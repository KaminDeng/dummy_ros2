"""障碍物真实场景差量、持久化和停车屏障回归。"""
import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

SOURCE = Path(__file__).resolve().parents[1]/'scripts/manual_scene_obstacles.py'
sys.path.insert(0, str(SOURCE.parent))
spec = importlib.util.spec_from_file_location('manual_scene_obstacles', SOURCE)
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)


def row(**kwargs):
    return dict(id='fixture', frame='base_link', x_mm=100., y_mm=-200., diameter_mm=50., height_mm=150., **kwargs)


def panel():
    calls = []
    client = NS(service_is_ready=lambda: True, call_async=lambda request: calls.append(request) or
                NS(done=lambda: False, result=lambda: NS(success=True)))
    state = dict(boot_epoch=1, reference_epoch=2, session_id=3, state_seq=11, motion_epoch=5,
        core_state=4, hold_ready=True, hold_valid_mask=63, position_valid_mask=63,
        velocity_valid_mask=63, hold_position_rad=[0.]*6, estimated_velocity_rad_s=[0.]*6)
    p = NS(node=NS(create_client=lambda *args: client,
                  create_publisher=lambda *args: NS(publish=lambda message: None)), last_state=state, actual_q=[0.]*6,
           fresh=lambda: True, busy=False, cancel_motion_hold=lambda: calls.append('CANCEL_HOLD'))
    from test_manual_moveit_service_topology import Graph
    graph = Graph()
    p.node.resolve_service_name = graph.resolve_service_name
    p.node.count_services = graph.count_services
    p.node.get_node_names_and_namespaces = graph.get_node_names_and_namespaces
    p.node.get_service_names_and_types_by_node = graph.get_service_names_and_types_by_node
    return p, calls


@pytest.mark.parametrize('key,value', [('height_mm',0), ('diameter_mm',-1),('x_mm',float('nan')),('height_mm',True),('frame','unknown')])
def test_invalid_geometry(key, value):
    item=row(); item[key]=value
    with pytest.raises(ValueError): m.validate_obstacles([item])


def test_cylinder_is_vertical_on_ground_and_scene_is_diff():
    item=row(); item['frame']='hole'
    scene=m.scene_message([item], {'fixture','deleted'}, {'hole':(20.,30.)})
    assert scene.is_diff
    assert {o.id for o in scene.world.collision_objects} == {'manual_cylinder_fixture','manual_cylinder_deleted'}
    obj=scene.world.collision_objects[-1]
    assert obj.primitives[0].dimensions == pytest.approx([.15,.025])
    pose=obj.primitive_poses[0]
    assert (pose.position.x,pose.position.y,pose.position.z) == pytest.approx((.12,-.17,.075))
    assert pose.orientation.w == 1.
    assert pose.position.z-obj.primitives[0].dimensions[0]/2 == 0.
    assert obj.header.frame_id == 'base_link'


def test_initial_load_requires_positive_service_ack_and_persists(tmp_path):
    p,calls=panel(); path=tmp_path/'obstacles.json'
    scene=m.ObstacleScene(p,path,{'base_link':(0.,0.)})
    assert not scene.scene_ready and scene.blocked
    scene.tick(); assert len(calls)==1 and not scene.scene_ready
    scene.phase='apply';scene.future=NS(done=lambda:True,result=lambda:NS(success=True))
    scene.tick(); assert scene.scene_ready and path.exists()
    scene.request_change([row()]); assert not scene.scene_ready and scene.scene_revision==1
    scene.tick(); scene.phase='apply';scene.future=NS(done=lambda:True,result=lambda:NS(success=True)); scene.tick()
    loaded=m.ObstacleScene(p,path,{'base_link':(0.,0.)})
    assert loaded.desired == [row()] and not loaded.scene_ready


def test_failed_service_never_opens_gate(tmp_path):
    p,_=panel(); scene=m.ObstacleScene(p,tmp_path/'s.json',{'base_link':(0.,0.)})
    scene.tick(); scene.phase='apply';scene.future=NS(done=lambda:True,result=lambda:NS(success=False));scene.tick()
    assert scene.blocked and not scene.scene_ready and scene.error
    assert not scene.path.exists()


def test_cancel_event_and_new_measured_hold_are_both_required(tmp_path):
    p,calls=panel(); scene=m.ObstacleScene(p,tmp_path/'s.json',{'base_link':(0.,0.)})
    p.busy=True
    scene.request_change([row()]); scene.tick()
    assert calls==['CANCEL_HOLD']
    scene.native_event('MOVE_CANCELED_HOLD',dict(boot='1',reference='2',session='3',state_seq='11',motion='5'))
    scene.tick(); assert calls==['CANCEL_HOLD']
    p.last_state['state_seq']=12; p.last_state['hold_ready']=False
    scene.tick(); assert calls==['CANCEL_HOLD']
    p.last_state['hold_ready']=True; p.last_state['estimated_velocity_rad_s'][0]=.02
    scene.tick(); assert calls==['CANCEL_HOLD']
    p.last_state['estimated_velocity_rad_s'][0]=0.
    scene.tick(); assert len(calls)==2 and not scene.wait_hold and not scene.scene_ready


def test_other_session_cannot_confirm_stop(tmp_path):
    p,calls=panel(); scene=m.ObstacleScene(p,tmp_path/'s.json',{'base_link':(0.,0.)})
    p.busy=True;scene.request_change([row()])
    scene.native_event('MOVE_CANCELED_HOLD',dict(boot='1',reference='2',session='99',state_seq='10',motion='5'))
    scene.tick(); assert calls==['CANCEL_HOLD'] and scene.wait_hold


def test_stop_timeout_cannot_be_retried_around_barrier(tmp_path):
    p,calls=panel(); scene=m.ObstacleScene(p,tmp_path/'s.json',{'base_link':(0.,0.)})
    p.busy=True;scene.request_change([row()]);scene.stop_deadline=0.;scene.tick()
    assert scene.blocked and scene.wait_hold and calls==['CANCEL_HOLD']
    with pytest.raises(ValueError):scene.request_change([])


def test_measured_origin_moves_all_references_invalidates_and_survives_reload(tmp_path):
    p,calls=panel(); invalidations=[]
    p.moveit_tab=NS(executing=False,auto_test=NS(active=False),scene_change_requested=lambda:invalidations.append(True))
    scene=m.ObstacleScene(p,tmp_path/'s.json',{'base_link':(0.,0.)})
    scene.request_origin_change('安装孔A',(20.,30.))
    item=row();item['frame']='安装孔A'
    scene.request_change([item]); old=m.cylinder_geometry(item,scene.frames)
    revision=scene.scene_revision
    scene.request_origin_change('安装孔A',(40.,50.))
    assert scene.scene_revision==revision+1 and scene.blocked and not scene.scene_ready
    assert len(invalidations)==3
    new=m.cylinder_geometry(item,scene.frames)
    assert new[:2] == pytest.approx((old[0]+.02,old[1]+.02))
    scene.tick();scene.phase='apply';scene.future=NS(done=lambda:True,result=lambda:NS(success=True));scene.tick()
    loaded=m.ObstacleScene(p,scene.path,{'base_link':(0.,0.)})
    assert loaded.frames['安装孔A']==(40.,50.) and loaded.desired[0]['frame']=='安装孔A'


def test_delete_referenced_origin_preserves_world_cylinder_and_still_invalidates(tmp_path):
    p,_=panel();scene=m.ObstacleScene(p,tmp_path/'s.json',{'base_link':(0.,0.),'孔':(20.,30.)})
    item=row();item['frame']='孔';scene.request_change([item]);before=m.cylinder_geometry(item,scene.frames)
    scene.request_origin_change('孔')
    assert '孔' not in scene.frames and scene.desired[0]['frame']=='base_link'
    assert m.cylinder_geometry(scene.desired[0],scene.frames)==pytest.approx(before)
    assert scene.scene_revision==2 and scene.blocked
    for offset in (None,(1.,2.)):
        with pytest.raises(ValueError):scene.request_origin_change('base_link',offset)


def test_referenced_origin_update_requires_stop_confirmation_when_running(tmp_path):
    p,calls=panel();scene=m.ObstacleScene(p,tmp_path/'s.json',{'base_link':(0.,0.),'孔':(20.,30.)})
    item=row();item['frame']='孔';scene.request_change([item]);p.busy=True
    scene.request_origin_change('孔',(40.,50.));scene.tick()
    assert calls==['CANCEL_HOLD'] and scene.wait_hold and scene.blocked


def test_offscreen_origin_editor_add_update_delete(tmp_path):
    from PyQt5 import QtWidgets
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    fake,_=panel();p=QtWidgets.QWidget()
    for key,value in vars(fake).items():setattr(p,key,value)
    p.cfg={'obstacles_file':str(tmp_path/'ui.json')}
    widget,scene=m.create_obstacle_tab(p)
    group=widget.findChildren(QtWidgets.QGroupBox)[0]
    name=group.findChild(QtWidgets.QLineEdit)
    coordinates=group.findChildren(QtWidgets.QDoubleSpinBox)
    buttons=group.findChildren(QtWidgets.QPushButton)
    name.setText('安装孔实测A');coordinates[0].setValue(25.);coordinates[1].setValue(-40.)
    buttons[0].click()
    assert scene.frames['安装孔实测A']==(25.,-40.) and scene.scene_revision==1
    assert widget.findChild(QtWidgets.QComboBox).currentText()=='安装孔实测A'
    coordinates[0].setValue(30.);buttons[0].click()
    assert scene.frames['安装孔实测A']==(30.,-40.) and scene.scene_revision==2
    buttons[1].click()
    assert '安装孔实测A' not in scene.frames and scene.scene_revision==3
    widget.deleteLater();p.deleteLater()


def test_acm_preserves_srdf_guard_pairs_and_only_allows_floor_cylinders():
    from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
    names=['base_link','link1','clearance_link1','link6_1_1']
    values=[[a==b or (a<2 and b<2) for b in range(4)] for a in range(4)]
    original=AllowedCollisionMatrix(entry_names=names,
        entry_values=[AllowedCollisionEntry(enabled=row) for row in values],
        default_entry_names=['existing_special'],default_entry_values=[True])
    result=m.obstacle_acm(original,{'a','b'})
    assert result.default_entry_names==original.default_entry_names
    assert result.default_entry_values==original.default_entry_values
    assert len(original.entry_names)==4
    for i in range(4): assert list(result.entry_values[i].enabled[:4])==values[i]
    floor=result.entry_names.index('manual_base_floor')
    for identifier in ('a','b'):
        cylinder=result.entry_names.index('manual_cylinder_'+identifier)
        assert result.entry_values[floor].enabled[cylinder]
        assert result.entry_values[cylinder].enabled[floor]
        for link in names:
            i=result.entry_names.index(link)
            assert not result.entry_values[i].enabled[cylinder]
            assert not result.entry_values[cylinder].enabled[i]


def test_get_acm_then_apply_are_both_required(tmp_path):
    from moveit_msgs.msg import PlanningScene, AllowedCollisionMatrix
    from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
    p,calls=panel();scene=m.ObstacleScene(p,tmp_path/'s.json',{'base_link':(0.,0.)})
    scene.request_change([row()]);scene.tick()
    assert isinstance(calls[-1],GetPlanningScene.Request)
    assert scene.blocked and scene.phase=='read'
    scene.future=NS(done=lambda:True,result=lambda:NS(scene=PlanningScene(allowed_collision_matrix=AllowedCollisionMatrix())))
    scene.tick()
    assert isinstance(calls[-1],ApplyPlanningScene.Request)
    acm=calls[-1].scene.allowed_collision_matrix
    assert acm.entry_values[0].enabled[1]
    assert scene.blocked and not scene.scene_ready
    scene.future=NS(done=lambda:True,result=lambda:NS(success=True));scene.tick()
    assert scene.scene_ready


@pytest.mark.parametrize('bad_response',[None,NS(scene=None)])
def test_failed_get_never_applies_or_opens_gate(tmp_path,bad_response):
    p,calls=panel();scene=m.ObstacleScene(p,tmp_path/'s.json',{'base_link':(0.,0.)})
    scene.tick();scene.future=NS(done=lambda:True,result=lambda:bad_response);scene.tick()
    assert len(calls)==1 and scene.error and scene.blocked and not scene.scene_ready


def test_get_timeout_keeps_transaction_blocked(tmp_path):
    p,calls=panel();scene=m.ObstacleScene(p,tmp_path/'s.json',{'base_link':(0.,0.)})
    scene.tick();scene.deadline=0.;scene.tick()
    assert scene.error and scene.blocked and not scene.scene_ready and scene.future is not None
    with pytest.raises(ValueError):scene.request_change([])


def test_remove_uses_actual_world_names_not_persisted_tombstones(tmp_path):
    """真实MoveIt拒绝REMOVE不存在对象：跨添加/删除/再修改/重启模拟此语义。"""
    from moveit_msgs.msg import PlanningScene, PlanningSceneWorld, CollisionObject
    from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
    from moveit_msgs.msg import PlanningSceneComponents
    world = {'external_fixture'}
    applied = []
    def call(request):
        if isinstance(request,GetPlanningScene.Request):
            assert request.components.components & PlanningSceneComponents.WORLD_OBJECT_NAMES
            response=NS(scene=PlanningScene(world=PlanningSceneWorld(collision_objects=[
                CollisionObject(id=name) for name in sorted(world)])))
        else:
            assert isinstance(request,ApplyPlanningScene.Request)
            for obj in request.scene.world.collision_objects:
                if obj.operation==CollisionObject.REMOVE:
                    assert obj.id in world, '真实MoveIt会拒绝重复REMOVE'
                    world.remove(obj.id)
                else:world.add(obj.id)
            applied.append(request.scene)
            response=NS(success=True)
        return NS(done=lambda:True,result=lambda:response)
    p,_=panel();client=NS(service_is_ready=lambda:True,call_async=call)
    p.node.create_client=lambda *args:client
    path=tmp_path/'world.json';scene=m.ObstacleScene(p,path,{'base_link':(0.,0.)})
    def sync(controller):
        for _ in range(3):controller.tick()
        assert controller.scene_ready and not controller.blocked
    scene.request_change([row()]);sync(scene)
    assert 'manual_cylinder_fixture' in world
    scene.request_change([]);sync(scene)
    assert 'manual_cylinder_fixture' not in world
    assert any(o.operation==CollisionObject.REMOVE for o in applied[-1].world.collision_objects)
    scene.request_origin_change('孔',(20.,30.));sync(scene)
    assert not applied[-1].world.collision_objects
    reloaded=m.ObstacleScene(p,path,{'base_link':(0.,0.)});sync(reloaded)
    assert not applied[-1].world.collision_objects
    assert reloaded.known_ids=={'fixture'} and world=={'external_fixture'}
