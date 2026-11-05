"""真实中央ROSIDL类验证面板的presence、Source和关节质量门；不启动网络。"""
import math
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'scripts'))
from manual_panel_root import (valid_state,state_view,source_identity,control_identity,ordered_joints)
from dpsdk_interfaces.msg import DummyJointState,DummyJointStateRecord


def state():
    value=DummyJointState()
    # 真生成类的has位，而非旧协议DTO；显式零必须仍有效。
    for name in value.get_fields_and_field_types():
        if name.startswith('has_'):setattr(value,name,True)
    value.profile_id=201
    value.source.device=list(range(1,17));value.source.boot_lo=13;value.source.boot_hi=256;value.source.context_id=7
    rows=[]
    for axis in range(6):
        row=DummyJointStateRecord()
        for name in row.get_fields_and_field_types():
            if name.startswith('has_'):setattr(row,name,True)
        row.joint_id=axis;row.position_rad=float(axis)
        row.configured_quality=True;row.position_valid_quality=True;row.velocity_valid_quality=True
        row.raw_position_valid_quality=True;row.hold_valid_quality=True
        rows.append(row)
    value.joints=rows
    return value


class CurrentRootPanel(unittest.TestCase):
    def test_actual_rosidl_complete_and_zero_presence(self):
        value=state();self.assertTrue(valid_state(value));self.assertEqual(state_view(value)['position_valid_mask'],63)
        self.assertEqual(source_identity(value)[1],(256<<32)|13)
        self.assertEqual(state_view(value)['session_id'],0)
        value.has_session_id=False;self.assertFalse(valid_state(value))

    def test_full_source_context_and_boot_change_identity(self):
        value=state();first=control_identity(value)
        value.source.context_id+=1;self.assertNotEqual(first,control_identity(value))
        value.source.context_id-=1;value.source.boot_hi+=1;self.assertNotEqual(first,control_identity(value))
        value.source.device=[0]*16;self.assertFalse(valid_state(value))

    def test_joint_ids_reorder_and_duplicate_are_real_records(self):
        value=state();value.joints=list(reversed(value.joints))
        self.assertEqual([x.joint_id for x in ordered_joints(value)],list(range(6)))
        self.assertEqual(state_view(value)['position_rad'],tuple(float(x) for x in range(6)))
        value.joints[0].joint_id=value.joints[1].joint_id;self.assertFalse(valid_state(value))

    def test_quality_invalid_nan_preserves_state_but_valid_nan_rejected(self):
        value=state();value.joints[0].position_rad=float('nan');value.joints[0].position_valid_quality=False
        view=state_view(value);self.assertEqual(view['position_valid_mask'],62);self.assertTrue(math.isnan(view['position_rad'][0]))
        value.joints[0].position_valid_quality=True
        with self.assertRaises(ValueError):state_view(value)

    def test_joint_missing_presence_and_source_are_refused(self):
        value=state();value.joints[0].has_position_rad=False;self.assertFalse(valid_state(value))
        value=state();value.source.context_id=0;self.assertFalse(valid_state(value))
        with self.assertRaises(ValueError):source_identity({'source':{'device':'01'*15,'boot_id':2,'context_id':3}})


    def test_actual_root_projects_freshness_and_hold_fields_in_joint_order(self):
        value=state();value.core_state=4;value.hold_ready=True
        for axis,row in enumerate(value.joints):
            row.can_rx_age_us=axis*1000;row.hold_position_rad=row.position_rad
        value.joints=list(reversed(value.joints))
        view=state_view(value)
        self.assertEqual(view['can_rx_age_us'],(0,1000,2000,3000,4000,5000))
        self.assertEqual(view['hold_position_rad'],tuple(float(i)for i in range(6)))
        self.assertEqual(view['hold_valid_mask'],63)
        # 运行实际MoveItTab.ready纯决策函数，不创建Qt窗口或ROS节点。
        import ast
        from types import SimpleNamespace as NS
        source=pathlib.Path(__file__).resolve().parents[1]/'scripts/manual_moveit_tab.py'
        cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef)and n.name=='MoveItTab')
        ready=next(n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name=='ready')
        namespace={'math':math};exec(compile(ast.Module(body=[ready],type_ignores=[]),str(source),'exec'),namespace)
        panel=NS(last_state=view,actual_q=list(view['position_rad']))
        tab=NS(panel=panel,request_ready=lambda:True,auto_test=NS(active=False))
        self.assertTrue(namespace['ready'](tab))
        value.joints[0].hold_valid_quality=False;value.joints[0].hold_position_rad=float('nan')
        panel.last_state=state_view(value)
        self.assertFalse(namespace['ready'](tab))

    def test_hold_invalid_nan_is_retained_but_cannot_be_used_as_valid_hold(self):
        value=state();value.joints[2].hold_position_rad=float('nan');value.joints[2].hold_valid_quality=False
        view=state_view(value)
        self.assertEqual(view['hold_valid_mask'],59)
        self.assertTrue(math.isnan(view['hold_position_rad'][2]))
        value.joints[2].hold_valid_quality=True
        with self.assertRaises(ValueError):state_view(value)

    def test_freshness_and_hold_missing_presence_are_not_filled(self):
        for name in ('can_rx_age_us','hold_position_rad','hold_valid_quality'):
            value=state();setattr(value.joints[0],'has_'+name,False)
            self.assertFalse(valid_state(value))
            with self.assertRaises(ValueError):state_view(value)
        value=state()
        for row in value.joints:row.hold_valid_quality=False
        view=state_view(value)
        self.assertEqual(view['can_rx_age_us'],(0,)*6)
        self.assertEqual(view['hold_valid_mask'],0)

if __name__=='__main__':unittest.main()
