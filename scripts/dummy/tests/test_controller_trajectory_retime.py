"""真实Jazzy JTC样条/20ms下发验证；不创建ROS节点或访问设备。"""
import importlib.util
import json
import math
from pathlib import Path
import subprocess
from types import SimpleNamespace as NS

import pytest

SCRIPT=Path(__file__).resolve().parents[1]/'ros_six_axis_movej.py'
spec=importlib.util.spec_from_file_location('movej_retime_test',SCRIPT)
movej=importlib.util.module_from_spec(spec);spec.loader.exec_module(movej)


def trajectory():
    points=[]
    for t,q,v,a in [(0,0.,0.,4.),(200_000_000,.1,.5,0.),(400_000_000,.15,0.,-4.)]:
        sec,ns=divmod(t,10**9)
        points.append(NS(positions=[q]+[0.]*5,velocities=[v]+[0.]*5,accelerations=[a]+[0.]*5,
                         effort=[],time_from_start=NS(sec=sec,nanosec=ns)))
    return NS(joint_names=[f'Joint{i}'for i in range(1,7)],points=points,
              header=NS(frame_id='',stamp=NS(sec=0,nanosec=0)))


def payload(msg):
    return {'points':[dict(t=p.time_from_start.sec*10**9+p.time_from_start.nanosec,
            p=p.positions,v=p.velocities,a=p.accelerations)for p in msg.points]}


@pytest.fixture(scope='session')
def jtc_oracle(tmp_path_factory):
    root=tmp_path_factory.mktemp('real-jtc-retime')
    (root/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.20)
project(jtc_retime_oracle CXX)
find_package(ament_cmake REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
add_executable(oracle oracle.cpp)
target_compile_features(oracle PRIVATE cxx_std_17)
ament_target_dependencies(oracle joint_trajectory_controller rclcpp trajectory_msgs)
''')
    (root/'oracle.cpp').write_text(r'''// 测试容器只传真实ROS轨迹字段；采样调用已安装Jazzy JTC，不复刻插值实现。
#include <joint_trajectory_controller/trajectory.hpp>
#include <joint_trajectory_controller/interpolation_methods.hpp>
#include <iomanip>
#include <iostream>
#include <cmath>

int main(){
 std::size_t count;std::cin>>count;if(count<2 || count>512)return 1;
 struct Input{std::int64_t t;std::array<double,6>p,v,a;};std::vector<Input> data(count);
 for(auto& row:data){std::cin>>row.t;for(auto* field:{&row.p,&row.v,&row.a})for(auto& value:*field)std::cin>>value;}
 std::array<double,6> vmax{},amax{};
 for(std::int64_t phase=0;phase<20000000LL;phase+=1000000LL){
  auto msg=std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  msg->joint_names={"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6"};
  for(const auto& row:data){trajectory_msgs::msg::JointTrajectoryPoint p;
   p.positions.assign(row.p.begin(),row.p.end());p.velocities.assign(row.v.begin(),row.v.end());p.accelerations.assign(row.a.begin(),row.a.end());
   p.time_from_start=rclcpp::Duration::from_nanoseconds(row.t);msg->points.push_back(p);}
  const rclcpp::Time origin(1000000000LL,RCL_ROS_TIME);
  trajectory_msgs::msg::JointTrajectoryPoint hold;hold.positions=msg->points.front().positions;hold.velocities.assign(6,0.);hold.accelerations.assign(6,0.);
  joint_trajectory_controller::Trajectory tr(origin,hold,msg);
  // 真JTC零时刻首点选择：与不同controller前点对照，不能臆造1ms前段。
  if(data.front().t==0){
   auto different=hold;for(auto& p:different.positions)p+=.123;
   different.velocities.assign(6,.5);different.accelerations.assign(6,-.5);
   joint_trajectory_controller::Trajectory probe(origin,different,msg);
   trajectory_msgs::msg::JointTrajectoryPoint at_zero;
   joint_trajectory_controller::TrajectoryPointConstIter pa,pb;
   if(!probe.sample(origin,joint_trajectory_controller::interpolation_methods::InterpolationMethod::VARIABLE_DEGREE_SPLINE,at_zero,pa,pb))return 3;
   for(std::size_t i=0;i<6;++i)if(std::abs(at_zero.positions[i]-msg->points.front().positions[i])>1e-12)return 4;
  }
  // header.stamp=0的真实JTC首次sample会锁定起点；先在origin初始化，
  // 再移动采样相位，避免把每个phase错误地重新当成轨迹零点。
  trajectory_msgs::msg::JointTrajectoryPoint prime;
  joint_trajectory_controller::TrajectoryPointConstIter prime_a,prime_b;
  if(!tr.sample(origin,joint_trajectory_controller::interpolation_methods::InterpolationMethod::VARIABLE_DEGREE_SPLINE,prime,prime_a,prime_b))return 5;
  std::array<double,6> prior{},velocity{};std::copy(hold.positions.begin(),hold.positions.end(),prior.begin());
  const auto end=rclcpp::Duration(msg->points.back().time_from_start).nanoseconds();
  for(std::int64_t ns=phase;ns<=end+60000000LL;ns+=20000000LL){
   trajectory_msgs::msg::JointTrajectoryPoint sample;joint_trajectory_controller::TrajectoryPointConstIter a,b;
   if(!tr.sample(origin+rclcpp::Duration::from_nanoseconds(ns),joint_trajectory_controller::interpolation_methods::InterpolationMethod::VARIABLE_DEGREE_SPLINE,sample,a,b))return 2;
   for(std::size_t i=0;i<6;++i){const double v=(sample.positions[i]-prior[i])/.02;
    vmax[i]=std::max(vmax[i],std::abs(v));amax[i]=std::max(amax[i],std::abs(v-velocity[i])/.02);prior[i]=sample.positions[i];velocity[i]=v;}
  }
 }
 std::cout<<std::setprecision(17);for(auto v:vmax)std::cout<<v<<" ";for(auto a:amax)std::cout<<a<<" ";std::cout<<"\n";
}''')
    subprocess.run(['cmake','-S',str(root),'-B',str(root/'build')],check=True,capture_output=True)
    subprocess.run(['cmake','--build',str(root/'build'),'--parallel','22'],check=True,capture_output=True)
    def run(msg):
        rows=payload(msg)['points']
        encoded=str(len(rows))+'\n'+'\n'.join(' '.join(map(str,[r['t'],*r['p'],*r['v'],*r['a']]))for r in rows)+'\n'
        p=subprocess.run([str(root/'build/oracle')],input=encoded,capture_output=True,text=True,check=True)
        values=list(map(float,p.stdout.split()));assert len(values)==12
        return {'v':values[:6],'a':values[6:]}
    return run


def test_real_jtc_exposes_pointwise_limit_gap_and_retime_fixes_twenty_ms(jtc_oracle):
    original=trajectory();before=jtc_oracle(original)
    assert before['a'][0]>4.0  # 真实JTC点间采样越帽，不能靠点值4证明可执行。
    result,proof=movej.retime_moveit_trajectory(original,velocity_caps=[math.radians(60)]*6,acceleration_caps=[4.]*6)
    after=jtc_oracle(result)
    assert after['a'][0]<=4.0 and max(after['v'])<=math.radians(60)
    assert proof['time_scale']>1.0
    assert [p.positions for p in result.points]==[p.positions for p in original.points]
    assert original.points[-1].time_from_start.nanosec==400_000_000


def test_missing_cap_nan_and_nonmonotonic_times_reject_without_mutation():
    p=trajectory();p.points[1].accelerations[0]=float('nan')
    with pytest.raises(ValueError):movej.retime_moveit_trajectory(p)
    p=trajectory();p.points[1].time_from_start.nanosec=0
    with pytest.raises(ValueError):movej.retime_moveit_trajectory(p)


def test_stationary_axes_and_cdr_float64_roundtrip(jtc_oracle):
    from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
    from rclpy.serialization import serialize_message,deserialize_message
    msg=JointTrajectory();msg.joint_names=trajectory().joint_names
    for src in trajectory().points:
        p=JointTrajectoryPoint();p.positions=src.positions;p.velocities=src.velocities;p.accelerations=src.accelerations
        p.time_from_start.sec=src.time_from_start.sec;p.time_from_start.nanosec=src.time_from_start.nanosec;msg.points.append(p)
    result,proof=movej.retime_moveit_trajectory(msg)
    actual=deserialize_message(serialize_message(result),JointTrajectory)
    assert payload(actual)==payload(result)
    sampled=jtc_oracle(actual)
    assert max(sampled['a'])<=4.0 and max(sampled['v'])<=math.radians(60)
    assert all(list(p.positions[1:])==[0.]*5 for p in actual.points)
    assert proof['sampled_max_acceleration'][0]<=4.0


def test_actual_execute_goal_sends_retimed_copy_and_reports_original_identity(monkeypatch):
    spec=importlib.util.spec_from_file_location('existing_movej_test',SCRIPT.parent/'tests/test_ros_six_axis_movej.py')
    existing=importlib.util.module_from_spec(spec);spec.loader.exec_module(existing)
    target,planned,planner,client,adapter,events,sent=existing.admission_case(monkeypatch,(),1)
    source=trajectory();source.points[-1].positions[0]=0.0
    for point in source.points:point.positions[2]=target[2]
    planned.points=source.points
    original=existing.movej.trajectory_digest(planned)
    report=existing.movej.execute_goal(object(),planner,client,target,existing.movej.LIMITS,
                                      existing.movej.time.monotonic()+5,adapter,
                                      velocity_scaling=60/63,acceleration_scaling=1.0)
    assert len(sent)==1 and sent[0].trajectory is not planned
    assert planned.points[-1].time_from_start.nanosec==400_000_000
    assert report['trajectory_retime']['original_trajectory_sha256']==original
    assert report['trajectory_retime']['time_scale']>1.0
    assert report['trajectory_sha256']==existing.movej.trajectory_digest(sent[0].trajectory)
    assert report['trajectory_retime']['sampled_max_acceleration'][0]<=4.0


def captured_home_trajectory():
    """载入实际规划原件字段，来源CDR指纹固定；不使用硬件或ROS节点。"""
    from trajectory_msgs.msg import JointTrajectory
    from rosidl_runtime_py.set_message import set_message_fields
    raw=json.loads((Path(__file__).parent/'fixtures/home_moveit_nonstationary_start.json').read_text())
    msg=JointTrajectory();set_message_fields(msg,raw['trajectory'])
    return msg


def test_actual_moveit_nonzero_start_preserves_tangent_and_jtc_caps(jtc_oracle):
    original=captured_home_trajectory();before=payload(original)
    assert before['points'][0]['t']==0  # 实际首点不是1ms；v/a的1ms比例不是时间戳。
    assert max(map(abs,original.points[0].velocities))>0.003
    assert max(map(abs,original.points[-1].velocities))<1e-10
    result,proof=movej.retime_moveit_trajectory(original)
    assert payload(original)==before
    assert [list(p.positions) for p in result.points]==[list(p.positions) for p in original.points]
    assert result.points[0].time_from_start.sec==0 and result.points[0].time_from_start.nanosec==0
    assert list(result.points[0].velocities)==[v/proof['time_scale'] for v in original.points[0].velocities]
    assert all(v!=0 for v in result.points[0].velocities)  # 不清零实际原首切向。
    actual=jtc_oracle(result)
    assert max(actual['a'])<=4.0 and max(actual['v'])<=math.radians(63)
    assert max(proof['continuous_max_acceleration'])<=4.0
    assert max(proof['sampled_acceleration_bound_all_phases'])<=4.0
    assert proof['hold_boundary_velocity_jump'][2]>0


def test_terminal_moving_trajectory_still_rejected_and_input_intact():
    original=captured_home_trajectory();original.points[-1].velocities[2]=0.1;before=payload(original)
    with pytest.raises(ValueError,match='stationary end'):
        movej.retime_moveit_trajectory(original)
    assert payload(original)==before


def test_positive_first_point_time_not_silently_shifted():
    original=captured_home_trajectory();original.points[0].time_from_start.nanosec=1_000_000
    before=payload(original)
    with pytest.raises(ValueError,match='zero origin'):
        movej.retime_moveit_trajectory(original)
    assert payload(original)==before
