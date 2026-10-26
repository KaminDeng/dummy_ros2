"""Offline guard for the real six-axis action gate's activation boundary."""
##
# @file test_device_action_activation_wiring.py
# @brief device_action_gate activation/续租/释放布线的源码文本回归。
#
# 只读取C++并split/regex，不编译或实例化Gate，不执行USB/ROS。能证明要求字符串/局部顺序仍存在，不能证明所有分支安全、实时期限或运行中的来源匹配；原文本oracle受源码布局影响。

import re
from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "src/device_action_gate.cpp"
REQUIRED = (
    "activation_path", "activation_sha256", "source_bundle_root",
    "prebuild_bundle_manifest_path", "compiled_policy_manifest_path",
    "firmware_image_path", "host_executable_path", "map_file_path",
    "resource_report_path", "d_regression_report_path",
    "d_regression_provenance_path", "hil_report_path", "hil_raw_index_path",
    "hil_evidence_root", "elf_path", "resource_base_root", "usb_serial",
)


##
# @brief transport构造前必须见全部activation参数、functional依据及带/不带activation两load路径，未执行loader。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_real_profile_requires_explicit_bound_activation_before_transport():
    source = SOURCE.read_text()
    pre_transport = source.split("transport_=std::make_unique<DpManagementTransport>", 1)[0]

    assert 'profile_name=="f405_6dof_bench"' in pre_transport
    for name in REQUIRED:
        assert f'"{name}"' in pre_transport, name
    assert "load_device_profile(bundle,profile_name,parameters_.profile,parameters_.mask,activation)" in pre_transport
    assert "device_profile_.functional_development_basis" in pre_transport
    assert "load_device_profile(bundle,profile_name,parameters_.profile,parameters_.mask);" in pre_transport


##
# @brief read_observation文本内身份匹配调用必须先于bootstrap observe，纯顺序检查。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_status_identity_is_checked_before_bootstrap_operations():
    source = SOURCE.read_text()
    observation = source.split("bool read_observation", 1)[1]
    assert observation.index("status_matches_bundle_identity(device_profile_,status_identity)") < observation.index(
        "bootstrap_->observe(latest_bootstrap_,now)")


##
# @brief Release回调文本queued/success false且无while/sleep，status另给pending/unconfirmed/最终观测门；未测回调时延。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_release_callback_only_queues_and_status_reports_terminal_result():
    source = SOURCE.read_text()
    callback = source.split('"~/disable_and_release"', 1)[1].split("server_=", 1)[0]
    status = source.split('"~/bootstrap_status"', 1)[1].split("const auto add_operation_service", 1)[0]

    assert 'response->message="RELEASE_QUEUED"' in callback
    assert "response->success=false" in callback
    assert "while(" not in callback
    assert "sleep_for" not in callback
    assert 'response->success=release==3 && observation_ready_.load()' in status
    assert '"RELEASE_PENDING"' in status
    assert '"RELEASE_UNCONFIRMED"' in status


##
# @brief 查30s lease配置、renewal_due、许可刷新/双Hold/post_complete门存在，不真实续租。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_application_hold_renews_once_in_due_window_without_restoring_permit():
    source = SOURCE.read_text()
    constructor = source.split("bootstrap_=std::make_unique<BootstrapCoordinator>", 1)[1].split(
        "gate_=std::make_unique<ActionGate>", 1)[0]
    observation = source.split("bool read_observation", 1)[1].split("void bootstrap_tick", 1)[0]

    assert "(device_profile_.functional_development_basis || manual_reference_mode_) ? 30000000U : 0U" in constructor
    assert "bootstrap_->renewal_due(now,requested_lease_us_)" in observation
    assert "gate_->permit_refresh_allowed()" in observation
    assert "reserve_lease_renewal()" in observation
    assert "status.core_state()==wire_value(WireCoreState::kHold)" in observation
    assert "state.core_state()==wire_value(WireCoreState::kHold)" in observation
    assert "transport_->post_complete_begin_ready()" in observation


##
# @brief regex定位实际cycle_impl并核Fault撤权/ready与reservation状态文本，不能当完整状态机实测。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_renewal_rejection_and_deadline_fault_cannot_leave_gate_ready():
    source = SOURCE.read_text()
    dispatch = source.split("const int requested=management_request_.exchange(0)", 1)[1].split(
        "port_.identity.boot=", 1)[0]
    tick = source.split("void bootstrap_tick", 1)[1].split("void check_execution", 1)[0]
    cycle_match = re.search(r"\bvoid\s+cycle_impl\s*\([^)]*\)\s*\{(?P<body>.*?)^  \}", source, re.S | re.M)
    assert cycle_match, "cycle_impl definition missing"
    cycle = cycle_match.group("body")
    status = source.split('"~/bootstrap_status"', 1)[1].split("const auto add_operation_service", 1)[0]

    assert 'bootstrap_->invalidate("LEASE_RENEWAL_REJECTED")' in dispatch
    assert "bootstrap_->phase()==BootstrapPhase::kFault" in tick
    assert "gate_->invalidate_authorization(now)" in tick
    assert re.search(r"const bool fault\s*=\s*bootstrap_->phase\(\)\s*==\s*BootstrapPhase::kFault", cycle)
    assert re.search(r"admission_prerequisites_\s*=\s*prerequisites\.ready\(\)\s*&&\s*!fault\s*;", cycle)
    assert re.search(r"ready_\s*=\s*admission_prerequisites_\.load\(\)\s*&&", cycle)
    assert "renewal_reserved_.load()" in status
    assert '"LEASE_RENEWAL_PENDING"' in status
    assert '"LEASE_RENEWED"' in status


##
# @brief renewal_complete表达式须在Hold且不依赖gate ready，renewal结果与新运动permit不同。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_committed_lease_renewal_reports_success_without_new_motion_permit():
    status = SOURCE.read_text().split('"~/bootstrap_status"', 1)[1].split(
        "const auto add_operation_service", 1)[0]
    renewal = status.split("const bool renewal_complete=", 1)[1].split(";", 1)[0]
    success = status.split("response->success=observation_ready_", 1)[1].split(";", 1)[0]

    assert "gate_->ready()" not in renewal
    assert "gate_->phase()==Phase::kHold" in renewal
    assert "renewal_complete" in success
