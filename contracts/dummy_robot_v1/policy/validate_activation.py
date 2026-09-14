#!/usr/bin/env python3
"""Strict structural validator for Dummy policy and activation artifacts."""

import argparse, hashlib, json, math, struct, sys
from datetime import datetime, timezone
from pathlib import Path

HEX=set("0123456789abcdef")

def strict_load(path):
    def pairs(items):
        result={}
        for key,value in items:
            if key in result: raise ValueError(f"duplicate key: {key}")
            result[key]=value
        return result
    return json.loads(path.read_text(),object_pairs_hook=pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))

def is_hash(value): return isinstance(value,str) and len(value)==64 and set(value)<=HEX
def canonical_policy(policy):
    value={key:item for key,item in policy.items() if key!="note"}
    return json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False).encode()
def parse_time(value):
    if not isinstance(value,str): raise ValueError("timestamp type")
    parsed=datetime.fromisoformat(value)
    if parsed.tzinfo is None or parsed.utcoffset() is None: raise ValueError("timezone required")
    return parsed.astimezone(timezone.utc)
def positive_int(value): return isinstance(value,int) and not isinstance(value,bool) and value>0

def validate(activation,policy,runtime,evidence_root):
    limit_keys={"position_min_rad","position_max_rad","velocity_max_rad_s","acceleration_max_rad_s2",
        "stop_deceleration_rad_s2","max_trial_travel_rad","stop_max_distance_rad","stationary_position_tolerance_rad",
        "stationary_velocity_tolerance_rad_s","max_tracking_error_rad","max_feedback_step_rad","can_velocity_cap_rev_s",
        "first_sample_position_tolerance_rad","hold_position_tolerance_rad","tracking_uncertainty_margin_rad"}
    timing_keys={"control_period_us","state_publish_period_us","command_timeout_us","freshness_ticket_window_us",
        "can_feedback_timeout_us","stationary_window_us","stop_max_duration_us","trial_max_duration_us",
        "first_sample_timeout_us","max_can_tx_residence_us","tracking_error_window_us","max_control_dt_us",
        "state_query_refresh_us","completion_max_duration_us"}
    capacity_keys={"ticket_ring_slots","management_client_slots","operation_result_slots","operation_retention_us",
                   "permit_ledger_slots","minimum_stationary_samples"}
    policy_keys={"format","profile_id","motion_supported","test_only","active_axes","mapping_source","reference_kind",
                 "limits","timing","capacity","note","model_sha256"}
    activation_keys={"format","motion_permitted","test_only","approval_state","device_id","firmware_sha256",
        "contract_sha256","profile_sha256","active_axes","reference_kind","limits","timing","evidence",
        "operator_acknowledgement","note","schema_revision","expected_boot_epoch_hex","not_before","expires_at",
        "reviewed_at","capacity","expected_reference_epoch"}
    if set(policy)!=policy_keys or set(activation)!=activation_keys or policy.get("format")!="dummy_j1_compiled_policy_v1" or activation.get("format")!="dummy_j1_activation_v1":
        raise ValueError("format")
    if type(policy.get("motion_supported")) is not bool or type(policy.get("test_only")) is not bool or \
       type(activation.get("motion_permitted")) is not bool or type(activation.get("test_only")) is not bool:
        raise ValueError("boolean type")
    if policy["test_only"] or activation["test_only"] or policy["active_axes"]!=["Joint1"] or activation["active_axes"]!=["Joint1"] or \
       policy["reference_kind"]!="BENCH_SESSION" or activation["reference_kind"]!="BENCH_SESSION": raise ValueError("identity")
    if set(policy["limits"])!=limit_keys or set(activation["limits"])!=limit_keys or set(policy["timing"])!=timing_keys or \
       set(activation["timing"])!=timing_keys or set(policy["capacity"])!=capacity_keys or set(activation["capacity"])!=capacity_keys:
        raise ValueError("parameter fields")
    for group,keys in (("limits",limit_keys),("timing",timing_keys),("capacity",capacity_keys)):
        for key in keys:
            value=policy[group][key]
            if group in {"timing","capacity"} and not positive_int(value): raise ValueError(key)
            if group=="limits" and (not isinstance(value,(int,float)) or isinstance(value,bool) or not math.isfinite(value) or \
               (key!="position_min_rad" and value<=0)): raise ValueError(key)
            activation_value=activation[group][key]
            if group in {"timing","capacity"} and not positive_int(activation_value): raise ValueError(f"activation {key}")
            if group=="limits" and (not isinstance(activation_value,(int,float)) or isinstance(activation_value,bool) or not math.isfinite(activation_value)):
                raise ValueError(f"activation {key}")
            if activation[group][key]!=value: raise ValueError("activation parameter mismatch")
    if not positive_int(policy.get("profile_id")) or policy.get("mapping_source")!="model-v1" or not is_hash(policy.get("model_sha256")):
        raise ValueError("policy identity")
    timing=policy["timing"]; capacity=policy["capacity"]
    if timing["state_publish_period_us"] < timing["control_period_us"]:
        raise ValueError("state faster than control")
    if capacity["ticket_ring_slots"]*timing["state_publish_period_us"] <= timing["freshness_ticket_window_us"]:
        raise ValueError("ticket ring overwrites live tickets")
    limits=policy["limits"]
    if limits["position_min_rad"]>=limits["position_max_rad"] or limits["hold_position_tolerance_rad"]>limits["stationary_position_tolerance_rad"] or \
       limits["stop_max_distance_rad"]>limits["max_trial_travel_rad"] or timing["stop_max_duration_us"]>timing["trial_max_duration_us"]:
        raise ValueError("cross constraint")
    latency=(timing["freshness_ticket_window_us"]+timing["command_timeout_us"]+timing["max_control_dt_us"]+timing["max_can_tx_residence_us"])/1_000_000.0
    stop_distance=limits["velocity_max_rad_s"]*latency + limits["velocity_max_rad_s"]**2/(2.0*limits["stop_deceleration_rad_s2"]) + limits["tracking_uncertainty_margin_rad"]
    if limits["stop_max_distance_rad"] < stop_distance: raise ValueError("insufficient stop envelope")
    cap32=struct.unpack("<f",struct.pack("<f",limits["can_velocity_cap_rev_s"]))[0]
    if not math.isfinite(cap32) or cap32*51200.0<1.0 or cap32*51200.0>=2147483648.0:
        raise ValueError("CAN cap")
    policy_hash=hashlib.sha256(canonical_policy(policy)).hexdigest()
    for key in ("device_id","firmware_sha256","contract_sha256","profile_sha256"):
        if not is_hash(activation.get(key)): raise ValueError(key)
    if activation["approval_state"] not in {"TRIAL_APPROVED","MEASURED_APPROVED"} or not isinstance(activation.get("operator_acknowledgement"),str) or not activation["operator_acknowledgement"]:
        raise ValueError("approval")
    try: boot=int(activation["expected_boot_epoch_hex"],16)
    except (TypeError,ValueError): raise ValueError("boot epoch")
    if boot<=0 or boot>0xffffffffffffffff or not positive_int(activation.get("expected_reference_epoch")) or activation["expected_reference_epoch"]>0xffffffff:
        raise ValueError("runtime epoch")
    not_before,expires,reviewed=parse_time(activation["not_before"]),parse_time(activation["expires_at"]),parse_time(activation["reviewed_at"])
    now=datetime.now(timezone.utc)
    if not_before>now or expires<=now or expires<=not_before or reviewed>not_before or (expires-now).total_seconds()*1e6 <= timing["stop_max_duration_us"]:
        raise ValueError("permit time")
    if activation["profile_sha256"]!=policy_hash: raise ValueError("compiled policy mismatch")
    expected_runtime={"device_id":activation["device_id"],"firmware_sha256":activation["firmware_sha256"],
        "contract_sha256":activation["contract_sha256"],"profile_sha256":activation["profile_sha256"],
        "active_axes":["Joint1"],"reference_kind":"BENCH_SESSION","boot_epoch":boot,
        "reference_epoch":activation["expected_reference_epoch"]}
    if runtime!=expected_runtime: raise ValueError("runtime mismatch")
    root=evidence_root.resolve(strict=True)
    evidence=activation.get("evidence")
    if not isinstance(evidence,list) or not evidence: raise ValueError("evidence")
    for entry in evidence:
        relative=Path(entry["path"])
        if relative.is_absolute() or ".." in relative.parts or not is_hash(entry.get("sha256")):
            raise ValueError("evidence path/hash")
        candidate=root/relative
        if candidate.is_symlink() or not candidate.resolve(strict=True).is_relative_to(root):
            raise ValueError("evidence escape")
        if hashlib.sha256(candidate.read_bytes()).hexdigest()!=entry["sha256"]:
            raise ValueError("evidence changed")
    return {"validation":"PASS","motion_allowed":False,"decision":"STRUCTURALLY_VALID_NOT_AUTHORIZED"}

def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--check-json",type=Path)
    parser.add_argument("--activation",type=Path); parser.add_argument("--policy",type=Path)
    parser.add_argument("--runtime",type=Path); parser.add_argument("--evidence-root",type=Path)
    parser.add_argument("--consumed-ledger",type=Path)
    args=parser.parse_args()
    try:
        if args.check_json: strict_load(args.check_json); return 0
        if not all((args.activation,args.policy,args.runtime,args.evidence_root)): return 2
        activation=strict_load(args.activation)
        if not args.consumed_ledger: raise ValueError("consumed ledger required")
        permit_key=f'{activation.get("device_id")}:{activation.get("expected_boot_epoch_hex")}:{activation.get("profile_sha256")}:{activation.get("expires_at")}'
        if permit_key in strict_load(args.consumed_ledger):
            raise ValueError("permit already consumed")
        print(json.dumps(validate(activation,strict_load(args.policy),strict_load(args.runtime),args.evidence_root),sort_keys=True))
        return 0
    except (OSError,KeyError,TypeError,ValueError,json.JSONDecodeError) as error:
        print(f"DENIED: {error}",file=sys.stderr); return 1
if __name__=="__main__": raise SystemExit(main())
