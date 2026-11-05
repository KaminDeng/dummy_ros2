"""当前Root/ROS消息的presence、真实来源及少量面板显示视图；不构造旧协议DTO。"""
import math

VIEW_FORMAT = 'dummy.current-root.state-view'
_STATE_SCALARS = ('session_id','reference_epoch','motion_epoch','management_epoch','profile_id',
                  'state_seq','core_state','hold_ready','first_failure_reason')
_JOINT_FIELDS = ('configured_quality','position_valid_quality','velocity_valid_quality',
                 'raw_position_valid_quality','feedback_seq','can_rx_age_us','position_rad',
                 'estimated_velocity_rad_s','raw_motor_position_rev','hold_position_rad','hold_valid_quality')
_STATUS_REQUIRED = ('profile_id','session_id','reference_epoch','motion_epoch','management_epoch',
                    'core_state','real_motion_permitted','permit_remaining_us','release_committed',
                    'first_failure_reason','evidence_lost','operation_id','transaction_phase')


def field(value, name):
    """None/has=false是缺失；显式0/False仍是实际值。"""
    if isinstance(value, dict):
        result = value.get(name)
    else:
        if not getattr(value, 'has_'+name, False):
            raise ValueError('Root字段缺失: '+name)
        result = getattr(value, name)
    if result is None:
        raise ValueError('Root字段缺失: '+name)
    return result


def source_identity(value):
    """使用SDK Source，禁止用domain_device_id/旧boot字段或topic推断来源。"""
    source = value.get('source') if isinstance(value,dict) else value.source
    if source is None:
        raise ValueError('Root缺少Source')
    def get(name):
        return source[name] if isinstance(source,dict) else getattr(source,name)
    device = get('device')
    device = bytes.fromhex(device) if isinstance(device,str) else bytes(device)
    boot = (get('boot_id') if isinstance(source,dict) and 'boot_id' in source else
            int(get('boot_lo')) | (int(get('boot_hi')) << 32))
    context_value=get('context_id')
    if isinstance(source,dict) and (type(context_value) is not int or type(boot) is not int):
        raise ValueError('Root Source须有整数boot/context')
    context = int(context_value)
    if len(device)!=16 or not any(device) or not 0<boot<1<<64 or not 0<context<1<<32:
        raise ValueError('Root Source device/boot/context非法')
    return tuple(device),boot,context


def ordered_joints(state):
    rows = field(state,'joints')
    if len(rows)!=6:
        raise ValueError('Root joints须有六条真实record')
    result=[None]*6
    for row in rows:
        ident=int(field(row,'joint_id'))
        if not 0<=ident<6 or result[ident] is not None:
            raise ValueError('Root joint_id重复或越界')
        for name in _JOINT_FIELDS:
            field(row,name)
        result[ident]=row
    return result


def joint_values(state, name):
    return tuple(field(row,name) for row in ordered_joints(state))


def quality_mask(state, name):
    return sum(1<<axis for axis,row in enumerate(ordered_joints(state)) if field(row,name))


def valid_state(state):
    try:
        source_identity(state)
        for name in _STATE_SCALARS:
            field(state,name)
        ordered_joints(state)
        return True
    except (AttributeError, KeyError, TypeError, ValueError):
        return False


def valid_status(status):
    try:
        source_identity(status)
        for name in _STATUS_REQUIRED:
            field(status,name)
        return True
    except (AttributeError, KeyError, TypeError, ValueError):
        return False


def state_view(state, source=None):
    """从真实joint_id排序生成UI显示、新鲜度及Hold判定所需视图。

    质量位为False时保留实际值（含NaN），不得补造有效Hold或默认零年龄；
    presence仍由原Root记录校验，来源与消息始终分开，不生成网络DTO。
    """
    if source is not None:
        state = {**state,'source':source}
    if not valid_state(state):
        raise ValueError('当前Root状态不完整')
    result={name:field(state,name) for name in _STATE_SCALARS}
    result.update(format=VIEW_FORMAT,source_identity=source_identity(state),boot_epoch=source_identity(state)[1])
    for output,quality in (('position_valid_mask','position_valid_quality'),
                           ('velocity_valid_mask','velocity_valid_quality'),
                           ('raw_position_valid_mask','raw_position_valid_quality'),
                           ('hold_valid_mask','hold_valid_quality')):
        result[output]=quality_mask(state,quality)
    for name,quality in (('position_rad','position_valid_quality'),
                         ('estimated_velocity_rad_s','velocity_valid_quality'),
                         ('raw_motor_position_rev','raw_position_valid_quality'),
                         ('hold_position_rad','hold_valid_quality')):
        values=joint_values(state,name)
        if any(field(row,quality) and not math.isfinite(value) for row,value in zip(ordered_joints(state),values)):
            raise ValueError('Root关节反馈非有限值')
        result[name]=values
    result['can_rx_age_us']=joint_values(state,'can_rx_age_us')
    return result


def control_identity(value):
    device,boot,context=source_identity(value)
    return ((device,context),field(value,'profile_id'),boot,field(value,'session_id'),
            field(value,'reference_epoch'),field(value,'motion_epoch'))


def reference_identity(value):
    identity=control_identity(value)
    return identity[0],identity[1],identity[2],identity[4]
