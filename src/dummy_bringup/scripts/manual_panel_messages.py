"""给操作员解释事件和错误；保留原始编号，未知原因不作推断。"""
import re

EVENTS = {
    'READONLY_BASELINE': '只读连接已就绪，尚未使能',
    'WAIT_MANUAL_HOME': '等待恢复控制或明确确认标定姿态',
    'GRANT_STATE_CONFIRMED': '控制会话已确认，尚未提交运动',
    'CALIBRATION_READY': '标定完成，设备零点有效',
    'CALIBRATION_RESUMED': '已恢复本次开机保留的零点',
    'MOVE_STARTED': '开始执行关节运动',
    'MOVE_CONVERGING': '轨迹已发送，正在确认实际到位',
    'MOVE_COMPLETED': '运动已确认完成，当前保持使能',
    'MOVE_CANCELED_HOLD': '测试运动已取消，受控停车并保持使能已确认',
    'CANCEL_HOLD_FAILED': '受控停车保持未确认，执行停止释放',
    'MOVE_NOT_READY': '本次运动未提交：保持状态、反馈或管理事务暂未就绪，可稍后重试',
    'MOVE_REJECTED_RANGE': '起点或目标超出当前工作范围，请缩小目标后重新规划',
    'MOVE_REJECTED_POLICY': '运动不满足当前控制策略，本次未执行',
    'READONLY_FAULT': '设备存在故障，仅只读显示；处理原因后显式清故障',
    'EXIT_CLEANUP': '控制程序正在退出并释放设备，请查看前面的原因',
    'RELEASE_CONFIRMED': '设备已确认失能和释放，已完成的同次开机零点保留',
    'RELEASE_UNCONFIRMED': '尚未确认设备释放，请检查连接和实际停止状态',
    'RESUME_NOT_READY': '当前位置或反馈不满足恢复条件，零点仍保留',
    'REST_CHAIN_COMMITTED': '回Rest及其验证已完成，零点已保留',
}
FAULTS = {
    0x1001: '目标或停车预测不满足安全范围，请检查位置并向工作区内退让',
    0x1002: '实际反馈越过软件限位，请检查位置并恢复到允许的工作区',
    0x1005: '控制会话或许可已到期，请确认停止后重新恢复控制',
    0x1008: '指令速度、加速度或连续性超过策略上限，已拒绝运动',
    0x1051: '检测到异常速度反馈，请检查电机反馈和实际运动',
    0x1067: '回Rest后未满足收敛或静止确认条件，请检查实际位置与反馈',
    0x2006: '目标范围或相邻运动指令不满足要求，请重新规划',
}
MOVEIT = {
    1: '服务报告成功但结果不完整，请重新请求',
    -1: '未找到可用路径，请缩小目标或换逆解偏好',
    -2: '规划结果未通过有效性检查，未授权执行',
    -3: '规划期间环境或路径失效，请重新规划',
    -6: '规划超时，请缩小目标或换偏好后重试',
    -10: '实际起点发生自碰撞，请先检查当前姿态',
    -11: '起点不满足规划约束，请检查反馈及关节范围',
    -12: '目标姿态发生碰撞，请调整位置、姿态或逆解偏好',
    -13: '目标不满足约束，请调整目标',
    -15: '规划关节组配置无效，请检查MoveIt配置',
    -21: '目标坐标转换失败，请检查TF与基座坐标系',
    -22: '碰撞检查服务不可用，请检查MoveIt是否已启动',
    -23: '机器人状态已失鲜，请等待新鲜反馈',
    -31: '目标没有可用逆解，可能超出可达范围或不满足姿态/碰撞条件；请缩小步长或换逆解偏好',
}
EXIT_CODES = {
    4: '启动只读检查未通过，请确认设备已释放且反馈新鲜',
    5: '控制授权或管理事务未通过，请检查前面的状态信息',
    6: '会话、身份或反馈条件变化，控制已结束',
    7: '运动条件或到位确认未通过，请查看前面的具体拒绝原因',
    8: '设备释放未获确认，请核实实际停止状态和连接',
    17: '本机计时异常，控制已结束，请检查系统时钟环境',
    56: '日志归档或封口未完成；动作是否完成应单独查看到位回执，不能将该日志当作完整归档',
}


def moveit_error(code):
    return f'{MOVEIT.get(code,"MoveIt未识别错误，请查看规划服务日志")}（MoveIt错误码 {code}）'


def explain_panel_log(text):
    if text in EVENTS:return f'{EVENTS[text]} [{text}]'
    match = re.search(r'(?:reason|failure_source)=(0x[0-9a-fA-F]+|\d+)',text)
    if match:
        value = int(match.group(1),16 if match.group(1).startswith('0x') else 10)
        if value:return f'{text}；说明：{FAULTS.get(value,"未识别的设备故障，请保留原始码并查看前后日志")}'
    exit_match = re.search(r'(?:code|exit_code)=(\d+)',text)
    if exit_match and int(exit_match.group(1)) in EXIT_CODES:
        return f'{text}；说明：{EXIT_CODES[int(exit_match.group(1))]}'
    return text
