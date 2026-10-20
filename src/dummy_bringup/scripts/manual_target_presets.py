##
# @file manual_target_presets.py
# @brief 提供三组可持久保存的六维目标输入。
# 关节页以度输入，MoveIt 页以毫米和角度输入；单位、范围和小数位都复制调用方控件。设置按钮只回调填入目标，是否允许运动由主面板单独控制。
"""三组六维快捷目标；设置只填入目标，数值由 Qt 用户配置保存。"""
import json
import math

from PyQt5 import QtCore, QtWidgets


##
# @class TargetPresets
# @brief 组合三行六维输入与 Qt 用户配置存储。
# rows 保存每组输入控件；buttons 供调用方统一启停；QSettings 仅保存用户目标值，不是设备状态或执行证据。
class TargetPresets(QtWidgets.QGroupBox):
    ##
    # @brief 建立三组输入，恢复合法的用户保存值。
    # 保存数据必须是包含六项的行；非法 JSON、布尔值、非数值或非有限数回退到对应目标控件当前值。QDoubleSpinBox 还会按自身范围处理恢复值。
    # @param key QSettings 的分组键；关节和末端目标使用不同键。
    # @param labels 六列表头文字，包含调用方选择的单位。
    # @param targets 六个目标 QDoubleSpinBox，用于复制范围、步长、小数位和默认值。
    # @param apply 接收六个数值列表的回调；其语义应是更新目标。
    # @param parent 可选 Qt 父控件，负责控件生命周期。
    # @return 无；通过实例状态或界面产生作用。
    def __init__(self, key, labels, targets, apply, parent=None):
        super().__init__('快捷目标（输入后点击设置）', parent)
        self.settings = QtCore.QSettings('DummyRobotics', 'ManualPanel')
        self.key, self.apply = key, apply
        self.rows, self.buttons = [], []
        grid = QtWidgets.QGridLayout(self)
        for column, label in enumerate(labels, 1):
            grid.addWidget(QtWidgets.QLabel(label), 0, column)
        try:
            saved = json.loads(self.settings.value(key, '[]'))
        except (ValueError, TypeError):
            saved = []
        for row in range(3):
            grid.addWidget(QtWidgets.QLabel(f'第 {row + 1} 组'), row + 1, 0)
            boxes = []
            values = saved[row] if isinstance(saved, list) and len(saved) > row else []
            for column, target in enumerate(targets):
                box = QtWidgets.QDoubleSpinBox()
                box.setDecimals(target.decimals())
                box.setRange(target.minimum(), target.maximum())
                box.setSingleStep(target.singleStep())
                box.setKeyboardTracking(False)
                value = values[column] if isinstance(values, list) and len(values) == 6 else target.value()
                if type(value) not in (int, float) or not math.isfinite(value):
                    value = target.value()
                box.setValue(value)
                box.valueChanged.connect(self.save)
                grid.addWidget(box, row + 1, column + 1)
                boxes.append(box)
            button = QtWidgets.QPushButton('设置')
            ##
            # 默认参数 index=row 在建按钮时捕获行号，避免所有按钮在点击时都引用循环最终行。
            button.clicked.connect(lambda _, index=row: self.apply_row(index))
            grid.addWidget(button, row + 1, 7)
            self.rows.append(boxes)
            self.buttons.append(button)

    ##
    # @brief 将三组当前控件值序列化保存。
    # valueChanged 会触发此槽；保存的是界面显示单位下的数字，不做弧度或米的转换。
    # @param _ Qt 信号附带的位置参数；这里不使用。
    # @return 无；通过实例状态或界面产生作用。
    def save(self, *_):
        self.settings.setValue(self.key, json.dumps([[box.value() for box in row] for row in self.rows]))

    ##
    # @brief 保存当前预设并将选中一组交给填目标回调。
    # 不检查设备会话、不发送运动命令；索引合法性由创建按钮时的绑定保证。
    # @param index 三组中的零基索引，由各行按钮闭包固定为 0、1、2。
    # @return 无；通过实例状态或界面产生作用。
    def apply_row(self, index):
        self.save()
        self.apply([box.value() for box in self.rows[index]])
