"""三组六维快捷目标；设置只填入目标，数值由 Qt 用户配置保存。"""
import json
import math

from PyQt5 import QtCore, QtWidgets


class TargetPresets(QtWidgets.QGroupBox):
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
            button.clicked.connect(lambda _, index=row: self.apply_row(index))
            grid.addWidget(button, row + 1, 7)
            self.rows.append(boxes)
            self.buttons.append(button)

    def save(self, *_):
        self.settings.setValue(self.key, json.dumps([[box.value() for box in row] for row in self.rows]))

    def apply_row(self, index):
        self.save()
        self.apply([box.value() for box in self.rows[index]])
