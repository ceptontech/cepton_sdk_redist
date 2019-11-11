import signal

import imageio

import cepton_util.common

_all_builder = cepton_util.common.AllBuilder(__name__)

from PyQt5.QtCore import *  # noqa isort:skip
from PyQt5.QtGui import *  # noqa isort:skip
from PyQt5.QtWidgets import *  # noqa isort:skip

from cepton_util.common import *  # noqa isort:skip


def create_toolbox_header(name):
    label = QLabel(name)
    label.setAlignment(Qt.AlignLeft)
    font = QFont()
    font.setBold(True)
    label.setFont(font)
    return label


def create_expanding_label(name):
    label = QLabel(name)
    label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    return label


def clear_layout(layout):
    while layout.count():
        layout.takeAt(0).widget().deleteLater()


class VariableLabel(QLabel):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setTextInteractionFlags(Qt.TextSelectableByMouse)

    def paintEvent(self, event):
        painter = QPainter(self)

        metrics = QFontMetrics(self.font())
        elided = metrics.elidedText(self.text(), Qt.ElideRight, self.width())

        painter.drawText(self.rect(), self.alignment(), elided)


class ButtonLineEdit(QLineEdit):
    def __init__(self, icon, parent=None):
        super().__init__(parent)

        self.button = QToolButton(self)
        self.button.setIcon(icon)
        self.button.setStyleSheet('border: 0px; padding: 0px;')
        self.button.setCursor(Qt.ArrowCursor)

        frame_width = self.style().pixelMetric(QStyle.PM_DefaultFrameWidth)
        button_size = self.button.sizeHint()
        self.setStyleSheet(
            'QLineEdit {padding-right: %dpx; }' %
            (button_size.width() + frame_width + 1))
        self.setMinimumSize(
            max(self.minimumSizeHint().width(),
                button_size.width() + frame_width*2 + 2),
            max(self.minimumSizeHint().height(),
                button_size.height() + frame_width*2 + 2))

    def resizeEvent(self, event):
        frame_width = self.style().pixelMetric(QStyle.PM_DefaultFrameWidth)
        button_size = self.button.sizeHint()
        self.button.move(
            self.rect().right() - frame_width - button_size.width(),
            (self.rect().bottom() - button_size.height() + 1)/2)
        super().resizeEvent(event)


__all__ = _all_builder.get()
