"""Chinese desktop controls for manual, read-only hand-eye calibration."""

import json
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QLabel, QPushButton, QLineEdit, QComboBox,
    QSpinBox, QGridLayout, QVBoxLayout, QHBoxLayout, QFrame,
    QFileDialog, QMessageBox, QTableWidget, QTableWidgetItem,
    QHeaderView, QAbstractItemView, QPlainTextEdit, QTabWidget, QSplitter,
)


class ImageView(QLabel):
    def __init__(self):
        super().__init__('等待相机图像\n请保持相机和 ArUco 识别节点运行')
        self.setObjectName('image_view')
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(480, 280)
        self._image = QImage()

    def set_image(self, image):
        self._image = image
        self.render_image()

    def render_image(self):
        if not self._image.isNull():
            self.setPixmap(QPixmap.fromImage(self._image).scaled(
                self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.render_image()


class CalibrationWindow(QMainWindow):
    connect_requested = pyqtSignal(dict)
    capture_requested = pyqtSignal()
    undo_requested = pyqtSignal()
    clear_requested = pyqtSignal()
    solve_requested = pyqtSignal()

    def __init__(self, config):
        super().__init__()
        self.setWindowTitle('手眼标定助手 · ROS2')
        self.resize(1280, 920)
        self._connected = False
        self._busy = False
        self._can_capture = False
        self._samples = []
        self._saved = False
        self.fields = {}
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(22, 18, 22, 18)
        root.setSpacing(12)
        header = QHBoxLayout()
        title = QLabel('手眼标定助手')
        title.setObjectName('title')
        header.addWidget(title)
        subtitle = QLabel('实时预览  /  手动采样  /  外参计算')
        subtitle.setObjectName('subtitle')
        header.addWidget(subtitle)
        header.addStretch()
        safety = QLabel('只读订阅 · 不控制机械臂')
        safety.setObjectName('safety')
        header.addWidget(safety)
        root.addLayout(header)

        config_bar = QHBoxLayout()
        self.expand_button = QPushButton('展开连接配置')
        self.expand_button.setCheckable(True)
        self.expand_button.toggled.connect(self.toggle_config)
        config_bar.addWidget(self.expand_button)
        self.summary = QLabel()
        config_bar.addWidget(self.summary, 1)
        self.connect_button = self.button('连接数据', 'connect_button',
                                           lambda: self.connect_requested.emit(self.get_config()))
        config_bar.addWidget(self.connect_button)
        root.addLayout(config_bar)
        self.config_panel = QFrame()
        self.config_panel.setObjectName('card')
        form = QGridLayout(self.config_panel)
        labels = [
            ('piper_topic', '机械臂位姿话题'), ('marker_topic', '标记位姿话题'),
            ('image_topic', '图像话题'), ('result_save_path', '结果保存目录'),
        ]
        for row, (key, label) in enumerate(labels):
            form.addWidget(QLabel(label), row, 0)
            field = QLineEdit(str(config[key]))
            field.setObjectName(key)
            self.fields[key] = field
            form.addWidget(field, row, 1, 1, 3 if key != 'result_save_path' else 2)
        self.browse_button = self.button('选择目录', 'browse_button', self.browse)
        form.addWidget(self.browse_button, 3, 3)
        form.addWidget(QLabel('标定模式'), 4, 0)
        self.mode = QComboBox()
        self.mode.setObjectName('mode')
        self.mode.addItem('眼在手上 · 相机随末端运动', 'eye_in_hand')
        self.mode.addItem('眼在手外 · 相机相对基座固定', 'eye_to_hand')
        self.mode.setCurrentIndex(max(0, self.mode.findData(config['mode'])))
        form.addWidget(self.mode, 4, 1)
        form.addWidget(QLabel('最少采样数'), 4, 2)
        self.min_num = QSpinBox()
        self.min_num.setObjectName('min_num')
        self.min_num.setRange(5, 200)
        self.min_num.setValue(int(config['min_num']))
        self.min_num.valueChanged.connect(self.update_buttons)
        form.addWidget(self.min_num, 4, 3)
        root.addWidget(self.config_panel)
        self.config_panel.hide()

        splitter = QSplitter(Qt.Vertical)
        live = QWidget()
        live_layout = QHBoxLayout(live)
        live_layout.setContentsMargins(0, 0, 0, 0)
        live_layout.setSpacing(16)
        self.image_view = ImageView()
        live_layout.addWidget(self.image_view, 3)
        controls = QFrame()
        controls.setObjectName('card')
        controls.setMinimumWidth(340)
        controls.setMaximumWidth(460)
        panel = QVBoxLayout(controls)
        panel.setContentsMargins(18, 18, 18, 18)
        panel.setSpacing(12)
        panel.addWidget(QLabel('数据状态'))
        self.status_robot = self.status_label('status_robot', '机械臂：未连接')
        self.status_marker = self.status_label('status_marker', '标记：未连接')
        self.status_image = self.status_label('status_image', '图像：未连接')
        for status in (self.status_robot, self.status_marker, self.status_image):
            panel.addWidget(status)
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        panel.addWidget(line)
        self.count = QLabel('0 组样本')
        self.count.setObjectName('sample_count')
        panel.addWidget(self.count)
        self.capture_button = self.button('采集当前姿态', 'capture_button', self.capture_requested.emit)
        self.capture_button.setMinimumHeight(48)
        self.capture_button.setObjectName('capture_button')
        panel.addWidget(self.capture_button)
        editing = QHBoxLayout()
        self.undo_button = self.button('撤销上一组', 'undo_button', self.undo_requested.emit)
        self.clear_button = self.button('清空样本', 'clear_button', self.confirm_clear)
        editing.addWidget(self.undo_button)
        editing.addWidget(self.clear_button)
        panel.addLayout(editing)
        self.solve_button = self.button('计算并保存结果', 'solve_button', self.solve_requested.emit)
        self.solve_button.setMinimumHeight(42)
        panel.addWidget(self.solve_button)
        self.hint = QLabel('采集时保持机械臂静止，等待“采集完成”后再移动。\n建议 15–25 组，多轴旋转，避免只平移。')
        self.hint.setWordWrap(True)
        self.hint.setObjectName('hint')
        panel.addWidget(self.hint)
        panel.addStretch()
        live_layout.addWidget(controls, 2)
        splitter.addWidget(live)

        self.tabs = QTabWidget()
        self.sample_table = QTableWidget(0, 7)
        self.sample_table.setObjectName('sample_table')
        self.sample_table.setHorizontalHeaderLabels(['序号', '末端 X / m', '末端 Y / m', '末端 Z / m',
                                                     '标记 X / m', '标记 Y / m', '标记 Z / m'])
        self.sample_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.sample_table.verticalHeader().hide()
        self.sample_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.sample_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.sample_table.setAlternatingRowColors(True)
        self.tabs.addTab(self.sample_table, '采样记录')
        self.result_text = QPlainTextEdit()
        self.result_text.setObjectName('result_text')
        self.result_text.setReadOnly(True)
        self.result_text.setPlaceholderText('采集足够且丰富的姿态后，点击“计算并保存结果”。\n输出位置单位 m，欧拉角单位 rad，四元数顺序 XYZW。')
        self.tabs.addTab(self.result_text, '标定结果')
        self.log_text = QPlainTextEdit()
        self.log_text.setObjectName('log_text')
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumBlockCount(300)
        self.tabs.addTab(self.log_text, '操作日志')
        splitter.addWidget(self.tabs)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([530, 220])
        root.addWidget(splitter, 1)
        self.setStyleSheet('''
            QMainWindow, QWidget { background: #17212d; color: #e1e8ee; font-size: 13px; }
            QLabel#title { font-size: 26px; font-weight: bold; }
            QLabel#subtitle, QLabel#hint { color: #a8b8c7; }
            QLabel#safety { color: #73d6c0; padding: 7px; }
            QFrame#card { background: #202e3d; border: 1px solid #354556; border-radius: 8px; }
            QFrame#card QLabel { background: transparent; }
            QLabel#image_view { background: #0d141c; color: #8ba2b7; border: 1px solid #354556; border-radius: 8px; }
            QLabel#sample_count { font-size: 24px; font-weight: bold; color: #76d9c4; }
            QPushButton { background: #314357; border: 1px solid #4b6278; border-radius: 5px; padding: 9px 14px; }
            QPushButton:hover { background: #3d556d; }
            QPushButton:disabled { color: #778797; background: #273443; border-color: #344252; }
            QPushButton#capture_button, QPushButton#solve_button { background: #166b62; border-color: #258d80; font-weight: bold; }
            QPushButton#capture_button:hover, QPushButton#solve_button:hover { background: #218579; }
            QPushButton#capture_button:disabled, QPushButton#solve_button:disabled { background: #293c40; color: #748c8b; border-color: #395453; }
            QLineEdit, QComboBox, QSpinBox { background: #101a25; border: 1px solid #485d72; border-radius: 4px; padding: 6px; }
            QLineEdit:disabled, QComboBox:disabled, QSpinBox:disabled { color: #8a9aab; border-color: #334455; }
            QPlainTextEdit, QTableWidget { background: #111c28; alternate-background-color: #1c2a38; border: 1px solid #354556; selection-background-color: #245d62; }
            QHeaderView::section { background: #263747; color: #c6d3df; padding: 7px; border: 0; }
            QTabWidget::pane { border: 1px solid #354556; }
            QTabBar::tab { background: #243343; padding: 9px 22px; }
            QTabBar::tab:selected { background: #17655e; }
            QSplitter::handle { background: #354556; height: 5px; }
        ''')
        self.update_summary()
        self.update_buttons()

    def button(self, text, name, callback):
        button = QPushButton(text)
        button.setObjectName(name)
        button.setAutoDefault(False)
        button.clicked.connect(callback)
        return button

    def status_label(self, name, text):
        label = QLabel(text)
        label.setObjectName(name)
        label.setWordWrap(True)
        label.setMinimumHeight(38)
        return label

    def toggle_config(self, visible):
        self.config_panel.setVisible(visible)
        self.expand_button.setText('收起连接配置' if visible else '展开连接配置')

    def browse(self):
        directory = QFileDialog.getExistingDirectory(self, '选择结果保存目录', self.fields['result_save_path'].text())
        if directory:
            self.fields['result_save_path'].setText(directory)

    def get_config(self):
        config = {key: field.text().strip() for key, field in self.fields.items()}
        config.update(mode=self.mode.currentData(), min_num=self.min_num.value())
        return config

    def update_summary(self):
        mode = '眼在手上' if self.mode.currentData() == 'eye_in_hand' else '眼在手外'
        self.summary.setText(mode + '  |  ' + self.fields['piper_topic'].text())

    def set_connection_state(self, connected):
        self._connected = connected
        self.connect_button.setText('断开订阅' if connected else '连接数据')
        self.update_summary()
        self.update_buttons()

    def set_status(self, robot_text, marker_text, image_text, can_capture):
        self.status_robot.setText(robot_text)
        self.status_marker.setText(marker_text)
        self.status_image.setText(image_text)
        for label in (self.status_robot, self.status_marker, self.status_image):
            bad = any(word in label.text() for word in ('未连接', '等待', '过期', '失败'))
            label.setStyleSheet('color: %s; background: transparent;' % ('#e6b86b' if bad else '#83d8bf'))
        self._can_capture = can_capture
        self.update_buttons()

    def set_busy(self, busy):
        self._busy = busy
        self.capture_button.setText('等待新数据…请保持静止' if busy else '采集当前姿态')
        self.update_buttons()

    def update_buttons(self):
        if not hasattr(self, 'solve_button'):
            return
        count = len(self._samples)
        for field in list(self.fields.values()) + [self.mode, self.min_num, self.browse_button]:
            field.setEnabled(not self._connected and not self._busy and count == 0)
        self.connect_button.setEnabled(not self._busy)
        self.capture_button.setEnabled(self._connected and self._can_capture and not self._busy)
        self.undo_button.setEnabled(count > 0 and not self._busy)
        self.clear_button.setEnabled(count > 0 and not self._busy)
        self.solve_button.setEnabled(count >= self.min_num.value() and not self._busy)
        self.count.setText('%d 组样本  /  最少 %d' % (count, self.min_num.value()))

    def set_image(self, image):
        self.image_view.set_image(image)

    def set_samples(self, samples):
        self._samples = list(samples)
        self._saved = False
        self.result_text.clear()
        self.sample_table.setRowCount(len(samples))
        for row, sample in enumerate(samples):
            values = [str(row + 1)] + ['%.5f' % v for v in sample['robot'][:3] + sample['marker'][:3]]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                self.sample_table.setItem(row, col, item)
        self.sample_table.scrollToBottom()
        self.update_buttons()

    def show_result(self, result, path):
        display = {key: value for key, value in result.items() if key not in ('samples', 'metadata')}
        self.result_text.setPlainText('保存位置：' + path + '\n\n' + json.dumps(display, ensure_ascii=False, indent=2))
        self._saved = True
        self.tabs.setCurrentWidget(self.result_text)

    def show_error(self, message):
        self.append_log('错误：' + message)
        QMessageBox.warning(self, '标定提示', message)

    def append_log(self, message):
        self.log_text.appendPlainText(message)

    def confirm_clear(self):
        answer = QMessageBox.question(self, '清空当前样本', '清空本次全部采样？已经保存的文件不会被删除。',
                                      QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer == QMessageBox.Yes:
            self.clear_requested.emit()

    def closeEvent(self, event):
        if self._samples and not self._saved:
            answer = QMessageBox.question(self, '退出标定', '本次样本尚未保存，确定退出？',
                                          QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                event.ignore()
                return
        event.accept()
