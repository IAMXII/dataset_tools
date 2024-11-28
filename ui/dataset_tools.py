import sys

from PyQt6.QtCore import QTimer, QDir
from PyQt6.QtWidgets import QPushButton, QMainWindow, QWidget, QApplication, \
    QVBoxLayout, QTabWidget, QProgressBar, QDialog, QLabel, QFileDialog, QHBoxLayout, QTextEdit, QLineEdit
from OpenGL.GL import glClear, GL_COLOR_BUFFER_BIT, glClearColor
from rename_files import rename_files
from copy_files import copy_files


def select_directory(my_dialog):
    """
    弹出目录选择对话框，获取用户选择的目录路径。
    """
    # 弹出选择对话框
    directory = QFileDialog.getExistingDirectory(
        my_dialog,
        "选择目录",  # 对话框标题
        QDir.homePath(),  # 默认路径（这里是用户主目录）
        QFileDialog.Option.ShowDirsOnly  # 仅显示目录，不显示文件
    )

    if directory:  # 如果用户选择了目录
        my_dialog.label.setText(f"已选择目录：{directory}")
        my_dialog.selected_directory = directory
        my_dialog.execute_button.setEnabled(True)  # 启用执行按钮
    else:  # 用户取消选择
        my_dialog.label.setText("未选择任何目录")
        my_dialog.execute_button.setEnabled(False)  # 禁用执行按

# class rename_files(QDialog):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("文件重命名")
#         # self.resize(600, 600)
#         layout = QVBoxLayout()
#
#         # 标签显示选定的目录路径
#         self.label = QLabel("当前未选择任何目录")
#         layout.addWidget(self.label)
#
#         # 添加按钮选择目录
#         self.select_dir_button = QPushButton("选择目录")
#         self.select_dir_button.clicked.connect(self.select_directory)
#         layout.addWidget(self.select_dir_button)
#         layout_text = QHBoxLayout()
#         text1 = QLineEdit()
#         text1.setPlaceholderText('请输入前缀')
#         text2 = QLineEdit()
#         text2.setPlaceholderText("请输入后缀")
#         layout_text.addWidget(text1)
#         layout_text.addWidget(text2)
#         layout.addLayout(layout_text)
#         # 添加另一个按钮执行操作
#         self.execute_button = QPushButton("执行操作")
#         self.execute_button.clicked.connect(self.execute_action)
#         self.execute_button.setEnabled(False)  # 默认禁用，直到选择目录
#         layout.addWidget(self.execute_button)
#         self.setLayout(layout)
#         # 设置布局
#
#
#     def select_directory(self):
#         """
#         弹出目录选择对话框，获取用户选择的目录路径。
#         """
#         # 弹出选择对话框
#         directory = QFileDialog.getExistingDirectory(
#             self,
#             "选择目录",  # 对话框标题
#             QDir.homePath(),  # 默认路径（这里是用户主目录）
#             QFileDialog.Option.ShowDirsOnly  # 仅显示目录，不显示文件
#         )
#
#         if directory:  # 如果用户选择了目录
#             self.label.setText(f"已选择目录：{directory}")
#             self.selected_directory = directory
#             self.execute_button.setEnabled(True)  # 启用执行按钮
#         else:  # 用户取消选择
#             self.label.setText("未选择任何目录")
#             self.execute_button.setEnabled(False)  # 禁用执行按钮
#
#     def execute_action(self):
#         """
#         执行与选定目录相关的操作。
#         """
#         if hasattr(self, "selected_directory"):
#             directory = self.selected_directory
#             # 在这里编写与目录相关的操作
#             self.label.setText(f"正在处理目录：{directory}")
#
#             print(f"操作已在目录 '{directory}' 执行！")
#         else:
#             self.label.setText("未选择目录，无法执行操作")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('数据处理工具')

        self.tab_widget = QTabWidget()
        self.setCentralWidget(self.tab_widget)
        centralWidget = QWidget()
        button1 = QPushButton('文件重命名')
        button2 = QPushButton('文件复制')

        button1.setStyleSheet("""
                    QPushButton {
                        background-color: gray;  /* 背景色 */           
                        color: white;               /* 字体颜色 */
                        border-radius: 8px;        /* 圆角半径 */
                        font-size: 14px;            /* 字体大小 */
                        padding: 4px;              /* 内边距 */
                    }
                    QPushButton:hover {
                        background-color: #45a049;  /* 鼠标悬停时的背景色 */
                    }
                    QPushButton:pressed {
                        background-color: #3e8e41;  /* 按下时的背景色 */
                    }
                """)
        button2.setStyleSheet(button1.styleSheet())
        self.styleSheet = button1.styleSheet()
        lay = QVBoxLayout()
        lay.addWidget(button1)
        lay.addWidget(button2)
        centralWidget.setLayout(lay)
        self.tab_widget.addTab(centralWidget, "文件操作")
        progress_tab = QWidget()
        self.setup_progress_tab(progress_tab)
        self.tab_widget.addTab(progress_tab, "进度条示例")
        button1.clicked.connect(self.open_new_window)
        button2.clicked.connect(self.open_copy_window)
        # layout = QVBoxLayout()
        # layout.addWidget(self.tab_widget)
        # centralWidget.setLayout(layout)

        # grid = QGridLayout(centralWidget)
        # grid.addWidget(button1)

    def setup_progress_tab(self, tab):
        """
        在指定的标签页中添加进度条和按钮。
        """
        layout = QVBoxLayout()

        # 添加进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)  # 设置范围
        self.progress_bar.setValue(0)  # 初始值
        layout.addWidget(self.progress_bar)

        self.label = QLabel("进度：0%")
        self.label.setStyleSheet("font-size: 10px;")  # 设置字体大小
        layout.addWidget(self.label)

        # 添加按钮
        self.start_button = QPushButton("开始")
        self.start_button.clicked.connect(self.start_progress)
        # self.start_button.setStyleSheet(self.styleSheet)
        layout.addWidget(self.start_button)

        # 设置布局
        tab.setLayout(layout)

    def start_progress(self):
        """
        模拟进度条的更新。
        """
        self.progress_value = 0

        # 创建定时器，模拟进度条更新
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_progress)
        self.timer.start(50)  # 每 50 毫秒更新一次

    def update_progress(self):
        """
        更新进度条的值。
        """
        self.progress_value += 1
        self.progress_bar.setValue(self.progress_value)
        self.label.setText(f"进度：{self.progress_value}%")

        # 停止定时器
        if self.progress_value >= 100:
            self.timer.stop()

    def open_new_window(self):
        # 实例化一个对话框类
        self.dlg = rename_files()
        # 显示对话框，代码阻塞在这里，
        # 等待对话框关闭后，才能继续往后执行
        self.dlg.exec()
    def open_copy_window(self):
        self.dlg = copy_files()
        self.dlg.exec()


app = QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec()