import shutil

from PyQt6.QtCore import QDir
from PyQt6.QtWidgets import QDialog, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QLineEdit, QFileDialog, QGridLayout
import sys
import os
# sys.path.append("/home/liuwei/project_code/dataset_tools")
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(parent_dir)
from slam_operations import get_digit_from_filename, sort_by_number


class copy_files(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("复制文件")
        # self.resize(600, 600)
        layout = QVBoxLayout()

        # 标签显示选定的目录路径
        self.label = QLabel("当前未选择任何目录")
        layout.addWidget(self.label)

        # 添加按钮选择目录
        self.select_src_button = QPushButton("选择源目录")
        self.select_src_button.clicked.connect(self.select_directory)
        layout.addWidget(self.select_src_button)
        self.select_dir_button = QPushButton("选择目标目录")
        self.select_dir_button.clicked.connect(self.select_directory)
        layout.addWidget(self.select_dir_button)
        layout_text = QHBoxLayout()
        grid_layout = QGridLayout()
        self.text1 = QLineEdit()
        self.text1.setPlaceholderText('请输入前缀')
        self.text2 = QLineEdit()
        self.text2.setPlaceholderText("请输入后缀")
        self.text3 = QLineEdit()
        self.text3.setPlaceholderText('请输入起始序号')
        self.text4 = QLineEdit()
        self.text4.setPlaceholderText("请输入终止序号")
        grid_layout.addWidget(self.text1,0,0)
        grid_layout.addWidget(self.text2,0,1)
        grid_layout.addWidget(self.text3,1,0)
        grid_layout.addWidget(self.text4,1,1)
        layout.addLayout(grid_layout)
        self.num = QLineEdit()
        self.num.setPlaceholderText('请输入数值变化')
        layout.addWidget(self.num)
        # 添加另一个按钮执行操作
        self.execute_button = QPushButton("执行操作")
        self.execute_button.clicked.connect(self.execute_action)
        self.execute_button.setEnabled(False)  # 默认禁用，直到选择目录
        layout.addWidget(self.execute_button)
        self.setLayout(layout)
        # 设置布局


    def select_directory(self):
        """
        弹出目录选择对话框，获取用户选择的目录路径。
        """
        # 弹出选择对话框
        directory = QFileDialog.getExistingDirectory(
            self,
            "选择目录",  # 对话框标题
            QDir.homePath(),  # 默认路径（这里是用户主目录）
            QFileDialog.Option.ShowDirsOnly  # 仅显示目录，不显示文件
        )

        if directory:  # 如果用户选择了目录
            self.label.setText(f"已选择目录：{directory}")
            self.selected_directory = directory
            self.execute_button.setEnabled(True)  # 启用执行按钮
        else:  # 用户取消选择
            self.label.setText("未选择任何目录")
            self.execute_button.setEnabled(False)  # 禁用执行按钮

    def execute_action(self):
        """
        执行与选定目录相关的操作。
        """
        if hasattr(self, "selected_directory"):
            directory = self.selected_directory
            prefix = self.text1.text()
            suffix = self.text2.text()
            start_index = int(self.text3.text())
            end_index = int(self.text4.text())
            num = self.num.text()
            num = int(num)
            file_names = sorted(os.listdir(directory),key=sort_by_number)
            for file_name in file_names[start_index:end_index]:
                old_file_name = os.path.join(directory, file_name)
                index = get_digit_from_filename(file_name)
                index = int(index)+num
                index = str(index).zfill(6)
                new_file_name = os.path.join(directory, f"{prefix}{index}{suffix}")
                shutil.copy(old_file_name, new_file_name)
            # 在这里编写与目录相关的操作
            self.label.setText(f"正在处理目录：{directory}")
            print(f"操作已在目录 '{directory}' 执行！")
        else:
            self.label.setText("未选择目录，无法执行操作")