from PyQt6.QtCore import QDir
from PyQt6.QtWidgets import QDialog, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QLineEdit, QFileDialog
import sys
import os
sys.path.append("/home/liuwei/project_code/dataset_tools")
from slam_operations import get_digit_from_filename, sort_by_number


class rename_files(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("文件重命名")
        # self.resize(600, 600)
        layout = QVBoxLayout()

        # 标签显示选定的目录路径
        self.label = QLabel("当前未选择任何目录")
        layout.addWidget(self.label)

        # 添加按钮选择目录
        self.select_dir_button = QPushButton("选择目录")
        self.select_dir_button.clicked.connect(self.select_directory)
        layout.addWidget(self.select_dir_button)
        layout_text = QHBoxLayout()
        self.text1 = QLineEdit()
        self.text1.setPlaceholderText('请输入前缀')
        self.text2 = QLineEdit()
        self.text2.setPlaceholderText("请输入后缀")
        layout_text.addWidget(self.text1)
        layout_text.addWidget(self.text2)
        layout.addLayout(layout_text)
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

            file_names = sorted(os.listdir(directory),key=sort_by_number)
            for file_name in file_names:
                old_file_name = os.path.join(directory, file_name)
                index = get_digit_from_filename(file_name)
                new_file_name = os.path.join(directory, f"{prefix}{index}{suffix}")
                os.rename(old_file_name, new_file_name)
            # 在这里编写与目录相关的操作
            self.label.setText(f"正在处理目录：{directory}")
            print(f"操作已在目录 '{directory}' 执行！")
        else:
            self.label.setText("未选择目录，无法执行操作")