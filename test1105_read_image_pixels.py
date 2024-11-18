import cv2
from pandas.io.formats.style import jinja2
from argparse import ArgumentParser
from tqdm import tqdm
# 定义鼠标回调函数
def get_pixel_coordinates(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:  # 检查是否为鼠标左键点击
        with open("pixels.txt","a") as f:
            f.write(f"Pixel coordinates: (x={x}, y={y})")
            f.write("\n")
        print(f"Pixel coordinates: (x={x}, y={y})")

# 读取图像
image_path = './img.png'  # 替换为你的图像路径
image = cv2.imread(image_path)

# 创建窗口并设置鼠标回调函数
cv2.namedWindow('Image')
cv2.setMouseCallback('Image', get_pixel_coordinates)

# 显示图像并等待键盘输入
while True:
    cv2.imshow('Image', image)
    if cv2.waitKey(1) & 0xFF == 27:  # 按下 'Esc' 键退出
        break

# 关闭窗口
cv2.destroyAllWindows()
print("done!")