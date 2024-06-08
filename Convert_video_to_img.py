import cv2
import os
from argparse import ArgumentParser

print('————欢迎来到视频连续截图自动生成系统2.0！————')
f = 30  # 截图速率默认为每30帧一张

parser = ArgumentParser(description='Convert_video_to_img')
parser.add_argument("--source_path", "-s", required=True)
parser.add_argument("--video", "-v", required=True)
parser.add_argument("--frame_size", "-f")
args = parser.parse_args()
f = eval(args.frame_size)

def transfer(args):  # 定义转换过程（核心代码）
    try:
        os.makedirs(pic_path)  # 自动在视频文件同一路径生成以视频文件名为名字的新文件夹
    except FileExistsError:
        print('————已存在与视频同名文件夹，请删除后再操作！————')
        quit()  # 存在同名文件夹系统报错，退出程序
    cap = cv2.VideoCapture(video)  # 导入视频文件
    num = 1  # 可在此处修改图片文件名起始序号
    print('————设置完成，即将开始导出图片！————')
    print('————' * 8)
    while True:
        if cap.grab():
            num += 1
            if num % f == 1:  # 每f帧截取一个图片
                flag, frame = cap.retrieve()  # 解码并返回一个帧
                if not flag:
                    continue
                else:
                    cv2.imshow('video', frame)
                    new = pic_path + "/" + pic_name + str(int(num / f)) + ".jpg"  # 定义图片的输出路径以及名字
                    print('正在导出：' + pic_name + str(int(num / f)) + ".jpg (按Esc停止运行)")
                    cv2.imencode('.jpg', frame)[1].tofile(new)  # 将生成的截图输出到新文件夹并命名
        else:  # 运行完毕自动退出
            break
        if cv2.waitKey(10) == 27:  # 检测到按下Esc时退出
            quit()

while True:
    try:
        video = args.source_path + "/" + args.video + ".mp4"  # 格式默认为mp4，如果你要转换别的格式，请在此处更改
        if not os.path.exists(video) or args.video == '':  # 判断文件是否存在
            print('你输入的文件名有误，请重新输入！')
        if os.path.exists(video) and args.video != '':
            print('————视频文件匹配成功！————')
            break
    except ValueError:
        print('你输入的文件名有误，请重新输入！')

# while True:
#     try:
#         f = int(input(
#             '请输入你想要的帧数(即每多少帧截一张图，如：最小间隔逐帧截图，则输入2，不输入则默认设置为30)：'))  # 获取用户的帧率需求，只限数字
#         if f < 0:
#             print('帧数不能为负！')
#         elif f == 0:
#             print('帧数不能为0！')
#         elif f == 1:
#             f += 1
#             print('————帧数不支持设为1，截图速率已设置为' + str(f) + '帧一张！————')  # 帧数为1时，系统不运作，这个是bug
#             break
#         else:
#             print('————截图速率已成功设置为' + str(f) + '帧一张！————')
#             break
#     except ValueError:
#         print('————截图速率已自动设置为默认的' + str(f) + '帧一张！————')
#         break


pic_name = ""  # 不输入则默认为空名字+数字
pic_path = video[:-4]  # 根据用户输入的视频文件路径来定义图片存放路径

transfer(args)  # 主程序启动

print('————' * 8)
print('运行完毕！图片已全部保存在：' + pic_path)
