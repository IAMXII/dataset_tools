import os

from moviepy.editor import VideoFileClip, concatenate_videoclips
from argparse import ArgumentParser
parser = ArgumentParser()
parser.add_argument('--video1', required=True)
parser.add_argument('--video2',required=True)
parser.add_argument('output_path',required=True)
args = parser.parse_args()
# 指定两个视频文件的路径
# video1_path = '../Downloads/360_v2/nyl/video1.mp4'
# video2_path = '../Downloads/360_v2/nyl/video2.mp4'

# 使用 VideoFileClip 加载视频文件
clip1 = VideoFileClip(args.video1)
clip2 = VideoFileClip(args.video2)

# 拼接两个视频（按时间轴顺序）
final_clip = concatenate_videoclips([clip1, clip2])
output = os.path.join(args.output_path,'output.mp4')
# 保存拼接后的视频
final_clip.write_videofile(output, codec='libx264')