import cv2

video_path = "/home/liuwei/mnt/vggt_dataset/milk_dragon/nailong.mp4"
output_path = "/home/liuwei/mnt/vggt_dataset/milk_dragon/output_cropped.mp4"


cap = cv2.VideoCapture(video_path)
ret, frame = cap.read()
if not ret or frame is None:
    raise RuntimeError("❌ 无法读取视频第一帧")

# ✅ 关键：显式创建窗口
cv2.namedWindow("选择裁剪区域", cv2.WINDOW_NORMAL)
cv2.imshow("选择裁剪区域", frame)

# ✅ selectROI 调用
r = cv2.selectROI("选择裁剪区域", frame, showCrosshair=True, fromCenter=False)
x, y, w, h = map(int, r)
cv2.destroyAllWindows()

if w == 0 or h == 0:
    raise RuntimeError("❌ 未选择有效ROI")

print(f"ROI: x={x}, y={y}, w={w}, h={h}")

# ✅ 执行裁剪
cap = cv2.VideoCapture(video_path)
fps = cap.get(cv2.CAP_PROP_FPS)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_path, fourcc, fps, (w, h))

while True:
    ret, frame = cap.read()
    if not ret:
        break
    out.write(frame[y:y+h, x:x+w])

cap.release()
out.release()
print("✅ 裁剪完成：", output_path)

