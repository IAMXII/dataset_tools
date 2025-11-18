import os
import json
from glob import glob
from PIL import Image, ImageDraw
from tqdm import tqdm


# ============================
# 请修改你的数据路径
# ============================
DATA_ROOT = "/home/liuwei/mnt/instant_vggt_dataset/liuweihitsz_4089962b/ADE20K_2021_17_01/images/ADE"      # 包含 training/ 和 validation/
OUTPUT_ROOT = "/home/liuwei/mnt/instant_vggt_dataset/liuweihitsz_4089962b/ade_sky"

def load_json_with_fallback(json_file):
    """尝试多种编码读取 JSON，全部失败则返回 None"""
    encodings = ["utf-8", "latin-1", "gbk", "utf-16"]

    for enc in encodings:
        try:
            with open(json_file, "r", encoding=enc) as f:
                return json.load(f)
        except Exception:
            continue

    return None

def ensure_dirs():
    for split in ["train", "val"]:
        os.makedirs(os.path.join(OUTPUT_ROOT, split, "imgs"), exist_ok=True)
        os.makedirs(os.path.join(OUTPUT_ROOT, split, "gts"), exist_ok=True)


# ============================
# 将 JSON 转天空掩膜（无天空 → 全白）
# ============================
def generate_sky_mask(json_file, size):
    data = load_json_with_fallback(json_file)

    # JSON 读取失败 → 返回 None 让上层跳过
    if data is None:
        return None

    w, h = size
    mask = Image.new("L", (w, h), 255)  # 默认全白
    draw = ImageDraw.Draw(mask)

    ann = data.get("annotation", {})
    objs = ann.get("object", [])

    for obj in objs:
        name = obj.get("name", "").lower()
        if "sky" not in name:
            continue

        poly = obj.get("polygon", {})
        xs, ys = poly.get("x", []), poly.get("y", [])
        if len(xs) == 0 or len(xs) != len(ys):
            continue

        points = [(int(x), int(y)) for x, y in zip(xs, ys)]
        draw.polygon(points, fill=0)

    return mask



# ============================
# 递归收集图像和标注对
# ============================
def collect_pairs(root_dir):
    # 支持 jpg/png
    imgs = glob(os.path.join(root_dir, "**/*.*"), recursive=True)
    valid_img_ext = [".jpg", ".jpeg", ".png"]

    pairs = []
    for img_path in imgs:
        ext = os.path.splitext(img_path)[1].lower()
        if ext not in valid_img_ext:
            continue

        base = os.path.splitext(img_path)[0]
        json_path = base + ".json"
        if os.path.exists(json_path):
            pairs.append((img_path, json_path))

    return pairs


# ============================
# 处理单个 split
# ============================
def process_split(split_name, input_dir):
    print(f"\n===== 处理 {split_name} 数据 =====")

    out_img_dir = os.path.join(OUTPUT_ROOT, split_name, "imgs")
    out_gt_dir = os.path.join(OUTPUT_ROOT, split_name, "gts")

    pairs = collect_pairs(input_dir)
    print(f"找到 {len(pairs)} 张图像")

    idx = 1
    skipped = 0

    for img_path, json_path in tqdm(pairs):

        img = Image.open(img_path).convert("RGB")

        mask = generate_sky_mask(json_path, img.size)

        # JSON 无法解析 → 跳过该样本
        if mask is None:
            skipped += 1
            continue

        # 正常保存
        fname = f"{idx:06d}"
        img.save(os.path.join(out_img_dir, fname + ".jpg"))
        mask.save(os.path.join(out_gt_dir, fname + ".png"))

        idx += 1

    print(f"{split_name} 跳过无效 JSON 数量：{skipped}")
    print(f"{split_name} 最终有效样本数量：{idx-1}")



def main():
    ensure_dirs()

    train_dir = os.path.join(DATA_ROOT, "training")
    val_dir = os.path.join(DATA_ROOT, "validation")

    process_split("train", train_dir)
    process_split("val", val_dir)

    print("\n全部完成！")


if __name__ == "__main__":
    main()
