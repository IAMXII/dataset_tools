import os
import sys
import glob
import subprocess
import datetime
from multiprocessing import Pool
from pytube import YouTube
from time import sleep

from skimage import io
from PIL import Image
import numpy as np


class Data:
    def __init__(self, url, seqname, list_timestamps):
        self.url = url
        self.list_seqnames = [seqname]
        self.list_list_timestamps = [list_timestamps]

    def add(self, seqname, list_timestamps):
        self.list_seqnames.append(seqname)
        self.list_list_timestamps.append(list_timestamps)

    def __len__(self):
        return len(self.list_seqnames)


def resize_image(image, scale=0.5):
    """使用 Pillow 代替 scipy.misc.imresize"""
    h, w = int(image.shape[0] * scale), int(image.shape[1] * scale)
    img = Image.fromarray(image)
    img = img.resize((w, h), Image.BILINEAR)
    return np.array(img)


def process(data, seq_id, videoname, output_root):
    seqname = data.list_seqnames[seq_id]
    seq_dir = os.path.join(output_root, seqname)

    if not os.path.exists(seq_dir):
        os.makedirs(seq_dir)
    else:
        print(f"[INFO] Skip {seqname}, directory already exists.")
        return True

    list_str_timestamps = []
    for timestamp in data.list_list_timestamps[seq_id]:
        timestamp = int(timestamp / 1000)
        str_hour = str(int(timestamp / 3600000)).zfill(2)
        str_min = str(int(int(timestamp % 3600000) / 60000)).zfill(2)
        str_sec = str(int(int(int(timestamp % 3600000) % 60000) / 1000)).zfill(2)
        str_mill = str(int(int(int(timestamp % 3600000) % 60000) % 1000)).zfill(3)
        _str_timestamp = f"{str_hour}:{str_min}:{str_sec}.{str_mill}"
        list_str_timestamps.append(_str_timestamp)

    # extract frames using ffmpeg
    for idx, str_timestamp in enumerate(list_str_timestamps):
        out_path = os.path.join(seq_dir, f"{data.list_list_timestamps[seq_id][idx]}.png")
        command = f'ffmpeg -y -ss {str_timestamp} -i "{videoname}" -vframes 1 -f image2 "{out_path}"'
        os.system(command)

    png_list = sorted(glob.glob(os.path.join(seq_dir, "*.png")))

    for pngname in png_list:
        image = io.imread(pngname)
        if int(image.shape[1] / 2) < 500:  # skip small frames
            break
        image = resize_image(image, 0.5)
        io.imsave(pngname, image)

    return False


def wrap_process(list_args):
    return process(*list_args)


class DataDownloader:
    def __init__(self, dataroot, mode='test'):
        print("[INFO] Loading data list ... ", end='')
        self.dataroot = dataroot
        self.list_seqnames = sorted(glob.glob(os.path.join(dataroot, '*.txt')))
        self.output_root = os.path.join('./dataset', mode)
        self.mode = mode

        self.isDone = False
        if not os.path.exists(self.output_root):
            os.makedirs(self.output_root)
        else:
            print("[INFO] The output dir already exists.")
            self.isDone = True

        self.list_data = []
        if not self.isDone:
            for txt_file in self.list_seqnames:
                seq_name = os.path.splitext(os.path.basename(txt_file))[0]

                # extract info from txt
                with open(txt_file, "r") as f:
                    lines = f.readlines()
                youtube_url = lines[0].strip()
                list_timestamps = [int(line.split(' ')[0]) for line in lines[1:] if line.strip()]

                # register or append
                for existing_data in self.list_data:
                    if youtube_url == existing_data.url:
                        existing_data.add(seq_name, list_timestamps)
                        break
                else:
                    self.list_data.append(Data(youtube_url, seq_name, list_timestamps))

            print("Done!")
            print(f"[INFO] {len(self.list_data)} movies are used in {self.mode} mode.")

    def Run(self):
        print(f"[INFO] Start downloading {len(self.list_data)} movies")

        for global_count, data in enumerate(self.list_data):
            print(f"[INFO] Downloading {data.url}")
            try:
                yt = YouTube(data.url)
                stream = yt.streams.filter(progressive=True, file_extension='mp4').first()
                if stream is None:
                    raise Exception("No MP4 stream found.")
                videoname = f"./current_{self.mode}.mp4"
                stream.download(filename=videoname)
            except Exception as e:
                print(f"[ERROR] Failed to download {data.url}: {e}")
                with open(f'failed_videos_{self.mode}.txt', 'a') as f:
                    for seqname in data.list_seqnames:
                        f.write(seqname + '\n')
                continue

            sleep(1)

            if len(data) == 1:
                process(data, 0, videoname, self.output_root)
            else:
                with Pool(processes=4) as pool:
                    pool.map(wrap_process, [(data, seq_id, videoname, self.output_root)
                                            for seq_id in range(len(data))])

            os.remove(videoname)

            if self.isDone:
                return False

        return True

    def Show(self):
        print("########################################")
        global_count = 0
        for data in self.list_data:
            print(f"URL : {data.url}")
            for idx in range(len(data)):
                print(f" SEQ_{idx} : {data.list_seqnames[idx]}")
                print(f" LEN_{idx} : {len(data.list_list_timestamps[idx])}")
                global_count += 1
            print("----------------------------------------")

        print(f"TOTAL : {global_count} sequences")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: generate_dataset.py [test or train]")
        quit()

    mode = sys.argv[1].lower()
    if mode not in ["test", "train"]:
        print("invalid mode")
        quit()

    dataroot = os.path.join("./RealEstate10K", mode)
    downloader = DataDownloader(dataroot, mode)
    downloader.Show()
    isOK = downloader.Run()

    if isOK:
        print("Done!")
    else:
        print("Failed")
