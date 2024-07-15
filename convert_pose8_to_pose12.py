import json
import numpy as np
from argparse import ArgumentParser
from scipy.spatial.transform import Rotation as R

parser = ArgumentParser()
parser.add_argument('--source_file', '-s')
args = parser.parse_args()

if __name__ == '__main__':
    H_list = []
    with open(args.source_file, 'r') as f:
        lines = f.readlines()
        for line in lines[1:]:
            line = line.strip().split(" ")
            line = list(map(float, line))
            trans = np.array(line[1:4])
            quat = line[4:8]
            # print(quat)
            R_t = R.from_quat(np.array(quat))
            R_t = R_t.as_matrix()
            H = np.concatenate((R_t, trans.reshape(-1, 1)), axis=1)
            H_list.append(H)
    with open("poses.txt", 'w') as f:
        for H in H_list:
            H = H.tolist()
            f.write(str(H[0][0]) + "\t" + str(H[0][1]) + "\t" + str(H[0][2]) + "\t" + str(H[0][3]) + "\t" + str(
                H[1][0]) + "\t" + str(
                H[1][1]) + "\t" + str(H[1][2]) + "\t" + str(H[1][3]) + "\t" +
                    str(H[2][0]) + "\t" + str(H[2][1]) + "\t" + str(H[2][2]) + "\t" + str(H[2][3]) +
                    "\n")
