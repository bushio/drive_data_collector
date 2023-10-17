#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import numpy as np
import sys
import glob
import argparse
import matplotlib.pyplot as plt

def load_npy(dirname: str, save_dirname: str, ext: str = ".npy"):
        files = glob.glob(dirname + "/*" + ext)
        for f in files:
            print(f)
            arr = np.load(f, allow_pickle=True)
            arr = arr.item()
            lidar_xyz = np.nan_to_num(arr["xyz"])
            intensity = np.nan_to_num(arr["intensity"])

            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            ax.scatter(lidar_xyz[:, 0], lidar_xyz[:, 1], lidar_xyz[:, 2], c=intensity, cmap='viridis')
            cbar = plt.colorbar(ax.scatter([], [], []))
            cbar.set_label('Intensity')


            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            save_file_name = save_dirname + "/{}.png".format(os.path.splitext(os.path.basename(f))[0])
            fig.savefig(save_file_name)
            plt.clf()

def argparser():
    parser = argparse.ArgumentParser()
    parser.add_argument('dir', help="Directory path in which lidar data are saved.")
    parser.add_argument('save_dir', help="Directory path for saving visualization data")
    args = parser.parse_args()
    return args

def main(args):
    print('Hi from point_cloud_publisher')
    
    if not os.path.isdir(args.dir):
        print("{} is not existed!!".format(args.dir))
        exit()
    os.makedirs(args.save_dir, exist_ok=True)

    load_npy(args.dir, args.save_dir)

    return 1

if __name__ == '__main__':
    args = argparser()
    main(args)