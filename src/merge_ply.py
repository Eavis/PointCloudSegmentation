import os
import glob
import argparse
import numpy as np
import plyfile


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder_path", required=True)
    parser.add_argument("--merged_path", required=True)
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    files = []
    for file_name in os.listdir(args.folder_path):
        if len(file_name) < 4 or file_name[-4:].lower() != ".ply":
            continue

        print ("Reading file", file_name)
        file = plyfile.PlyData.read(os.path.join(args.folder_path, file_name))
        for element in file.elements:
            files.append(element.data)
    #import pdb;pdb.set_trace()
    print ("Merging files")
    merged_file = np.concatenate(files, -1)
    merged_el = plyfile.PlyElement.describe(merged_file, 'vertex')

    print ("Writing merged file")
    plyfile.PlyData([merged_el]).write(args.merged_path)


if __name__ == '__main__':
    main()
