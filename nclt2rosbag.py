#!/usr/bin/env python

import argparse
from src.downloader.dataset import LoadDataset
from src.extractor.extract import ExtractRawData


def main():

    parser = argparse.ArgumentParser(description="Downloads, extracts, converts and visualizes the NCLT dataset")
    parser.add_argument('action', help='Specify an action command to execute (e.g. download, extract, convert etc)')
    parser.add_argument('date', metavar='Date', type=str, help='Name one date from the dataset')
    parser.add_argument('--lb3',    dest='lb3',      action='store_true', help='Download Ladybug3 Images')
    parser.add_argument('--sen',    dest='sen',      action='store_true', help='Download sensor data (includes odometry)')
    parser.add_argument('--vel',    dest='vel',      action='store_true', help='Download velodyne data')
    parser.add_argument('--hokuyo', dest='hokuyo',   action='store_true', help='Download hokuyo data')
    parser.add_argument('--gt',     dest='gt',       action='store_true', help='Download ground truth')
    parser.add_argument('--gt_cov', dest='gt_cov',   action='store_true', help='Download ground truth covariance')
    parser.add_argument('--bag',    dest='bag_name', action='append', help='Name of the rosbag file')
    parser.add_argument('--cam_folder', dest='cam_folder', action='append', help='Number between 0 and 5 for camera folder')
    args = parser.parse_args()

    if args.action == 'download':
        print("Download NCLT dataset from %s" % args.date)
        LoadDataset(args=args)  # multiple args could be provided

    elif args.action == 'extract':
        print("Extracting data from %s" % args.date)
        ExtractRawData(date=args.date)


    """
    elif args.action == 'convert':
        print("Converting data from %s" % args.date)
        if args.cam_folder is None:
            converter = ConvertToRosbag(date=args.date, bag_name=args.bag_name[0])
        else:
            converter = ConvertToRosbag(date=args.date, bag_name=args.bag_name[0], cam_folder=int(args.cam_folder[0]))
        converter.process()

    elif args.action == 'visualize':
        print("Visualize data from %s" % args.date)

    """


if __name__ == '__main__':
    main()
