#!/usr/bin/env python

import argparse
from src.downloader.dataset import LoadDataset
from src.extractor.extract import ExtractRawData
from src.converter.convert import Convert
from src.visualizer.visualize import Visualize


def main():

    parser = argparse.ArgumentParser(description="Downloads, extracts, converts and visualizes the NCLT dataset")
    # mandatory arguments
    parser.add_argument('action', help='Specify an action command to execute (e.g. download, extract, convert etc)')
    parser.add_argument('date', metavar='Date', type=str, help='Name one date from the dataset')

    # arguments for downloading
    parser.add_argument('--lb3',    dest='lb3',      action='store_true', help='Download Ladybug3 Images')
    parser.add_argument('--sen',    dest='sen',      action='store_true', help='Download sensor data (includes odometry)')
    parser.add_argument('--vel',    dest='vel',      action='store_true', help='Download velodyne data')
    parser.add_argument('--hokuyo', dest='hokuyo',   action='store_true', help='Download hokuyo data')
    parser.add_argument('--gt',     dest='gt',       action='store_true', help='Download ground truth')
    parser.add_argument('--gt_cov', dest='gt_cov',   action='store_true', help='Download ground truth covariance')

    # arguments for visualizing
    parser.add_argument('--gt_kml',      dest='v_gt_kml',      action='store_true', help='Visualize ground truth as a kml file')
    parser.add_argument('--gt_png',      dest='v_gt_png',      action='store_true', help='Visualize ground truth as a png file')
    parser.add_argument('--gps_kml',     dest='v_gps_kml',     action='store_true', help='Visualize gps as a kml file')
    parser.add_argument('--gps_png',     dest='v_gps_png',     action='store_true', help='Visualize gps as a png file')
    parser.add_argument('--gps_rtk_kml', dest='v_gps_rtk_kml', action='store_true', help='Visualize gps_rtk as a kml file')
    parser.add_argument('--gps_rtk_png', dest='v_gps_rtk_png', action='store_true', help='Visualize gps_rtk as a png file')
    parser.add_argument('--odom_kml',    dest='v_odom_kml',    action='store_true', help='Visualize wheel odometry as a kml file')
    parser.add_argument('--odom_png',    dest='v_odom_png',    action='store_true', help='Visualize wheel odometry as a png file')
    parser.add_argument('--all',         dest='v_all',         action='store_true', help='Visualizes all sensor data as a png file')

    parser.add_argument('--bag',    dest='bag_name', action='append', help='Name of the rosbag file')
    parser.add_argument('--cam_folder', dest='cam_folder', action='append', help='Number between 0 and 5 for camera folder')
    args = parser.parse_args()

    if args.action == 'download':
        print("Download NCLT dataset from %s" % args.date)
        LoadDataset(args=args)  # multiple args could be provided

    elif args.action == 'extract':
        print("Extracting data from %s" % args.date)
        ExtractRawData(date=args.date)

    elif args.action == 'visualize':
        Visualize(args=args)

    elif args.action == 'convert':
        print("Converting data from %s" % args.date)
        Convert(args=args)


if __name__ == '__main__':
    main()
