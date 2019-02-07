from src.converter.to_rosbag import ToRosbag


class Convert:

    def __init__(self, args):

        self.args = args
        self.date = args.date

        if args.cam_folder is None:
            converter = ToRosbag(date=args.date, bag_name=args.bag_name[0])
        else:
            converter = ToRosbag(date=args.date, bag_name=args.bag_name[0], cam_folder=int(args.cam_folder[0]))

        converter.process()