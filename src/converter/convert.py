from src.converter.to_rosbag import ToRosbag


class Convert:
    """Class to decide how to convert the dataset to rosbag

    USAGE:
            Convert(args=args)

    """
    def __init__(self, args):

        self.args = args
        self.date = args.date

        # check cam_folder argument
        if args.cam_folder is not None:
            if 'all' in args.cam_folder:
                self.cam_folder = 'all'
            else:
                self.cam_folder = int(args.cam_folder[0])
        else:
            self.cam_folder = None

        print("Converting data from %s" % self.date)

        try:
            self.bag_name = self.args.bag_name[0]

        except TypeError:
            print("no bag name was specified, 'example' was selected as default!")
            self.bag_name = "example"

        if (not self.args.lb3) and (not self.args.sen) and (not self.args.vel) and (not self.args.hokuyo) and \
           (not self.args.gt) and (not self.args.gt_cov):
            self.args.gt     = True
            self.args.gt_cov = True
            self.args.hokuyo = True
            self.args.sen    = True
            self.args.vel    = True
            self.args.lb3    = True
            converter = ToRosbag(date=self.date, args=self.args, bag_name=self.bag_name, cam_folder=self.cam_folder)
        else:
            converter = ToRosbag(date=self.date, args=self.args, bag_name=self.bag_name, cam_folder=self.cam_folder)


        converter.process()
