from src.visualizer.gt import GroundTruth
from src.visualizer.gps import GPS
from src.visualizer.gps_rtk import GPS_RTK
from src.visualizer.wheel_odom import WheelOdom
from src.visualizer.all import AllSensors


class Visualize:
    """Class to visualize depending on the arguments

    USAGE:
            Visualize(args=args)

    """
    def __init__(self, args):

        self.args = args
        self.date = args.date

        if self.args.v_gt_kml:
            print("visualize ground truth kml from %s" % self.args.date)
            self.gt = GroundTruth(date=self.date)
            self.gt.save_kml_line()
            print("successfully created ground truth kml file in /plots/%s/kml/" % self.args.date)

        elif self.args.v_gt_png:
            print("visualize ground truth png from %s" % self.args.date)
            self.gt = GroundTruth(date=self.date)
            self.gt.save_gt_png(offset=True)
            print("successfully created ground truth png file in /plots/%s/png/" % self.args.date)

        elif self.args.v_gps_kml:
            print("visualize gps kml from %s" % self.args.date)
            self.gps = GPS(date=self.date)
            self.gps.save_kml_line()
            print("successfully created gps kml file in /plots/%s/kml/" % self.args.date)

        elif self.args.v_gps_png:
            print("visualize gps png from %s" % self.args.date)
            self.gps = GPS(date=self.date)
            self.gps.save_gps_png()
            print("successfully created gps png file in /plots/%s/png/" % self.args.date)

        elif self.args.v_gps_rtk_kml:
            print("visualize gps_rtk kml from %s" % self.args.date)
            self.gps_rtk = GPS_RTK(date=self.date)
            self.gps_rtk.save_kml_line()
            print("successfully created gps_rtk kml file in /plots/%s/kml/" % self.args.date)

        elif self.args.v_gps_rtk_png:
            print("visualize gps_rtk png from %s" % self.args.date)
            self.gps_rtk = GPS_RTK(date=self.date)
            self.gps_rtk.save_gps_rtk_png()
            print("successfully created gps_rtk png file in /plots/%s/png/" % self.args.date)

        elif self.args.v_odom_kml:
            print("visualize wheel odometry kml from %s" % self.args.date)
            self.wheel_odom = WheelOdom(date=self.date)
            self.wheel_odom.save_kml_line()
            print("successfully created wheel odometry kml file in /plots/%s/kml/" % self.args.date)

        elif self.args.v_odom_png:
            print("visualize wheel odometry png from %s" % self.args.date)
            self.wheel_odom = WheelOdom(date=self.date)
            self.wheel_odom.save_wheel_odom_png()
            print("successfully created wheel odometry png file in /plots/%s/png/" % self.args.date)

        elif self.args.v_all:
            print("visualizes all data from %s" % self.args.date)
            self.all = AllSensors(date=self.date)
            self.all.plot()
            print("successfully created raw_data_all png file in /plots/%s/png/" % self.args.date)

        else:
            print("no matching arguments were found")

