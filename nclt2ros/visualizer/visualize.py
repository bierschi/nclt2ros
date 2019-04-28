from nclt2ros.visualizer.gt         import GroundTruth
from nclt2ros.visualizer.gps        import GPS
from nclt2ros.visualizer.gps_rtk    import GPS_RTK
from nclt2ros.visualizer.wheel_odom import WheelOdom
from nclt2ros.visualizer.all        import AllSensors


class Visualize:
    """Class to visualize depending on the arguments

    USAGE:
            Visualize(args=args)

    """
    def __init__(self, date, **kwargs):

        self.date = date

        self.v_gt_kml = None
        self.v_gt_png = None
        self.v_gps_kml = None
        self.v_gps_png = None
        self.v_gps_rtk_kml = None
        self.v_gps_rtk_png = None
        self.v_odom_kml = None
        self.v_odom_png = None
        self.v_all = None

        for (key, value) in kwargs.iteritems():
            if hasattr(self, key):
                setattr(self, key, value)

        if self.v_gt_kml:
            print("visualize ground truth kml from %s" % self.date)
            self.gt = GroundTruth(date=self.date)
            self.gt.save_kml_line()
            print("successfully created ground truth kml file in /plots/%s/kml/" % self.date)

        if self.v_gt_png:
            print("visualize ground truth png from %s" % self.date)
            self.gt = GroundTruth(date=self.date)
            self.gt.save_gt_png(offset=True)
            print("successfully created ground truth png file in /plots/%s/png/" % self.date)

        if self.v_gps_kml:
            print("visualize gps kml from %s" % self.date)
            self.gps = GPS(date=self.date)
            self.gps.save_kml_line()
            print("successfully created gps kml file in /plots/%s/kml/" % self.date)

        if self.v_gps_png:
            print("visualize gps png from %s" % self.date)
            self.gps = GPS(date=self.date)
            self.gps.save_gps_png()
            print("successfully created gps png file in /plots/%s/png/" % self.date)

        if self.v_gps_rtk_kml:
            print("visualize gps_rtk kml from %s" % self.date)
            self.gps_rtk = GPS_RTK(date=self.date)
            self.gps_rtk.save_kml_line()
            print("successfully created gps_rtk kml file in /plots/%s/kml/" % self.date)

        if self.v_gps_rtk_png:
            print("visualize gps_rtk png from %s" % self.date)
            self.gps_rtk = GPS_RTK(date=self.date)
            self.gps_rtk.save_gps_rtk_png()
            print("successfully created gps_rtk png file in /plots/%s/png/" % self.date)

        if self.v_odom_kml:
            print("visualize wheel odometry kml from %s" % self.date)
            self.wheel_odom = WheelOdom(date=self.date)
            self.wheel_odom.save_kml_line()
            print("successfully created wheel odometry kml file in /plots/%s/kml/" % self.date)

        if self.v_odom_png:
            print("visualize wheel odometry png from %s" % self.date)
            self.wheel_odom = WheelOdom(date=self.date)
            self.wheel_odom.save_wheel_odom_png()
            print("successfully created wheel odometry png file in /plots/%s/png/" % self.date)

        if self.v_all:
            print("visualizes all data from %s" % self.date)
            self.all = AllSensors(date=self.date)
            self.all.plot()
            print("successfully created raw_data_all png file in /plots/%s/png/" % self.date)

        else:
            print("no matching arguments were found")

