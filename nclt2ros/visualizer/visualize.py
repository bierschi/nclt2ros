import rospy
from nclt2ros.visualizer.gt import GroundTruth
from nclt2ros.visualizer.gps import GPS
from nclt2ros.visualizer.gps_rtk import GPS_RTK
from nclt2ros.visualizer.wheel_odom import WheelOdom
from nclt2ros.visualizer.all import AllSensors


class Visualize:
    """Class to visualize the raw data depending on the parameter specified in the launch file

    USAGE:
            Visualize(date='2013-01-10')

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

        # TODO expand with more commands

        if self.v_gt_kml:
            rospy.loginfo("visualize ground truth kml from date %s" % self.date)
            self.gt = GroundTruth(date=self.date)
            self.gt.save_kml_line()
            kml_dir = self.gt.get_kml_dir()
            rospy.loginfo("successfully created ground truth kml file in %s" % kml_dir)

        if self.v_gt_png:
            rospy.loginfo("visualize ground truth png from date %s" % self.date)
            self.gt = GroundTruth(date=self.date)
            self.gt.save_gt_png(offset=True)
            png_gt_dir = self.gt.get_png_gt_dir()
            rospy.loginfo("successfully created ground truth png file in %s" % png_gt_dir)

        if self.v_gps_kml:
            rospy.loginfo("visualize gps kml from date %s" % self.date)
            self.gps = GPS(date=self.date)
            self.gps.save_kml_line()
            kml_dir = self.gps.get_kml_dir()
            rospy.loginfo("successfully created gps kml file in %s" % kml_dir)

        if self.v_gps_png:
            rospy.loginfo("visualize gps png from %s" % self.date)
            self.gps = GPS(date=self.date)
            self.gps.save_gps_png()
            png_gps_dir = self.gps.get_png_gps_dir()
            rospy.loginfo("successfully created gps png file in %s" % png_gps_dir)

        if self.v_gps_rtk_kml:
            rospy.loginfo("visualize gps_rtk kml from date %s" % self.date)
            self.gps_rtk = GPS_RTK(date=self.date)
            self.gps_rtk.save_kml_line()
            kml_dir = self.gps_rtk.get_kml_dir()
            rospy.loginfo("successfully created gps_rtk kml file in %s" % kml_dir)

        if self.v_gps_rtk_png:
            rospy.loginfo("visualize gps_rtk png from date %s" % self.date)
            self.gps_rtk = GPS_RTK(date=self.date)
            self.gps_rtk.save_gps_rtk_png()
            png_gps_rtk_dir = self.gps_rtk.get_png_gps_rtk_dir()
            rospy.loginfo("successfully created gps_rtk png file in %s" % png_gps_rtk_dir)

        if self.v_odom_kml:
            rospy.loginfo("visualize wheel odometry kml from date %s" % self.date)
            self.wheel_odom = WheelOdom(date=self.date)
            self.wheel_odom.save_kml_line()
            kml_dir = self.wheel_odom.get_kml_dir()
            rospy.loginfo("successfully created wheel odometry kml file in %s" % kml_dir)

        if self.v_odom_png:
            rospy.loginfo("visualize wheel odometry png from date %s" % self.date)
            self.wheel_odom = WheelOdom(date=self.date)
            self.wheel_odom.save_wheel_odom_png()
            png_odom_dir = self.wheel_odom.get_png_odom_dir()
            rospy.loginfo("successfully created wheel odometry png file in %s/" % png_odom_dir)

        if self.v_all:
            rospy.loginfo("visualizes all data from date %s" % self.date)
            self.all = AllSensors(date=self.date)
            self.all.plot()
            png_all_dir = self.all.get_png_all_dir()
            rospy.loginfo("successfully created raw_data_all png file in %s" % png_all_dir)

        else:
            rospy.loginfo("no matching parameters were found")

