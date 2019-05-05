import os
import simplekml
import rospy
from definitions import ROOT_DIR
from src.nclt2ros import ReadSensorData
from src.nclt2ros import ReadGroundTruth


class Plotter:
    """Base class for visualizing

    USAGE:
            Plotter('2013-01-10')

    """
    def __init__(self, date):

        if isinstance(date, str):
            self.date = date
            self.reader    = ReadSensorData(date=date)
            self.reader_gt = ReadGroundTruth(date=date)
        else:
            raise TypeError('"date" must be of type string')

        PLOT_PATH_DFLT = ROOT_DIR + '/plots/'
        self.plot_path = rospy.get_param('~plot_path', PLOT_PATH_DFLT)

        if self.plot_path.endswith('/'):
            self.plot_dir = self.plot_path
        else:
            self.plot_dir = self.plot_path + '/'

        # define directories
        self.visualization_kml_dir            = self.plot_dir + '%s/kml/'            % self.date
        self.visualization_png_gt_dir         = self.plot_dir + '%s/png/gt/'         % self.date
        self.visualization_png_gps_rtk_dir    = self.plot_dir + '%s/png/gps_rtk/'    % self.date
        self.visualization_png_gps_dir        = self.plot_dir + '%s/png/gps/'        % self.date
        self.visualization_png_wheel_odom_dir = self.plot_dir + '%s/png/wheel_odom/' % self.date
        self.visualization_png_all_dir        = self.plot_dir + '%s/png/all/'        % self.date

        # check directories
        if not os.path.exists(self.visualization_kml_dir):
            os.makedirs(self.visualization_kml_dir)

        if not os.path.exists(self.visualization_png_gt_dir):
            os.makedirs(self.visualization_png_gt_dir)

        if not os.path.exists(self.visualization_png_gps_rtk_dir):
            os.makedirs(self.visualization_png_gps_rtk_dir)

        if not os.path.exists(self.visualization_png_gps_dir):
            os.makedirs(self.visualization_png_gps_dir)

        if not os.path.exists(self.visualization_png_wheel_odom_dir):
            os.makedirs(self.visualization_png_wheel_odom_dir)

        if not os.path.exists(self.visualization_png_all_dir):
            os.makedirs(self.visualization_png_all_dir)

        # kml settings
        self.kml = simplekml.Kml()
        self.green = simplekml.Color.lime
        self.red = simplekml.Color.red
        self.yellow = simplekml.Color.yellow
        self.magenta = simplekml.Color.magenta

    def get_kml_dir(self):
        """get the kml directory

        :return: path to kml directory
        """
        return self.visualization_kml_dir