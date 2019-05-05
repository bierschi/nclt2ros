import os
import rospy
import subprocess


class Download:
    """Class to download the NCLT Dataset from http://robots.engin.umich.edu/nclt/

    USAGE:
            Download(date='2013-01-10', raw_data_path='/home/christian/nclt2ros/raw_data', gt=True)
    """

    def __init__(self, date, raw_data_path, lb3=False, sen=False, hokuyo=False, vel=False, gt=False, gt_cov=False):

        self.download_url_dir = 'http://robots.engin.umich.edu/nclt'
        self.dates = ['2012-01-08', '2012-01-15', '2012-01-22', '2012-02-02', '2012-02-04', '2012-02-05', '2012-02-12',
                      '2012-02-18', '2012-02-19', '2012-03-17', '2012-03-25', '2012-03-31', '2012-04-29', '2012-05-11',
                      '2012-05-26', '2012-06-15', '2012-08-04', '2012-08-20', '2012-09-28', '2012-10-28', '2012-11-04',
                      '2012-11-16', '2012-11-17', '2012-12-01', '2013-01-10', '2013-02-23', '2013-04-05']

        self.date = date
        self.raw_data_path = str(raw_data_path)

        self.lb3    = lb3
        self.sen    = sen
        self.hokuyo = hokuyo
        self.vel    = vel
        self.gt     = gt
        self.gt_cov = gt_cov

        if self.raw_data_path.endswith('/'):
            self.raw_data_dir = raw_data_path + str(self.date)
        else:
            self.raw_data_dir = raw_data_path + '/' + str(self.date)

        if not os.path.exists(self.raw_data_dir):
            os.makedirs(self.raw_data_dir)

        self.saved_path = None

        if self.check_date(self.date):
            self.print_log()
            self.load_dataset()
        else:
            raise ValueError("Given 'Date' is not in dataset")

    def print_log(self):
        """method for logging purposes
        """

        rospy.loginfo("Download: ")

        if self.lb3:
            rospy.loginfo("- Images")
        if self.sen:
            rospy.loginfo("- Sensors")
        if self.hokuyo:
            rospy.loginfo("- Hokuyo")
        if self.vel:
            rospy.loginfo("- Velodyne")
        if self.gt:
            rospy.loginfo("- Ground Truth Pose")
        if self.gt_cov:
            rospy.loginfo("- Ground Truth Covariance")

        rospy.loginfo("from date %s " % self.date)

    def load_dataset(self):
        """load data depending on the arguments specified in the launch file
        """

        self.saved_path = os.getcwd()
        os.chdir(self.raw_data_dir)

        if self.lb3:
            self.get_images()

        if self.sen:
            self.get_sensors()

        if self.vel:
            self.get_velodyne()

        if self.hokuyo:
            self.get_hokuyo()

        if self.gt:
            self.get_ground_truth_pose()

        if self.gt_cov:
            self.get_ground_truth_cov()

        os.chdir(self.saved_path)

    def check_date(self, date):
        """checks if date is in self.dates

        :param date: String
        :return: Bool, True if date is in self.dates
                  else False
        """

        if date in self.dates:
            return True
        else:
            return False

    def get_images(self):
        """fetching the raw image data with wget and the continue flag for later continuation
        """

        cmd = ['wget', '--continue', '%s/images/%s_lb3.tar.gz' % (self.download_url_dir, self.date), '-P', 'images']
        cmd_str = ' '.join(cmd)
        rospy.loginfo("Calling: %s" % cmd_str)
        subprocess.call(cmd)

    def get_sensors(self):
        """fetching the raw sensors data with wget and the continue flag for later continuation
        """

        cmd = ['wget', '--continue', '%s/sensor_data/%s_sen.tar.gz' % (self.download_url_dir, self.date), '-P', 'sensor_data']
        cmd_str = ' '.join(cmd)
        rospy.loginfo("Calling: %s" % cmd_str)
        subprocess.call(cmd)

    def get_velodyne(self):
        """fetching the raw velodyne data with wget and the continue flag for later continuation
        """

        cmd = ['wget', '--continue', '%s/velodyne_data/%s_vel.tar.gz' % (self.download_url_dir, self.date), '-P', 'velodyne_data']
        cmd_str = ' '.join(cmd)
        rospy.loginfo("Calling: %s" % cmd_str)
        subprocess.call(cmd)

    def get_hokuyo(self):
        """fetching the raw hokuyo data with wget and the continue flag for later continuation
        """

        cmd = ['wget', '--continue', '%s/hokuyo_data/%s_hokuyo.tar.gz' % (self.download_url_dir, self.date), '-P', 'hokuyo_data']
        cmd_str = ' '.join(cmd)
        rospy.loginfo("Calling: %s" % cmd_str)
        subprocess.call(cmd)

    def get_ground_truth_pose(self):
        """fetching the raw ground truth pose data with wget and the continue flag for later continuation
        """

        cmd = ['wget', '--continue', '%s/ground_truth/groundtruth_%s.csv' % (self.download_url_dir, self.date), '-P', 'ground_truth']
        cmd_str = ' '.join(cmd)
        rospy.loginfo("Calling: %s" % cmd_str)
        subprocess.call(cmd)

    def get_ground_truth_cov(self):
        """fetching the raw ground truth covariance data with wget and the continue flag for later continuation
        """

        cmd = ['wget', '--continue', '%s/covariance/cov_%s.csv' % (self.download_url_dir, self.date), '-P', 'ground_truth_covariance']
        cmd_str = ' '.join(cmd)
        rospy.loginfo("Calling: %s" % cmd_str)
        subprocess.call(cmd)
