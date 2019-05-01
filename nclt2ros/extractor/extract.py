import os
import tarfile
import rospy
from base_raw_data import BaseRawData


class Extract(BaseRawData):
    """Class to extract the tarballs in param 'raw_data_path'

    USAGE:
            ExtractRawData('2013-01-10')

    """

    def __init__(self, date, lb3=False, sen=False, vel=False, hokuyo=False):

        if isinstance(date, str):
            self.date = date
        else:
            raise TypeError("'date' must be type of string")

        # init base class
        BaseRawData.__init__(self, date=date)

        self.lb3    = lb3
        self.sen    = sen
        self.vel    = vel
        self.hokuyo = hokuyo

        if self.lb3 or self.sen or self.vel or self.hokuyo:
            rospy.loginfo("Extracting data from %s" % self.date)
            self.extract_data()
        else:
            rospy.loginfo("Nothing to extract")

    def extract_data(self):
        """extracts the data from tar.gz format
        """

        # check hokuyo_data
        if self.hokuyo:
            if os.path.exists(self.hokuyo_data_dir):
                os.chdir(self.hokuyo_data_dir)
                files = os.listdir(self.hokuyo_data_dir)
                if not (self.date in files):
                    for file in files:
                        if file.endswith('tar.gz'):
                            rospy.loginfo("unpacking {}".format(file))
                            tar = tarfile.open(file, 'r:gz')
                            tar.extractall()
                            tar.close()
                            # remove tar.gz file
                            os.remove(file)
                else:
                    rospy.loginfo("hokuyo_data already exists")

        # check sensor_data
        if self.sen:
            if os.path.exists(self.sensor_data_dir):
                os.chdir(self.sensor_data_dir)
                files = os.listdir(self.sensor_data_dir)
                if not (self.date in files):
                    for file in files:
                        if file.endswith('tar.gz'):
                            rospy.loginfo("unpacking {}".format(file))
                            tar = tarfile.open(file, 'r:gz')
                            tar.extractall()
                            tar.close()
                            # remove tar.gz file
                            os.remove(file)

                else:
                    rospy.loginfo("sensor_data already exists")

        # check velodyne_data
        if self.vel:
            if os.path.exists(self.velodyne_data_dir):
                os.chdir(self.velodyne_data_dir)
                files = os.listdir(self.velodyne_data_dir)
                if not (self.date in files):
                    for file in files:
                        if file.endswith('tar.gz'):
                            rospy.loginfo("unpacking {}".format(file))
                            tar = tarfile.open(file, 'r:gz')
                            tar.extractall()
                            tar.close()
                            # remove tar.gz file
                            os.remove(file)
                else:
                    rospy.loginfo("velodyne_data already exists")

        # check image_data
        if self.lb3:
            if os.path.exists(self.images_dir):
                os.chdir(self.images_dir)
                files = os.listdir(self.images_dir)
                if not (self.date in files):
                    for file in files:
                        if file.endswith('tar.gz'):
                            rospy.loginfo("unpacking {}".format(file))
                            tar = tarfile.open(file, 'r:gz')
                            tar.extractall()
                            tar.close()
                            # remove tar.gz file
                            os.remove(file)
                else:
                    rospy.loginfo("image_data already exists")
