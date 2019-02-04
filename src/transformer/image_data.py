import os
import rospy
import rosbag
import numpy as np
import cv2
import cv_bridge

from PIL import Image as pilImage
from src.transformer.data import Data


class ImageData(Data):
    """Class to transform the images from the ladybug camera

    USAGE:
            ImageData('2013-01-10', write_to_bag=False)

    """
    def __init__(self, date, write_to_bag=False):

        # init base class
        Data.__init__(self, date=date)

        self.num_cameras = 6
        self.bridge = cv_bridge.CvBridge()

        if write_to_bag:
            self.bag = rosbag.Bag('image_bag', 'w')

    def get_image_timestamps(self):
        """returns the image timestamps in a sorted manner

        :return: timestamps_microsec, list containing the sorted timestamps
        """

        files = os.listdir(self.images_dir + 'Cam0')

        timestamps_microsec = sorted([long(os.path.splitext(f)[0]) for f in files if f.endswith('.tiff')])

        return timestamps_microsec

    def write_images(self, utime, cam_file):
        """converts the images into a ros image message

        :return: timestamp: ros time object
                 image_msg: ROS image msg
        """

        timestamp = rospy.Time.from_sec(utime / 1e6)

        cv_img = cv2.imread(cam_file)
        cv_img = cv2.rotate(cv_img, rotateCode=0)  # 90 deg
        img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        img_msg.header.frame_id = self.json_configs['frame_ids']['ladybug_sensor']

        return timestamp, img_msg

    def read_image(self, cam_nr=2):
        """reads images from a cam_nr directory

        :param cam_nr: number between 0-5
        """

        files = sorted(os.listdir(self.images_dir + 'Cam' + str(cam_nr)))
        os.chdir(self.images_dir + 'Cam2')
        for f in files:
            im = pilImage.open(f)
            im_array = np.array(im)
            im_array = np.rot90(im_array, -1)
            im = pilImage.fromarray(im_array)
            im.show()
            raw_input("Please press enter...")

    def images_to_bag(self):

        timestamps = self.get_image_timestamps()

        for (i, utime) in enumerate(timestamps):

            for camera_id in range(self.num_cameras):
                cam_file = os.path.join(self.images_dir, 'Cam' + str(camera_id), str(utime) + '.tiff')
                cv_img = cv2.imread(cam_file)
                cv_img = cv2.rotate(cv_img, rotateCode=0)  # 90 deg
                img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                rostime = rospy.Time.from_sec(utime / 1e6)

                self.bag.write(self.json_configs['topics']['ladybug_sensor'] + str(camera_id), img_msg, t=rostime)