import os
import rospy
import numpy as np
import cv2
import cv_bridge
import geometry_msgs.msg
import tf2_msgs.msg
import tf.transformations

from PIL import Image as pilImage
from nclt2ros.extractor.base_raw_data import BaseRawData
from nclt2ros.converter.base_convert import BaseConvert


class ImageData(BaseRawData, BaseConvert):
    """Class to transform the images from the ladybug camera

    USAGE:
            ImageData('2013-01-10')

    """
    def __init__(self, date):

        # init base class
        BaseRawData.__init__(self, date=date)
        BaseConvert.__init__(self, date=date)

        # create a cv bridge
        self.bridge = cv_bridge.CvBridge()

    def get_image_timestamps(self):
        """returns the image timestamps in a sorted manner

        :return: timestamps_microsec, list containing the sorted timestamps
        """
        # same timestamps in all folders
        files = os.listdir(self.images_lb3_dir + 'Cam0')

        timestamps_microsec = sorted([long(os.path.splitext(f)[0]) for f in files if f.endswith('.tiff')])

        return timestamps_microsec

    def write_images(self, utime, cam_file):
        """converts the images into a ros image message

        :return: timestamp: ros time object
                 image_msg: ROS image msg
                 tf_static_msg: static transformation
        """

        # create ros timestamp
        timestamp = rospy.Time.from_sec(utime / 1e6)

        # get image and base link
        camera_link = self.ladybug_frame
        base_link = self.body_frame

        cv_img = cv2.imread(cam_file)
        cv_img = cv2.rotate(cv_img, rotateCode=0)  # 90 deg
        img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")

        img_msg.header.frame_id = camera_link

        # create base_link camera_link static transformer
        img_static_transform_stamped = geometry_msgs.msg.TransformStamped()
        img_static_transform_stamped.header.stamp = timestamp
        img_static_transform_stamped.header.frame_id = base_link
        img_static_transform_stamped.child_frame_id = camera_link

        img_static_transform_stamped.transform.translation.x = 0.035
        img_static_transform_stamped.transform.translation.y = -0.002
        img_static_transform_stamped.transform.translation.z = 1.23

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        img_static_transform_stamped.transform.rotation.x = quat[0]
        img_static_transform_stamped.transform.rotation.y = quat[1]
        img_static_transform_stamped.transform.rotation.z = quat[2]
        img_static_transform_stamped.transform.rotation.w = quat[3]

        # publish static transform
        tf_static_msg = tf2_msgs.msg.TFMessage([img_static_transform_stamped])

        return timestamp, img_msg, tf_static_msg

    def read_image(self, cam_nr=2):
        """reads images from a cam_nr directory

        :param cam_nr: number between 0-5
        """

        files = sorted(os.listdir(self.images_dir + 'Cam' + str(cam_nr)))
        os.chdir(self.images_dir + 'Cam' + str(cam_nr))
        for f in files:
            im = pilImage.open(f)
            im_array = np.array(im)
            im_array = np.rot90(im_array, -1)
            im = pilImage.fromarray(im_array)
            im.show()
            raw_input("Please press enter...")
