import rosbag
import os
import subprocess
import sys
import rospy

from src.nclt2ros import ReadGroundTruth
from src.nclt2ros import ReadGroundTruthCovariance
from src.nclt2ros import ReadSensorData
from src.nclt2ros.transformer.hokuyo_data import HokuyoData
from src.nclt2ros import VelodyneSyncData
from src.nclt2ros.transformer.image_data import ImageData
from src.nclt2ros.extractor.base_raw_data import BaseRawData
from src.nclt2ros import RosSensorMsg
from src.nclt2ros.converter import BaseConvert


class ToRosbag(BaseRawData, BaseConvert):
    """Class to convert the NCLT Dataset into a rosbag file

    USAGE:
            ToRosbag('2013-01-10', 'example.bag', cam_folder=None)

    """
    def __init__(self, date, bag_name, cam_folder, gt, sen, hokuyo, vel, lb3):

        if isinstance(date, str):
            self.date = date
        else:
            raise TypeError('"date" must be of type string')

        self.gt = gt
        self.sen = sen
        self.hokuyo = hokuyo
        self.vel = vel
        self.lb3 = lb3

        # init base classes
        BaseRawData.__init__(self, date=self.date)
        BaseConvert.__init__(self, date=self.date)

        # load class instances, if available
        if self.ground_truth_flag and self.gt:
            self.gt = ReadGroundTruth(self.date)
            if self.ground_truth_covariance_flag:
                self.gt_cov = ReadGroundTruthCovariance(self.date)
            else:
                raise TypeError('No ground_truth_covariance directory available, please check!')

        if self.sensor_data_flag and self.sen:
            self.raw_data = ReadSensorData(self.date)

        if self.hokuyo_data_flag and self.hokuyo:
            self.hokuyo_data = HokuyoData(self.date)

        if self.velodyne_data_flag and self.vel:
            self.velodyne_sync_data = VelodyneSyncData(self.date)

        if self.images_flag and self.lb3:

            # create instance
            self.image_data = ImageData(self.date)

            # check cam_folder argument
            if cam_folder is None:
                self.cam_folder = 5
            else:
                self.cam_folder = cam_folder
        else:
            self.cam_folder = None

        # load ros msg converter class
        self.ros_sensor_msg = RosSensorMsg(self.date)

        # create rosbag file and check name
        self.bag_name = str(bag_name)

        if not self.bag_name.endswith('.bag'):
            self.bag_name = self.bag_name + '.bag'

        os.chdir(self.rosbag_dir)
        # check if file already exists
        if os.path.isfile(self.rosbag_dir + '/' + self.bag_name):
            rospy.loginfo("rosbag file %s already exists" % self.bag_name)
            self.bag_name = self.bag_name.split('.')[0] + '_2' + '.bag'

        # create rosbag file
        self.bag = rosbag.Bag(self.bag_name, 'w')

    def __del__(self):
        """destructor
        """
        rospy.loginfo("close rosbag file")
        self.bag.close()

    def process(self):
        """loads and converts the data into a rosbag file

        """
        # init counter with 0
        i_gt      = 0
        i_gps     = 0
        i_gps_rtk = 0
        i_ms25    = 0
        i_odom    = 0
        i_vel     = 0
        i_img     = 0

        rospy.loginfo("loading data ...")

        # load ground_truth data
        if self.gt:
            gt_list       = self.gt.read_gt_csv(all_in_one=True)
            gt_cov_list   = self.gt_cov.read_gt_cov_csv(all_in_one=True)

        # load sensor data
        if self.sen:
            gps_list      = self.raw_data.read_gps_csv(all_in_one=True)
            gps_rtk_list  = self.raw_data.read_gps_rtk_csv(all_in_one=True)
            ms25_list     = self.raw_data.read_ms25_csv(all_in_one=True)
            odom_list     = self.raw_data.read_odometry_mu_100hz_csv(all_in_one=True)
            odom_cov_list = self.raw_data.read_odometry_cov_100hz_csv(all_in_one=True)
            wheels_list   = self.raw_data.read_wheels_csv(all_in_one=True)
            kvh_list      = self.raw_data.read_kvh_csv(all_in_one=True)

        # load hokuyo data
        if self.hokuyo:
            utime_hok4, data_hok4   = self.hokuyo_data.read_next_hokuyo_4m_packet()
            utime_hok30, data_hok30 = self.hokuyo_data.read_next_hokuyo_30m_packet()

        # load velodyne sync data
        if self.vel:
            vel_sync_timestamps_microsec, vel_sync_bin_files = self.velodyne_sync_data.get_velodyne_sync_timestamps_and_files()

        # load image data
        if self.cam_folder is not None and self.lb3:
            images_timestamps_microsec = self.image_data.get_image_timestamps()

        rospy.loginfo("data loaded, writing to rosbag file %s" % self.bag_name)

        max_num_messages = 1e20
        num_messages = 0

        while not rospy.is_shutdown():
            next_packet = "done"
            next_utime = -1

            if self.gt:
                if i_gt < len(gt_list) and (gt_list[i_gt, 0] < next_utime or next_utime < 0):
                    next_utime = gt_list[i_gt, 0]
                    next_packet = "gt"

            if self.sen:
                if i_gps < len(gps_list) and (gps_list[i_gps, 0] < next_utime or next_utime < 0):
                    next_utime = gps_list[i_gps, 0]
                    next_packet = "gps"

                if i_gps_rtk < len(gps_rtk_list) and (gps_rtk_list[i_gps_rtk, 0] < next_utime or next_utime < 0):
                    next_utime = gps_rtk_list[i_gps_rtk, 0]
                    next_packet = "gps_rtk"

                if i_ms25 < len(ms25_list) and (ms25_list[i_ms25, 0] < next_utime or next_utime < 0):
                    next_utime = ms25_list[i_ms25, 0]
                    next_packet = "ms25"

                if i_odom < len(odom_list) and (odom_list[i_odom, 0] < next_utime or next_utime < 0):
                    next_utime = odom_list[i_odom, 0]
                    next_packet = "odom"

            if self.hokuyo:
                if utime_hok4 > 0 and (utime_hok4 < next_utime or next_utime < 0):
                    next_packet = "hok4"

                if utime_hok30 > 0 and (utime_hok30 < next_utime or next_utime < 0):
                    next_packet = "hok30"

            if self.vel:
                if i_vel < len(vel_sync_timestamps_microsec) and (vel_sync_timestamps_microsec[i_vel] < next_utime or next_utime < 0):
                    next_utime = vel_sync_timestamps_microsec[i_vel]
                    next_packet = "vel_sync"

            if self.cam_folder is not None and self.lb3:
                if i_img < len(images_timestamps_microsec) and (images_timestamps_microsec[i_img] < next_utime or next_utime < 0):
                    next_utime = images_timestamps_microsec[i_img]
                    next_packet = "img"

            if next_packet == "done":
                break

            elif next_packet == "gps":
                # print("write gps")
                navsat, track, speed, timestamp, tf_static_msg = self.ros_sensor_msg.gps_to_navsat(gps_list=gps_list, i=i_gps)
                self.bag.write(self.gps_fix_topic, navsat, timestamp)
                self.bag.write(self.gps_track_topic, track, timestamp)
                self.bag.write(self.gps_speed_topic, speed, timestamp)
                self.bag.write('/tf_static', tf_static_msg, t=timestamp)
                i_gps += 1

            elif next_packet == "gps_rtk":
                # print("write gps_rtk")
                navsat, track, speed, timestamp, tf_static_msg = self.ros_sensor_msg.gps_rtk_to_navsat(gps_rtk_list=gps_rtk_list, i=i_gps_rtk)
                self.bag.write(self.gps_rtk_fix_topic, navsat, t=timestamp)
                self.bag.write(self.gps_rtk_track_topic, track, t=timestamp)
                self.bag.write(self.gps_rtk_speed_topic, speed, t=timestamp)
                self.bag.write('/tf_static', tf_static_msg, t=timestamp)
                i_gps_rtk += 1

            elif next_packet == "ms25":
                # print("write ms25")
                imu, mag, timestamp, tf_static_msg = self.ros_sensor_msg.ms25_to_imu(imu_list=ms25_list, i=i_ms25)
                self.bag.write(self.imu_data_topic, imu, t=timestamp)
                self.bag.write(self.imu_mag_topic, mag, t=timestamp)
                self.bag.write('/tf_static', tf_static_msg, t=timestamp)
                i_ms25 += 1

            elif next_packet == "odom":
                # print("write odom")
                odom, timestamp, tf_msg, tf_static_msg = self.ros_sensor_msg.wheel_odom_to_odometry(odom_list=odom_list, odom_cov_list=odom_cov_list, wheels_list=wheels_list, kvh_list=kvh_list, i=i_odom)
                self.bag.write(self.odom_topic, odom, t=timestamp)
                self.bag.write('/tf', tf_msg, t=timestamp)
                self.bag.write('/tf_static', tf_static_msg, t=timestamp)
                i_odom += 1

            elif next_packet == "gt":
                # print("write gt")
                gt, timestamp, tf_static_msg = self.ros_sensor_msg.gt_to_odometry(gt_list=gt_list, gt_cov_list=gt_cov_list, i=i_gt)
                self.bag.write(self.ground_truth_topic, gt, t=timestamp)
                self.bag.write('/tf_static', tf_static_msg, t=timestamp)
                i_gt += 1

            elif next_packet == "hok4":
                # print("write hok4")
                timestamp, scan, tf_static_msg = self.hokuyo_data.write_hokuyo_4m_to_laserscan(utime_hok4, data_hok4)
                self.bag.write(self.hokuyo_urg_topic, scan, t=timestamp)
                self.bag.write('/tf_static', tf_static_msg, t=timestamp)
                utime_hok4, data_hok4 = self.hokuyo_data.read_next_hokuyo_4m_packet()

            elif next_packet == "hok30":
                # print("write hok30")
                timestamp, scan, tf_static_msg = self.hokuyo_data.write_hokuyo_30m_to_laserscan(utime_hok30, data_hok30)
                self.bag.write(self.hokuyo_utm_topic, scan, t=timestamp)
                self.bag.write('/tf_static', tf_static_msg, t=timestamp)
                utime_hok30, data_hok30 = self.hokuyo_data.read_next_hokuyo_30m_packet()

            elif next_packet == "vel_sync":
                # print("vel_sync")
                try:
                    hits = self.velodyne_sync_data.read_next_velodyne_sync_packet(vel_sync_bin_files[i_vel])
                except ValueError:
                    rospy.logerr("error in sync velodyne packet")

                timestamp, pc2_msg, tf_static_msg = self.velodyne_sync_data.xyzil_array_to_pointcloud2(utime=vel_sync_timestamps_microsec[i_vel], hits=hits)
                self.bag.write(self.velodyne_topic, pc2_msg, t=timestamp)
                self.bag.write('/tf_static', tf_static_msg, t=timestamp)
                i_vel += 1

            elif next_packet == "img":
                if self.cam_folder is 'all':
                    for camera_id in range(self.num_cameras):
                        cam_file = os.path.join(self.images_lb3_dir, 'Cam' + str(camera_id), str(next_utime) + '.tiff')
                        timestamp, image_msg, tf_static_msg = self.image_data.write_images(utime=next_utime, cam_file=cam_file)
                        self.bag.write(self.ladybug_topic + str(camera_id), image_msg, t=timestamp)
                        self.bag.write('/tf_static', tf_static_msg, t=timestamp)
                else:
                    cam_file = os.path.join(self.images_lb3_dir, 'Cam' + str(self.cam_folder), str(next_utime) + '.tiff')
                    timestamp, image_msg, tf_static_msg = self.image_data.write_images(utime=next_utime, cam_file=cam_file)
                    self.bag.write(self.ladybug_topic + str(self.cam_folder), image_msg, t=timestamp)
                    self.bag.write('/tf_static', tf_static_msg, t=timestamp)
                i_img += 1
            else:
                rospy.logerr("unknown packet type")

            num_messages += 1
            if (num_messages % 5000) == 0:
                rospy.loginfo("number messages written: %d" % num_messages)

            if num_messages >= max_num_messages:
                break

        if not rospy.is_shutdown():
            self.bag.close()
            rospy.loginfo("successfully finished converting!")

        #self.compress_bag()

    def compress_bag(self):

        # TODO compress bag if file not to big
        files = os.listdir(self.rosbag_dir)
        try:

            if self.bag_name in files:

                os.chdir(self.rosbag_dir)
                reindex_cmd = "rosbag reindex " + self.bag_name
                compress_cmd = "rosbag compress --lz4 " + self.bag_name

                #reindex_proc = subprocess.Popen(reindex_cmd, stdin=subprocess.PIPE, shell=True, executable='/bin/bash')
                #output, error = reindex_proc.communicate()
                #if error:
                #    print("error occured while running rosbag reindex")
                #    sys.exit(0)

                #reindex_proc.wait()

                compress_proc = subprocess.Popen(compress_cmd, stdin=subprocess.PIPE, shell=True, executable='/bin/bash')

                output, error = compress_proc.communicate()

                if error:
                    print("error occured while running rosbag compress")
                    sys.exit(0)

                compress_proc.wait()

                # remove orig file
                orig_bag_str = self.bag_name[:-4] + '.orig.bag'
                all_files = os.listdir(self.rosbag_dir)
                for f in all_files:
                    if f == orig_bag_str:
                        os.remove(f)

        except Exception as e:
            print(e)
