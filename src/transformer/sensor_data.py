import rospy
import numpy as np
import tf.transformations
import tf2_msgs.msg
import tf2_ros
import geometry_msgs.msg

from sensor_msgs.msg import NavSatStatus, NavSatFix, Imu, MagneticField
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16, Float64
from src.transformer.data import Data

# default COVARIANCE matrices
IMU_ORIENT_COVAR = [1e-3, 0, 0,
                    0, 1e-3, 0,
                    0, 0, 1e-3]

IMU_VEL_COVAR    = [1e-3, 0, 0,
                    0, 1e-3, 0,
                    0, 0, 1e-3]

IMU_ACCEL_COVAR  = [1e-3, 0, 0,
                    0, 1e-3, 0,
                    0, 0, 1e-3]

NAVSAT_COVAR     = [1, 0, 0,
                    0, 1, 0,
                    0, 0, 1]

POSE_COVAR       = [1e-3, 0, 0, 0, 0, 0,
                    0, 1e-3, 0, 0, 0, 0,
                    0, 0, 1e-3, 0, 0, 0,
                    0, 0, 0, 1e-3, 0, 0,
                    0, 0, 0, 0, 1e-3, 0,
                    0, 0, 0, 0, 0, 1e-3]


class SensorData(Data):
    """Class to convert the sensor_data directory to ROS messages

    USAGE:
            SensorData('2013-01-10')

    """
    def __init__(self, date):

        # init base class
        Data.__init__(self, date=date)

        self.tf_broadcast = tf.TransformBroadcaster()
        self.static_transform = tf2_ros.StaticTransformBroadcaster()

        self.i_wheel = 0
        self.last_twist = None
        self.init_twist = False

        self.i_kvh = 0
        self.last_twist_heading = None
        self.init_twist_heading = False

        self.i_gt = 0
        self.last_gt_cov = None
        self.init_gt_cov = False

    def gps_to_navsat(self, gps_list, i):
        """ converts gps data to ROS NavSatFix messages

        :param gps_list: list, containing gps data
        :param        i: row counter

        :return: fill bag with navsat, track, speed, timestamp
        """

        # load data from list
        utime     = gps_list[i, 0]
        mode      = gps_list[i, 1]
        num_satss = gps_list[i, 2]
        lat       = gps_list[i, 3]
        lon       = gps_list[i, 4]
        alt       = gps_list[i, 5]
        track_raw = gps_list[i, 6]
        speed_raw = gps_list[i, 7]

        # create ros timestamp
        timestamp = rospy.Time.from_sec(utime / 1e6)

        # get gps and base link
        gps_link = self.json_configs['frame_ids']['gps_sensor']
        base_link = self.json_configs['frame_ids']['body']

        # fill NavSat message
        status = NavSatStatus()

        if mode == 0 or mode == 1:
            status.status = NavSatStatus.STATUS_NO_FIX
        else:
            status.status = NavSatStatus.STATUS_FIX

        status.service = NavSatStatus.SERVICE_GPS

        num_sats = UInt16()
        num_sats.data = num_satss

        navsat = NavSatFix()
        navsat.header.stamp = timestamp
        navsat.header.frame_id = gps_link
        navsat.status = status

        navsat.latitude = np.rad2deg(lat)
        navsat.longitude = np.rad2deg(lon)
        navsat.altitude = alt

        navsat.position_covariance = NAVSAT_COVAR
        navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        track = Float64()
        track.data = track_raw

        speed = Float64()
        speed.data = speed_raw

        # create base_link gps_link static transformer
        gps_static_transform_stamped = geometry_msgs.msg.TransformStamped()
        gps_static_transform_stamped.header.stamp = timestamp
        gps_static_transform_stamped.header.frame_id = base_link
        gps_static_transform_stamped.child_frame_id = gps_link

        gps_static_transform_stamped.transform.translation.x = 0
        gps_static_transform_stamped.transform.translation.y = 0.25
        gps_static_transform_stamped.transform.translation.z = 0.51

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        gps_static_transform_stamped.transform.rotation.x = quat[0]
        gps_static_transform_stamped.transform.rotation.y = quat[1]
        gps_static_transform_stamped.transform.rotation.z = quat[2]
        gps_static_transform_stamped.transform.rotation.w = quat[3]

        # publish static transform
        tf_static_msg = tf2_msgs.msg.TFMessage([gps_static_transform_stamped])

        return navsat, track, speed, timestamp, tf_static_msg

    def gps_rtk_to_navsat(self, gps_rtk_list, i):
        """ converts gps_rtk data to ROS NavSatFix messages

        :param gps_rtk_list: list, containing gps_rtk data
        :param            i: row counter

        :return: fill bag with navsat, track, speed, timestamp
        """

        # load data from list
        utime     = gps_rtk_list[i, 0]
        mode      = gps_rtk_list[i, 1]
        num_satss = gps_rtk_list[i, 2]
        lat       = gps_rtk_list[i, 3]
        lon       = gps_rtk_list[i, 4]
        alt       = gps_rtk_list[i, 5]
        track_raw = gps_rtk_list[i, 6]
        speed_raw = gps_rtk_list[i, 7]

        # create ros timestamp
        timestamp = rospy.Time.from_sec(utime / 1e6)

        # get gps_rtk and base link
        gps_rtk_link = self.json_configs['frame_ids']['gps_rtk_sensor']
        base_link    = self.json_configs['frame_ids']['body']

        # fill NavSat message
        status = NavSatStatus()

        if mode == 0 or mode == 1:
            status.status = NavSatStatus.STATUS_NO_FIX
        else:
            status.status = NavSatStatus.STATUS_FIX

        status.service = NavSatStatus.SERVICE_GPS

        num_sats = UInt16()
        num_sats.data = num_satss

        navsat = NavSatFix()
        navsat.header.stamp = timestamp
        navsat.header.frame_id = gps_rtk_link
        navsat.status = status

        navsat.latitude = np.rad2deg(lat)
        navsat.longitude = np.rad2deg(lon)
        navsat.altitude = alt

        navsat.position_covariance = NAVSAT_COVAR
        navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        track = Float64()
        track.data = track_raw

        speed = Float64()
        speed.data = speed_raw

        # create base_link gps_rtk_link static transformer
        gps_rtk_static_transform_stamped = geometry_msgs.msg.TransformStamped()
        gps_rtk_static_transform_stamped.header.stamp = timestamp
        gps_rtk_static_transform_stamped.header.frame_id = base_link
        gps_rtk_static_transform_stamped.child_frame_id = gps_rtk_link

        gps_rtk_static_transform_stamped.transform.translation.x = -0.24
        gps_rtk_static_transform_stamped.transform.translation.y = 0
        gps_rtk_static_transform_stamped.transform.translation.z = 1.24

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        gps_rtk_static_transform_stamped.transform.rotation.x = quat[0]
        gps_rtk_static_transform_stamped.transform.rotation.y = quat[1]
        gps_rtk_static_transform_stamped.transform.rotation.z = quat[2]
        gps_rtk_static_transform_stamped.transform.rotation.w = quat[3]

        # publish static transform
        tf_static_msg = tf2_msgs.msg.TFMessage([gps_rtk_static_transform_stamped])

        return navsat, track, speed, timestamp, tf_static_msg

    def wheel_odom_to_odometry(self, odom_list, odom_cov_list, wheels_list, kvh_list, i):
        """converts recorded wheel odom to ROS Odometry messages

        :param     odom_list: list containing the wheel encoder data
        :param odom_cov_list: list containing the covariance matrices
        :param   wheels_list: list containing the velocity from left and right wheel
        :param      kvh_list: list containing the heading from the KVH
        :param             i: row counter

        :return: fill bag with odom, timestamp, tf_msg
        """

        # get 6 DoF odometry
        utime = odom_list[i, 0]
        x     = odom_list[i, 1]
        y     = odom_list[i, 2]
        z     = odom_list[i, 3]
        r     = odom_list[i, 4]
        p     = odom_list[i, 5]
        h     = odom_list[i, 6]

        # get upper diagonal of the covariance matrix
        xx = odom_cov_list[i, 1];  xy = odom_cov_list[i, 2];  xz = odom_cov_list[i, 3];  xr = odom_cov_list[i, 4]
        xp = odom_cov_list[i, 5];  xh = odom_cov_list[i, 6];  yy = odom_cov_list[i, 7];  yz = odom_cov_list[i, 8]
        yr = odom_cov_list[i, 9];  yp = odom_cov_list[i, 10]; yh = odom_cov_list[i, 11]; zz = odom_cov_list[i, 12]
        zr = odom_cov_list[i, 13]; zp = odom_cov_list[i, 14]; zh = odom_cov_list[i, 15]; rr = odom_cov_list[i, 16]
        rp = odom_cov_list[i, 17]; rh = odom_cov_list[i, 18]; pp = odom_cov_list[i, 19]; ph = odom_cov_list[i, 20]
        hh = odom_cov_list[i, 21]

        # get wheel velocity
        utime_wheel = wheels_list[self.i_wheel, 0]
        vl          = wheels_list[self.i_wheel, 1]
        vr          = wheels_list[self.i_wheel, 2]

        # get heading from kvh
        utime_kvh   = kvh_list[self.i_kvh, 0]
        kvh_heading = kvh_list[self.i_kvh, 1]

        # create ros timestamp
        timestamp = rospy.Time.from_sec(utime / 1e6)

        # get wheel odometry and body link
        wheel_odom_link = str(self.json_configs['frame_ids']['wheel_odometry'])
        base_link       = str(self.json_configs['frame_ids']['body'])

        # create ros odometry message
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = wheel_odom_link
        odom.child_frame_id  = base_link

        odom.pose.pose.position.x = x                                                                       # x
        odom.pose.pose.position.y = -y  # change recorded coordinate system to ros coordinate system *(-1)  # -y
        odom.pose.pose.position.z = -z  # change recorded coordinate system to ros coordinate system *(-1)  x -z

        # create quaternion from euler angles
        quaternion = tf.transformations.quaternion_from_euler(r, -p, -h)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        # fill covariance matrix
        odom.pose.covariance[0] = xx; odom.pose.covariance[1] = xy; odom.pose.covariance[2] = xz; odom.pose.covariance[3] = xr
        odom.pose.covariance[4] = xp; odom.pose.covariance[5] = xh

        odom.pose.covariance[7] = yy; odom.pose.covariance[8] = yz; odom.pose.covariance[9] = yr; odom.pose.covariance[10] = yp
        odom.pose.covariance[11] = yh

        odom.pose.covariance[14] = zz; odom.pose.covariance[15] = zr; odom.pose.covariance[16] = zp; odom.pose.covariance[17] = zh

        odom.pose.covariance[21] = rr; odom.pose.covariance[22] = rp; odom.pose.covariance[23] = rh

        odom.pose.covariance[28] = pp; odom.pose.covariance[29] = ph

        odom.pose.covariance[35] = hh

        if utime > utime_wheel:
            speed = (vl + vr) / 2.0

            odom.twist.twist.linear.x = speed

            self.last_twist = odom.twist.twist.linear.x
            self.i_wheel += 1

        elif self.init_twist:
            odom.twist.twist.linear.x = self.last_twist

        else:
            odom.twist.twist.linear.x = 0.0

            self.init_twist = True
            self.last_twist = odom.twist.twist.linear.x

        if utime > utime_kvh:
            odom.twist.twist.angular.z = kvh_heading
            self.last_twist_heading = odom.twist.twist.angular.z
            self.i_kvh += 1
        elif self.init_twist_heading:
            odom.twist.twist.angular.z = self.last_twist_heading

        else:
            odom.twist.twist.angular.z = 0.0
            self.init_twist_heading = True
            self.last_twist_heading = odom.twist.twist.angular.z

        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.linear.z  = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0

        # just an example
        odom.twist.covariance[0]  = 0.001
        odom.twist.covariance[7]  = 0.001
        odom.twist.covariance[14] = 99999.9
        odom.twist.covariance[21] = 99999.9
        odom.twist.covariance[28] = 99999.9
        odom.twist.covariance[35] = 0.001

        # broadcast odom base_link transform
        geo_msg = geometry_msgs.msg.TransformStamped()
        geo_msg.header.stamp = timestamp
        geo_msg.header.frame_id = wheel_odom_link
        geo_msg.child_frame_id  = base_link
        geo_msg.transform.translation.x = x
        geo_msg.transform.translation.y = -y
        geo_msg.transform.translation.z = -z
        geo_msg.transform.rotation.x = quaternion[0]
        geo_msg.transform.rotation.y = quaternion[1]
        geo_msg.transform.rotation.z = quaternion[2]
        geo_msg.transform.rotation.w = quaternion[3]

        tf_msg = tf2_msgs.msg.TFMessage([geo_msg])

        # create world odom static transformer
        odom_static_transform_stamped = geometry_msgs.msg.TransformStamped()
        odom_static_transform_stamped.header.stamp = timestamp
        odom_static_transform_stamped.header.frame_id = "world"
        odom_static_transform_stamped.child_frame_id = wheel_odom_link

        odom_static_transform_stamped.transform.translation.x = 0
        odom_static_transform_stamped.transform.translation.y = 0
        odom_static_transform_stamped.transform.translation.z = 0

        quat = tf.transformations.quaternion_from_euler(0, 0, 1.57)
        odom_static_transform_stamped.transform.rotation.x = quat[0]
        odom_static_transform_stamped.transform.rotation.y = quat[1]
        odom_static_transform_stamped.transform.rotation.z = quat[2]
        odom_static_transform_stamped.transform.rotation.w = quat[3]

        # publish static transform
        tf_static_msg = tf2_msgs.msg.TFMessage([odom_static_transform_stamped])

        return odom, timestamp, tf_msg, tf_static_msg

    def ms25_to_imu(self, imu_list, i):
        """converts ms25 data to ROS imu messages

        :param imu_list: list containing the imu data from sensor ms25
        :param        i: list row counter

        :return: fill bag with imu, mag, timestamp
        """

        # load data from list
        utime    = imu_list[i, 0]
        mag_xs   = imu_list[i, 1]
        mag_ys   = imu_list[i, 2]
        mag_zs   = imu_list[i, 3]
        accel_xs = imu_list[i, 4]
        accel_ys = imu_list[i, 5]
        accel_zs = imu_list[i, 6]
        rot_rs   = imu_list[i, 7]
        rot_ps   = imu_list[i, 8]
        rot_hs   = imu_list[i, 9]

        # create ros timestamp
        timestamp = rospy.Time.from_sec(utime / 1e6)

        # get imu and base link
        imu_link   = self.json_configs['frame_ids']['imu_sensor']
        base_link  = self.json_configs['frame_ids']['body']

        # create ros imu message
        imu = Imu()
        imu.header.stamp = timestamp
        imu.header.frame_id = imu_link

        # swap x,y due to the enu frame, negate z
        quaternion = tf.transformations.quaternion_from_euler(
            rot_ps, rot_rs, -rot_hs
        )

        # alternative
        #quaternion = tf.transformations.quaternion_from_euler(
        #    rot_rs, -rot_ps, -rot_hs
        #)

        imu.orientation.x = quaternion[0]
        imu.orientation.y = quaternion[1]
        imu.orientation.z = quaternion[2]
        imu.orientation.w = quaternion[3]

        # swap x,y due to the enu frame, negate z
        imu.angular_velocity.x = rot_ps  # rot_rs
        imu.angular_velocity.y = rot_rs  # -rot_ps
        imu.angular_velocity.z = -rot_hs
        imu.angular_velocity_covariance = IMU_VEL_COVAR

        # swap x,y due to the enu frame, negate z
        imu.linear_acceleration.x = accel_ys   # accel_xs
        imu.linear_acceleration.y = accel_xs   # -accel_ys
        imu.linear_acceleration.z = -accel_zs
        imu.linear_acceleration_covariance = IMU_ACCEL_COVAR

        # enu frame
        mag = MagneticField()
        mag.header.stamp     = timestamp
        mag.magnetic_field.x = mag_ys
        mag.magnetic_field.y = mag_xs
        mag.magnetic_field.z = -mag_zs

        # create base_link imu static transformer
        imu_static_transform_stamped = geometry_msgs.msg.TransformStamped()
        imu_static_transform_stamped.header.stamp = timestamp
        imu_static_transform_stamped.header.frame_id = base_link
        imu_static_transform_stamped.child_frame_id = imu_link

        imu_static_transform_stamped.transform.translation.x = -0.11
        imu_static_transform_stamped.transform.translation.y = 0.18
        imu_static_transform_stamped.transform.translation.z = 0.71

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        imu_static_transform_stamped.transform.rotation.x = quat[0]
        imu_static_transform_stamped.transform.rotation.y = quat[1]
        imu_static_transform_stamped.transform.rotation.z = quat[2]
        imu_static_transform_stamped.transform.rotation.w = quat[3]

        # publish static transform
        tf_static_msg = tf2_msgs.msg.TFMessage([imu_static_transform_stamped])

        return imu, mag, timestamp, tf_static_msg

    def gt_to_odometry(self, gt_list, gt_cov_list, i):
        """converts ground_truth odometry to ROS odometry messages

        :param gt_list: list containing the ground truth data
        :param       i: row counter

        :return: fill bag with gt, timestamp
        """

        # load data from list
        utime     = gt_list[i, 0]
        x         = gt_list[i, 1]
        y         = gt_list[i, 2]
        z         = gt_list[i, 3]
        roll_rad  = gt_list[i, 4]
        pitch_rad = gt_list[i, 5]
        yaw_rad   = gt_list[i, 6]

        # get upper diagonal of the covariance matrix
        xx = gt_cov_list[self.i_gt, 1];  xy = gt_cov_list[self.i_gt, 2];  xz = gt_cov_list[self.i_gt, 3];  xr = gt_cov_list[self.i_gt, 4]
        xp = gt_cov_list[self.i_gt, 5];  xh = gt_cov_list[self.i_gt, 6];  yy = gt_cov_list[self.i_gt, 7];  yz = gt_cov_list[self.i_gt, 8]
        yr = gt_cov_list[self.i_gt, 9];  yp = gt_cov_list[self.i_gt, 10]; yh = gt_cov_list[self.i_gt, 11]; zz = gt_cov_list[self.i_gt, 12]
        zr = gt_cov_list[self.i_gt, 13]; zp = gt_cov_list[self.i_gt, 14]; zh = gt_cov_list[self.i_gt, 15]; rr = gt_cov_list[self.i_gt, 16]
        rp = gt_cov_list[self.i_gt, 17]; rh = gt_cov_list[self.i_gt, 18]; pp = gt_cov_list[self.i_gt, 19]; ph = gt_cov_list[self.i_gt, 20]
        hh = gt_cov_list[self.i_gt, 21]

        # load utime from gt_cov list
        utime_gt_cov = gt_cov_list[self.i_gt, 0]

        # create ros timestamp
        timestamp = rospy.Time.from_sec(utime / 1e6)

        # get ground truth link
        gt_link = str(self.json_configs['frame_ids']['ground_truth'])

        # create odometry message for ground truth
        gt = Odometry()
        gt.header.stamp = timestamp
        gt.header.frame_id = gt_link
        # odom.child_frame_id = 'base_link'

        # change due to the ENU frame
        gt.pose.pose.position.x = y - 107.724666286     # x
        gt.pose.pose.position.y = x - 75.829339527800   # y
        gt.pose.pose.position.z = -z  # z

        # create quaternion from euler angles
        quaternion = tf.transformations.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        gt.pose.pose.orientation.x = quaternion[0]
        gt.pose.pose.orientation.y = quaternion[1]
        gt.pose.pose.orientation.z = quaternion[2]
        gt.pose.pose.orientation.w = quaternion[3]

        # fill covariance matrices
        if utime > utime_gt_cov:
            gt.pose.covariance[0] = xx;  gt.pose.covariance[1] = xy;  gt.pose.covariance[2] = xz;  gt.pose.covariance[3] = xr
            gt.pose.covariance[4] = xp;  gt.pose.covariance[5] = xh

            gt.pose.covariance[7] = yy;  gt.pose.covariance[8] = yz;  gt.pose.covariance[9] = yr;  gt.pose.covariance[10] = yp
            gt.pose.covariance[11] = yh

            gt.pose.covariance[14] = zz; gt.pose.covariance[15] = zr; gt.pose.covariance[16] = zp; gt.pose.covariance[17] = zh

            gt.pose.covariance[21] = rr; gt.pose.covariance[22] = rp; gt.pose.covariance[23] = rh

            gt.pose.covariance[28] = pp; gt.pose.covariance[29] = ph

            gt.pose.covariance[35] = hh

            self.last_gt_cov = gt.pose.covariance
            self.i_gt += 1

        elif self.init_gt_cov:
            gt.pose.covariance = self.last_gt_cov

        else:
            gt.pose.covariance = POSE_COVAR

            self.init_gt_cov = True
            self.last_gt_cov = gt.pose.covariance

        # TODO fill TWIST
        gt.twist.twist.linear.x = 0.0
        gt.twist.twist.linear.y = 0.0
        gt.twist.twist.linear.z = 0.0
        gt.twist.twist.angular.x = 0.0
        gt.twist.twist.angular.y = 0.0
        gt.twist.twist.angular.z = 0.0

        # create world ground truth static transformer
        gt_static_transform_stamped = geometry_msgs.msg.TransformStamped()
        gt_static_transform_stamped.header.stamp = timestamp
        gt_static_transform_stamped.header.frame_id = "world"
        gt_static_transform_stamped.child_frame_id = gt_link

        gt_static_transform_stamped.transform.translation.x = 0
        gt_static_transform_stamped.transform.translation.y = 0
        gt_static_transform_stamped.transform.translation.z = 0

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        gt_static_transform_stamped.transform.rotation.x = quat[0]
        gt_static_transform_stamped.transform.rotation.y = quat[1]
        gt_static_transform_stamped.transform.rotation.z = quat[2]
        gt_static_transform_stamped.transform.rotation.w = quat[3]

        # publish static transform
        tf_static_msg = tf2_msgs.msg.TFMessage([gt_static_transform_stamped])

        return gt, timestamp, tf_static_msg
