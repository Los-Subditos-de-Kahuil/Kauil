#!/usr/bin/env python
"""
Use EKF to estimate the state of the robot using map based localization.

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/05/25
"""
import rospy
import tf
import numpy as np
import tf2_ros
import json
import os
import sys
import tf

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Point,
    Quaternion,
    TransformStamped,
    PoseStamped,
)
from fiducial_msgs.msg import FiducialTransformArray
from visualization_msgs.msg import MarkerArray, Marker

# * Robot parameters
R, L = 0.160, 0.51  # Wheel radius and distance between wheels

# * Covariance constants
KL = 50  # Covariance constant of the left wheel
KR = 50  # Covariance constant of the right wheel
SIGMA0 = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])  # Initial covariance matrix

# * Frequency and period
FS = 50
T = 1.0 / FS

# * Map
MAP_PATH = os.path.join(sys.path[0], "./../resources/aruco_coords.json")


def get_camera_robot_transformation(th=np.pi / 2):
    """Get the transformation between the camera and the robot.

    Args:
        th (float, optional): Angle by which to rotate the transformation. Defaults to np.pi/2.

    Returns:
        np.NDArray: Transformation matrix.
    """
    transform = np.eye(4)
    rotation_matrix_z = np.array(
        [[np.cos(-th), -np.sin(-th), 0], [np.sin(-th), np.cos(-th), 0], [0, 0, 1]],
        dtype=float,
    )
    rotation_matrix_y = np.array(
        [[np.cos(th), 0, np.sin(th)], [0, 1, 0], [-np.sin(th), 0, np.cos(th)]],
        dtype=float,
    )
    full_rotation_matrix = np.matmul(rotation_matrix_y, rotation_matrix_z)
    transform[:3, :3] = full_rotation_matrix

    return transform


class KalmanFilter:
    """
    # Kalman Filter
    This node implements the Extended Kalman Filter algorithm to estimate the state of the robot using map based localization.

    ## Attributes
    - rate (`rospy.Rate`): Rate of the node.
    - fiducial_poses (`FiducialTransformArray`): Fiducial poses.
    - wl (`float`): Angular velocity of the left wheel.
    - wr (`float`): Angular velocity of the right wheel.
    - camera_robot_transformation (`np.NDArray`): Transformation between the camera and the robot.
    - Rk (`np.NDArray`): Covariance matrix of the measurement.
    - pOD (`rospy.Publisher`): Publisher of the odometry.
    - kalman_publisher (`rospy.Publisher`): Publisher of the kalman pose.
    - markers_publisher (`rospy.Publisher`): Publisher of the seen ArUco markers.
    - br (`tf2_ros.TransformBroadcaster`): Transform broadcaster.
    - brStatic (`tf2_ros.StaticTransformBroadcaster`): Static transform broadcaster.
    - map (`dict`): Map of the environment.

    ## Methods
    - __init__(): Initializes the node, broadcasters, publishers and subscribers, and sets the initial conditions.
    - callback_wl(`msg`): Callback of the topic /wl. It updates the angular velocity of the left wheel.
    - callback_wr(`msg`): Callback of the topic /wr. It updates the angular velocity of the right wheel.
    - callback_fiducial(`msg`): Callback of the topic /fiducial_transforms. It updates the fiducial poses.
    - send_odom_frame_fixed(`x`, `y`): Broadcasts the odometry frame.
    - calculate_covariance(`q`, `v`, `sigma`): Calculates the covariance matrix of the robot from encoders.
    - covariance_3_to_6(`sigma`): Converts a 3x3 covariance matrix to a 6x6 covariance matrix.
    - initialize_odometry(`cTime`, `x`, `y`, `qRota`, `v`, `w`, `covariance`): Initializes the odometry message.
    - initialize_pose(`cTime`, `odom`): Initializes the pose message.
    - create_dp(`aruco_pose_map`, `robot_pose_map`): Creates the dp vector, containing [[dx], [dy], [p]].
    - get_Gk(`dp`): Calculates the jacobian G.
    - get_z(`dp`, `aruco_r`): Calculates the measurement vector, containing [[p], [theta]].
    - run(): Runs the node.

    ## Publishers
    - /odom: Odometry of the robot.
    - /kalman_pose: Pose of the robot.
    - /aruco_markers: ArUco markers seen by the camera.

    ## Subscribers
    - /wl: Angular velocity of the left wheel.
    - /wr: Angular velocity of the right wheel.
    - /fiducial_transforms: Fiducial poses.

    ## Broadcasters
    - /odom: Fixed odometry frame.
    - /kalman/base_link: Base link frame.
    """
    def __init__(self):
        """Initializes the node, broadcasters, publishers and subscribers, and sets the initial conditions."""
        # * Initialize node
        rospy.init_node("EKF")
        self.rate = rospy.Rate(FS)

        # * Initial values
        self.fiducaial_poses = None
        self.wl, self.wr = 0.0, 0.0
        self.camera_robot_transformation = get_camera_robot_transformation()
        self.Rk = np.eye(2) * 0.2

        # * Publishers
        self.pOD = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.kalman_publisher = rospy.Publisher(
            "/kalman_pose", PoseWithCovarianceStamped, queue_size=10
        )
        self.markers_publisher = rospy.Publisher(
            "/aruco_markers", MarkerArray, queue_size=10
        )

        # * Transform broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        self.brStatic = tf2_ros.StaticTransformBroadcaster()

        # * Subscribers
        rospy.Subscriber("/wl", Float32, self.callback_wl)
        rospy.Subscriber("/wr", Float32, self.callback_wr)
        rospy.Subscriber(
            "/fiducial_transforms",
            FiducialTransformArray,
            callback=self.callback_fiducial,
        )

        # * Load map
        with open(MAP_PATH) as f:
            self.map = json.load(f)

    def callback_wl(self, msg):
        """Callback of the topic /wl. It updates the angular velocity of the left wheel.

        Args:
            msg (Float32): Message of the topic /wl.
        """
        self.wl = msg.data

    def callback_wr(self, msg):
        """Callback of the topic /wr. It updates the angular velocity of the right wheel.

        Args:
            msg (Float32): Message of the topic /wr.
        """
        self.wr = msg.data

    def callback_fiducial(self, msg):
        """Callback of the topic /fiducial_transforms. It updates the fiducial poses.

        Args:
            msg (FiducialTransformArray): Message of the topic /fiducial_transforms.
        """
        self.fiducaial_poses = msg

    def send_odom_frame_fixed(self, x, y):
        """Send the fixed transform between the map and the odom frame.

        Args:
            x (float): x coordinate of the transform.
            y (float): y coordinate of the transform.
        """
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.brStatic.sendTransform(t)
        self.initial_state = PoseStamped()
        self.initial_state.header = t.header
        self.initial_state.pose.position = t.transform.translation
        self.initial_state.pose.orientation = t.transform.rotation

    def calculate_covariance(self, q, v, sigma):
        """Calculate the covariance matrix of the robot from encoders.

        Args:
            q (float): Angle of the robot.
            v (float): Linear velocity of the robot.

        Returns:
            np.NDArray: Covariance matrix of the robot from encoders.
        """
        nabla_w = (
            (1.0 / 2.0)
            * R
            * T
            * np.array(
                [
                    [np.cos(q), np.cos(q)],
                    [np.sin(q), np.sin(q)],
                    [2 / L, -2 / L],
                ]
            )
        )
        sigma_delta = np.array([[KR * abs(self.wr), 0], [0, KL * abs(self.wl)]])
        Q = np.matmul(nabla_w, np.matmul(sigma_delta, nabla_w.T))
        H = np.array(
            [
                [1, 0, -T * v * np.sin(q)],
                [0, 1, T * v * np.cos(q)],
                [0, 0, 1],
            ]
        )

        sigma = np.matmul(H, np.matmul(sigma, H.T)) + Q

        return sigma

    def covariance_3_to_6(self, sigma):
        """Convert a 3x3 covariance matrix to a 6x6 covariance matrix.

        Args:
            sigma (np.NDArray): 3x3 covariance matrix.

        Returns:
            np.NDArray: 6x6 covariance matrix.
        """
        covariance = np.zeros((6, 6))
        covariance[:2, :2] = sigma[:2, :2]
        covariance[5, 5] = sigma[2, 2]
        covariance[5, :2] = sigma[2, :2]
        covariance[:2, 5] = sigma[:2, 2]

        return covariance

    def initialize_odometry(self, cTime, x, y, qRota, v, w, covariance):
        """Initialize the odometry message.

        Args:
            cTime (rospy.Time): Time since the node started.
            x (float): x coordinate of the robot.
            y (float): y coordinate of the robot.
            qRota (ListLike): Quaternion of the robot.
            v (float): Linear velocity of the robot.
            w (float): Angular velocity of the robot.
            covariance (np.NDArray): 6x6 Covariance matrix of the robot.

        Returns:
            Odometry: Odometry message.
        """
        odom = Odometry()
        odom.header.stamp = cTime
        odom.header.frame_id = "odom"
        odom.child_frame_id = "odom/base_link"
        odom.pose.pose.position = Point(x, y, R)
        odom.pose.pose.orientation = Quaternion(qRota[0], qRota[1], qRota[2], qRota[3])
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        odom.pose.covariance = covariance.flatten().tolist()

        return odom

    def initialize_pose(self, cTime, odom):
        """Initialize the pose message.

        Args:
            cTime (rospy.Time): Time since the node started.
            odom (Odometry): Odometry message.

        Returns:
            PoseWithCovarianceStamped: Pose message.
        """
        pose_with_covariance_stamped = PoseWithCovarianceStamped()
        pose_with_covariance_stamped.header.stamp = cTime
        pose_with_covariance_stamped.header.frame_id = "map"
        pose_with_covariance_stamped.pose.pose.position = odom.pose.pose.position
        pose_with_covariance_stamped.pose.pose.orientation = odom.pose.pose.orientation
        pose_with_covariance_stamped.pose.covariance = odom.pose.covariance

        return pose_with_covariance_stamped

    def create_dp(self, aruco_pose_map, robot_pose_map):
        """Create the dp vector, containing [[dx], [dy], [p]], where p is the linear distance between the robot and the aruco marker. Used for calculating the jacobian G.

        Args:
            aruco_pose_map (np.NDArray): 3x1 Aruco marker pose in the map frame.
            robot_pose_map (np.NDArray): 3x1 Robot pose in the map frame.

        Returns:
            np.NDArray: 3x1 dp vector.
        """
        dp = aruco_pose_map - robot_pose_map
        dp[-1] = np.linalg.norm(dp[:2])

        return dp

    def get_Gk(self, dp):
        """Calculate the jacobian G.

        Args:
            dp (np.NDArray): 3x1 dp vector.

        Returns:
            np.NDArray: 2x3 Jacobian G.
        """
        Gk = np.array(
            [
                [(-dp[0][0] / dp[-1][0]), (-dp[1][0] / dp[-1][0]), 0],
                [(dp[1][0] / (dp[-1][0] ** 2)), (-dp[0][0] / (dp[-1][0] ** 2)), -1],
            ]
        )

        return Gk

    def get_z(self, dp, aruco_r):
        """Calculate the z vector, containing [[p], [theta]], where p is the linear distance between the robot and the aruco marker and theta is the angle between the robot and the aruco marker. Used for calculating the innovation.

        Args:
            dp (np.NDArray): 3x1 dp vector.
            aruco_r (np.NDArray): 3x1 Aruco marker pose in the robot frame.

        Returns:
            np.NDArray: 2x1 z vector.
        """
        return np.array(
            [[np.linalg.norm(dp[:2])], [np.arctan2(aruco_r[1], aruco_r[0])]]
        )

    def run(self):
        """Run the node. Estimate the pose of the robot based on the wheel velocities, then correct the pose based on all the aruco markers detected."""
        # * Initial conditions
        x, y, q = 0.0, 0.0, 0.0
        sigma = SIGMA0
        self.send_odom_frame_fixed(x, y)

        while not rospy.is_shutdown():
            cTime = rospy.Time.now()  # Time since the node started

            # * Calculate the linear and angular velocities based on the angular velocities of the wheels
            v = R * ((self.wr + self.wl) / 2.0)
            w = R * ((self.wr - self.wl) / L)

            # * Calculate the pose based on the calculated velocities and the previous pose
            # ! mu^
            x += T * v * np.cos(q)
            y += T * v * np.sin(q)
            q += T * w

            # * To quaternion
            qRota = tf.transformations.quaternion_from_euler(0, 0, q)

            # * Encoder covariance
            # ! Sigma^
            sigma = self.calculate_covariance(q, v, sigma)
            covariance = self.covariance_3_to_6(sigma)

            # * First estimation
            odom = self.initialize_odometry(cTime, x, y, qRota, v, w, covariance)
            pose_with_covariance_stamped = self.initialize_pose(cTime, odom)

            # * Prepare transformation matrices
            rotation_matrix_z = np.array(
                [[np.cos(q), -np.sin(q), 0], [np.sin(q), np.cos(q), 0], [0, 0, 1]],
                dtype=float,
            )
            robot_map_transformation = np.eye(4)
            robot_map_transformation[:3, :3] = rotation_matrix_z
            robot_map_transformation[:3, -1] = np.array([x, y, 0]).T
            map_robot_transformation = np.linalg.inv(robot_map_transformation)

            # * Correct with all seen arucos
            if self.fiducaial_poses is not None:
                robot_pose_map = np.array([[x], [y], [0]])  # Robot in map frame
                markers = []
                for fiducial_pose in self.fiducaial_poses.transforms:
                    fiducial_id = fiducial_pose.fiducial_id
                    true_aruco = self.map.get(str(fiducial_id), None)

                    # ! Sometimes sees the wrong aruco
                    if true_aruco is None:
                        continue

                    # * Create the dp vector for the real aruco
                    true_aruco = np.array(true_aruco)
                    aruco_dp = self.create_dp(true_aruco, robot_pose_map)

                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = cTime
                    marker.type = 1
                    marker.id = int(fiducial_id)
                    marker.scale.x = 0.01
                    marker.scale.y = 0.14
                    marker.scale.z = 0.14
                    marker.color.r = 76.0 / 255.0
                    marker.color.g = 185.0 / 255.0
                    marker.color.b = 68.0 / 255.0
                    marker.color.a = 1
                    marker.pose.position.x = true_aruco[0][0]
                    marker.pose.position.y = true_aruco[1][0]
                    marker.pose.position.z = true_aruco[2][0]

                    markers.append(marker)

                    # * Transform the aruco pose to the map frame
                    fiducial_pose_homogenous = np.array(
                        [
                            [fiducial_pose.transform.translation.x],
                            [fiducial_pose.transform.translation.y],
                            [fiducial_pose.transform.translation.z],
                            [1],
                        ]
                    )
                    aruco_r = np.matmul(  # Aruco in robot frame
                        self.camera_robot_transformation, fiducial_pose_homogenous
                    )
                    aruco_m = np.matmul(  # Aruco in map frame
                        robot_map_transformation, aruco_r
                    )

                    # * Create the dp vector for the seen aruco
                    dp = self.create_dp(aruco_m[:3], robot_pose_map)

                    # * Calculate the Kalman gain
                    Gk = self.get_Gk(dp)
                    Zk = np.matmul(Gk, np.matmul(sigma, Gk.T)) + self.Rk
                    Kk = np.matmul(sigma, np.matmul(Gk.T, np.linalg.inv(Zk)))

                    # * Calculate the innovation
                    z = self.get_z(dp, aruco_r)
                    true_aruco_homogenous = true_aruco
                    true_aruco_homogenous = np.append(true_aruco_homogenous, 1)
                    aruco_r = np.matmul(map_robot_transformation, true_aruco_homogenous)
                    z_hat = self.get_z(aruco_dp, aruco_r)

                    # * Correction
                    mu = np.array([[x], [y], [q]]) + np.matmul(Kk, (z - z_hat))
                    sigma = np.matmul((np.eye(3) - np.matmul(Kk, Gk)), sigma)

                    # * Update for next iteration
                    x = mu[0][0]
                    y = mu[1][0]
                    q = mu[2][0]

                self.markers_publisher.publish(markers)

                # * Correct the pose
                pose_with_covariance_stamped.pose.pose.position.x = x
                pose_with_covariance_stamped.pose.pose.position.y = y
                qRota = tf.transformations.quaternion_from_euler(0, 0, q)
                pose_with_covariance_stamped.pose.pose.orientation = Quaternion(
                    qRota[0], qRota[1], qRota[2], qRota[3]
                )
                covariance = self.covariance_3_to_6(sigma)
                pose_with_covariance_stamped.pose.covariance = (
                    covariance.flatten().tolist()
                )

                # * Clean the fiducial poses for the next iteration
                self.fiducaial_poses = None

            # * Publish the final pose
            self.kalman_publisher.publish(pose_with_covariance_stamped)

            # * Reconstruction of the odometry message
            odom.pose.pose.position = Point(x, y, R)
            odom.pose.pose.orientation = Quaternion(
                qRota[0], qRota[1], qRota[2], qRota[3]
            )
            odom.pose.covariance = covariance.flatten().tolist()

            # * Publish the final odometry
            self.pOD.publish(odom)

            # * Send transform
            t = TransformStamped()
            t.header.stamp = cTime
            t.header.frame_id = "odom"
            t.child_frame_id = "kalman/base_link"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation = pose_with_covariance_stamped.pose.pose.orientation
            self.br.sendTransform(t)

            # * Sleep
            self.rate.sleep()


if __name__ == "__main__":
    try:
        KalmanFilter().run()
    except rospy.exceptions.ROSInterruptException:
        pass
