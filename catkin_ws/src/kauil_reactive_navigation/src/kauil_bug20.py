#!/usr/bin/env python
"""
Follow the Bug20 Reactive Navigation Algorithm (designed by the team mentioned below) to reach a goal.

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/05/09
"""
import rospy
import argparse
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from kauil_reactive_navigation.srv import *
from std_msgs.msg import String

# * Constants for State Machine
DIST_THRESHOLD = 0.10  # Distance threshold to goal
OBSTACLE_THRESHOLD = 1.3  # Distance threshold to obstacle
PUZZLEBOT_SIZE = 1  # Size of the robot (from xacro)
VISION_ANGLE = np.arctan((PUZZLEBOT_SIZE / 2.0) / OBSTACLE_THRESHOLD)  # Vision angle

# * Constants for go to goal
V_MAX = 0.28  # Max linear velocity
ALPHA = 2.0  # For calculating linear velocity
KW = 1  # Angular velocity P controller constant

# * Constants for obstacle avoidance
P_ORIENTATION = 1.0  # P controller constant for orientation
P_V = 0.1  # P controller constant for linear velocity
W_MAX = 0.4  # Max angular velocity
DISTANCE_TO_LINE_TH = 0.05  # Distance threshold to main line
ANGLE_THRESHOLD = np.deg2rad(5)  # Angle threshold to obstacle
DESIRED_WALL_DISTANCE = ROBOT_SIZE * 1.5  # Desired distance to wall
MAX_WALL_DISTANCE = 3.0  # Max distance to wall


class Bug20Algorithm:
    """
    # Bug20 Algorithm
    This node implements the Bug20 Reactive Navigation Algorithm (designed by the team mentioned above).

    ## Attributes
    - goal_x (`float`): x coordinate of the goal
    - goal_y (`float`): y coordinate of the goal
    - verbose (`bool`): verbosity of the program
    - rate (`rospy.Rate`): rate of the node
    - x (`float`): current x coordinate of the robot
    - y (`float`): current y coordinate of the robot
    - theta (`float`): current theta coordinate of the robot
    - scan (`LaserScan`): current LiDAR scan
    - angles (`numpy.ndarray`): angles of the LiDAR scan
    - state (`str`): current state of the state machine
    - a (`float`): a constant for the line equation
    - b (`float`): b constant for the line equation
    - c (`float`): c constant for the line equation
    - closest_distance (`float`): closest distance travelled in the line
    - cmd_vel_pub (`rospy.Publisher`): publisher for the cmd_vel topic
    - goal_pub (`rospy.Publisher`): publisher for the goal topic

    ## Methods
    - __init__(): Initializes the node (with arguments), publishers and subscribers, and sets the initial values.
    - odom_callback(msg): Callback for current x, y, and theta position
    - scan_callback(msg): Callback for LiDAR scan
    - end_callback(): Callback for when the node is shutdown
    - get_line(): Calculates the line equation for the goal and the robot
    - get_goal(): Gets the goal from the service
    - get_scan_between_angles(start_angle, end_angle): Gets the scan between two angles
    - distance_to_goal(): Calculates the linear distance to the goal
    - orientation_error(): Calculates the orientation error to the goal
    - go_to_goal(): Implements the go to goal state
    - calculate_w(r_1, theta_1, r_2, theta_2, beta, v): Calculates the angular velocity to follow the wall
    - go_around_obstacle(beta): Implements the go around obstacle state
    - distance_to_line(): Calculates the distance to the main line
    - run(): Runs the node

    ## Subscribers
    - `/odom`: current x, y, and theta position
    - `/scan`: LiDAR scan

    ## Publishers
    - `/cmd_vel`: cmd_vel to control robot
    """

    def __init__(self):
        """Initializes the node (with arguments), publishers and subscribers, and sets the initial values."""
        # * Arguments
        parser = argparse.ArgumentParser(
            description="Runs the Bug20 Reactive Navigation Algorithm"
        )
        parser.add_argument(  # Verbosity
            "--verbose",
            "-v",
            action="store_true",
            help="Sets the verbosity of the program",
            default=False,
        )
        args = parser.parse_args(rospy.myargv()[1:])
        self.goal_x, self.goal_y = 0, 0
        self.verbose = args.verbose

        # * Initialize node
        rospy.init_node("Bug20")
        self.rate = rospy.Rate(60)
        rospy.on_shutdown(self.end_callback)

        # * Initial values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.scan = None
        self.angles = None
        self.state = "GO_TO_GOAL"
        self.a, self.b, self.c = 0, 0, 0
        self.get_line()

        self.closest_distance = self.distance_to_goal()

        # * Subscribe to current x, y, and theta position
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # * Subscribe to LiDAR scan
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # * Publish to cmd_vel to control robot
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.goal_pub = rospy.Publisher("/goal", String, queue_size=10)

    def odom_callback(self, msg):
        """Callback for current x, y, and theta position

        Args:
            msg (Odometry): current x, y, and theta position
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )[2]

    def scan_callback(self, msg):
        """Callback for LiDAR scan

        Args:
            msg (LaserScan): LiDAR scan
        """
        self.scan = msg
        self.angles = np.arange(  # Get all the angles in the scan
            self.scan.angle_min, self.scan.angle_max, self.scan.angle_increment
        )

    def end_callback(self):
        """If node dies, for instance, by keyboard interrupt, we stop
        the robot
        """
        tw = Twist()
        tw.linear.x = 0
        tw.angular.z = 0
        self.cmd_vel_pub.publish(tw)

    def get_line(self):
        """Create a line from start point to end point.

        Returns:
            tuple[float, float, float]: a, b, and c constants for line equation.
        """
        # Following the equation ax + by + c = 0
        x0 = self.x
        y0 = self.y
        xf = self.goal_x
        yf = self.goal_y
        self.a = y0 - yf
        self.b = xf - x0
        self.c = x0 * yf - xf * y0

    def get_goal(self):
        """Gets the next goal from the service"""
        rospy.wait_for_service("get_new_goal")
        try:
            get_new_goal = rospy.ServiceProxy("get_new_goal", getNewGoal)
            response = get_new_goal(self.goal_x, self.goal_y)
            self.goal_x, self.goal_y = response.nx, response.ny
            if True:
                print("Current goal: ")
                print(self.goal_x, self.goal_y)
            self.get_line()
        except rospy.ServiceException as e:
            print("Service call failed")

    def get_scan_between_angles(self, start_angle, end_angle):
        """Gets the mean scan data between two angles

        Args:
            start_angle (float): start angle in radians
            end_angle (float): end angle in radians

        Returns:
            float: mean scan data between two angles
        """
        # * Normalize angles
        if abs(start_angle) > np.pi:
            start_angle = np.sign(start_angle) * (abs(start_angle) - 2 * np.pi)
        if abs(end_angle) > np.pi:
            end_angle = np.sign(end_angle) * (abs(end_angle) - 2 * np.pi)

        start_index = np.argmin(  # Find angle closest to start_angle
            np.abs(self.angles - start_angle)
        )
        end_index = np.argmin(  # Find angle closest to end_angle
            np.abs(self.angles - end_angle)
        )

        # * If front is requested, we need to split the ranges
        if start_index > end_index:
            return np.mean(
                np.concatenate(  # Concatenate the two ranges
                    (self.scan.ranges[start_index:], self.scan.ranges[:end_index])
                )
            )

        scan = np.mean(self.scan.ranges[start_index:end_index])
        if np.isnan(scan):
            scan = self.scan.range_max
        return scan

    def distance_to_goal(self):
        """Calculates the distance to the goal

        Returns:
            float: distance to goal
        """
        return np.sqrt((self.goal_x - self.x) ** 2 + (self.goal_y - self.y) ** 2)

    def orientation_error(self):
        """Calculates the orientation error

        Returns:
            float: orientation error
        """
        T_OR = np.linalg.inv(  # Transformation matrix from odom to robot
            np.array(
                [
                    [np.cos(self.theta), -np.sin(self.theta), 0, self.x],
                    [np.sin(self.theta), np.cos(self.theta), 0, self.y],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                ],
                dtype=np.float64,
            )
        )
        goal_o = np.array([self.goal_x, self.goal_y, 0, 1], dtype=np.float64)
        goal_r = np.dot(T_OR, goal_o)

        return np.arctan2(goal_r[1], goal_r[0])

    def go_to_goal(self):
        """Controls the robot to go to the goal. Calculates the linear and angular velocities based on the distance to the goal and the orientation error. Publishes the velocities to the cmd_vel topic."""
        if self.verbose:
            pass
        rospy.logwarn("Going to goal")

        # * Calculate errors
        distance_err = self.distance_to_goal()
        orientation_error = self.orientation_error()

        # * Calculate velocities
        v = V_MAX * (1 - np.exp(-ALPHA * (distance_err**2)))
        if distance_err > DIST_THRESHOLD:
            w = orientation_error * KW
        else:
            self.get_goal()
            w = 0

        # * Publish velocities
        tw = Twist()
        tw.linear.x = v
        tw.angular.z = w
        self.cmd_vel_pub.publish(tw)

    def calculate_w(self, r_1, theta_1, r_2, theta_2, beta, v):
        """Calculates the angular velocity to follow a wall.

        Args:
            r_1 (float): Distance to wall on absolute right
            theta_1 (float): Angle to wall on absolute right (in LiDAR frame)
            r_2 (float): Distance to wall on right-front
            theta_2 (float): Angle to wall on right-front (in LiDAR frame)
            beta (float): Weighting of orientation error
            v (float): Linear velocity

        Returns:
            float: Angular velocity
        """
        # * Tangent vector
        p1 = np.array([r_1 * np.cos(theta_1 + np.pi), r_1 * np.sin(theta_1 + np.pi)])
        p2 = np.array([r_2 * np.cos(theta_2 + np.pi), r_2 * np.sin(theta_2 + np.pi)])
        u_tan = p2 - p1
        u_tan_hat = u_tan / np.linalg.norm(u_tan)

        # * Perpendicular vector
        u_per = p1 - np.dot(p1, u_tan_hat) * u_tan_hat

        # * Calculate errors
        err_orientation = beta * np.arctan2(u_tan[1], u_tan[0])
        err_dist = (1 - beta) * (DESIRED_WALL_DISTANCE - np.linalg.norm(u_per))
        if self.verbose:
            rospy.logwarn("ed: {:.4f} | eo: {:.4f}".format(err_dist, err_orientation))

        # * Calculate angular velocity
        w = max(
            min(
                (err_orientation + err_dist) * P_ORIENTATION,
                W_MAX,
            ),
            -W_MAX,
        )

        return w

    def go_around_obstacle(self, beta=0.25):
        """Controls the robot to go around the obstacle using right-hand rule. Calculates the linear and angular velocities based on the distance to the obstacle and the direction of the wall. Publishes the velocities to the cmd_vel topic.

        Args:
            beta (float, optional): Weighting of orientation error, between 0 and 1 the larger the value, the more the robot will try to align with the wall, 0.5 being equal weighting. Defaults to 0.25.
        """
        # * -------- Get laser scans --------
        amount_of_scans = 15
        theta_1 = - np.pi/2 # Full right
        r_1 = self.get_scan_between_angles(
            theta_1 - self.scan.angle_increment * amount_of_scans,
            theta_1 + self.scan.angle_increment * amount_of_scans,
        )

        theta_2 = theta_1 + (np.pi/4)  # Right-front
        r_2 = min(
            self.get_scan_between_angles(
                theta_2 - self.scan.angle_increment * amount_of_scans,
                theta_2 + self.scan.angle_increment * amount_of_scans,
            ),
            MAX_WALL_DISTANCE * 0.5,  # Limit to max wall distance
        )

        if r_2 == MAX_WALL_DISTANCE:
            rospy.logerr("No wall right-front")

        distance_ahead = self.get_scan_between_angles(
        - self.scan.angle_increment * 15, self.scan.angle_increment * 15
        )
        # * ---------------------------------
        no_obstacle_right = r_1 > OBSTACLE_THRESHOLD
        no_obstacle_right_front = r_2 > OBSTACLE_THRESHOLD
        no_obstacle_front = distance_ahead > OBSTACLE_THRESHOLD
        if no_obstacle_right and no_obstacle_front and no_obstacle_right_front:
            self.state = "GO_TO_GOAL"
            return

        if distance_ahead > MAX_WALL_DISTANCE:  # If no wall ahead
            v = V_MAX * MAX_WALL_DISTANCE * 0.5  # Go slow
            if self.verbose:
                pass
            rospy.logerr("No wall front")

            if r_1 > MAX_WALL_DISTANCE:  # And no wall on the right
                w = W_MAX
                if self.verbose:
                    pass
                rospy.logerr("No wall right")
            else:
                w = self.calculate_w(r_1, theta_1, r_2, theta_2, beta, v)
        else:
            v = min((distance_ahead - DESIRED_WALL_DISTANCE * 0.3) * P_V, V_MAX)

            if r_1 > MAX_WALL_DISTANCE:  # If no wall on the right
                w = W_MAX
                if self.verbose:  # Force turn left
                    pass
                rospy.logerr("No wall right")
            else:
                w = self.calculate_w(r_1, theta_1, r_2, theta_2, beta, v)

        if self.verbose:
            rospy.logwarn("v: {:.4f} | w: {:.4f}".format(v, w))

        # * Publish velocities
        tw = Twist()
        tw.linear.x = v
        tw.angular.z = w
        self.cmd_vel_pub.publish(tw)

    def distance_to_line(self):
        """Calculate distance from current point to main line

        Returns:
            float: distance to line
        """
        return abs(self.a * self.x + self.b * self.y + self.c) / np.sqrt(
            self.a**2 + self.b**2
        )

    def run(self):
        """Runs the Bug20 algorithm until goal is reached"""
        self.get_goal()  # Get first goal
        while not rospy.is_shutdown():
            if self.scan is not None:  # Only move if scan is available
                self.goal_pub.publish(str(self.goal_x) + " " + str(self.goal_y))
                if self.verbose:
                    rospy.logwarn("")  # For spacing each iteration

                obstacle_in_front_distance = self.get_scan_between_angles(
                - VISION_ANGLE, VISION_ANGLE
                )
                theta_1 = - np.pi/2
                obstacle_in_right_distance = self.get_scan_between_angles(theta_1 - self.scan.angle_increment * 10, theta_1 + self.scan.angle_increment * 10)

                if self.state == "GO_TO_GOAL":
                    if obstacle_in_front_distance <= OBSTACLE_THRESHOLD or obstacle_in_right_distance <= OBSTACLE_THRESHOLD:
                        self.state = "GO_AROUND_OBSTACLE"
                    else:
                        self.go_to_goal()

                if self.state == "GO_AROUND_OBSTACLE":
                    self.go_around_obstacle()
                    angle_to_goal = self.orientation_error()
                    if abs(angle_to_goal) <= ANGLE_THRESHOLD:  # Exit through bug0
                        self.state = "GO_TO_GOAL"
                    elif (  # Exit through bug2
                        self.distance_to_line() <= DISTANCE_TO_LINE_TH
                    ):
                        if (  # Only if closer to goal
                            self.closest_distance > self.distance_to_goal()
                        ):
                            self.closest_distance = self.distance_to_goal()
                            self.state = "GO_TO_GOAL"

            # * Sleep
            self.rate.sleep()


if __name__ == "__main__":
    try:
        Bug20Algorithm().run()
    except rospy.ROSInterruptException:
        pass
