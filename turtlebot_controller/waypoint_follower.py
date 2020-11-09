#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from math import sin, cos, atan2
import matplotlib.pyplot as plt


class SplineGenerator:
    def __init__(self):
        self.waypoints = None
        self.spline_pts = None

    @property
    def spline_data(self):
        return self.spline_pts.copy()

    def create_splines(self, waypoints):
        self.waypoints = waypoints.reshape((-1, 2))
        waypoints = np.insert(waypoints, 0, waypoints[0], axis=0)
        waypoints = np.insert(waypoints, -1, waypoints[-1], axis=0)
        splines = []
        for i in range(len(waypoints)-3):
            this_spline = self.cubic_spline(waypoints[i], waypoints[i+1], waypoints[i+2], waypoints[i+3])
            splines.append(this_spline)

        self.spline_pts = np.array(splines).reshape((-1, 3))
        return self.spline_pts

    @staticmethod
    def cubic_spline(y0, y1, y2, y3, delt_mu=.001):
        mu = 0.
        points = []
        prev_x = 0.
        prev_y = 0.
        while mu <= 1.:
            mu2 = mu*mu
            a0 = y3 - y2 - y0 + y1
            a1 = y0 - y1 - a0
            a2 = y2 - y0
            a3 = y1
            mu += delt_mu
            point = a0*mu*mu2+a1*mu2+a2*mu+a3
            slope = atan2(point[1]-prev_y, point[0]-prev_x)
            point = np.append(point, slope)
            points.append(point)
            prev_x = point[0]
            prev_y = point[1]

        return points

    def plot(self):
        plt.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'bx')
        plt.plot(self.spline_pts[:, 0], self.spline_pts[:, 1], 'r-')
        plt.show()


class WaypointFollower:
    def __init__(self, full_waypoints=None, init_x=0., init_y=0., init_theta=0., kp=8., ka=3.):
        self.x = init_x
        self.y = init_y
        self.theta = init_theta

        self.kp = kp
        self.ka = ka

        self.waypoint_thresh = .175
	self.idx = 50

	# self.current_time = rospy.get_rostime()
        self.original_waypoints = full_waypoints
        self.gen = SplineGenerator()
        self.gen.create_splines(self.original_waypoints)
        
	self.xdata = self.gen.spline_data[:, 0]
        self.ydata = self.gen.spline_pts[:, 1]
        self.adata = self.gen.spline_pts[:, 2]

        self.ref_x = self.xdata[self.idx]
        self.ref_y = self.ydata[self.idx]
        self.ref_a = self.adata[self.idx]

    def nonlinear_controller(self, xr, yr, ar):
        p = np.sqrt(np.abs(xr - self.x)**2 + np.abs(yr - self.y)**2)
        alph = ar-self.theta

        vcmd = self.kp*p*cos(alph)
        wcmd = self.kp*sin(alph)*cos(alph) + self.ka*alph

        if vcmd >= .25:
	    vcmd = .25
        if vcmd <= -.25:
            vcmd = -.25

        if wcmd >= .45:
	    wcmd = .45
        if wcmd <= -.45:
            wcmd = -.45

        return vcmd, wcmd

    def odom_callback(self, odom_msg):
	self.x = odom_msg.pose.pose.position.x
	self.y = odom_msg.pose.pose.position.y

        siny_cosp = 2*(odom_msg.pose.pose.orientation.w*odom_msg.pose.pose.orientation.z + odom_msg.pose.pose.orientation.x*odom_msg.pose.pose.orientation.y)
        cosy_cosp = 1 - 2*(odom_msg.pose.pose.orientation.y*odom_msg.pose.pose.orientation.y + odom_msg.pose.pose.orientation.z*odom_msg.pose.pose.orientation.z)
        yaw_rad = atan2(siny_cosp, cosy_cosp)  #+ 3.141592654
        self.theta = yaw_rad 
	
    def subscribe_odom(self):
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
    
    def move_ref(self, robot_pos, curr_pt):
        if np.linalg.norm(robot_pos-curr_pt) <= self.waypoint_thresh:
            return True
        return False

    def plot_results(self, x_pos, y_pos):
        plt.plot(self.original_waypoints[:, 0], self.original_waypoints[:, 1], 'gx')
        plt.plot(x_pos, y_pos, 'b-', label='Robot Trajectory')
        plt.plot(self.gen.spline_data[:, 0], self.gen.spline_data[:, 1], 'r-', label='Desired Trajectory')
        plt.xlabel('X pos (m)')
        plt.ylabel('Y pos (m)')
        plt.grid()
        plt.legend()
        plt.show()
	
    def __call__(self):
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('controller', anonymous=True)
        rate = rospy.Rate(20) # 20hz
        vel_msg = Twist()
	
        vel_msg.linear.x = 0.
        vel_msg.linear.y = 0.
        vel_msg.linear.z = 0.
        vel_msg.angular.x = 0.
        vel_msg.angular.y = 0.
        vel_msg.angular.z = 0.

        self.ref_x = 0.
        self.ref_y = 0.
        self.ref_a = 0.

        pos_x = []
        pos_y = []
	
        ref_x_hist = []
        ref_y_hist = []
	
        while not rospy.is_shutdown():
            self.subscribe_odom()
            pos_x.append(self.x)
            pos_y.append(self.y)
            ref_x_hist.append(self.ref_x)
            ref_y_hist.append(self.ref_y)

            goal_dist = np.linalg.norm(np.array([self.x, self.y])-np.array([self.original_waypoints[-1, 0], self.original_waypoints[-1, 1]]))

            if self.move_ref(np.array([self.x, self.y]), np.array([self.ref_x, self.ref_y])):
                self.idx += 25

            if self.idx >= len(self.xdata):
                self.idx = len(self.xdata)-1

            if goal_dist <= self.waypoint_thresh:
                vel_msg.linear.x = 0.
                vel_msg.angular.z = 0.
                pub.publish(vel_msg)
                self.plot_results(pos_x, pos_y)
                break

            self.ref_x = self.xdata[self.idx]
            self.ref_y = self.ydata[self.idx]
            self.ref_a = self.adata[self.idx]

            vcmd, wcmd = self.nonlinear_controller(self.ref_x, self.ref_y, self.ref_a)
	    vel_msg.linear.x = vcmd
	    vel_msg.angular.z = wcmd

	    # rospy.loginfo("X POSITION: %s", self.x)
	    # rospy.loginfo("Y POSITION: %s", self.y)
	    # rospy.loginfo("HEADING: %s", self.theta)
            # rospy.loginfo("---REF HEADING: %s", self.ref_a)
	    
            pub.publish(vel_msg)
            rate.sleep()


if __name__ == '__main__':
    waypts = np.array([[0., 0.], [1., .5], [2., 1.5], [3., 1.], [4.5, 0.]])
    waypoint_follow = WaypointFollower(waypts)
    waypoint_follow()
