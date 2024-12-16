#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped # 控制机器人运动
from visualization_msgs.msg import Marker
import smach
import numpy as np
from actionlib import SimpleActionClient
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import time
import threading


class NavigateToPoint(smach.State):
    def __init__(self, target):
        smach.State.__init__(self, outcomes=['reached', 'failed'])
        self.target = target  # 导航目标点 (x, y)

    def execute(self, userdata):
        try:        
            self.sub=rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
            rospy.loginfo(f"Navigating to point: {self.target}")
            result = self.move_to_goal(self.target[0], self.target[1])
            if result:
                return 'reached'
            else:
                return 'failed'
        
        finally:
            self.client.cancel_all_goals()  # 清理：取消所有目标任务
            self.sub.unregister()


        
        

    def amcl_callback(self,msg):
        # 从 Odometry 消息中获取位置
        global current_x, current_y
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        rospy.loginfo(f"Robot position from amcl: x={self.current_x}, y={self.current_y}")

    def move_to_goal(self,x, y):
        # 创建 ActionClient 来控制导航
        self.client = SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()

        # 设置目标位置
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # 朝向默认值

        # 发送目标并等待结果
        self.client.send_goal(goal)
        
        # 等待目标完成，或者检查距离
        while not self.client.wait_for_result(rospy.Duration(0.1)):  # 每0.1秒检查一次
            # 获取机器人当前位置
            robot_x, robot_y = self.get_robot_position()
            
            # 计算当前位置与目标点的距离
            distance = distance_to_goal(robot_x, robot_y, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
            print(f"Distance to goal: {distance:.2f} meters")
            # 如果距离小于0.15米，认为已经到达目标
            if distance < 0.20:
                rospy.loginfo(f"Goal reached: ({x}, {y}) within 0.15 meters")
                return True

            # 检查结果
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Goal reached: ({x}, {y})")
                return True
        # 如果任务没有成功
        if self.client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logwarn(f"Failed to reach goal: ({x}, {y})")
            return False

    def get_robot_position(self):
        """
        获取机器人当前的 x, y 坐标
        """
        print("Getting robot position...")
        return self.current_x, self.current_y

def distance_to_goal(x1, y1, x2, y2):
    # 计算两个点之间的欧氏距离
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)




class MovetoPillar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached', 'not_reached'])
        self.pillar_detected = False   

    def execute(self, userdata):
        try:          
            self.controller = Turtlebot3HighlevelController(rospy)
            rospy.loginfo("Detecting red point...")
            # 等待控制器进行柱子检测并处理
            rate = rospy.Rate(10)  # 控制检测频率
            start=time.time()
            end = start
            while not self.pillar_detected and end-start < 20:
                with self.controller.lock:
                    if self.controller.pillar_detected:
                        self.pillar_detected = True
                        rospy.loginfo("Pillar detected!")
                        return 'reached'
            end = time.time()
            rate.sleep()
            

            rospy.loginfo("Red point not detected.")
            return 'not_reached'
            
            
        finally:
            if self.controller:
                self.controller.vel_pub.unregister()
                self.controller.subscriber.unregister()
                self.controller.viz_pub.unregister()
            



class StopAtPole(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stopped'])

    def execute(self, userdata):
        try:
            rospy.loginfo("Stopping at pillar...")
            self.stop_robot(duration=2)
            return 'stopped' 
        finally:
            self.vel_pub.unregister()   

    def stop_robot(self,duration=2):
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        Twist().linear.x = 0.0
        Twist().angular.z = 0.0
        self.stop_cmd = Twist()
        self.vel_pub.publish(self.stop_cmd)  # 发布停止命令
        rospy.sleep(duration)  # 静止指定时间
    
    
class Turtlebot3HighlevelController():
    def __init__(self, nh):
        self.node_handle = nh
        self.pillar_pos = [0.0, 0.0]
        self.p_vel = 0.8  # Proportional gain for linear velocity
        self.p_ang = 0.2  # Proportional gain for angular velocity
        self.pillar_detected = False
        self.lock = threading.Lock()  # 锁机制，确保线程安全
        self.pillar_found = False



        # Load parameters from config file
        self.p_vel = rospy.get_param("controller_gain", self.p_vel)  # default value if param not set
        self.subscriber_topic = rospy.get_param("subscriber_topic", "scan")  # default topic name
        self.queue_size = rospy.get_param("queue_size", 10)  # default queue size

        # Create subscriber and publisher
        self.subscriber = rospy.Subscriber(self.subscriber_topic, LaserScan, self.laser_callback, queue_size=self.queue_size)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.viz_pub = rospy.Publisher("visualization_marker", Marker, queue_size=0)

        # Initialize marker for pillar visualization
        self.init_pillar_marker()
        rospy.loginfo("TurtleBot3 high-level controller node launched!")
        

    def laser_callback(self, msg):
        cluster_size_threshold = 10
        width_threshold = 0.2
        curvature_threshold = 0.05
        min_dist = float('inf')
        min_index = -1
        j=0

        for i in range(1, len(msg.ranges) - 1):
            dist = msg.ranges[i]
            if not np.isfinite(dist) or dist <= 0.0:
                continue
            
            prev_dist = msg.ranges[i - 1]
            next_dist = msg.ranges[i + 1]
            curvature = abs(prev_dist - 2 * dist + next_dist)

            if curvature > curvature_threshold:
                cluster_size = 0
                cluster_width = 0.0

                j = i
                while j < len(msg.ranges) and abs(msg.ranges[j] - dist) < width_threshold:
                    cluster_size += 1
                    cluster_width += msg.angle_increment * msg.ranges[j]
                    j += 1
                    
                if cluster_size < cluster_size_threshold and cluster_width < width_threshold and dist < min_dist:
                    self.pillar_found = True
                    min_dist = dist
                    min_index = i

        if self.pillar_found and min_index >= 0:
            ang = msg.angle_min + msg.angle_increment * min_index

            self.pillar_pos[0] = min_dist * math.cos(ang)
            self.pillar_pos[1] = min_dist * math.sin(ang)

            rospy.loginfo("Pillar detected at %.2f m and %.2f degrees" % (min_dist, ang * 180.0 / math.pi))
            rospy.loginfo("Pillar's coordinate to Turtlebot is [%f, %f]" % (self.pillar_pos[0], self.pillar_pos[1]))

            # Drive the Husky robot towards the pillar
            self.drive_husky()       

        else:
            rospy.loginfo("No pillar detected.")
            
            
    # 当距离足够接近时，调用停止
        if min_dist < 0.2:
            rospy.loginfo("Pillar is close enough!")
            with self.lock:
                self.pillar_detected = True  # 设置标志为 True，表示已检测到柱子
            rospy.loginfo("Shutting down the task after pillar detection.")
            

            


    def viz_pillar(self):
        """
        Visualize pillar position with a marker in RViz
        """
        self.marker.pose.position.x = self.pillar_pos[0]
        self.marker.pose.position.y = self.pillar_pos[1]
        self.marker.pose.position.z = -1.0
        self.viz_pub.publish(self.marker)

            
    def drive_husky(self):
        # Combine heading and speed adjustments
        cmd = Twist()
        cmd.linear.x = min(0.5, (self.pillar_pos[0] - 0.2))  # Ensure safe approach
        cmd.angular.z = 0.8 * math.atan2(self.pillar_pos[1], self.pillar_pos[0])
        self.vel_pub.publish(cmd)




    def init_pillar_marker(self):
        """
        Initialize the pillar marker for RViz visualization
        """
        self.marker = Marker()
        self.marker.header.frame_id = "base_laser"
        self.marker.ns = "pillar"
        self.marker.id = 1
        self.marker.type = Marker.CYLINDER
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = self.pillar_pos[0]
        self.marker.pose.position.y = self.pillar_pos[1]
        self.marker.pose.position.z = -1.0
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 2.0
        self.marker.color.a = 1.0  # Alpha transparency
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

    def update_marker_time(self):
        """
        Update the timestamp of the marker for RViz
        """
        self.marker.header.stamp = rospy.Time.now()


def main():
    rospy.init_node('smach_turtlebot_navigation', anonymous=True)

    # 创建状态机
    sm = smach.StateMachine(outcomes=['task_completed', 'task_failed'])

    # 定义导航点 (x, y) 坐标
    points = {
        "P1": (-4.28, -0.72),
        "P2": (-1.61, -0.554),
        "P3": (0.211, 1.8),
        "P4": (-3.16,2.66 ),
    }

    # 打开状态机容器
    with sm:
        # P1 → P2
        smach.StateMachine.add('NAVIGATE_TO_P2', NavigateToPoint(points["P2"]), 
                               transitions={'reached': 'MovetoPillar1', 
                                            'failed': 'task_failed'})
        smach.StateMachine.add('MovetoPillar1', MovetoPillar(), 
                               transitions={'reached': 'STOP_AT_P2', 
                                            'not_reached': 'task_failed'})
        smach.StateMachine.add('STOP_AT_P2', StopAtPole(), 
                               transitions={'stopped': 'NAVIGATE_TO_P3'})

        # P2 → P3
        smach.StateMachine.add('NAVIGATE_TO_P3', NavigateToPoint(points["P3"]), 
                               transitions={'reached': 'MovetoPillar2', 
                                            'failed': 'task_failed'})
        smach.StateMachine.add('MovetoPillar2', MovetoPillar(), 
                               transitions={'reached': 'STOP_AT_P3', 
                                            'not_reached': 'task_failed'})
        smach.StateMachine.add('STOP_AT_P3', StopAtPole(), 
                               transitions={'stopped': 'NAVIGATE_TO_P4'})

        # P3 → P4
        smach.StateMachine.add('NAVIGATE_TO_P4', NavigateToPoint(points["P4"]), 
                               transitions={'reached': 'MovetoPillar3', 
                                            'failed': 'task_failed'})
        smach.StateMachine.add('MovetoPillar3', MovetoPillar(), 
                               transitions={'reached': 'STOP_AT_P4', 
                                            'not_reached': 'task_failed'})
        smach.StateMachine.add('STOP_AT_P4', StopAtPole(), 
                               transitions={'stopped': 'NAVIGATE_TO_P1'})

        # P4 → P1
        smach.StateMachine.add('NAVIGATE_TO_P1', NavigateToPoint(points["P1"]), 
                               transitions={'reached': 'task_completed', 
                                            'failed': 'task_failed'})

    # 执行状态机
    outcome = sm.execute()

if __name__=='__main__':
    main()