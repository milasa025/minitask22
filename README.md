# minitask22
#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

class Behaviors:
    def __init__(self):
        rospy.init_node('behaviors', anonymous=True)
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.move_cmd = Twist()
        self.state = 'random_walk'

    def laser_callback(self, data):
        # 检查激光扫描数据
        front_ranges = data.ranges[0:45] + data.ranges[-45:]  # 前方的范围
        self.min_distance_front = min(front_ranges)

        right_ranges = data.ranges[270:300]  # 右侧的范围
        self.min_distance_right = min(right_ranges)

        # 状态切换逻辑
        if self.min_distance_front < 0.3:
            self.state = 'obstacle_avoidance'
        elif self.min_distance_right < 0.5:
            self.state = 'wall_following'
        else:
            self.state = 'random_walk'

    def obstacle_avoidance(self):
        if self.min_distance_front < 0.3:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.1            
        else:
            self.state = 'random_walk'


    def follow_wall(self):
        if self.min_distance_right < 0.5:
            self.move_cmd.linear.x = 0.1
            self.move_cmd.angular.z = 0.0
        else:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = -0.1

    def random_walk(self):
        self.move_cmd.linear.x = 0.1
        self.move_cmd.angular.z = random.uniform(-1.0, 1.0)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.state == 'random_walk':
                self.random_walk()
            else:
                self.obstacle_avoidance()
            
            self.pub.publish(self.move_cmd) 
            rate.sleep()

if __name__ == '__main__':
    turtlebot_behaviors = Behaviors()
    turtlebot_behaviors.run()
