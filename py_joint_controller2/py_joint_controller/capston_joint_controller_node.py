import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from numpy.linalg import norm
import time
import numpy as np
from numpy import inf, r_


class InitPublisher(Node):
    def __init__(self):
        super().__init__('init_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = [-1.24, 0.0, 3.9, -2.05, -1.94, 0.0]
        point.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        point.time_from_start = rclpy.time.Duration(seconds=4.0).to_msg()
        msg.points.append(point)
        self.publisher_.publish(msg)
        time.sleep(5)
        self.get_logger().info('Initialization Done')

class RestartPublisher(Node):
    def __init__(self):
        super().__init__('restart_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = [-1.24, 0.0, 3.9, -2.05, -1.94, 0.0]
        point.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        point.time_from_start = rclpy.time.Duration(seconds=4.0).to_msg()
        msg.points.append(point)
        self.publisher_.publish(msg)
        time.sleep(5)
        self.get_logger().info('Restart Done')


class CapstonJointController(Node):
    def __init__(self):
        super().__init__('capston_joint_controller')
        # Subscriber
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_controller/state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Read .txt file
        self.i = 0
        self.j = 0
        self.f1 = open('/home/user/ros2_ws/src/py_joint_controller/py_joint_controller/policies.txt', 'r')
        self.policies = self.f1.readlines()
        self.policies=[x.strip("\n[]") for x in self.policies]
        self.policies=[y.split(' ') for y in self.policies]



    def listener_callback(self, msg):
        # Publish Topic setting 1
        pub_msg = JointTrajectory()
        pub_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        
        # Init Position
        init_position = [-1.24, 0.0, 3.9, -2.05, -1.94, 0.0]

        # Current position
        current_position = list(np.round(msg.actual.positions,3))

        # Tartget postion
        target_position = [float(self.policies[self.i][self.j]), float(self.policies[self.i][self.j+1]), float(self.policies[self.i][self.j+2]), -2.05, -1.94, 0.0]
        
        # Max velocity
        max_vel = float(self.policies[self.i][6])


        print("current: ", current_position)
        print("target: ", point.positions)


        # Tar1 to Tar2
        if (target_position == current_position) and (self.j == 0):
            self.j = 3
            print("Target 1 Reached")
        elif target_position == current_position:
            print("Target 2 Reached")
            input("Press Enter")
            restart_publisher = RestartPublisher()
            rclpy.spin_once(restart_publisher)
            self.i = self.i + 1
            self.j = 0

        # Publish Topic setting 2
        duration = norm((r_[target_position] - r_[current_position])/ max_vel, ord = inf)
        point.time_from_start = rclpy.time.Duration(seconds = duration).to_msg()
        point.positions = target_position
        pub_msg.points.append(point)
        self.publisher_.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)
    init_publisher = InitPublisher()
    rclpy.spin_once(init_publisher)
    capston_joint_controller = CapstonJointController()
    rclpy.spin(capston_joint_controller)
    capston_joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
