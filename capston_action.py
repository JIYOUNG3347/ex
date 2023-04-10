import rclpy
import time
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main():
    a = 1

    # Read .txt file
    i = 0
    f1 = open('/home/user/ros2_ws/src/py_joint_controller/py_joint_controller/policies.txt', 'r')
    policies = f1.readlines()
    policies=[x.strip("\n[]") for x in policies]
    policies=[y.split(' ') for y in policies]
    
    
    while a == 1:

        # Initialize rclpy
        rclpy.init(args=None)

        # Create ROS node
        node = rclpy.create_node('test_action_client')

        # Create an action client of type FollowJointTrajectory
        action_client = ActionClient(node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        # Check if the action server is available and wait for 2 seconds
        if not action_client.wait_for_server(timeout_sec=2.0):
            node.get_logger().error('Action server not available')
            rclpy.shutdown()
            return

        # Output log message indicating connection to action server
        node.get_logger().info('Action server available')

        # Add set of positions
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # Init positions
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = [0.0, -1.57, 0.0, 0.0, 0.0, 0.0] # joint position
        joint_trajectory_point.time_from_start = rclpy.duration.Duration(seconds=3).to_msg()
        joint_trajectory.points.append(joint_trajectory_point)

        # Target 1 positions
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = [float(policies[i][0]), -1.57, float(policies[i][1]), 0.0, 0.0, 0.0]
        joint_trajectory_point.time_from_start = rclpy.duration.Duration(seconds=int(policies[i][4])).to_msg()
        joint_trajectory.points.append(joint_trajectory_point)

        # Target 2 positions
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = [float(policies[i][2]), -1.57, float(policies[i][3]), 0.0, 0.0, 0.0]
        joint_trajectory_point.time_from_start = rclpy.duration.Duration(seconds=int(policies[i][5])).to_msg()
        joint_trajectory.points.append(joint_trajectory_point)

        # Create an action target of type FollowJointTrajectory
        follow_joint_traj_goal = FollowJointTrajectory.Goal()

        # Assign the given joint trajectory to the action target
        follow_joint_traj_goal.trajectory = joint_trajectory

        # Sends the action target to the action server and returns a future object that monitors the state change of the action
        future = action_client.send_goal_async(follow_joint_traj_goal)

        while rclpy.ok():
            rclpy.spin_once(node)
            if future.done():                       # When a request to an action goal completes, do the following
                status = future.result().status     # Check the status of the result of a request to an action goal
                if status == 2:
                    node.get_logger().info('Action succeeded')
                else:
                    node.get_logger().info('Action failed with status:')
                    node.get_logger().info(str(status))
                break



        action_client.destroy()
        rclpy.shutdown()

        i = i + 1
        a = int(input('1 - To start a program, 2 - To end a program >> '))
    
    exit() 


    

if __name__ == '__main__':
    main()
