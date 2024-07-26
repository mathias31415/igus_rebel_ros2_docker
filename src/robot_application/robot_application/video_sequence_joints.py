import rclpy
import numpy as np
import time
from igus_moveit_clients.transform import Affine #für 6D Transformation 

#import clients
from igus_moveit_clients.igus_moveit import ARMClient # igus robot

### without IK Computation, because this is very slow on rasperry pi ###

def main():
    # start ros application
    rclpy.init(args=None)

    # init and start client node
    robot = ARMClient()
    robot.home_position = [-3.1, 0.87, 1.57, 0.0, -0.87, 0.0] # [joint1, joint2, joint3, joint4, joint5, joint6]
    robot.setVelocity(0.8)
    print('Robot initialized')
    time.sleep(2)

    # home the robot to safe travel pose
    robot.home()
    print('Robot homed')
    time.sleep(5)


    # move the robot to approach pose over desk (defined in robot base coordinates)
    approach_pick_pose = Affine((0.364, 0.0, 0.38), (0.717, -0.015, -0.699, -0.013))    # defined in base coordinates 

    approach1_feedback = robot.ptp_joint([0.0, 0.311, 1.169, 0.0, 1.63, -3.1])
    print(f'PTP to approach_pick_pose feedback: {approach1_feedback}')


    # move the robot to pick pose
    pick_movenemt = Affine((0.15, 0.0, 0.0))    # defined in tcp coordinates
    pick_pose = approach_pick_pose * pick_movenemt    # transform multiplication to get the pick pose in base coordinates
    
    robot.setVelocity(0.25)
    pick_feedback = robot.ptp_joint([0.0, 0.428, 1.55, 0.00, 1.14, -3.1])
    print(f'LIN to pick_pose feedback: {pick_feedback}')
    time.sleep(2)


    # move the robot back to approach pose
    approach2_feedback = robot.ptp_joint([0.0, 0.311, 1.169, 0.0, 1.63, -3.1])
    print(f'PTP to approach_pick_pose feedback: {approach2_feedback}')


    # move the robot to approach place pose
    approach_place_pose = Affine((-0.285, 0.227, 0.38), (0.717, -0.015, -0.699, -0.013))    # defined in base coordinates
    
    robot.setVelocity(0.8)
    approach3_feedback = robot.ptp_joint([-2.47, 0.339, 1.13, -0.01, 1.69, -0.62])
    print(f'PTP to approch_place_pose feedback: {approach3_feedback}')


    # move the robot to place pose
    place_movenemt = Affine((0.32, 0.0, 0.0))    # defined in tcp coordinates
    place_pose = approach_place_pose * place_movenemt    # transform multiplication to get the pick pose in base coordinates

    robot.setVelocity(0.25)
    place_feedback = robot.ptp_joint([-2.47, 0.846, 1.62, -0.02, 0.7, -0.62])
    print(f'LIN to place_pose feedback: {place_feedback}')
    time.sleep(2)


    # move the robot back to approach place pose
    approach4_feedback = robot.ptp_joint([-2.47, 0.339, 1.13, -0.01, 1.69, -0.62])
    print(f'PTP to approch_place_pose feedback: {approach4_feedback}')


    # home the robot to safe travel pose
    robot.setVelocity(0.8)
    robot.home()
    print('Robot homed')
    time.sleep(5)


    # destroy the node
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()