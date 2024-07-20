import rclpy
import numpy as np
import time
from igus_moveit_clients.transform import Affine #für 6D Transformation 

#import clients
from igus_moveit_clients.igus_moveit import ARMClient


def main():
    # initialize ros communications for a given context 
    rclpy.init(args=None)

    # initialize/ bring up node with agv clients
    robot = ARMClient()

    #define a home position (when want to use default [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] you don't need this definition) -> floats required
    robot.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    robot.setVelocity(0.5)

    # move robot to home position
    robot.home()

    #pose = robot.get_transform('igus_tool0', 'igus_base_link', affine=False)
    affine = robot.get_transform('igus_tool0', 'igus_base_link', affine=True)
 
    test_affine = Affine((0.50038992, -0.195618,    0.147), (0.00,0.00,-0.17110917,  0.98525208))


    # move robot to a specific position
    print('Test Affine:', test_affine)
    print('Test_Affine_Translation:', test_affine.translation)
    print('Test_Affine_Quaternion:', test_affine.quat)

    feedback = robot.ptp(test_affine)
    
    print('Feedback:', feedback)

    # move robot to home position
    robot.home()
    # destroy the robot node, stop execution
    robot.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()


if __name__ == '__main__':
    main()