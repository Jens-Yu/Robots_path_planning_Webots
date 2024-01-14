"""my_lab4_controller controller."""

# my_lab4_controller Class Definition
# File: my_lab4_controller.py
# Date: 15th Nov 2022
# Description: Simple Controller based on 2021 version (2022)
# Author: Terry Payne (trp@liv.ac.uk)
#

from controller import Supervisor
import pioneer_nav2 as pn
import pioneer_proxsensors1 as pps
import math
import pose
from pioneer_nav2 import MoveState

  
def bug2_modified(robot):
    timestep = 64
    robot_pose = pose.Pose(0.0, 0.0, 0.0)
    target_pose = pose.Pose(2.75, -3.26, 0)
    camera = robot.getDevice('camera')
    if camera is not None:
        camera.enable(timestep)
    display = robot.getDevice('display2')
    display.setFont('Arial', 24, True)

    prox_sensors = pps.PioneerProxSensors(robot, "display", robot_pose)
    nav = pn.PioneerNavigation(robot, robot_pose, target_pose, prox_sensors)

    while robot.step(timestep) != -1:
        print(nav.state)
        robot_pose = nav.get_real_pose()
        prox_sensors.set_pose(robot_pose)
        prox_sensors.paint()
        angle_to_target = nav.calculate_heading_angle()
  
        if nav.close_to_target():
            nav.stop()
            print("Total distance traveled: {:.2f} meters".format(nav.total_distance))
            break
        if nav.movement_timer > 0:
            nav.movement_timer -= timestep
            continue
        if nav.state == MoveState.STOP:
            if not nav.close_to_target():
                nav.state = MoveState.ARC

        elif nav.state == MoveState.ARC:
            if nav.detect_obstacle(1, 7):
                nav.arc(math.pi/2.0)
                nav.state = MoveState.FOLLOW_WALL
            else:
                nav.arc(angle_to_target)
                nav.state = MoveState.FORWARD


        elif nav.state == MoveState.FORWARD:
            nav.forward()
            if nav.detect_obstacle(1, 7) or abs(angle_to_target) > 0.16:
                nav.stop()


        elif nav.state == MoveState.FOLLOW_WALL:
            if nav.detect_obstacle(7, 10, 0.5):
                nav.forward()
            elif nav.detect_obstacle(6,7):
                nav.arc(math.pi/4.0)
            else:
                nav.stop()
 
    pass


if __name__ == "__main__":
    # create the Supervised Robot instance.
    my_robot = Supervisor()
    bug2_modified(my_robot)
