from controller import Supervisor, Node

import math
import pose
from enum import Enum


class MoveState(Enum):
    STOP = 0
    FORWARD = 1
    ARC = 2
    FOLLOW_WALL = 3


class PioneerNavigation:
    """ A custom class to initialise and manage simple navigation on a Pioneer Adept robot """

    WHEEL_RADIUS = 0.0957  # in meters - found using CONFIGURE
    AXEL_LENGTH = 0.323  # in meters- found using CONFIGURE

    def __init__(self, robot, init_pose, target_pose, ps):

        self.robot = robot  # reference to the robot
        self.robot_node = self.robot.getSelf()  # reference to the robot node
        self.state = MoveState.STOP
        self.prox_sensors = ps
        # enable motors
        self.left_motor = self.robot.getDevice('left wheel')
        self.right_motor = self.robot.getDevice('right wheel')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        # set up pose
        self.robot_pose = pose.Pose(init_pose.x, init_pose.y, init_pose.theta)

        # Initialise motor velocity
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.max_vel = self.left_motor.getMaxVelocity() - 0.1
        self.prev_error = 0
        self.total_error = 0
        self.target_pose = target_pose
        self.movement_timer = 0
        self.total_distance = 0.0
        self.last_pose = init_pose

    def get_real_pose(self):
        if self.robot_node is None:
            return pose.Pose(0, 0, 0)

        real_pos = self.robot_node.getPosition()
        rot = self.robot_node.getOrientation()
        theta = math.atan2(-rot[0], rot[3])
        halfpi = math.pi / 2
        theta2 = theta + halfpi
        if (theta > halfpi):
            theta2 = -(3 * halfpi) + theta
        new_pose = pose.Pose(real_pos[0], real_pos[1], theta2)

        # 更新总距离
        self.total_distance += math.sqrt((new_pose.x - self.last_pose.x) ** 2 +
                                         (new_pose.y - self.last_pose.y) ** 2)
        self.last_pose = new_pose  # 更新上一次的位置
        return new_pose

    def close_to_target(self, threshold=0.2):
        return self.get_distance_to_target() <= threshold

    def get_distance_to_target(self):
        current_pose = self.get_real_pose()
        return math.sqrt((current_pose.x - self.target_pose.x) ** 2 + (current_pose.y - self.target_pose.y) ** 2)

    def detect_obstacle(self, sensor_num1, sensor_num2, threshold_distance=0.3):
        # 检测前方是否有障碍物
        # 假设前方传感器的索引是3到4
        for i in range(sensor_num1, sensor_num2):
            if self.prox_sensors.get_value(i) < threshold_distance:
                return True  # 检测到障碍物
        return False  # 未检测到障碍物

    def calculate_heading_angle(self):
        # 当前机器人的位置
        current_position = self.robot_node.getPosition()
        # 目标位置
        target_x, target_y = self.target_pose.x, self.target_pose.y

        # 计算机器人位置到目标位置的向量
        direction_x, direction_y = target_x - current_position[0], target_y - current_position[1]

        # 计算目标方向的角度（相对于北方）
        target_angle = math.atan2(direction_y, direction_x)

        # 机器人当前朝向
        robot_angle = self.robot_node.getOrientation()
        theta = math.atan2(-robot_angle[0], robot_angle[3])
        halfpi = math.pi / 2
        theta2 = theta + halfpi
        if (theta > halfpi):
            theta2 = -(3 * halfpi) + theta
        # 计算角度差
        angle_diff = target_angle - theta2

        # 规范化角度到[-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        return angle_diff

    def forward(self, robot_linearvelocity = 0.2):
        wheel_av = (robot_linearvelocity / self.WHEEL_RADIUS)

        self.left_motor.setVelocity(wheel_av)
        self.right_motor.setVelocity(wheel_av)



    def arc(self, icr_angle, robot_linearvelocity = 0.2):
        target_time = (abs(icr_angle) * self.AXEL_LENGTH) / (4 * robot_linearvelocity)

        wheel_av = (robot_linearvelocity / self.WHEEL_RADIUS)
        if icr_angle < 0:
            self.left_motor.setVelocity(wheel_av)
            self.right_motor.setVelocity(-wheel_av)
        else:
            self.left_motor.setVelocity(-wheel_av)
            self.right_motor.setVelocity(wheel_av)
        self.movement_timer = 1000.0 * target_time
        # return target_time as millisecs
        return self.movement_timer

    def stop(self):
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.state = MoveState.STOP

    def set_velocity(self, base, control=0):
        # base gives the velocity of the wheels in m/s
        # control is an adjustment on the main velocity
        base_av = (base / self.WHEEL_RADIUS)

        if (control != 0):
            control_av = (control / self.WHEEL_RADIUS)
            # Check if we exceed max velocity and compensate
            correction = 1
            lv = base_av - control_av
            rv = base_av + control_av

            if (lv > self.max_vel):
                correction = self.max_vel / lv
                lv = lv * correction
                rv = rv * correction

            if (rv > self.max_vel):
                correction = self.max_vel / rv
                lv = lv * correction
                rv = rv * correction

        else:
            lv = rv = base_av

        self.left_motor.setVelocity(lv)
        self.right_motor.setVelocity(rv)

    def pid(self, error):
        kp = 0.6  # proportional weight (may need tuning)
        kd = 3.0  # differential weight (may need tuning)
        ki = 0.0  # integral weight (may need tuning)

        prop = error
        diff = error - self.prev_error
        self.total_error += error
        control = (kp * prop) + (ki * self.total_error) + (kd * diff)
        self.prev_error = error

        return control

    def follow_wall(self, robot_linearvelocity=0.2, set_point=0.2, right=True):
        if right:
            direction_coeff = -1
        else:
            direction_coeff = 1

        # Approaching a wall, turn
        if (min(self.prox_sensors.get_value(1),
                self.prox_sensors.get_value(2),
                self.prox_sensors.get_value(3),
                self.prox_sensors.get_value(4),
                self.prox_sensors.get_value(5),
                self.prox_sensors.get_value(6)) < set_point):
            self.set_velocity(robot_linearvelocity / 3, -0.2 * direction_coeff)
        else:
            if not right:
                wall_dist = min(self.prox_sensors.get_value(1),
                                self.prox_sensors.get_value(0))
            else:
                wall_dist = min(self.prox_sensors.get_value(7),
                                self.prox_sensors.get_value(8))

            # Running aproximately parallel to the wall
            if (wall_dist < self.prox_sensors.max_range):
                error = wall_dist - set_point
                control = self.pid(error)
                # adjust for right wall
                self.set_velocity(robot_linearvelocity, control * direction_coeff)
            else:
                # No wall, so turn
                self.set_velocity(robot_linearvelocity, 0.08 * direction_coeff)

    """def bug1(self):
        while not self.close_to_target():
            # 计算机器人朝向与目标点连线之间的角度
            angle_to_target = self.calculate_heading_angle()

            # 转向使机器人朝向目标点
            self.arc(angle_to_target, 0, 0.2)

            # 前行直到发现障碍物
            while not self.detect_obstacle():
                self.forward(0.2, 0.2)

                if self.close_to_target():
                    self.stop()
                    break
            # 如果发现障碍物，绕过障碍物
            if self.detect_obstacle():
                self.stop
                self.arc(math.pi / 2.0, 0, 0.2)
                self.follow_wall()
        # 到达目标附近
        self.stop()
        return"""