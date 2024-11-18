"""NaoController controller."""
from os.path import curdir

from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath.base import negative_one

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Keyboard, Motion, Node
from enum import Enum, unique, auto
import math
import logging
import logging.config
# from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
from simple_pid import PID
import numpy as np
import struct

cur_dir = Path(__file__).resolve().parents[2]

#initial logging
config_path = cur_dir / 'libraries' /'logging.conf'

log_path = cur_dir / "log"
log_path.mkdir(parents=True, exist_ok=True)
# logger.info(log_path)

# #set two handlers
log_file_name= log_path / "{}.log".format(Path(__file__).stem)
logging.config.fileConfig(config_path, defaults={'logfilename': str(log_file_name)})
logger = logging.getLogger()

@unique
class move_status(Enum):
    INITIAL = auto()
    PREPARE = auto()
    MOVING  = auto()
    FINISH  = auto()
    END     = auto()

class SALUTE_MOTION(Enum):
    INITIAL      = auto()
    PREPARE      = auto()
    RAISE_HAND   = auto()
    SALUTE       = auto()
    RESET        = auto()
    FINISH       = auto()
    END          = auto()

class WALK_MOTION(Enum):
    INITIAL = auto()
    PREPARE = auto()
    IS_WALKING = auto()
    WALKING = auto()
    FINISH = auto()
    END = auto()

# initial NAO robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# initial each joints
# left leg and sensor
l_hip_yaw_pitch = robot.getDevice('LHipYawPitch')
l_hip_pitch = robot.getDevice('LHipPitch')
l_hip_roll = robot.getDevice('LHipRoll')
l_knee_pitch = robot.getDevice('LKneePitch')
l_ankle_pitch = robot.getDevice('LAnklePitch')
l_ankle_roll = robot.getDevice('LAnkleRoll')

l_hip_yaw_pitch_sensor = l_hip_yaw_pitch.getPositionSensor()
l_hip_roll_sensor = l_hip_roll.getPositionSensor()
l_hip_pitch_sensor = l_hip_pitch.getPositionSensor()
l_knee_pitch_sensor = l_knee_pitch.getPositionSensor()
l_ankle_pitch_sensor = l_ankle_pitch.getPositionSensor()
l_ankle_roll_sensor = l_ankle_roll.getPositionSensor()

l_hip_yaw_pitch_v = l_hip_yaw_pitch.getVelocity()
l_hip_pitch_v = l_hip_pitch.getVelocity()
l_hip_roll_v  = l_hip_roll.getVelocity()
l_knee_pitch_v = l_knee_pitch.getVelocity()
l_ankle_pitch_v = l_ankle_pitch.getVelocity()
l_ankle_roll_v = l_ankle_roll.getVelocity()

# right leg and sensor
r_hip_yaw_pitch = robot.getDevice('RHipYawPitch')
r_hip_roll = robot.getDevice('RHipRoll')
r_hip_pitch = robot.getDevice('RHipPitch')
r_knee_pitch = robot.getDevice('RKneePitch')
r_ankle_pitch = robot.getDevice('RAnklePitch')
r_ankle_roll = robot.getDevice('RAnkleRoll')

r_hip_yaw_pitch_sensor = r_hip_yaw_pitch.getPositionSensor()
r_hip_roll_sensor = r_hip_roll.getPositionSensor()
r_hip_pitch_sensor = r_hip_pitch.getPositionSensor()
r_knee_pitch_sensor = r_knee_pitch.getPositionSensor()
r_ankle_pitch_sensor = r_ankle_pitch.getPositionSensor()
r_ankle_roll_sensor = r_ankle_roll.getPositionSensor()

r_hip_yaw_pitch_v = r_hip_yaw_pitch.getVelocity()
r_hip_pitch_v   = r_hip_pitch.getVelocity()
r_hip_roll_v    = r_hip_roll.getVelocity()
r_knee_pitch_v  = r_knee_pitch.getVelocity()
r_ankle_pitch_v = r_ankle_pitch.getVelocity()
r_ankle_roll_v  = r_ankle_roll.getVelocity()

# left arm and sensor
l_shoulder_pitch = robot.getDevice('LShoulderPitch')
l_shoulder_roll = robot.getDevice('LShoulderRoll')
l_elbow_yaw = robot.getDevice('LElbowYaw')
l_elbow_roll = robot.getDevice('LElbowRoll')
# l_wrist_yaw = robot.getDevice('WristYaw')
# l_hand = robot.getDevice('LHand')

l_shoulder_pitch_sensor = l_shoulder_pitch.getPositionSensor()
l_shoulder_roll_sensor  = l_shoulder_roll.getPositionSensor()
l_elbow_yaw_sensor      = l_elbow_yaw.getPositionSensor()
l_elbow_roll_sensor     = l_elbow_roll.getPositionSensor()

l_shoulder_pitch_v = l_shoulder_pitch.getVelocity()
l_shoulder_roll_v  = l_shoulder_roll.getVelocity()
l_elbow_yaw_v      = l_elbow_yaw.getVelocity()
l_elbow_roll_v     = l_elbow_roll.getVelocity()

# right arm and sensor
r_shoulder_pitch = robot.getDevice('RShoulderPitch')
r_shoulder_roll = robot.getDevice('RShoulderRoll')
r_elbow_yaw = robot.getDevice('RElbowYaw')
r_elbow_roll = robot.getDevice('RElbowRoll')
# r_wrist_yaw = robot.getDevice('WristYaw')
# r_hand = robot.getDevice('LHand')

r_shoulder_pitch_sensor = r_shoulder_pitch.getPositionSensor()
r_shoulder_roll_sensor  = r_shoulder_roll.getPositionSensor()
r_elbow_yaw_sensor      = r_elbow_yaw.getPositionSensor()
r_elbow_roll_sensor     = r_elbow_roll.getPositionSensor()

r_shoulder_pitch_v = r_shoulder_pitch.getVelocity()
r_shoulder_roll_v  = r_shoulder_roll.getVelocity()
r_elbow_yaw_v      = r_elbow_yaw.getVelocity()
r_elbow_roll_v     = r_elbow_roll.getVelocity()

# Enable
l_hip_yaw_pitch_sensor.enable(timestep)
l_hip_roll_sensor.enable(timestep)
l_hip_pitch_sensor.enable(timestep)
l_knee_pitch_sensor.enable(timestep)
l_ankle_pitch_sensor.enable(timestep)
l_ankle_roll_sensor.enable(timestep)

r_hip_yaw_pitch_sensor.enable(timestep)
r_hip_roll_sensor.enable(timestep)
r_hip_pitch_sensor.enable(timestep)
r_knee_pitch_sensor.enable(timestep)
r_ankle_pitch_sensor.enable(timestep)
r_ankle_roll_sensor.enable(timestep)

l_shoulder_roll_sensor.enable(timestep)
l_shoulder_pitch_sensor.enable(timestep)
l_elbow_yaw_sensor.enable(timestep)
l_elbow_roll_sensor.enable(timestep)

r_shoulder_roll_sensor.enable(timestep)
r_shoulder_pitch_sensor.enable(timestep)
r_elbow_yaw_sensor.enable(timestep)
r_elbow_roll_sensor.enable(timestep)

# keyboard
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

class move:
    def __init__(self):
        self.moved = False

    def set_position(self):
        self.moved = True

class left_shoulder(move):
    def __init__(self):
        move.__init__(self)
    __pitch_status = move_status.INITIAL
    __roll_status  = move_status.INITIAL
    __limitation = 0.000001
    __max_shoulder_pitch_radian = 2.0857
    __min_shoulder_pitch_radian = -2.0857

    __max_shoulder_roll_radian = 1.3265
    __min_shoulder_roll_radian = -0.3142

    __pitch_previous_target = None
    __roll_previous_target = None
    def get_velocity(self, joints):
        velocity = None
        if joints == "LShoulderPitch":
            velocity = l_shoulder_pitch_v
        elif joints == "LShoulderRoll":
            velocity = l_shoulder_roll_v
        return velocity

    def getJointsStatus(self, joints):
        if joints == "LShoulderPitch":
            return self.__pitch_status
        elif joints == "LShoulderRoll":
            return self.__roll_status
        else:
            logger.info(f"{joints} is not finding!")
            return None
    def position_is_arrive(self, joints="", targets=None):
        if joints == "":
            logger.info("You do not set any joints")
            return

        if targets is None:
            logger.info("You do not set any positions")
            return

        if joints == "LShoulderPitch":
            if targets is self.__pitch_previous_target:
                position = l_shoulder_pitch_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        elif joints == "LShoulderRoll":
            if targets is self.__roll_previous_target:
                position = l_shoulder_roll_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        else:
            logger.info("Can not find any joints, please set again in position_is_arrive")
            return

    def set_position(self, joints="", positions=None):
        if joints == "":
            logger.info("You do not set any joints")
            return

        if positions is None:
            logger.info("You do not set any positions")
            return

        if joints == "LShoulderPitch":
            if positions > self.__max_shoulder_pitch_radian or positions < self.__min_shoulder_pitch_radian:
                logger.info(f"The position you set is {positions}, is out of range in [{self.__max_shoulder_pitch_radian}, {self.__min_shoulder_pitch_radian}]")
                return
            # if self.__pitch_previous_target is positions and self.__pitch_status is move_status.INITIAL:
            #     return
            if self.__pitch_status is move_status.INITIAL:
                logger.info(f"{joints} is in Initial")
                self.__pitch_previous_target = positions
                self.__pitch_status = move_status.PREPARE
            elif self.__pitch_status is move_status.PREPARE:
                if positions is self.__pitch_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    # if l_elbow_roll_sensor.getValue() < -1:
                    #     if l_shoulder_roll_sensor.getValue() >= 1:
                    #         self.__pitch_status = move_status.MOVING
                    #     else:
                    #         return
                    # else:
                    self.__pitch_status = move_status.MOVING
                else:
                    return
            elif self.__pitch_status is move_status.MOVING:
                if positions is self.__pitch_previous_target:
                    logger.info(f"{joints} is in Moving")
                    l_shoulder_pitch.setPosition(positions)
                    # logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
                    if self.position_is_arrive(joints, positions):
                        self.__pitch_status = move_status.FINISH
                else:
                    return
            elif self.__pitch_status is move_status.FINISH:
                if positions is self.__pitch_previous_target:
                    logger.info(f"{joints} is in Finish")
                    if self.position_is_arrive(joints, positions):
                        self.__pitch_status = move_status.END
                else:
                    return
            elif self.__pitch_status is move_status.END:
                logger.info(f"{joints} is in End")
                if positions is not self.__pitch_previous_target:
                    self.__pitch_status = move_status.INITIAL
                else:
                    return
            else:
                logger.info(f"{joints} is in error status!")
                return
        elif joints == "LShoulderRoll":
             if positions > self.__max_shoulder_roll_radian or positions < self.__min_shoulder_roll_radian:
                    logger.info(f"The position you set is {positions}, is out of range in [{self.__max_shoulder_roll_radian}, {self.__min_shoulder_roll_radian}]")
                    return
             # if self.__roll_previous_target is positions and self.__roll_status is move_status.INITIAL:
             #    return
             if self.__roll_status is move_status.INITIAL:
                    logger.info(f"{joints} is in Initial")
                    self.__roll_previous_target = positions
                    self.__roll_status = move_status.PREPARE
             elif self.__roll_status is move_status.PREPARE:
                 if positions is self.__roll_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    # if l_shoulder_pitch_sensor.getValue() >= 1:
                    #     if l_elbow_roll_sensor.getValue() > -1:
                    #         self.__roll_status = move_status.MOVING
                    #     else:
                    #         return
                    # else:
                    self.__roll_status = move_status.MOVING
                 else:
                     return
             elif self.__roll_status is move_status.MOVING:
                    if positions is self.__roll_previous_target:
                        logger.info(f"{joints} is in Moving")
                        l_shoulder_roll.setPosition(positions)
                        # logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
                        if self.position_is_arrive(joints, positions):
                            self.__roll_status = move_status.FINISH
                    else:
                        return
             elif self.__roll_status is move_status.FINISH:
                    if positions is self.__roll_previous_target:
                        logger.info(f"{joints} is in Finish")
                        if self.position_is_arrive(joints, positions):
                            self.__roll_status = move_status.END
                    else:
                        return
             elif self.__roll_status is move_status.END:
                    logger.info(f"{joints} is in End")
                    if positions is not self.__roll_previous_target:
                        self.__roll_status = move_status.INITIAL
                    else:
                        return
             else:
                    logger.info(f"{joints} is in error status!")
                    return
        else:
            logger.info("Can not find any joints, please set again in set_position")
            return
        # logger.info("End moved is", self.moved)

class left_elbow(move):
    def __init__(self):
        move.__init__(self)
    __yaw_status = move_status.INITIAL
    __roll_status = move_status.INITIAL
    __limitation = 0.000001
    __max_elbow_yaw_radian = 2.0857
    __min_elbow_yaw_radian = -2.0857

    __max_elbow_roll_radian = -0.0349
    __min_elbow_roll_radian = -1.5446

    __yaw_previous_target = None
    __roll_previous_target = None
    def getJointsStatus(self, joints):
        if joints == "LElbowYaw":
            return self.__yaw_status
        elif joints == "LElbowRoll":
            return self.__roll_status
        else:
            logger.info(f"{joints} is not finding!")
            return None

    def position_is_arrive(self, joints="", targets=None):
        if joints == "":
            logger.info("You do not set any joints")
            return

        if targets is None:
            logger.info("You do not set any positions")
            return

        if joints == "LElbowYaw":
            if targets is self.__yaw_previous_target:
                position = l_elbow_yaw_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        elif joints == 'LElbowRoll':
            if targets is self.__roll_previous_target:
                position = l_elbow_roll_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        else:
            logger.info("Can not find any joints, please set again in position_is_arrive")
            return

    def set_position(self, joints="", positions=None):
        if joints == "":
            logger.info("You do not set any joints")
            return

        if positions is None:
            logger.info("You do not set any positions")
            return

        if joints == "LElbowYaw":
            if positions > self.__max_elbow_yaw_radian or positions < self.__min_elbow_yaw_radian:
                logger.info(f"The position you set is {positions}, is out of range in [{self.__max_elbow_yaw_radian}, {self.__min_elbow_yaw_radian}]")
                return
            # if self.__yaw_previous_target is positions and self.__yaw_status is move_status.INITIAL:
            #     return
            if self.__yaw_status is move_status.INITIAL:
                logger.info(f"{joints} is in Initial")
                self.__yaw_previous_target = positions
                self.__yaw_status = move_status.MOVING
            elif self.__yaw_status is move_status.MOVING:
                if positions is self.__yaw_previous_target:
                    logger.info(f"{joints} is in Moving")
                    l_elbow_yaw.setPosition(positions)
                    if self.position_is_arrive(joints, positions):
                        self.__yaw_status = move_status.FINISH
                else:
                    return
            elif self.__yaw_status is move_status.FINISH:
                if positions is self.__yaw_previous_target:
                    logger.info(f"{joints} is in Finish")
                    if self.position_is_arrive(joints, positions):
                        self.__yaw_status = move_status.END
                else:
                    return
            elif self.__yaw_status is move_status.END:
                logger.info(f"{joints} is in End")
                if positions is not self.__yaw_previous_target:
                    self.__yaw_status = move_status.INITIAL
                    return
                else:
                    return
            else:
                logger.info(f"{joints} is in error status!")
                return
        elif joints == 'LElbowRoll':
            if positions > self.__max_elbow_roll_radian or positions < self.__min_elbow_roll_radian:
                logger.info(f"The position you set is {positions}, is out of range in [{self.__max_elbow_roll_radian}, {self.__min_elbow_roll_radian}]")
                return
            # if self.__roll_previous_target is positions and self.__roll_status is move_status.INITIAL:
            #     return
            if self.__roll_status is move_status.INITIAL:
                logger.info(f"{joints} is in Initial")
                self.__roll_previous_target = positions
                self.__roll_status = move_status.PREPARE
            elif self.__roll_status is move_status.PREPARE:
                logger.info(f"{joints} is in Prepare")
                self.__roll_status = move_status.MOVING
            elif self.__roll_status is move_status.MOVING:
                if positions is self.__roll_previous_target:
                    logger.info(f"{joints} is in Moving")
                    l_elbow_roll.setPosition(positions)
                    if self.position_is_arrive(joints, positions):
                        self.__roll_status = move_status.FINISH
                else:
                    return
            elif self.__roll_status is move_status.FINISH:
                if positions is self.__roll_previous_target:
                    logger.info(f"{joints} is in Finish")
                    if self.position_is_arrive(joints, positions):
                        self.__roll_status = move_status.END
                else:
                    return
            elif self.__roll_status is move_status.END:
                if positions is not self.__roll_previous_target:
                    logger.info(f"{joints} is in End")
                    self.__roll_status = move_status.INITIAL
                    return
                else:
                    return
            else:
                logger.info(f"{joints} is in error status!")
                return
        else:
            logger.info("Can not find any joints, please set again in set_position")
            return

class right_shoulder(move):
    def __init__(self):
        move.__init__(self)
    __pitch_status = move_status.INITIAL
    __roll_status  = move_status.INITIAL
    __limitation = 0.000001
    __max_shoulder_pitch_radian = 2.0857
    __min_shoulder_pitch_radian = -2.0857

    __max_shoulder_roll_radian = 0.3142
    __min_shoulder_roll_radian = -1.3265

    __pitch_previous_target = None
    __roll_previous_target = None

    def getJointsStatus(self, joints):
        if joints == "RShoulderPitch":
            return self.__pitch_status
        elif joints == "RShoulderRoll":
            return self.__roll_status
        else:
            logger.info(f"{joints} is not finding!")
            return None
    def position_is_arrive(self, joints="", targets=None):
        if joints == "":
            logger.info("You do not set any joints")
            return

        if targets is None:
            logger.info("You do not set any positions")
            return

        if joints == "RShoulderPitch":
            if targets is self.__pitch_previous_target:
                position = r_shoulder_pitch_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    return True
                else:
                    return False
            else:
                return True
        elif joints == "RShoulderRoll":
            if targets is self.__roll_previous_target:
                position = r_shoulder_roll_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        else:
            logger.info("Can not find any joints, please set again in position_is_arrive")
            return

    def set_position(self, joints="", positions=None):
        if joints == "":
            logger.info("You do not set any joints")
            return

        if positions is None:
            logger.info("You do not set any positions")
            return

        if joints == "RShoulderPitch":
            if positions > self.__max_shoulder_pitch_radian or positions < self.__min_shoulder_pitch_radian:
                logger.info(f"The position you set is {positions}, is out of range in [{self.__max_shoulder_pitch_radian}, {self.__min_shoulder_pitch_radian}]")
                return
            # if self.__pitch_previous_target is positions and self.__pitch_status is move_status.INITIAL:
            #     return
            if self.__pitch_status is move_status.INITIAL:
                logger.info(f"{joints} is in Initial")
                self.__pitch_previous_target = positions
                self.__pitch_status = move_status.PREPARE
            elif self.__pitch_status is move_status.PREPARE:
                if positions is self.__pitch_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    # if l_elbow_roll_sensor.getValue() < -1:
                    #     if l_shoulder_roll_sensor.getValue() >= 1:
                    #         self.__pitch_status = move_status.MOVING
                    #     else:
                    #         return
                    # else:
                    self.__pitch_status = move_status.MOVING
                else:
                    return
            elif self.__pitch_status is move_status.MOVING:
                if positions is self.__pitch_previous_target:
                    logger.info(f"{joints} is in Moving")
                    r_shoulder_pitch.setPosition(positions)
                    if self.position_is_arrive(joints, positions):
                        self.__pitch_status = move_status.FINISH
                else:
                    return
            elif self.__pitch_status is move_status.FINISH:
                if positions is self.__pitch_previous_target:
                    logger.info(f"{joints} is in Finish")
                    if self.position_is_arrive(joints, positions):
                        self.__pitch_status = move_status.END
                else:
                    return
            elif self.__pitch_status is move_status.END:
                logger.info(f"{joints} is in End")
                if positions is not self.__pitch_previous_target:
                    self.__pitch_status = move_status.INITIAL
                else:
                    return
            else:
                logger.info(f"{joints} is in error status!")
                return
        elif joints == "RShoulderRoll":
             if positions > self.__max_shoulder_roll_radian or positions < self.__min_shoulder_roll_radian:
                    logger.info(f"The position you set is {positions}, is out of range in [{self.__max_shoulder_roll_radian}, {self.__min_shoulder_roll_radian}]")
                    return
             # if self.__roll_previous_target is positions and self.__roll_status is move_status.INITIAL:
             #    return
             if self.__roll_status is move_status.INITIAL:
                    logger.info(f"{joints} is in Initial")
                    self.__roll_previous_target = positions
                    self.__roll_status = move_status.PREPARE
             elif self.__roll_status is move_status.PREPARE:
                 if positions is self.__roll_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    self.__roll_status = move_status.MOVING
                 else:
                     return
             elif self.__roll_status is move_status.MOVING:
                    if positions is self.__roll_previous_target:
                        logger.info(f"{joints} is in Moving")
                        r_shoulder_roll.setPosition(positions)
                        if self.position_is_arrive(joints, positions):
                            self.__roll_status = move_status.FINISH
                    else:
                        return
             elif self.__roll_status is move_status.FINISH:
                    if positions is self.__roll_previous_target:
                        logger.info(f"{joints} is in Finish")
                        if self.position_is_arrive(joints, positions):
                            self.__roll_status = move_status.END
                    else:
                        return
             elif self.__roll_status is move_status.END:
                    logger.info(f"{joints} is in End")
                    if positions is not self.__roll_previous_target:
                        self.__roll_status = move_status.INITIAL
                    else:
                        return
             else:
                    logger.info(f"{joints} is in error status!")
                    return
        else:
            logger.info("Can not find any joints, please set again in set_position")
            return
        # logger.info("End moved is", self.moved)

class right_elbow(move):
    def __init__(self):
        move.__init__(self)
    __yaw_status = move_status.INITIAL
    __roll_status = move_status.INITIAL
    __limitation = 0.000001
    __max_elbow_yaw_radian = 2.0857
    __min_elbow_yaw_radian = -2.0857

    __max_elbow_roll_radian = 1.5446
    __min_elbow_roll_radian = 0.0349

    __yaw_previous_target = None
    __roll_previous_target = None
    def getJointsStatus(self, joints):
        if joints == "RElbowYaw":
            return self.__yaw_status
        elif joints == "RElbowRoll":
            return self.__roll_status
        else:
            logger.info(f"{joints} is not finding!")
            return None

    def position_is_arrive(self, joints="", targets=None):
        if joints == "":
            logger.info("You do not set any joints")
            return

        if targets is None:
            logger.info("You do not set any positions")
            return

        if joints == "RElbowYaw":
            if targets is self.__yaw_previous_target:
                position = r_elbow_yaw_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        elif joints == 'RElbowRoll':
            if targets is self.__roll_previous_target:
                position = r_elbow_roll_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        else:
            logger.info("Can not find any joints, please set again in position_is_arrive")
            return

    def set_position(self, joints="", positions=None):
        if joints == "":
            logger.info("You do not set any joints")
            return

        if positions is None:
            logger.info("You do not set any positions")
            return

        if joints == "RElbowYaw":
            if positions > self.__max_elbow_yaw_radian or positions < self.__min_elbow_yaw_radian:
                logger.info(f"The position you set is {positions}, is out of range in [{self.__max_elbow_yaw_radian}, {self.__min_elbow_yaw_radian}]")
                return
            # if self.__yaw_previous_target is positions and self.__yaw_status is move_status.INITIAL:
            #     return
            if self.__yaw_status is move_status.INITIAL:
                logger.info(f"{joints} is in Initial")
                self.__yaw_previous_target = positions
                self.__yaw_status = move_status.MOVING
            elif self.__yaw_status is move_status.MOVING:
                if positions is self.__yaw_previous_target:
                    logger.info(f"{joints} is in Moving")
                    r_elbow_yaw.setPosition(positions)
                    if self.position_is_arrive(joints, positions):
                        self.__yaw_status = move_status.FINISH
                else:
                    return
            elif self.__yaw_status is move_status.FINISH:
                if positions is self.__yaw_previous_target:
                    logger.info(f"{joints} is in Finish")
                    if self.position_is_arrive(joints, positions):
                        self.__yaw_status = move_status.END
                else:
                    return
            elif self.__yaw_status is move_status.END:
                logger.info(f"{joints} is in End")
                if positions is not self.__yaw_previous_target:
                    self.__yaw_status = move_status.INITIAL
                    return
                else:
                    return
            else:
                logger.info(f"{joints} is in error status!")
                return
        elif joints == 'RElbowRoll':
            if positions > self.__max_elbow_roll_radian or positions < self.__min_elbow_roll_radian:
                logger.info(f"The position you set is {positions}, is out of range in [{self.__max_elbow_roll_radian}, {self.__min_elbow_roll_radian}]")
                return
            # if self.__roll_previous_target is positions and self.__roll_status is move_status.INITIAL:
            #     return
            if self.__roll_status is move_status.INITIAL:
                logger.info(f"{joints} is in Initial")
                self.__roll_previous_target = positions
                self.__roll_status = move_status.PREPARE
            elif self.__roll_status is move_status.PREPARE:
                logger.info(f"{joints} is in Prepare")
                self.__roll_status = move_status.MOVING
            elif self.__roll_status is move_status.MOVING:
                if positions is self.__roll_previous_target:
                    logger.info(f"{joints} is in Moving")
                    r_elbow_roll.setPosition(positions)
                    if self.position_is_arrive(joints, positions):
                        self.__roll_status = move_status.FINISH
                else:
                    return
            elif self.__roll_status is move_status.FINISH:
                if positions is self.__roll_previous_target:
                    logger.info(f"{joints} is in Finish")
                    if self.position_is_arrive(joints, positions):
                        self.__roll_status = move_status.END
                else:
                    return
            elif self.__roll_status is move_status.END:
                if positions is not self.__roll_previous_target:
                    logger.info(f"{joints} is in End")
                    self.__roll_status = move_status.INITIAL
                    return
                else:
                    return
            else:
                logger.info(f"{joints} is in error status!")
                return
        else:
            logger.info("Can not find any joints, please set again in set_position")
            return

class left_leg(move):
    def __init__(self):
        move.__init__(self)
    __hip_yaw_pitch_status = move_status.INITIAL
    __hip_pitch_status     = move_status.INITIAL
    __hip_roll_status      = move_status.INITIAL
    __knee_pitch_status    = move_status.INITIAL
    __ankle_pitch_status   = move_status.INITIAL
    __ankle_roll_status    = move_status.INITIAL

    __limitation = 0.000001

    __max_hip_yaw_pitch_radian = 0.740810
    __min_hip_yaw_pitch_radian = -1.145303

    __max_hip_pitch_radian = 0.484090
    __min_hip_pitch_radian = -1.535889

    __max_hip_roll_radian = 0.790477
    __min_hip_roll_radian = -0.379472

    __max_knee_pitch_radian = 2.112528
    __min_knee_pitch_radian = -0.092346

    __max_ankle_pitch_radian = 0.922747
    __min_ankle_pitch_radian = -1.189516

    __max_ankle_roll_radian = 0.769001
    __min_ankle_roll_radian = -0.397880

    __hip_yaw_pitch_previous_target = None
    __hip_pitch_previous_target     = None
    __hip_roll_previous_target      = None
    __knee_pitch_previous_target    = None
    __ankle_pitch_previous_target   = None
    __ankle_roll_previous_target    = None

    def get_velocity(self, joints):
        velocity = None
        if joints == "LHipPitch":
            velocity = l_hip_pitch_v
        elif joints == "LHipRoll":
            velocity = l_hip_roll_v
        elif joints == "LKneePitch":
            velocity = l_knee_pitch_v
        elif joints == "LAnklePitch":
            velocity = l_ankle_pitch_v
        elif joints == "LAnkleRoll":
            velocity = l_ankle_roll_v
        elif joints == "LHipYawPitch":
            velocity = l_hip_yaw_pitch_v
        else:
            logger.info(f"Can not find any joints, please set again in get_velocity")
            return
        return velocity

    def getJointsStatus(self, joints):
        if joints == "LHipPitch":
            return self.__hip_pitch_status
        elif joints == "LHipRoll":
            return self.__hip_roll_status
        elif joints == "LKneePitch":
            return self.__knee_pitch_status
        elif joints == "LAnklePitch":
            return self.__ankle_pitch_status
        elif joints == "LAnkleRoll":
            return self.__ankle_roll_status
        elif joints == "LHipYawPitch":
            return self.__hip_yaw_pitch_status
        else:
            logger.info(f"{joints} is not finding!")
            return None
    def position_is_arrive(self, joints="", targets=None):
        if joints == "":
            logger.info("You do not set any joints")
            return

        if targets is None:
            logger.info("You do not set any positions")
            return

        if joints == "LHipPitch":
            if targets is self.__hip_pitch_previous_target:
                position = l_hip_pitch_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        elif joints == "LHipRoll":
            if targets is self.__hip_roll_previous_target:
                position = l_hip_roll_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        elif joints == "LKneePitch":
            if targets is self.__knee_pitch_previous_target:
                position = l_knee_pitch_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        elif joints == "LAnklePitch":
            if targets is self.__ankle_pitch_previous_target:
                position = l_ankle_pitch_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        elif joints == "LAnkleRoll":
            if targets is self.__ankle_roll_previous_target:
                position = l_ankle_roll_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        elif joints == "LHipYawPitch":
            if targets is self.__hip_yaw_pitch_previous_target:
                position = l_hip_yaw_pitch_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        else:
            logger.info("Can not find any joints, please set again in position_is_arrive")
            return

    def set_position(self, joints="", positions=None):
        if joints == "":
            logger.info("You do not set any joints")
            return

        if positions is None:
            logger.info("You do not set any positions")
            return

        if joints == "LHipPitch":
            if positions > self.__max_hip_pitch_radian or positions < self.__min_hip_pitch_radian:
                logger.info(f"The position you set is {positions}, is out of range in [{self.__max_hip_pitch_radian}, {self.__min_hip_pitch_radian}]")
                return
            if self.__hip_pitch_status is move_status.INITIAL:
                logger.info(f"{joints} is in Initial")
                self.__hip_pitch_previous_target = positions
                self.__hip_pitch_status = move_status.PREPARE
            elif self.__hip_pitch_status is move_status.PREPARE:
                if positions is self.__hip_pitch_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    self.__hip_pitch_status = move_status.MOVING
                else:
                    return
            elif self.__hip_pitch_status is move_status.MOVING:
                if positions is self.__hip_pitch_previous_target:
                    logger.info(f"{joints} is in Moving")
                    l_hip_pitch.setPosition(positions)
                    # logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
                    if self.position_is_arrive(joints, positions):
                        self.__hip_pitch_status = move_status.FINISH
                else:
                    return
            elif self.__hip_pitch_status is move_status.FINISH:
                if positions is self.__hip_pitch_previous_target:
                    logger.info(f"{joints} is in Finish")
                    if self.position_is_arrive(joints, positions):
                        self.__hip_pitch_status = move_status.END
                else:
                    return
            elif self.__hip_pitch_status is move_status.END:
                logger.info(f"{joints} is in End")
                if positions is not self.__hip_pitch_previous_target:
                    self.__hip_pitch_status = move_status.INITIAL
                else:
                    return
            else:
                logger.info(f"{joints} is in error status!")
                return
        elif joints == "LHipRoll":
             if positions > self.__max_hip_roll_radian or positions < self.__min_hip_roll_radian:
                    logger.info(f"The position you set is {positions}, is out of range in [{self.__max_hip_roll_radian}, {self.__min_hip_roll_radian}]")
                    return
             if self.__hip_roll_status is move_status.INITIAL:
                    logger.info(f"{joints} is in Initial")
                    self.__hip_roll_previous_target = positions
                    self.__hip_roll_status = move_status.PREPARE
             elif self.__hip_roll_status is move_status.PREPARE:
                 if positions is self.__hip_roll_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    self.__hip_roll_status = move_status.MOVING
                 else:
                     return
             elif self.__hip_roll_status is move_status.MOVING:
                    if positions is self.__hip_roll_previous_target:
                        logger.info(f"{joints} is in Moving")
                        l_hip_roll.setPosition(positions)
                        # logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
                        if self.position_is_arrive(joints, positions):
                            self.__hip_roll_status = move_status.FINISH
                    else:
                        return
             elif self.__hip_roll_status is move_status.FINISH:
                    if positions is self.__hip_roll_previous_target:
                        logger.info(f"{joints} is in Finish")
                        if self.position_is_arrive(joints, positions):
                            self.__hip_roll_status = move_status.END
                    else:
                        return
             elif self.__hip_roll_status is move_status.END:
                    logger.info(f"{joints} is in End")
                    if positions is not self.__hip_roll_previous_target:
                        self.__hip_roll_status = move_status.INITIAL
                    else:
                        return
             else:
                    logger.info(f"{joints} is in error status!")
                    return
        elif joints == "LKneePitch":
             if positions > self.__max_knee_pitch_radian or positions < self.__min_knee_pitch_radian:
                    logger.info(f"The position you set is {positions}, is out of range in [{self.__max_knee_pitch_radian}, {self.__min_knee_pitch_radian}]")
                    return
             if self.__knee_pitch_status is move_status.INITIAL:
                    logger.info(f"{joints} is in Initial")
                    self.__knee_pitch_previous_target = positions
                    self.__knee_pitch_status = move_status.PREPARE
             elif self.__knee_pitch_status is move_status.PREPARE:
                 if positions is self.__knee_pitch_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    self.__knee_pitch_status = move_status.MOVING
                 else:
                     return
             elif self.__knee_pitch_status is move_status.MOVING:
                    if positions is self.__knee_pitch_previous_target:
                        logger.info(f"{joints} is in Moving")
                        l_knee_pitch.setPosition(positions)
                        # logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
                        if self.position_is_arrive(joints, positions):
                            self.__knee_pitch_status = move_status.FINISH
                    else:
                        return
             elif self.__knee_pitch_status is move_status.FINISH:
                    if positions is self.__knee_pitch_previous_target:
                        logger.info(f"{joints} is in Finish")
                        if self.position_is_arrive(joints, positions):
                            self.__knee_pitch_status = move_status.END
                    else:
                        return
             elif self.__knee_pitch_status is move_status.END:
                    logger.info(f"{joints} is in End")
                    if positions is not self.__knee_pitch_previous_target:
                        self.__knee_pitch_status = move_status.INITIAL
                    else:
                        return
             else:
                    logger.info(f"{joints} is in error status!")
                    return
        elif joints == "LAnklePitch":
             if positions > self.__max_knee_pitch_radian or positions < self.__min_knee_pitch_radian:
                    logger.info(f"The position you set is {positions}, is out of range in [{self.__max_knee_pitch_radian}, {self.__min_knee_pitch_radian}]")
                    return
             if self.__ankle_pitch_status is move_status.INITIAL:
                    logger.info(f"{joints} is in Initial")
                    self.__ankle_pitch_previous_target = positions
                    self.__ankle_pitch_status = move_status.PREPARE
             elif self.__ankle_pitch_status is move_status.PREPARE:
                 if positions is self.__ankle_pitch_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    self.__ankle_pitch_status = move_status.MOVING
                 else:
                     return
             elif self.__ankle_pitch_status is move_status.MOVING:
                    if positions is self.__ankle_pitch_previous_target:
                        logger.info(f"{joints} is in Moving")
                        l_ankle_pitch.setPosition(positions)
                        # logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
                        if self.position_is_arrive(joints, positions):
                            self.__ankle_pitch_status = move_status.FINISH
                    else:
                        return
             elif self.__ankle_pitch_status is move_status.FINISH:
                    if positions is self.__ankle_pitch_previous_target:
                        logger.info(f"{joints} is in Finish")
                        if self.position_is_arrive(joints, positions):
                            self.__ankle_pitch_status = move_status.END
                    else:
                        return
             elif self.__ankle_pitch_status is move_status.END:
                    logger.info(f"{joints} is in End")
                    if positions is not self.__ankle_pitch_previous_target:
                        self.__ankle_pitch_status = move_status.INITIAL
                    else:
                        return
             else:
                    logger.info(f"{joints} is in error status!")
                    return
        elif joints == "LAnkleRoll":
             if positions > self.__max_ankle_roll_radian or positions < self.__min_ankle_roll_radian:
                    logger.info(f"The position you set is {positions}, is out of range in [{self.__max_ankle_roll_radian}, {self.__min_ankle_roll_radian}]")
                    return
             if self.__ankle_roll_status is move_status.INITIAL:
                    logger.info(f"{joints} is in Initial")
                    self.__ankle_roll_previous_target = positions
                    self.__ankle_roll_status = move_status.PREPARE
             elif self.__ankle_roll_status is move_status.PREPARE:
                 if positions is self.__ankle_roll_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    self.__ankle_roll_status = move_status.MOVING
                 else:
                     return
             elif self.__ankle_roll_status is move_status.MOVING:
                    if positions is self.__ankle_roll_previous_target:
                        logger.info(f"{joints} is in Moving")
                        l_ankle_roll.setPosition(positions)
                        # logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
                        if self.position_is_arrive(joints, positions):
                            self.__ankle_roll_status = move_status.FINISH
                    else:
                        return
             elif self.__ankle_roll_status is move_status.FINISH:
                    if positions is self.__ankle_roll_previous_target:
                        logger.info(f"{joints} is in Finish")
                        if self.position_is_arrive(joints, positions):
                            self.__ankle_roll_status = move_status.END
                    else:
                        return
             elif self.__ankle_roll_status is move_status.END:
                    logger.info(f"{joints} is in End")
                    if positions is not self.__ankle_roll_previous_target:
                        self.__ankle_roll_status = move_status.INITIAL
                    else:
                        return
             else:
                    logger.info(f"{joints} is in error status!")
                    return
        elif joints == "LHipYawPitch":
             if positions > self.__max_hip_yaw_pitch_radian or positions < self.__min_hip_yaw_pitch_radian:
                    logger.info(f"The position you set is {positions}, is out of range in [{self.__max_hip_yaw_pitch_radian}, {self.__min_hip_yaw_pitch_radian}]")
                    return
             if self.__hip_yaw_pitch_status is move_status.INITIAL:
                    logger.info(f"{joints} is in Initial")
                    self.__hip_yaw_pitch_previous_target = positions
                    self.__hip_yaw_pitch_status = move_status.PREPARE
             elif self.__hip_yaw_pitch_status is move_status.PREPARE:
                 if positions is self.__hip_yaw_pitch_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    self.__hip_yaw_pitch_status = move_status.MOVING
                 else:
                     return
             elif self.__hip_yaw_pitch_status is move_status.MOVING:
                    if positions is self.__hip_yaw_pitch_previous_target:
                        logger.info(f"{joints} is in Moving")
                        l_hip_yaw_pitch.setPosition(positions)
                        # logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
                        if self.position_is_arrive(joints, positions):
                            self.__hip_yaw_pitch_status = move_status.FINISH
                    else:
                        return
             elif self.__hip_yaw_pitch_status is move_status.FINISH:
                    if positions is self.__hip_yaw_pitch_previous_target:
                        logger.info(f"{joints} is in Finish")
                        if self.position_is_arrive(joints, positions):
                            self.__hip_yaw_pitch_status = move_status.END
                    else:
                        return
             elif self.__hip_yaw_pitch_status is move_status.END:
                    logger.info(f"{joints} is in End")
                    if positions is not self.__hip_yaw_pitch_previous_target:
                        self.__hip_yaw_pitch_status = move_status.INITIAL
                    else:
                        return
             else:
                    logger.info(f"{joints} is in error status!")
                    return
        else:
            logger.info("Can not find any joints, please set again in set_position")
            return
        # logger.info("End moved is", self.moved)

class right_leg(move):
    def __init__(self):
        move.__init__(self)

    __hip_yaw_pitch_status = move_status.INITIAL
    __hip_pitch_status = move_status.INITIAL
    __hip_roll_status = move_status.INITIAL
    __knee_pitch_status = move_status.INITIAL
    __ankle_pitch_status = move_status.INITIAL
    __ankle_roll_status = move_status.INITIAL

    __limitation = 0.00001

    __max_hip_yaw_pitch_radian = 0.740810
    __min_hip_yaw_pitch_radian = -1.145303

    __max_hip_pitch_radian = 0.484090
    __min_hip_pitch_radian = -1.535889

    __max_hip_roll_radian = 0.379472
    __min_hip_roll_radian = -0.790477

    __max_knee_pitch_radian = 2.120198
    __min_knee_pitch_radian = -0.092346

    __max_ankle_pitch_radian = 0.932056
    __min_ankle_pitch_radian = -1.186448

    __max_ankle_roll_radian = 0.397935
    __min_ankle_roll_radian = -0.768992

    __hip_yaw_pitch_previous_target = None
    __hip_pitch_previous_target     = None
    __hip_roll_previous_target      = None
    __knee_pitch_previous_target    = None
    __ankle_pitch_previous_target   = None
    __ankle_roll_previous_target    = None

    def get_velocity(self, joints):
        velocity = None
        if joints == "RHipPitch":
            velocity = r_hip_pitch_v
        elif joints == "RHipRoll":
            velocity = r_hip_roll_v
        elif joints == "RKneePitch":
            velocity = r_knee_pitch_v
        elif joints == "RAnklePitch":
            velocity = r_ankle_pitch_v
        elif joints == "RAnkleRoll":
            velocity = r_ankle_roll_v
        elif joints == "RHipYawPitch":
            velocity = r_hip_yaw_pitch_v
        else:
            logger.info(f"Can not find any joints, please set again in get_velocity")
            return
        return velocity

    def getJointsStatus(self, joints):
        if joints == "RHipPitch":
            return self.__hip_pitch_status
        elif joints == "RHipRoll":
            return self.__hip_roll_status
        elif joints == "RKneePitch":
            return self.__knee_pitch_status
        elif joints == "RAnklePitch":
            return self.__ankle_pitch_status
        elif joints == "RAnkleRoll":
            return self.__ankle_roll_status
        elif joints == "RHipYawPitch":
            return self.__hip_yaw_pitch_status
        else:
            logger.info(f"{joints} is not finding!")
            return None
    def position_is_arrive(self, joints="", targets=None):
        if joints == "":
            logger.info("You do not set any joints")
            return

        if targets is None:
            logger.info("You do not set any positions")
            return

        if joints   == "RHipPitch":
            if targets is self.__hip_pitch_previous_target:
                position = r_hip_pitch_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    return True
                else:
                    return False
            else:
                return True
        elif joints == "RHipRoll":
            if targets is self.__hip_roll_previous_target:
                position = r_hip_roll_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    return True
                else:
                    return False
            else:
                return True
        elif joints == "RKneePitch":
            if targets is self.__knee_pitch_previous_target:
                position = r_knee_pitch_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    return True
                else:
                    return False
            else:
                return True
        elif joints == "RAnklePitch":
            if targets is self.__ankle_pitch_previous_target:
                position = r_ankle_pitch_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    return True
                else:
                    return False
            else:
                return True
        elif joints == "RAnkleRoll":
            if targets is self.__ankle_roll_previous_target:
                position = r_ankle_roll_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        elif joints == "RHipYawPitch":
            if targets is self.__hip_yaw_pitch_previous_target:
                position = r_hip_yaw_pitch_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                #
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
        else:
            logger.info("Can not find any joints, please set again in position_is_arrive")
            return

    def set_position(self, joints="", positions=None):
        if joints == "":
            logger.info("You do not set any joints")
            return

        if positions is None:
            logger.info("You do not set any positions")
            return

        if joints == "RHipPitch":
            if positions > self.__max_hip_pitch_radian or positions < self.__min_hip_pitch_radian:
                logger.info(f"The position you set is {positions}, is out of range in [{self.__max_hip_pitch_radian}, {self.__min_hip_pitch_radian}]")
                return
            if self.__hip_pitch_status is move_status.INITIAL:
                logger.info(f"{joints} is in Initial")
                self.__hip_pitch_previous_target = positions
                self.__hip_pitch_status = move_status.PREPARE
                return
            elif self.__hip_pitch_status is move_status.PREPARE:
                if positions is self.__hip_pitch_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    self.__hip_pitch_status = move_status.MOVING
                else:
                    return
            elif self.__hip_pitch_status is move_status.MOVING:
                if positions is self.__hip_pitch_previous_target:
                    logger.info(f"{joints} is in Moving")
                    r_hip_pitch.setPosition(positions)
                    # logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
                    if self.position_is_arrive(joints, positions):
                        self.__hip_pitch_status = move_status.FINISH
                else:
                    return
            elif self.__hip_pitch_status is move_status.FINISH:
                if positions is self.__hip_pitch_previous_target:
                    logger.info(f"{joints} is in Finish")
                    if self.position_is_arrive(joints, positions):
                        self.__hip_pitch_status = move_status.END
                else:
                    return
            elif self.__hip_pitch_status is move_status.END:
                logger.info(f"{joints} is in End")
                if positions is not self.__hip_pitch_previous_target:
                    self.__hip_pitch_status = move_status.INITIAL
                else:
                    return
            else:
                logger.info(f"{joints} is in error status!")
                return
        elif joints == "RHipRoll":
             if positions > self.__max_hip_roll_radian or positions < self.__min_hip_roll_radian:
                    logger.info(f"The position you set is {positions}, is out of range in [{self.__max_hip_roll_radian}, {self.__min_hip_roll_radian}]")
                    return
             if self.__hip_roll_status is move_status.INITIAL:
                    logger.info(f"{joints} is in Initial")
                    self.__hip_roll_previous_target = positions
                    self.__hip_roll_status = move_status.PREPARE
             elif self.__hip_roll_status is move_status.PREPARE:
                 if positions is self.__hip_roll_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    self.__hip_roll_status = move_status.MOVING
                 else:
                     return
             elif self.__hip_roll_status is move_status.MOVING:
                    if positions is self.__hip_roll_previous_target:
                        logger.info(f"{joints} is in Moving")
                        r_hip_roll.setPosition(positions)
                        # logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
                        if self.position_is_arrive(joints, positions):
                            self.__hip_roll_status = move_status.FINISH
                    else:
                        return
             elif self.__hip_roll_status is move_status.FINISH:
                    if positions is self.__hip_roll_previous_target:
                        logger.info(f"{joints} is in Finish")
                        if self.position_is_arrive(joints, positions):
                            self.__hip_roll_status = move_status.END
                    else:
                        return
             elif self.__hip_roll_status is move_status.END:
                    logger.info(f"{joints} is in End")
                    if positions is not self.__hip_roll_previous_target:
                        self.__hip_roll_status = move_status.INITIAL
                    else:
                        return
             else:
                    logger.info(f"{joints} is in error status!")
                    return
        elif joints == "RKneePitch":
             if positions > self.__max_knee_pitch_radian or positions < self.__min_knee_pitch_radian:
                    logger.info(f"The position you set is {positions}, is out of range in [{self.__max_knee_pitch_radian}, {self.__min_knee_pitch_radian}]")
                    return
             if self.__knee_pitch_status is move_status.INITIAL:
                    logger.info(f"{joints} is in Initial")
                    self.__knee_pitch_previous_target = positions
                    self.__knee_pitch_status = move_status.PREPARE
             elif self.__knee_pitch_status is move_status.PREPARE:
                 if positions is self.__knee_pitch_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    self.__knee_pitch_status = move_status.MOVING
                 else:
                     return
             elif self.__knee_pitch_status is move_status.MOVING:
                    if positions is self.__knee_pitch_previous_target:
                        logger.info(f"{joints} is in Moving")
                        r_knee_pitch.setPosition(positions)
                        # logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
                        if self.position_is_arrive(joints, positions):
                            self.__knee_pitch_status = move_status.FINISH
                    else:
                        return
             elif self.__knee_pitch_status is move_status.FINISH:
                    if positions is self.__knee_pitch_previous_target:
                        logger.info(f"{joints} is in Finish")
                        if self.position_is_arrive(joints, positions):
                            self.__knee_pitch_status = move_status.END
                    else:
                        return
             elif self.__knee_pitch_status is move_status.END:
                    logger.info(f"{joints} is in End")
                    if positions is not self.__knee_pitch_previous_target:
                        self.__knee_pitch_status = move_status.INITIAL
                    else:
                        return
             else:
                    logger.info(f"{joints} is in error status!")
                    return
        elif joints == "RAnklePitch":
             if positions > self.__max_knee_pitch_radian or positions < self.__min_knee_pitch_radian:
                    logger.info(f"The position you set is {positions}, is out of range in [{self.__max_knee_pitch_radian}, {self.__min_knee_pitch_radian}]")
                    return
             if self.__ankle_pitch_status is move_status.INITIAL:
                    logger.info(f"{joints} is in Initial")
                    self.__ankle_pitch_previous_target = positions
                    self.__ankle_pitch_status = move_status.PREPARE
             elif self.__ankle_pitch_status is move_status.PREPARE:
                 if positions is self.__ankle_pitch_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    self.__ankle_pitch_status = move_status.MOVING
                 else:
                     return
             elif self.__ankle_pitch_status is move_status.MOVING:
                    if positions is self.__ankle_pitch_previous_target:
                        logger.info(f"{joints} is in Moving")
                        r_ankle_pitch.setPosition(positions)
                        # logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
                        if self.position_is_arrive(joints, positions):
                            self.__ankle_pitch_status = move_status.FINISH
                    else:
                        return
             elif self.__ankle_pitch_status is move_status.FINISH:
                    if positions is self.__ankle_pitch_previous_target:
                        logger.info(f"{joints} is in Finish")
                        if self.position_is_arrive(joints, positions):
                            self.__ankle_pitch_status = move_status.END
                    else:
                        return
             elif self.__ankle_pitch_status is move_status.END:
                    logger.info(f"{joints} is in End")
                    if positions is not self.__ankle_pitch_previous_target:
                        self.__ankle_pitch_status = move_status.INITIAL
                    else:
                        return
             else:
                    logger.info(f"{joints} is in error status!")
                    return
        elif joints == "RAnkleRoll":
             if positions > self.__max_ankle_roll_radian or positions < self.__min_ankle_roll_radian:
                    logger.info(f"The position you set is {positions}, is out of range in [{self.__max_ankle_roll_radian}, {self.__min_ankle_roll_radian}]")
                    return
             if self.__ankle_roll_status is move_status.INITIAL:
                    logger.info(f"{joints} is in Initial")
                    self.__ankle_roll_previous_target = positions
                    self.__ankle_roll_status = move_status.PREPARE
             elif self.__ankle_roll_status is move_status.PREPARE:
                 if positions is self.__ankle_roll_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    self.__ankle_roll_status = move_status.MOVING
                 else:
                     return
             elif self.__ankle_roll_status is move_status.MOVING:
                    if positions is self.__ankle_roll_previous_target:
                        logger.info(f"{joints} is in Moving")
                        r_ankle_roll.setPosition(positions)
                        # logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
                        if self.position_is_arrive(joints, positions):
                            self.__ankle_roll_status = move_status.FINISH
                    else:
                        return
             elif self.__ankle_roll_status is move_status.FINISH:
                    if positions is self.__ankle_roll_previous_target:
                        logger.info(f"{joints} is in Finish")
                        if self.position_is_arrive(joints, positions):
                            self.__ankle_roll_status = move_status.END
                    else:
                        return
             elif self.__ankle_roll_status is move_status.END:
                    logger.info(f"{joints} is in End")
                    if positions is not self.__ankle_roll_previous_target:
                        self.__ankle_roll_status = move_status.INITIAL
                    else:
                        return
             else:
                    logger.info(f"{joints} is in error status!")
                    return
        elif joints == "RHipYawPitch":
             if positions > self.__max_hip_yaw_pitch_radian or positions < self.__min_hip_yaw_pitch_radian:
                    logger.info(f"The position you set is {positions}, is out of range in [{self.__max_hip_yaw_pitch_radian}, {self.__min_hip_yaw_pitch_radian}]")
                    return
             if self.__hip_yaw_pitch_status is move_status.INITIAL:
                    logger.info(f"{joints} is in Initial")
                    self.__hip_yaw_pitch_previous_target = positions
                    self.__hip_yaw_pitch_status = move_status.PREPARE
             elif self.__hip_yaw_pitch_status is move_status.PREPARE:
                 if positions is self.__hip_yaw_pitch_previous_target:
                    logger.info(f"{joints} is in Prepare")
                    self.__hip_yaw_pitch_status = move_status.MOVING
                 else:
                     return
             elif self.__hip_yaw_pitch_status is move_status.MOVING:
                    if positions is self.__hip_yaw_pitch_previous_target:
                        logger.info(f"{joints} is in Moving")
                        r_hip_yaw_pitch.setPosition(positions)
                        # logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
                        if self.position_is_arrive(joints, positions):
                            self.__hip_yaw_pitch_status = move_status.FINISH
                    else:
                        return
             elif self.__hip_yaw_pitch_status is move_status.FINISH:
                    if positions is self.__hip_yaw_pitch_previous_target:
                        logger.info(f"{joints} is in Finish")
                        if self.position_is_arrive(joints, positions):
                            self.__hip_yaw_pitch_status = move_status.END
                    else:
                        return
             elif self.__hip_yaw_pitch_status is move_status.END:
                    logger.info(f"{joints} is in End")
                    if positions is not self.__hip_yaw_pitch_previous_target:
                        self.__hip_yaw_pitch_status = move_status.INITIAL
                    else:
                        return
             else:
                    logger.info(f"{joints} is in error status!")
                    return
        else:
            logger.info("Can not find any joints, please set again in set_position")
            return
        # logger.info("End moved is", self.moved)

class NAO_MOTION:
    def __init__(self):
        logger.info("Init NAO_MOTION!")

    __l_shoulder = left_shoulder()
    __l_elbow = left_elbow()
    __r_shoulder = right_shoulder()
    __r_elbow = right_elbow()

    __l_leg = left_leg()
    __r_leg = right_leg()
    __time = 0.0  # 初始化时间变量
    __step_duration = 0.05  # 每一步的持续时间（秒）
    __amplitude = 0.05  # 关节摆动的幅度（弧度）
    __sway_amplitude = 0.02  # 侧向摆动的幅度（弧度）
    __balance_adjustment = 0.01  # 前后平衡调整参数（弧度）

    __salute_status = SALUTE_MOTION.INITIAL
    __walk_status = WALK_MOTION.INITIAL
    __last_walk_status = None
    __left_joints_name = ['LHipPitch', 'LKneePitch', 'LAnklePitch']
    __right_joints_name = ['RHipPitch', 'RKneePitch', 'RAnklePitch']
    __positions = {}

    __is_right_support = True

    def set_leg_position(self, left_hip, left_knee, left_ankle, right_hip, right_knee, right_ankle):
        self.__l_leg.set_position('LHipPitch', left_hip)
        self.__l_leg.set_position('LKneePitch', left_knee)
        self.__l_leg.set_position('LAnklePitch', left_ankle)
        self.__r_leg.set_position('RHipPitch', right_hip)
        self.__r_leg.set_position('RKneePitch', right_knee)
        self.__r_leg.set_position('RAnklePitch', right_ankle)

    def salute_motion(self):
        if self.__salute_status is SALUTE_MOTION.INITIAL:
            logger.info("In SALUTE_MOTION.INITIAL")
            self.__salute_status = SALUTE_MOTION.PREPARE
        elif self.__salute_status is SALUTE_MOTION.PREPARE:
            logger.info("In SALUTE_MOTION.PREPARE")
            self.__r_shoulder.set_position("RShoulderPitch", 1.5)
            self.__l_shoulder.set_position("LShoulderPitch", 1.5)
            self.__l_shoulder.set_position("LShoulderRoll", 0)
            self.__l_elbow.set_position('LElbowRoll', -0.035)
            if (self.__r_shoulder.getJointsStatus('RShoulderPitch') is move_status.END and
                self.__l_shoulder.getJointsStatus('LShoulderPitch') is move_status.END and
                self.__l_shoulder.getJointsStatus("LShoulderRoll") is move_status.END and
                self.__l_elbow.getJointsStatus('LElbowRoll') is move_status.END):
                self.__salute_status = SALUTE_MOTION.RAISE_HAND
            else:
                return
        elif self.__salute_status is SALUTE_MOTION.RAISE_HAND:
            logger.info("In SALUTE_MOTION.PREPARE")
            self.__l_shoulder.set_position("LShoulderPitch", -1)
            self.__l_shoulder.set_position("LShoulderRoll", 1)
            if (self.__l_shoulder.getJointsStatus('LShoulderPitch') is move_status.END and
                self.__l_shoulder.getJointsStatus('LShoulderRoll') is move_status.END):
                self.__salute_status = SALUTE_MOTION.SALUTE
            else:
                return
        elif self.__salute_status is SALUTE_MOTION.SALUTE:
            logger.info("In SALUTE_MOTION.SALUTE")
            self.__l_elbow.set_position('LElbowRoll', -1.5)
            if self.__l_elbow.getJointsStatus('LElbowRoll') is move_status.END:
                self.__salute_status = SALUTE_MOTION.RESET
            else:
                return
        elif self.__salute_status is SALUTE_MOTION.RESET:
            logger.info("In SALUTE_MOTION.RESET")
            self.__l_elbow.set_position('LElbowRoll', -0.035)
            if self.__l_elbow.getJointsStatus('LElbowRoll') is move_status.END:
                self.__salute_status = SALUTE_MOTION.FINISH
            else:
                return
        elif self.__salute_status is SALUTE_MOTION.FINISH:
            logger.info("In SALUTE_MOTION.FINISH")
            self.__salute_status = SALUTE_MOTION.END
        elif self.__salute_status is SALUTE_MOTION.END:
            logger.info("In SALUTE_MOTION.END")
            self.__salute_status = SALUTE_MOTION.INITIAL
        else:
            logger.info("salute motion in error status")
            return

    def walk_motion(self):
        if self.__walk_status is WALK_MOTION.INITIAL:
            logger.info("In WALK_MOTION.INITIAL")
            self.__walk_status = WALK_MOTION.WALKING
        elif self.__walk_status is WALK_MOTION.PREPARE:
            logger.info("In WALK_MOTION.PREPARE")
            for name in self.__left_joints_name:
                self.__l_leg.set_position(name, 0)
            for name in self.__right_joints_name:
                self.__r_leg.set_position(name, 0)
            l_leg_all_in_end = all(self.__l_leg.getJointsStatus(n) is move_status.END for n in self.__left_joints_name)
            r_leg_all_in_end = all(self.__r_leg.getJointsStatus(n) is move_status.END for n in self.__right_joints_name)
            if  l_leg_all_in_end and r_leg_all_in_end:
                self.__walk_status = WALK_MOTION.FINISH
            else:
                return
        # elif self.__walk_status is WALK_MOTION.IS_WALKING:
        #     logger.info("In WALK_MOTION.IS_WALKING")
        #     key = keyboard.getKey()
        #     logger.info(f'key: {key}')
        #     if key == keyboard.UP:
        #         self.__walk_status = WALK_MOTION.WALKING
        #     else:
        #         self.__walk_status = WALK_MOTION.FINISH
        elif self.__walk_status is WALK_MOTION.WALKING:
            logger.info("In WALK_MOTION.WALKING")
            if self.__is_right_support:
                self.set_leg_position(-1.5, 1.55, 0.46, 0, 0.01, -0.01)
            else:
                self.set_leg_position(0, 0.01, -0.01, -1.5, 1.06, 0.46)
            l_leg_all_in_end = all(self.__l_leg.getJointsStatus(n) is move_status.END for n in self.__left_joints_name)
            r_leg_all_in_end = all(self.__r_leg.getJointsStatus(n) is move_status.END for n in self.__right_joints_name)
            if l_leg_all_in_end and r_leg_all_in_end:
                self.__walk_status = WALK_MOTION.FINISH
                self.__is_right_support = not self.__is_right_support
            else:
                return
        elif self.__walk_status is WALK_MOTION.FINISH:
            logger.info("In WALK_MOTION.FINISH")
            self.__walk_status = WALK_MOTION.END
        elif self.__walk_status is WALK_MOTION.END:
            logger.info("In WALK_MOTION.END")
            self.__walk_status = WALK_MOTION.INITIAL
        else:
            logger.info("walk motion in error status")


# l_leg = left_leg()
# r_leg = right_leg()
# def set_leg_position(left_hip, left_knee, left_ankle, right_hip, right_knee, right_ankle):
#     # l_leg.set_position('LHipYawPitch', left_hip[0])
#     l_leg.set_position('LHipPitch', left_hip)
#     # l_leg.set_position('LHipRoll', left_hip[2])
#     l_leg.set_position('LKneePitch', left_knee)
#     l_leg.set_position('LAnklePitch', left_ankle)
#     # l_leg.set_position('LAnkleRoll', left_ankle[1])
#     # r_leg.set_position('RHipYawPitch', right_hip[0])
#     r_leg.set_position('RHipPitch', right_hip)
#     # r_leg.set_position('RHipRoll', right_hip[2])
#     r_leg.set_position('RKneePitch', right_knee)
#     r_leg.set_position('RAnklePitch', right_ankle)
#     # r_leg.set_position('RAnkleRoll', right_ankle[1])

def set_leg_position(left_hip, left_knee, left_ankle, right_hip, right_knee, right_ankle):
    l_hip_pitch.setPosition(left_hip)
    l_knee_pitch.setPosition(left_knee)
    l_ankle_pitch.setPosition(left_ankle)
    r_hip_pitch.setPosition(right_hip)
    r_knee_pitch.setPosition(right_knee)
    r_ankle_pitch.setPosition(right_ankle)

have_found = False

def down_motion(h):
    l_kp = 0
    r_kp = 0
    l_hp = 0
    r_hp = 0
    l_ap = 0
    r_ap = 0
    h1 = h2 = 100/1000
    d1 = d2 = 102.9/1000
    for i in np.arange(0,1.53,0.001):
        for j in np.arange(0,0.92,0.001):
            print(i)
            print(f"left leg is: {h1 * np.cos(np.rad2deg(np.abs(l_hp - i))) + d1 * np.cos(np.rad2deg(l_kp + j))}, "
                  f"right leg is: {h2 * np.cos(np.rad2deg(np.abs(r_hp - i))) + d2 * np.cos(np.rad2deg(r_kp + j))}, ")
            tmp_lhp = np.abs(l_hp - i)
            tmp_lkp = l_kp + j
            tmp_rhp = np.abs(r_hp - i)
            tmp_rkp = r_kp + j
            tmp_lap = np.abs(l_ap - j)
            tmp_rap = np.abs(r_ap - j)
            if (
                    (np.abs(h - (np.abs(h1 * np.cos(np.rad2deg(tmp_lhp))) + np.abs(d1 * np.cos(np.rad2deg(tmp_lkp))))) < 0.001 and tmp_lap == tmp_lkp)
                    and (np.abs(h - (np.abs(h2 * np.cos(np.rad2deg(tmp_rhp))) + np.abs(d2 * np.cos(np.rad2deg(tmp_rkp))))) < 0.001 and tmp_rap == tmp_rkp)
            ):
                print(tmp_lhp, tmp_lkp, tmp_lap, tmp_rhp, tmp_rkp, tmp_lap)
                set_leg_position(tmp_lhp, tmp_lkp, tmp_lap, tmp_rhp, tmp_rkp, tmp_rap)
                return True
            else:
                return False

T_sup = 0.2
T = 0
delta_sx = 0.3
delta_sy = 0.1
n = 0


while robot.step(timestep) != -1:
    pass
# # 初始化部件及质量
# part_names = ["Head", "Torso", "RightArm", "LeftArm", "RightLeg", "LeftLeg"]
# masses = []
# positions = []
#
# # 获取各部件质量和位置
# for part_name in part_names:
#     part = robot.getDevice(part_name)
#     mass = part.getMass()
#     position = part.getPosition()
#     masses.append(mass)
#     positions.append(position)
#
# # 计算质心
# total_mass = sum(masses)
# center_of_mass = [0, 0, 0]
# for i in range(len(masses)):
#     center_of_mass[0] += masses[i] * positions[i][0]
#     center_of_mass[1] += masses[i] * positions[i][1]
#     center_of_mass[2] += masses[i] * positions[i][2]
#
# center_of_mass = [coord / total_mass for coord in center_of_mass]
#
# print("NAO 机器人的质心位置:", center_of_mass)

# while robot.step(timestep) != -1:
#     have_found = down_motion(0.1)
#     if have_found:
#         break
#     else:
#         pass


# is_right_support = True
# left_joints_name = ['LHipYawPitch','LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll']
# right_joints_name = ['RHipYawPitch','RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
# left_hip = [0,0,0]
# left_knee = 0
# left_ankle = [0,0]
# right_hip = [0,0,0]
# right_knee = 0
# right_ankle = [0,0]

# while robot.step(timestep) != -1:

    # set_leg_position(left_hip, left_knee, left_ankle, right_hip, right_knee, right_ankle)
    # logger.info('----------inertial unit----------')
    # logger.info('x/y/z: [%f %f %f]' % (xyz[0], xyz[1], xyz[2]))

    # if is_right_support:
    #     set_leg_position(-1.5, 1.55, 0.46, 0, 0.01, -0.01)
    # else:
    #     set_leg_position(0, 0.01, -0.01, -1.5, 1.06, 0.46)
    # l_leg_all_in_end = all(l_leg.getJointsStatus(n) is move_status.END for n in left_joints_name)
    # r_leg_all_in_end = all(r_leg.getJointsStatus(n) is move_status.END for n in right_joints_name)
    # if l_leg_all_in_end and r_leg_all_in_end:
    #     is_right_support = not is_right_support
    # pass