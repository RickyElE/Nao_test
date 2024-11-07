"""NaoController controller."""
from os.path import curdir

from roboticstoolbox import DHRobot, RevoluteDH

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Keyboard
from enum import Enum, unique, auto
import math
import logging
import logging.config
# from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import numpy as np

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
l_hip_pitch = robot.getDevice('LHipPitch')
l_hip_roll = robot.getDevice('LHipRoll')
l_knee_pitch = robot.getDevice('LKneePitch')
l_ankle_pitch = robot.getDevice('LAnklePitch')
l_ankle_roll = robot.getDevice('LAnkleRoll')

l_hip_roll_sensor = l_hip_roll.getPositionSensor()
l_hip_pitch_sensor = l_hip_pitch.getPositionSensor()
l_knee_pitch_sensor = l_knee_pitch.getPositionSensor()
l_ankle_pitch_sensor = l_ankle_pitch.getPositionSensor()
l_ankle_roll_sensor = l_ankle_roll.getPositionSensor()

l_hip_pitch_v = l_hip_pitch.getVelocity()
l_hip_roll_v  = l_hip_roll.getVelocity()
l_knee_pitch_v = l_knee_pitch.getVelocity()
l_ankle_pitch_v = l_ankle_pitch.getVelocity()
l_ankle_roll_v = l_ankle_roll.getVelocity()

# right leg and sensor
r_hip_roll = robot.getDevice('RHipRoll')
r_hip_pitch = robot.getDevice('RHipPitch')
r_knee_pitch = robot.getDevice('RKneePitch')
r_ankle_pitch = robot.getDevice('RAnklePitch')
r_ankle_roll = robot.getDevice('RAnkleRoll')

r_hip_roll_sensor = r_hip_roll.getPositionSensor()
r_hip_pitch_sensor = r_hip_pitch.getPositionSensor()
r_knee_pitch_sensor = r_knee_pitch.getPositionSensor()
r_ankle_pitch_sensor = r_ankle_pitch.getPositionSensor()
r_ankle_roll_sensor = r_ankle_roll.getPositionSensor()

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
l_hip_roll_sensor.enable(timestep)
l_hip_pitch_sensor.enable(timestep)
l_knee_pitch_sensor.enable(timestep)
l_ankle_pitch_sensor.enable(timestep)
l_ankle_roll_sensor.enable(timestep)

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
                    logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
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
                        logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
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
    __hip_pitch_status   = move_status.INITIAL
    __hip_roll_status    = move_status.INITIAL
    __knee_pitch_status  = move_status.INITIAL
    __ankle_pitch_status = move_status.INITIAL
    __ankle_roll_status  = move_status.INITIAL

    __limitation = 0.000001

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

    __hip_pitch_previous_target   = None
    __hip_roll_previous_target    = None
    __knee_pitch_previous_target  = None
    __ankle_pitch_previous_target = None
    __ankle_roll_previous_target  = None

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
                    logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
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
                        logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
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
                        logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
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
                        l_shoulder_roll.setPosition(positions)
                        logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
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
                        logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
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
        else:
            logger.info("Can not find any joints, please set again in set_position")
            return
        # logger.info("End moved is", self.moved)

class right_leg(move):
    def __init__(self):
        move.__init__(self)
    __hip_pitch_status   = move_status.INITIAL
    __hip_roll_status    = move_status.INITIAL
    __knee_pitch_status  = move_status.INITIAL
    __ankle_pitch_status = move_status.INITIAL
    __ankle_roll_status  = move_status.INITIAL

    __limitation = 0.000001

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

    __hip_pitch_previous_target   = None
    __hip_roll_previous_target    = None
    __knee_pitch_previous_target  = None
    __ankle_pitch_previous_target = None
    __ankle_roll_previous_target  = None

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
                    logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
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
                        logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
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
                        logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
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
                        r_shoulder_roll.setPosition(positions)
                        logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
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
                        logger.info(f"{joints}'s velocity is {self.get_velocity(joints)}")
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
    __left_joints_name = ['LHipPitch', 'LKneePitch', 'LHipRoll', 'LAnkleRoll', 'LAnklePitch']
    __right_joints_name = ['RHipPitch', 'RKneePitch', 'RHipRoll', 'RAnkleRoll', 'RAnklePitch']
    __positions = {}

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
            l_leg_all_in_zero = all(self.__l_leg.position_is_arrive(n,0) for n in self.__left_joints_name)
            r_leg_all_in_zero = all(self.__r_leg.position_is_arrive(n,0) for n in self.__right_joints_name)
            if l_leg_all_in_zero and r_leg_all_in_zero:
                self.__walk_status = WALK_MOTION.WALKING
            else:
                self.__walk_status = WALK_MOTION.PREPARE
            # self.__last_walk_status = WALK_MOTION.INITIAL
        elif self.__walk_status is WALK_MOTION.PREPARE:
            logger.info("In WALK_MOTION.PREPARE")
            for name in self.__left_joints_name:
                self.__l_leg.set_position(name, 0)
            for name in self.__right_joints_name:
                self.__r_leg.set_position(name, 0)
            l_leg_all_in_end = all(self.__l_leg.getJointsStatus(n) is move_status.END for n in self.__left_joints_name)
            r_leg_all_in_end = all(self.__r_leg.getJointsStatus(n) is move_status.END for n in self.__right_joints_name)
            if  l_leg_all_in_end and r_leg_all_in_end:
                self.__walk_status = WALK_MOTION.IS_WALKING
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
            l_leg_all_in_init = all(
                self.__l_leg.getJointsStatus(n) is move_status.INITIAL for n in self.__left_joints_name)
            r_leg_all_in_init = all(
                self.__r_leg.getJointsStatus(n) is move_status.INITIAL for n in self.__right_joints_name)
            if l_leg_all_in_init and r_leg_all_in_init:
                self.__time += timestep / 1000.0
                phase = (self.__time % self.__step_duration) / self.__step_duration
                left_phase = phase
                right_phase = (phase + 0.5) % 1.0

                # 髋关节前后摆动（HipPitch）
                self.__positions['LHipPitch'] = self.__amplitude * math.sin(2 * math.pi * left_phase)
                self.__positions['RHipPitch'] = self.__amplitude * math.sin(2 * math.pi * right_phase)

                # 膝关节弯曲（KneePitch）
                self.__positions['LKneePitch'] = self.__amplitude * math.sin(2 * math.pi * left_phase)
                self.__positions['RKneePitch'] = self.__amplitude * math.sin(2 * math.pi * right_phase)

                # 踝关节调整（AnklePitch）
                self.__positions['LAnklePitch'] = -self.__positions['LHipPitch'] - self.__positions['LKneePitch']
                self.__positions['RAnklePitch'] = -self.__positions['RHipPitch'] - self.__positions['RKneePitch']

                # 侧向平衡
                self.__positions['LHipRoll'] = self.__sway_amplitude * math.sin(2 * math.pi * phase)
                self.__positions['RHipRoll'] = self.__positions['LHipRoll']
                self.__positions['LAnkleRoll'] = -self.__positions['LHipRoll']
                self.__positions['RAnkleRoll'] = -self.__positions['RHipRoll']

                # 前后平衡调整
                self.__positions['LAnklePitch'] += self.__balance_adjustment * math.sin(2 * math.pi * left_phase)
                self.__positions['RAnklePitch'] += self.__balance_adjustment * math.sin(2 * math.pi * right_phase)

            # 设置关节的目标位置
            for name, position in self.__positions.items():
                print(name, position)
                if name in self.__left_joints_name:
                    self.__l_leg.set_position(name,position)
                elif name in self.__right_joints_name:
                    self.__r_leg.set_position(name,position)
                else:
                    logger.error(f'{name} is not in leg joints!')

            l_leg_all_in_end = all(self.__l_leg.getJointsStatus(n) is move_status.END for n in self.__left_joints_name)
            r_leg_all_in_end = all(self.__r_leg.getJointsStatus(n) is move_status.END for n in self.__right_joints_name)
            if (l_leg_all_in_end and r_leg_all_in_end):
                self.__walk_status = WALK_MOTION.FINISH
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

nao_motion = NAO_MOTION()

# 获取键盘设备并启用
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# 创建字典来存储机器人的关节电机和位置传感器
joints = {}
sensors = {}
# 定义需要控制的关节名称列表，包括头部、手臂和下肢关节
joint_names = [
    # 头部关节
    'HeadYaw', 'HeadPitch',
    # 左右臂关节
    'LShoulderPitch', 'RShoulderPitch',
    'LShoulderRoll', 'RShoulderRoll',
    'LElbowYaw', 'RElbowYaw',
    'LElbowRoll', 'RElbowRoll',
    # 左右髋关节
    'LHipYawPitch', 'RHipYawPitch',
    'LHipRoll',     'RHipRoll',
    'LHipPitch',    'RHipPitch',
    # 左右膝关节
    'LKneePitch',   'RKneePitch',
    # 左右踝关节
    'LAnklePitch',  'RAnklePitch',
    'LAnkleRoll',   'RAnkleRoll'
]

# 遍历关节名称，获取对应的电机设备和位置传感器
for name in joint_names:
    joint = robot.getDevice(name)
    if joint is None:
        print(f"关节 '{name}' 未找到")
        continue
    else:
        print(f"关节 '{name}' 成功获取")
    joint.setPosition(0.0)
    joints[name] = joint
    # 获取关节的角度限制
    # min_position = joint.getMinPosition()
    # max_position = joint.getMaxPosition()
    # 获取并启用关节的角度传感器
    sensor = joint.getPositionSensor()
    if sensor is not None:
        sensor.enable(timestep)
        sensors[name] = sensor
    else:
        print(f"关节 '{name}' 没有位置传感器")

# 定义用于检查单个关节是否到达目标位置的函数
def is_at_zero(name):
    if name in sensors:
        return abs(sensors[name].getValue() - 0.0) < 0.01  # 允许微小的误差
    else:
        return True  # 如果没有传感器，假设关节已到达目标位置

# 定义用于检查所有关节的函数
# def are_all_joints_at_zero(names):
#     return all(is_at_zero(name) for name in names)
#
# # 设置初始姿态，将所有关节位置设置为 0
# initial_positions = {name: 0.0 for name in joint_names}
# for name, position in initial_positions.items():
#     if name in joints:
#         joints[name].setPosition(position)

# 提示信息，等待关节初始化
print("初始化中，等待所有关节到达 0 位置...")

# 等待关节到达初始位置
# while robot.step(timestep) != -1:
#     if are_all_joints_at_zero(joint_names):
#         print("所有关节已到达 0 位置，可以通过键盘控制机器人。")
#         break

# 定义行走参数
time = 0.0  # 时间变量
step_duration = int(2000 / timestep)  # 每一步的持续时间（秒）

is_left_leg_support = True
step_counter = 0

# PID 控制器参数（用于前后平衡）
Kp = 0.5  # 比例系数
Ki = 0.0  # 积分系数
Kd = 0.1  # 微分系数

# 初始化 PID 控制器变量
error_sum_pitch = 0.0  # 误差积分项
last_error_pitch = 0.0  # 上一次的误差

# 启用陀螺仪传感器，用于获取机器人的俯仰角速度
gyro = robot.getDevice('gyro')
if gyro is not None:
    gyro.enable(timestep)
else:
    print("陀螺仪传感器未找到，无法进行反馈控制。")

# 定义一个变量来跟踪机器人是否在行走
is_walking = False

hip_offset_y = 50/1000
hip_offset_z = 85/1000
thigh_length = 100/1000
tibia_length = 102.9/1000
foot_height = 45.19/1000

step_length = [0.1 * (459.59 / 1000), 0.2 * (459.59 / 1000)] #x
step_width  = [0.4 * (98 / 1000), 0.6 * (98 / 1000)] #y
step_height = [0.05 * (333.09 / 1000), 0.1 * (333.09 / 1000)] #z

hip_side_shift = 0.03
hip_forward_shift = 0.03

nao_left_leg = DHRobot(
    [
        RevoluteDH(a=0, alpha=0, d=-hip_offset_z, offset=0), #Base to L_Hip_Yaw_Pitch
        RevoluteDH(a=0, alpha=-3*np.pi/4, d=0, offset=-np.pi/2), # L Hip Yaw Pitch
        RevoluteDH(a=0, alpha=-np.pi/2, d=0, offset=np.pi/4), # L Hip Roll
        RevoluteDH(a=0, alpha=np.pi/2, d=0), # L Hip Pitch
        RevoluteDH(a=-thigh_length, alpha=0, d=0), # L Knee Pitch
        RevoluteDH(a=-tibia_length, alpha=0, d=0), # L Ankle Pitch
        RevoluteDH(a=0, alpha=-np.pi/2, d=0), # L Ankle Roll
        RevoluteDH(a=0, alpha=np.pi, d=0, offset=-np.pi/2)
     ],
    name="NAO_LEFT_LEG"
)

nao_right_leg = DHRobot(
    [
        RevoluteDH(a=0, alpha=0, d=-hip_offset_z, offset=0), #Base to L_Hip_Yaw_Pitch
        RevoluteDH(a=0, alpha=-np.pi/4, d=0, offset=-np.pi/2), # L Hip Yaw Pitch
        RevoluteDH(a=0, alpha=-np.pi/2, d=0, offset=-np.pi/4), # L Hip Roll
        RevoluteDH(a=0, alpha=np.pi/2, d=0), # L Hip Pitch
        RevoluteDH(a=-thigh_length, alpha=0, d=0), # L Knee Pitch
        RevoluteDH(a=-tibia_length, alpha=0, d=0), # L Ankle Pitch
        RevoluteDH(a=0, alpha=-np.pi/2, d=0), # L Ankle Roll
        RevoluteDH(a=0, alpha=np.pi, d=0, offset=-np.pi/2)
     ],
    name="NAO_RIGHT_LEG"
)

left_leg_joints = [
 # 左右髋关节
 'LHipYawPitch',
 'LHipRoll',
 'LHipPitch',
 # 左右膝关节
 'LKneePitch',
 # 左右踝关节
 'LAnklePitch',
 'LAnkleRoll',]

right_leg_joints = [
'RHipYawPitch',
'RHipRoll',
'RHipPitch',
'RKneePitch',
'RAnklePitch',
'RAnkleRoll'
]

# 主控制循环
while robot.step(timestep) != -1:
    step_counter += 1

    # x = np.random.uniform(step_length[0], step_length[1])  # 随机步长
    # y = np.random.uniform(step_width[0], step_width[1])    # 随机步宽
    # z = np.random.uniform(step_height[0], step_height[1])  # 随机步高

    x = step_length[1]
    y = step_width[1]
    z = step_height[1]


    if is_left_leg_support:
        target_position_right = np.array([x + hip_forward_shift, -y - hip_offset_y - hip_side_shift, z - hip_offset_z - foot_height])
        target_transform_right = np.eye(4)
        target_transform_right[0:3, 3] = target_position_right
        ik_solution_right = nao_right_leg.ikine_LM(target_transform_right)
        r_leg_rad = []
        if ik_solution_right.success:
            r_leg_rad = np.deg2rad(ik_solution_right.q)
            r_leg_rad = r_leg_rad[1:len(r_leg_rad) - 1]
            rad_of_r_leg = {}
            for i in range(len(r_leg_rad)):
                rad_of_r_leg.update({right_leg_joints[i]: r_leg_rad[i]})
            for (name, position)in rad_of_r_leg.items():
                if name in joints:
                    joints[name].setPosition(position)
        else:
            print("右脚前摆无法找到合适的关节角度")

        target_position_left = np.array([0, y + hip_offset_y + hip_side_shift, -foot_height - hip_offset_z])
        target_transform_left = np.eye(4)
        target_transform_left[0:3, 3] = target_position_left
        ik_solution_left = nao_left_leg.ikine_LM(target_transform_left)
        l_leg_rad = []
        if ik_solution_left.success:
            l_leg_rad = np.deg2rad(ik_solution_right.q)
            l_leg_rad = l_leg_rad[1:len(l_leg_rad) - 1]
            rad_of_l_leg = {}
            for i in range(len(l_leg_rad)):
                rad_of_l_leg.update({right_leg_joints[i]: l_leg_rad[i]})
            for (name, position) in rad_of_l_leg.items():
                if name in joints:
                    joints[name].setPosition(position)
        else:
            print("左脚支撑无法找到合适的关节角度")

        logger.info(f'left_leg_position: {sensors[left_leg_joints[0]].getValue()}, {sensors[left_leg_joints[1]].getValue()}, {sensors[left_leg_joints[2]].getValue()}, {sensors[left_leg_joints[3]].getValue()},{sensors[left_leg_joints[4]].getValue()}, {sensors[left_leg_joints[5]].getValue()}')
        logger.info(f'right_leg_position: {sensors[right_leg_joints[0]].getValue()}, {sensors[right_leg_joints[1]].getValue()}, {sensors[right_leg_joints[2]].getValue()}, {sensors[right_leg_joints[3]].getValue()},{sensors[right_leg_joints[4]].getValue()}, {sensors[right_leg_joints[5]].getValue()}')

    else:
        target_position_left = np.array([x + hip_forward_shift, y + hip_offset_y + hip_side_shift, z -foot_height - hip_offset_z])
        target_transform_left = np.eye(4)
        target_transform_left[0:3, 3] = target_position_left
        ik_solution_left = nao_left_leg.ikine_LM(target_transform_left)
        l_leg_rad = []
        if ik_solution_left.success:
            l_leg_rad = np.deg2rad(ik_solution_right.q)
            l_leg_rad = l_leg_rad[1:len(l_leg_rad) - 1]
            rad_of_l_leg = {}
            for i in range(len(l_leg_rad)):
                rad_of_l_leg.update({right_leg_joints[i]: l_leg_rad[i]})
            for (name, position) in rad_of_l_leg.items():
                if name in joints:
                    joints[name].setPosition(position)
        else:
            print("左脚前摆无法找到合适的关节角度")

        target_position_right = np.array([0, -y - hip_offset_y - hip_side_shift, - hip_offset_z - foot_height])
        target_transform_right = np.eye(4)
        target_transform_right[0:3, 3] = target_position_right
        ik_solution_right = nao_right_leg.ikine_LM(target_transform_right)
        r_leg_rad = []
        if ik_solution_right.success:
            r_leg_rad = np.deg2rad(ik_solution_right.q)
            r_leg_rad = r_leg_rad[1:len(r_leg_rad) - 1]
            rad_of_r_leg = {}
            for i in range(len(r_leg_rad)):
                rad_of_r_leg.update({right_leg_joints[i]: r_leg_rad[i]})
            for (name, position)in rad_of_r_leg.items():
                if name in joints:
                    joints[name].setPosition(position)
        else:
            print("右脚支撑无法找到合适的关节角度")

        logger.info(
            f'left_leg_position: {sensors[left_leg_joints[0]].getValue()}, {sensors[left_leg_joints[1]].getValue()}, {sensors[left_leg_joints[2]].getValue()}, {sensors[left_leg_joints[3]].getValue()},{sensors[left_leg_joints[4]].getValue()}, {sensors[left_leg_joints[5]].getValue()}')
        logger.info(
            f'right_leg_position: {sensors[right_leg_joints[0]].getValue()}, {sensors[right_leg_joints[1]].getValue()}, {sensors[right_leg_joints[2]].getValue()}, {sensors[right_leg_joints[3]].getValue()},{sensors[right_leg_joints[4]].getValue()}, {sensors[right_leg_joints[5]].getValue()}')

    if step_counter >= step_duration:
        is_left_leg_support = not is_left_leg_support
        step_counter = 0

    # # 检测键盘输入
    # key = keyboard.getKey()
    #
    # # 更新行走状态
    # if key == ord('W'):
    #     is_walking = True
    # elif key == -1:
    #     is_walking = False
    #
    # if is_walking:
    #     # 更新时间
    #     time += timestep / 1000.0  # 将毫秒转换为秒
    #     phase = (time % step_duration) / step_duration  # 计算当前步态相位，范围在 0 到 1 之间
    #     left_phase = phase  # 左腿的相位
    #     right_phase = (phase + 0.5) % 1.0  # 右腿的相位，相位差 0.5，实现左右腿交替
    #
    #     positions = {}  # 创建一个字典来存储当前循环中各关节的目标位置
    #
    #     # 髋关节前后摆动（HipPitch），控制大腿的前后运动，并加入身体前倾
    #     positions['LHipPitch'] = amplitude * math.sin(2 * math.pi * left_phase) + forward_tilt
    #     positions['RHipPitch'] = amplitude * math.sin(2 * math.pi * right_phase) + forward_tilt
    #
    #     # 膝关节弯曲（KneePitch），控制小腿的弯曲程度
    #     positions['LKneePitch'] = amplitude * math.sin(2 * math.pi * left_phase)
    #     positions['RKneePitch'] = amplitude * math.sin(2 * math.pi * right_phase)
    #
    #     # 踝关节调整（AnklePitch），用于保持脚部的平衡
    #     positions['LAnklePitch'] = -positions['LHipPitch'] - positions['LKneePitch']
    #     positions['RAnklePitch'] = -positions['RHipPitch'] - positions['RKneePitch']
    #
    #     # 添加侧向平衡，通过髋关节和踝关节的侧摆（HipRoll 和 AnkleRoll）来实现
    #     positions['LHipRoll'] = sway_amplitude * math.sin(2 * math.pi * phase)
    #     positions['RHipRoll'] = positions['LHipRoll']
    #     positions['LAnkleRoll'] = -positions['LHipRoll']
    #     positions['RAnkleRoll'] = -positions['RHipRoll']
    #
    #     # 前后平衡调整，在踝关节的俯仰角度中加入微调项
    #     positions['LAnklePitch'] += balance_adjustment * math.sin(2 * math.pi * left_phase)
    #     positions['RAnklePitch'] += balance_adjustment * math.sin(2 * math.pi * right_phase)
    #
    #     # 使用 PID 控制器进行前后平衡调整
    #     if gyro is not None:
    #         gyro_values = gyro.getValues()  # 获取陀螺仪数据（角速度，单位：rad/s）
    #         pitch_rate = gyro_values[1]  # 获取俯仰角速度（Y 轴）
    #
    #         # 理想的俯仰角速度为 0，计算误差
    #         error_pitch = -pitch_rate
    #
    #         # 积分误差
    #         error_sum_pitch += error_pitch * (timestep / 1000.0)
    #
    #         # 误差变化率
    #         error_rate_pitch = (error_pitch - last_error_pitch) / (timestep / 1000.0)
    #
    #         # PID 控制输出
    #         pid_output_pitch = Kp * error_pitch + Ki * error_sum_pitch + Kd * error_rate_pitch
    #
    #         # 更新上一次的误差
    #         last_error_pitch = error_pitch
    #
    #         # 调整踝关节俯仰角度
    #         positions['LAnklePitch'] += pid_output_pitch
    #         positions['RAnklePitch'] += pid_output_pitch
    #
    #     # 控制上半身姿态
    #     # 让肩部前后摆动，模拟手臂的摆动，增加平衡性
    #     arm_swing = 0.5 * amplitude * math.sin(2 * math.pi * phase)
    #     positions['LShoulderPitch'] = 1.5 + arm_swing  # 左肩关节俯仰
    #     positions['RShoulderPitch'] = 1.5 - arm_swing  # 右肩关节俯仰，方向相反
    #
    #     # 手肘弯曲，固定角度
    #     positions['LElbowRoll'] = -0.5  # 左肘关节内旋
    #     positions['RElbowRoll'] = 0.5   # 右肘关节内旋
    #
    #     # 头部略微前倾，增加重心前移
    #     positions['HeadPitch'] = -0.1
    #
    #     # 设置关节的目标位置
    #     for name, position in positions.items():
    #         if name in joints:
    #             print(f'name {name}, position {position}')
    #             joints[name].setPosition(position)
    # else:
    #     # 未按下 'W' 键时，机器人保持站立姿态并稍微前倾
    #     for name in joint_names:
    #         if name in joints:
    #             joints[name].setPosition(0.0)
    #
    #     # 设置踝关节的俯仰角度，使机器人稍微前倾
    #     if 'LAnklePitch' in joints:
    #         joints['LAnklePitch'].setPosition(-0.1)
    #     if 'RAnklePitch' in joints:
    #         joints['RAnklePitch'].setPosition(-0.1)
    #
    #     # 保持手臂下垂姿态
    #     if 'LShoulderPitch' in joints:
    #         joints['LShoulderPitch'].setPosition(1.5)
    #     if 'RShoulderPitch' in joints:
    #         joints['RShoulderPitch'].setPosition(1.5)
    #     if 'LElbowRoll' in joints:
    #         joints['LElbowRoll'].setPosition(-0.5)
    #     if 'RElbowRoll' in joints:
    #         joints['RElbowRoll'].setPosition(0.5)
    #
    #     # 重置 PID 控制器变量
    #     error_sum_pitch = 0.0
    #     last_error_pitch = 0.0
# while robot.step(timestep) != -1:
    # key = keyboard.getKey()
    # if key == ord('W'):
    #     nao_motion.walk_motion()
    # pass
# Enter here exit cleanup code.
