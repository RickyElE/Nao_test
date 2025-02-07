import math
import typing

from controller import Supervisor, Motion, motion, Robot
import numpy as np
from enum import Enum,auto,unique
import json
import os
import logging.config
from pathlib import Path

# cur_dir = Path(__file__).resolve().parents[2]
#
# #initial logging
# config_path = cur_dir / 'libraries' /'logging.conf'
#
# log_path = cur_dir / "log"
# log_path.mkdir(parents=True, exist_ok=True)
# # logger.info(log_path)
#
# # #set two handlers
# log_file_name= log_path / "{}.log".format(Path(__file__).stem)
# logging.config.fileConfig(config_path, defaults={'logfilename': str(log_file_name)})
# logger = logging.getLogger()

@unique
class KICK_STAGE(Enum):
    INITIAL = auto()
    PREPARE = auto()
    WEIGHT_SHIFT = auto()
    BEND_LEFT_LEG = auto()
    KICK = auto()
    LEG_IN = auto()
    COMPLETE = auto()
    IS_BALANCE = auto()
    END = auto()

class move_status(Enum):
    INITIAL = auto()
    PREPARE = auto()
    MOVING  = auto()
    FINISH  = auto()
    END     = auto()

class MOTION_PLAY(Enum):
    INITIAL = auto()
    PREPARE = auto()
    PLAYING = auto()
    ADJUSTING_ANGLE = auto()
    SIDE_STEP_ADJUST = auto()
    FINISH = auto()
    END = auto()

class DRIBBLE(Enum):
    INITIAL = auto()
    PREPARE = auto()
    BALLFINDING = auto()
    ADJUSTING_ANGLE = auto()
    ADJUSTING_COORDINATE = auto()
    DRIBBLING = auto()
    CHECK_SHOOT = auto()
    FINISH = auto()
    END = auto()

class STRIKER(Enum):
    INITIAL = auto()
    PREPARE = auto()
    DRIBBLE = auto()
    KICK = auto()
    BACK2MIDLINE = auto()
    STAND_UP = auto()
    FINISH = auto()
    END = auto()

class STAND_UP(Enum):
    INITIAL = auto()
    PREPARE = auto()
    FROM_FRONT = auto()
    FROM_BACK = auto()
    FINISH = auto()
    END = auto()

class BACK_MIDDLE(Enum):
    INITIAL = auto()
    PREPARE = auto()
    ANGLE_ADJUSTING = auto()
    FACING_TO_MIDDLE = auto()
    MOVING = auto()
    FINISH = auto()
    END = auto()

class NAO_RedTeam_Striker(Robot):
    PHALANX_MAX = 8
    kick_stage = KICK_STAGE.INITIAL

    def loadMotionFiles(self):
        current_path = os.path.abspath(__file__)
        current_folder_path = os.path.dirname(current_path)
        pre_folder_path = os.path.dirname(current_folder_path)
        pre_pre_folder_path = os.path.dirname(pre_folder_path)
        self.forwards = Motion(os.path.join(pre_pre_folder_path,'libraries/Forwards.motion'))
        self.backwards = Motion(os.path.join(pre_pre_folder_path,'libraries/Backwards.motion'))
        self.shoot = Motion(os.path.join(pre_pre_folder_path,'libraries/Shoot.motion'))
        self.turnleft40 = Motion(os.path.join(pre_pre_folder_path,'libraries/TurnLeft40.motion'))
        self.turnright40 = Motion(os.path.join(pre_pre_folder_path,'libraries/TurnRight40.motion'))
        self.sidestepleft = Motion(os.path.join(pre_pre_folder_path,'libraries/SideStepLeft.motion'))
        self.sidestepright = Motion(os.path.join(pre_pre_folder_path,'libraries/SideStepRight.motion'))
        self.KICK = Motion(os.path.join(pre_pre_folder_path,'libraries/KICK.motion'))
        self.StandUpFromFront = Motion(os.path.join(pre_pre_folder_path,'libraries/StandUpFromFront.motion'))
        self.StandUpFromBack = Motion(os.path.join(pre_pre_folder_path,'libraries/StandUpFromBack.motion'))
        self.ReturnFromSide = Motion(os.path.join(pre_pre_folder_path,'libraries/ReturnFromSide.motion'))

    def startMotion(self, motion):
        # interrupt current motion
        # if self.currentlyPlaying:
        #     self.currentlyPlaying.stop()

        # start new motion
        motion.play()
        self.currentlyPlaying = motion

    def stopMotion(self):
        if self.currentlyPlaying:
            duration = self.currentlyPlaying.getDuration()
            print("duration is: ",duration)
            self.currentlyPlaying.stop()
            self.currentlyPlaying.setTime(duration)

    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())

        self.__threshold = 0.0001

        # camera
        self.cameraTop = self.getDevice("CameraTop")
        self.cameraBottom = self.getDevice("CameraBottom")
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom.enable(4 * self.timeStep)

        # accelerometer
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(4 * self.timeStep)

        # gyro
        self.gyro = self.getDevice('gyro')
        self.gyro.enable(4 * self.timeStep)

        # gps
        self.gps = self.getDevice('gps')
        self.gps.enable(4 * self.timeStep)

        # inertial unit
        self.inertialUnit = self.getDevice('inertial unit')
        self.inertialUnit.enable(self.timeStep)

        # ultrasound sensors
        self.us = []
        usNames = ['Sonar/Left', 'Sonar/Right']
        for i in range(0, len(usNames)):
            self.us.append(self.getDevice(usNames[i]))
            self.us[i].enable(self.timeStep)

        # foot sensors
        self.fsr = []
        fsrNames = ['LFsr', 'RFsr']
        for i in range(0, len(fsrNames)):
            self.fsr.append(self.getDevice(fsrNames[i]))
            self.fsr[i].enable(self.timeStep)

        # foot bumpers
        self.lfootlbumper = self.getDevice('LFoot/Bumper/Left')
        self.lfootrbumper = self.getDevice('LFoot/Bumper/Right')
        self.rfootlbumper = self.getDevice('RFoot/Bumper/Left')
        self.rfootrbumper = self.getDevice('RFoot/Bumper/Right')
        self.lfootlbumper.enable(self.timeStep)
        self.lfootrbumper.enable(self.timeStep)
        self.rfootlbumper.enable(self.timeStep)
        self.rfootrbumper.enable(self.timeStep)

        # there are 7 controlable LED groups in Webots
        self.leds = []
        self.leds.append(self.getDevice('ChestBoard/Led'))
        self.leds.append(self.getDevice('RFoot/Led'))
        self.leds.append(self.getDevice('LFoot/Led'))
        self.leds.append(self.getDevice('Face/Led/Right'))
        self.leds.append(self.getDevice('Face/Led/Left'))
        self.leds.append(self.getDevice('Ears/Led/Right'))
        self.leds.append(self.getDevice('Ears/Led/Left'))

        # get phalanx motor tags
        # the real Nao has only 2 motors for RHand/LHand
        # but in Webots we must implement RHand/LHand with 2x8 motors
        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        for i in range(0, self.PHALANX_MAX):
            self.lphalanx.append(self.getDevice("LPhalanx%d" % (i + 1)))
            self.rphalanx.append(self.getDevice("RPhalanx%d" % (i + 1)))

            # assume right and left hands have the same motor position bounds
            self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
            self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())

        # shoulder pitch motors
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

        # Initialize motors
        self.tracking_stage = MOTION_PLAY.INITIAL
        self.motors = {}
        self.sensors = {}
        self.movestage = {}
        self.__previous_targets = {}
        self.motor_names = [
            'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
            'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll',
            'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
            'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'
        ]
        print('Initializing motors...', flush=True)
        for name in self.motor_names:
            motor = self.getDevice(name)
            if motor is not None:
                self.motors[name] = motor
                self.sensors[name] = self.motors[name].getPositionSensor()
                self.sensors[name].enable(self.timeStep)
                self.movestage[name] = move_status.INITIAL
                self.__previous_targets[name] = None
                print(f'Motor {name} initialized', flush=True)
            else:
                print(f'Failed to get motor: {name}', flush=True)

        # Initialize Emitter & Receiver
        self.emitter = self.getDevice('emitter')
        if self.emitter:
            self.emitter.setChannel(1)
        else:
            print("emitter not set")
        self.receiver = self.getDevice("receiver")
        if self.receiver:
            self.receiver.setChannel(1)
            self.receiver.enable(self.timeStep)
        else:
            print("receiver not set")

    def position_is_arrive(self,joints="",targets=None):
        if joints == "":
            print("You do not set any joints")
            return

        if targets is None:
            print("You do not set any positions")
            return

        if joints in self.motor_names:
            position = self.sensors[joints].getValue()
            passing = np.abs(np.abs(targets) - np.abs(position))
            if np.isclose(passing, self.__threshold, atol=0.1):
                return True
            # if passing <= self.__threshold:
            #     return True
            else:
                return False

    def getMoveStage(self, joints=""):
        if joints == "":
            print("You do not set any joints")
            return

        if joints in self.motor_names:
            return self.movestage[joints]
        else:
            print(f"{joints} cannot not found")
            return

    def setMotorPosition(self, joints="", targets=None):
        if joints == "":
            print("You do not set any joints")
            return

        if targets is None:
            print("You do not set any positions")
            return

        if joints in self.motor_names:
            if self.movestage[joints] == move_status.INITIAL:
                print(f"{joints} is in Initial")
                self.__previous_targets[joints] = targets
                self.movestage[joints] = move_status.PREPARE
                return
            elif self.movestage[joints] == move_status.PREPARE:
                print(f"{joints} is in Prepare")
                if targets is self.__previous_targets[joints]:
                    self.movestage[joints] = move_status.MOVING
                    return
                else:
                    return
            elif self.movestage[joints] == move_status.MOVING:
                print(f"{joints} is in Moving")
                if targets is self.__previous_targets[joints]:
                    self.motors[joints].setPosition(targets)
                    if self.position_is_arrive(joints, targets):
                        self.movestage[joints] = move_status.FINISH
                        return
                    else:
                        return
                else:
                    return
            elif self.movestage[joints] == move_status.FINISH:
                print(f"{joints} is in Finish")
                if targets is self.__previous_targets[joints]:
                    self.movestage[joints] = move_status.END
                    return
                else:
                    return
            elif self.movestage[joints] == move_status.END:
                print(f"{joints} is in End")
                if targets is not self.__previous_targets[joints]:
                    self.movestage[joints] = move_status.INITIAL
                else:
                    return
            else:
                print(f"{joints} in error stage!")

    def __init__(self):
        Robot.__init__(self)
        print('NAO_RedTeamStriker has initialized')
        self.currentlyPlaying = False
        self.wait_frames = 0
        # initialize stuff
        self.findAndEnableDevices()
        self.loadMotionFiles()


        # self.initial_angle = np.rad2deg(self.nao.getField("rotation").getSFFloat()[3])
        # print("initial angle is ", self.initial_angle)

        # Initializes the status value
        self.dribbling_status = DRIBBLE.INITIAL
        self.striker_stage = STRIKER.INITIAL
        self.__standup_stage = STAND_UP.INITIAL
        self.__pre_run_stage = STRIKER.INITIAL
        self.b2mp_stage = BACK_MIDDLE.INITIAL
        self.__pre_dribble_stage = DRIBBLE.INITIAL
        self.__side_count = 0
        self.__temp_angle = 0

    def position_refresh(self):
        if self.receiver.getQueueLength() > 0:
            data = self.receiver.getString()
            shared_info = json.loads(data)
            robot_position = np.float64(shared_info["striker_red"]["position"])
            robot_orientation = np.float64(shared_info["striker_red"]["orientation"])

            football_position = np.float64(shared_info["football"]["position"])

            stadiumgoal_blue_position = np.float64(shared_info["stadiumgoal_blue"]["position"])
            stadiumgoal_blue_orientation = np.float64(shared_info["stadiumgoal_blue"]["orientation"])

            self.receiver.nextPacket()
            self.__robot_position = robot_position
            self.__robot_orientation = robot_orientation
            self.__football_position = football_position
            self.__stadiumgoal_blue_position = stadiumgoal_blue_position
            self.__stadiumgoal_blue_orientation= stadiumgoal_blue_orientation
            # return robot_position, robot_orientation, football_position
        else:
            # return None, None, None
            self.__robot_position = None
            self.__robot_orientation = None
            self.__football_position = None

    def set_stage(self, stage=None):
        if stage is None:
            print("You do not set any stage")
            return

        if isinstance(stage, MOTION_PLAY):
            self.tracking_stage = stage
        elif isinstance(stage, KICK_STAGE):
            self.kick_stage = stage
        elif isinstance(stage, DRIBBLE):
            self.dribbling_status = stage
        elif isinstance(stage, STRIKER):
            self.striker_stage = stage
        elif isinstance(stage, STAND_UP):
            self.__standup_stage = stage
        elif isinstance(stage, BACK_MIDDLE):
            self.b2mp_stage = stage
        else:
            print(f"Stage {stage} not supported")
        return

    def angleCalculaor(self, football_position, robot_position, orientation):
        pass
        # modified at 18/11 Mon
        dx = football_position[0] - robot_position[0]
        dy = football_position[1] - robot_position[1]
        # target_vector = [dx, dy]
        target_magnitude = np.sqrt(dx ** 2 + dy ** 2)
        target_vector_normalized = [dx / target_magnitude, dy / target_magnitude]
        # front_vector = [orientation[0], orientation[3]]
        front_magnitude = np.sqrt(orientation[0] ** 2 + orientation[3] ** 2)
        front_vector_normalized = [orientation[0] / front_magnitude, orientation[3] / front_magnitude]
        dot_product = sum(f * t for f, t in zip(front_vector_normalized, target_vector_normalized))
        angle = np.rad2deg(np.arccos(dot_product))
        cross_product = front_vector_normalized[0] * target_vector_normalized[1] - \
                        front_vector_normalized[1] * target_vector_normalized[0]
        if cross_product < 0:
            angle = -angle
        distance = np.sqrt(dx ** 2 + dy ** 2)
        return angle, distance
        # print(f"distance is {distance}")
        # print(f"angle is {angle}")
        # print(f"cross_product is {cross_product}")

    def trackingBall(self):
        epsilon_ = 0.2
        robot_position = self.__shared_info["striker_red"]["position"]
        football_position = self.__shared_info["football"]["position"]
        orientation = self.__shared_info["striker_red"]["orientation"]

        # # modified at 18/11 Mon
        # dx = football_position[0] - robot_position[0]
        # dy = football_position[1] - robot_position[1]
        # # target_vector = [dx, dy]
        # target_magnitude = np.sqrt(dx ** 2 + dy ** 2)
        # target_vector_normalized = [dx / target_magnitude, dy / target_magnitude]
        # orientation = self.__shared_info["striker_red"]["orientation"]
        # # front_vector = [orientation[0], orientation[3]]
        # front_magnitude = np.sqrt(orientation[0] ** 2 + orientation[3] ** 2)
        # front_vector_normalized = [orientation[0] / front_magnitude, orientation[3] / front_magnitude]
        # dot_product = sum(f * t for f, t in zip(front_vector_normalized, target_vector_normalized))
        # angle = np.rad2deg(np.arccos(dot_product))
        # cross_product = front_vector_normalized[0] * target_vector_normalized[0] - \
        #                 front_vector_normalized[1] * target_vector_normalized[1]
        # if cross_product < 0:
        #     angle = -angle
        # distance = np.sqrt(dx ** 2 + dy ** 2)
        # print(f"distance is {distance}")
        # print(f"angle is {angle}")
        # print(f"cross_product is {cross_product}")
        angle, distance = self.angleCalculaor(football_position, robot_position, orientation)
        print(f"The angle is {angle}, and the distance is {distance}")
        if angle is None or distance is None:
            print("Angle or distance not calculated")
            return
        if self.tracking_stage == MOTION_PLAY.INITIAL:
            print("MOTION PLAY INITIAL")
            # L_pitch = self.sensors['LShoulderPitch'].getValue()
            # R_pitch = self.sensors['RShoulderPitch'].getValue()
            # print("L_pitch", L_pitch)
            # print("R_pitch", R_pitch)
            self.setMotorPosition('LShoulderPitch',1.49)
            self.setMotorPosition('RShoulderPitch', 1.49)
            self.setMotorPosition('LShoulderRoll', 0.000000086)
            self.setMotorPosition('RShoulderRoll', -0.000000086)
            self.setMotorPosition('LElbowRoll', -0.49)
            self.setMotorPosition('RElbowRoll', 0.49)
            self.setMotorPosition('LElbowYaw', 0.000000049)
            self.setMotorPosition('RElbowYaw', -0.000000049)
            # print(f"L Shoulder Roll is {self.sensors['LShoulderRoll'].getValue()}, LElbowRoll is {self.sensors['LElbowRoll'].getValue()}, "
            #       f"LElbowYaw is {self.sensors['LElbowYaw'].getValue()}")
            # print(
            #     f"R Shoulder Roll is {self.sensors['RShoulderRoll'].getValue()}, RElbowRoll is {self.sensors['RElbowRoll'].getValue()}, "
            #     f"RElbowYaw is {self.sensors['RElbowYaw'].getValue()}")
            if (self.getMoveStage('LShoulderPitch') is move_status.END and
                self.getMoveStage('RShoulderPitch') is move_status.END and
                self.getMoveStage('LShoulderRoll') is move_status.END and
                self.getMoveStage('RShoulderRoll') is move_status.END and
                self.getMoveStage('LElbowRoll') is move_status.END and
                self.getMoveStage('RElbowRoll') is move_status.END and
                self.getMoveStage('LElbowYaw') is move_status.END and
                self.getMoveStage('RElbowYaw') is move_status.END):
                self.tracking_stage = MOTION_PLAY.PREPARE
                return
            else:
                return
        elif self.tracking_stage == MOTION_PLAY.PREPARE:
            print("MOTION PLAY PREPARE")

            if 180.0 >= angle >= 15.0 or -15.0 >= angle >= -180.0:
                self.__temp_angle = angle
                self.tracking_stage = MOTION_PLAY.ADJUSTING_ANGLE
                return
            else:
                self.tracking_stage = MOTION_PLAY.PLAYING
                return

        elif self.tracking_stage == MOTION_PLAY.ADJUSTING_ANGLE:
            print("MOTION PLAY ADJUSTING_ANGLE")
            if 180.0 >= self.__temp_angle >= 15.0:
                self.startMotion(self.turnright40)
            elif -15.0 >= self.__temp_angle >= -180.0:
                self.startMotion(self.turnleft40)

            if np.abs(angle) <= 15.0:
                self.stopMotion()
                self.tracking_stage = MOTION_PLAY.PLAYING
                return
            else:
                return
            # else:
            #     print("Distance is smaller than epsilon_")
            #     if x < 0 or y > 0:
            #         self.startMotion(self.turnright40)
            #     elif x > 0 or y < 0:
            #         self.startMotion(self.turnleft40)
            #     else:
            #         self.stopMotion()
            #         self.tracking_stage = MOTION_PLAY.FINISH
            #     return
        elif self.tracking_stage == MOTION_PLAY.PLAYING:
            print("MOTION PLAY PLAYING")
            print(f"distance is {distance}")
            # print(f"distance in x : {dx} y: {dy}")
            print(f"angle is {angle}")
            if distance >= epsilon_:
                if 180.0 >= angle >= 15.0 or -15.0 >= angle >= -180.0:
                    self.__temp_angle = angle
                    self.tracking_stage = MOTION_PLAY.ADJUSTING_ANGLE
                else:
                    self.startMotion(self.forwards)
                return
            else:
                self.stopMotion()
                self.__temp_angle = angle
                self.tracking_stage = MOTION_PLAY.SIDE_STEP_ADJUST
                return

        elif self.tracking_stage == MOTION_PLAY.SIDE_STEP_ADJUST:
            print("MOTION PLAY SIDE_STEP_ADJUST")
            print(f"distance is {distance}")
            print(f"angle is {angle}")
            if 180.0 >= self.__temp_angle >= 60.0 or -60.0 >= self.__temp_angle >= -180.0:
                if 90.0 >= self.__temp_angle >= 60.0:
                    self.startMotion(self.turnright40)

                if -60.0 >= self.__temp_angle >= -90.0:
                    self.startMotion(self.turnleft40)

            if 60.0 > self.__temp_angle >= 15.0 or -15.0 >= self.__temp_angle > -60.0:
                if 60.0 > self.__temp_angle >= 15.0:
                    self.startMotion(self.sidestepright)

                if -15.0 >= self.__temp_angle > -60.0:
                    self.startMotion(self.sidestepleft)

            else:
                self.tracking_stage = MOTION_PLAY.FINISH
                return

            if np.abs(angle) <= 15:
                self.stopMotion()
                self.tracking_stage = MOTION_PLAY.FINISH
                return
            else:
                return

        elif self.tracking_stage == MOTION_PLAY.FINISH:
            print("MOTION PLAY FINISH")
            if self.is_balanced():
                self.tracking_stage = MOTION_PLAY.END
            return
        elif self.tracking_stage == MOTION_PLAY.END:
            print("MOTION PLAY END")
            return True
        else:
            print("UNKNOWN MOTION PLAY STATUS!")

    def prepare_kick(self):
        print("Prepare to kick...", flush=True)
        initial_positions = {
            'LHipYawPitch': 0.0,
            'LHipRoll': 0.1,
            'LHipPitch': -0.4,  # -22.93
            'LKneePitch': 0.7,  # 40.127  #0.7
            'LAnklePitch': -0.3,
            'LAnkleRoll': -0.1,
            'RHipYawPitch': 0.0,
            'RHipRoll': -0.1,
            'RHipPitch': -0.4,
            'RKneePitch': 0.7,
            'RAnklePitch': -0.3,  # -17.197
            'RAnkleRoll': 0.1,
            # hand
            'LShoulderPitch': 1.57,
            'LShoulderRoll': 0.3,
            'LElbowYaw': -1.0,
            'LElbowRoll': -0.5,
            'RShoulderPitch': 1.57,
            'RShoulderRoll': -0.3,
            'RElbowYaw': 1.0,
            'RElbowRoll': 0.5
        }
        for name, position in initial_positions.items():
            if name in self.motors:
                self.setMotorPosition(name, position)

        all_in_end = all(self.getMoveStage(n) is move_status.END for n in self.motor_names)
        if all_in_end:
            return True
        else:
            return False

    def is_balanced(self):
        vel = self.gyro.getValues()
        # print('angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1]))
        # print(vel[0], vel[1])
        all_in_balance = np.isclose(np.float64(vel[0]),0.0, atol=0.1) and np.isclose(np.float64(vel[1]),0.0, atol=0.1)

        # print(f"all_in_balance: {all_in_balance}")
        return all_in_balance

    def kick_ball(self):
        if self.kick_stage is KICK_STAGE.INITIAL:
            print("KICK INITIAL")
            for name in self.motor_names:
                if name in self.motors:
                    self.setMotorPosition(name, 0.0)
            all_in_end = all(self.getMoveStage(n) is move_status.END for n in self.motor_names)
            if all_in_end and self.is_balanced():
                self.kick_stage = KICK_STAGE.PREPARE
                return
            else:
                return
        elif self.kick_stage is KICK_STAGE.PREPARE:
            print('Stage 0: PREPARE', flush=True)
            if self.prepare_kick() :
                self.kick_stage = KICK_STAGE.WEIGHT_SHIFT
                return
            else:
                return
        elif self.kick_stage is KICK_STAGE.WEIGHT_SHIFT:
            print('Stage 1: Weight shift', flush=True)
            self.setMotorPosition('LHipRoll',0.3) # 8.59
            self.setMotorPosition('RHipRoll',0.2)
            self.setMotorPosition('LAnkleRoll',-0.2)
            self.setMotorPosition('RAnkleRoll',-0.2)

            if (self.getMoveStage('LHipRoll') is move_status.END and
                    self.getMoveStage('RHipRoll') is move_status.END and
                    self.getMoveStage('LAnkleRoll') is move_status.END and
                    self.getMoveStage('RAnkleRoll') is move_status.END):
                self.kick_stage = KICK_STAGE.BEND_LEFT_LEG
                return
            else:
                return
        elif self.kick_stage is KICK_STAGE.BEND_LEFT_LEG:
            print('Stage 2: bend the left leg', flush=True)
            self.setMotorPosition('LKneePitch',2.1)
            self.setMotorPosition('LHipPitch',0.17) # 9.75
            self.setMotorPosition('LAnklePitch',-1.0)
            self.setMotorPosition('RShoulderRoll',-0.26)
            self.setMotorPosition('LShoulderRoll',-0.26)

            if (self.getMoveStage('LKneePitch') is move_status.END and
                self.getMoveStage('LHipPitch') is move_status.END and
                self.getMoveStage('LAnklePitch') is move_status.END and
                self.getMoveStage('RShoulderRoll') is move_status.END and
                self.getMoveStage('LShoulderRoll') is move_status.END):
                self.kick_stage = KICK_STAGE.KICK
                return
            else:
                return
        elif self.kick_stage is KICK_STAGE.KICK:
            print('Stage 3: Kick', flush=True)
            self.setMotorPosition('LHipPitch', -1.22)
            self.setMotorPosition('LAnklePitch', -0.3)
            if (self.getMoveStage('LHipPitch') is move_status.END and
                self.getMoveStage('LAnklePitch') is move_status.END):
                self.kick_stage = KICK_STAGE.LEG_IN
                return
            else:
                return
        elif self.kick_stage is KICK_STAGE.LEG_IN:
            print('Stage 4: LEG_IN', flush=True)
            self.setMotorPosition('LKneePitch', -0.09)
            if (self.getMoveStage('LKneePitch') is move_status.END):
                self.kick_stage = KICK_STAGE.COMPLETE
                return
            else:
                return
        elif self.kick_stage is KICK_STAGE.COMPLETE:
            print('Stage 5: Kick complete', flush=True)
            for name in self.motor_names:
                if name in self.motors:
                    self.setMotorPosition(name,0.0)

            all_in_end = all(self.getMoveStage(n) is move_status.END for n in self.motor_names)
            if all_in_end:
                self.kick_stage = KICK_STAGE.IS_BALANCE
                return
            else:
                return
        elif self.kick_stage is KICK_STAGE.IS_BALANCE:
            print('Stage 6: Is balance', flush=True)
            if self.is_balanced():
                self.kick_stage = KICK_STAGE.END
                return
            else:
                return
        elif self.kick_stage is KICK_STAGE.END:
            print('Stage 7: Kick end', flush=True)
            return True
        else:
            print("KICK STAGE ERROR", flush=True)

    def kick_motion(self):
        if self.kick_stage is KICK_STAGE.INITIAL:
            print("KICK INITIAL")
            if self.is_balanced():
                self.kick_stage = KICK_STAGE.PREPARE
                return
            else:
                return
        elif self.kick_stage is KICK_STAGE.PREPARE:
            print('Stage 0: PREPARE', flush=True)
            for name in self.motor_names:
                if name in self.motors:
                    self.setMotorPosition(name, 0.0)
            all_in_end = all(self.getMoveStage(n) is move_status.END for n in self.motor_names)
            if all_in_end and self.is_balanced():
                self.kick_stage = KICK_STAGE.KICK
            return
        elif self.kick_stage is KICK_STAGE.KICK:
            print('Stage 1: KICK', flush=True)
            self.startMotion(self.KICK)
            gettime = self.currentlyPlaying.getTime()
            if gettime == 2784: # the real time to stop
                self.stopMotion()
                self.kick_stage = KICK_STAGE.COMPLETE
            return
        elif self.kick_stage is KICK_STAGE.COMPLETE:
            print('Stage 2: KICK complete', flush=True)
            self.kick_stage = KICK_STAGE.IS_BALANCE
            return
        elif self.kick_stage is KICK_STAGE.IS_BALANCE:
            print('Stage 3: KICK IS_BALANCE', flush=True)
            if self.is_balanced():
                self.kick_stage = KICK_STAGE.END
            return
        elif self.kick_stage is KICK_STAGE.END:
            print('Stage 4: KICK END', flush=True)
            return True
        else:
            print("KICK STAGE ERROR", flush=True)

    def standupIfnecessary(self):
        Acc = self.accelerometer.getValues()
        # print(f"Acc is {Acc}")
        if (
                (Acc[2] < 5.0 and Acc[0] < -4.0)
                or (Acc[2] < 5.0 and Acc[0] > 4.0)
                or (Acc[2] < 5.0 and Acc[1] < -4.0)
                or (Acc[2] < 5.0 and Acc[1] > 4.0)
        ):
            return True
        else:
            return False
        # if Acc[2] < 5.0 and Acc[0] < -4.0:
        #     self.startMotion(self.StandUpFromFront)
        #     self.__is_standup = True
        #     return True
        # elif Acc[2] < 5.0 and Acc[0] > 4.0:
        #     self.startMotion(self.StandUpFromBack)
        #     self.__is_standup = True
        #     return True
        # elif Acc[2] < 5.0 and Acc[1] < -4.0:
        #     self.startMotion(self.ReturnFromSide)
        #     self.__is_standup = True
        #     return True
        # elif Acc[2] < 5.0 and Acc[1] > 4.0:
        #     self.startMotion(self.ReturnFromSide)
        #     self.__is_standup = True
        #     return True
        # else:
        #     self.__is_standup = False
        #     return False
        # roll, pitch, _ = self.inertialUnit.getRollPitchYaw()
        # if np.rad2deg(roll) < -120 and np.rad2deg(pitch) > 0:
        #     self.startMotion(self.StandUpFromFront)
        #     self.__is_standup = True
        #     return True
        # elif np.rad2deg(roll) < -120 and np.rad2deg(pitch) < 0:
        #     self.startMotion(self.StandUpFromBack)
        #     self.__is_standup = True
        #     return True
        # elif np.rad2deg(roll) > 80 and 0 < np.rad2deg(pitch) < 45:
        #     self.startMotion(self.ReturnFromSide)
        #     self.__is_standup = True
        #     return True
        # elif np.rad2deg(roll) > 170 and 0 < np.rad2deg(pitch) < 5:
        #     self.startMotion(self.ReturnFromSide)
        #     self.__is_standup = True
        #     return True
        # else:
        #     self.__is_standup = False
        #     return False

    def is_standup(self):
        pass
        if self.__standup_stage == STAND_UP.INITIAL:
            print("Stand-Up INITIAL")
            self.__standup_stage = STAND_UP.PREPARE
            return
        elif self.__standup_stage == STAND_UP.PREPARE:
            print("Stand-Up PREPARE")
            self.__standup_stage = STAND_UP.FROM_BACK
            return
        elif self.__standup_stage == STAND_UP.FROM_FRONT:
            print("Stand-Up From FRONT")
            return
        elif self.__standup_stage == STAND_UP.FROM_BACK:
            print("Stand-Up From BACK")
            self.startMotion(self.StandUpFromBack)
            if self.is_balanced():
                self.__standup_stage = STAND_UP.FINISH
            return
        elif self.__standup_stage == STAND_UP.FINISH:
            print("Stand-Up FINISH")
            self.__standup_stage = STAND_UP.END
            return
        elif self.__standup_stage == STAND_UP.END:
            print("Stand-Up END")
            return
        else:
            print("Unknown stand-up stage")

    def ballisonline(self):
        angbetsta, distbetsta = self.angleCalculaor(self.__stadiumgoal_blue_position, self.__robot_position, self.__robot_orientation)
        angbetball, disbetball = self.angleCalculaor(self.__football_position, self.__robot_position, self.__robot_orientation)
        print(f"angbetsta is {angbetsta}")
        print(f"angbetball is {angbetball}")
        # print(f"lfootrbumper is {self.lfootlbumper.getValue()},{self.lfootrbumper.getValue()},"
        #       f"rfootrbumper is {self.rfootlbumper.getValue()},{self.rfootrbumper.getValue()}")
        # dx_stadium2bot = stadium_position[0] - robot_position[0]
        # dy_stadium2bot = stadium_position[1] - robot_position[1]
        # dist_stadium2bot = np.sqrt(dx_stadium2bot ** 2 + dy_stadium2bot ** 2)
        #
        # k = dy_stadium2bot / dx_stadium2bot
        # b = robot_position[1] -  k * robot_position[0]
        # result = k * football_position[0] + b
        if self.__isonline(self.__robot_position, self.__stadiumgoal_blue_position, self.__football_position):
            print("Is online!")
        else:
            print("Is offline!")

    def __isonline(self, pos_1, pos_2, object):
        dx = pos_2[0] - pos_1[0]
        dy = pos_2[1] - pos_1[1]
        if np.isclose(dx, 0.0, atol=0.1) and np.isclose(object[0], dx, atol=0.1):
            return True
        k = dy / dx
        b = pos_2[1] - k * pos_2[0]
        # check the object is on line of y=kx+b
        result = k * object[0] + b
        # print(f"result is {result}")
        # print(f"object position is {object[1]}")
        # print(f"dist is {object[1] - result}")
        if np.isclose(object[1], result, atol=0.1):
            return True
        else:
            return False

    def dribble2stadium(self):
        angbetsta, distbetsta = self.angleCalculaor(self.__stadiumgoal_blue_position, self.__robot_position,
                                                    self.__robot_orientation)
        angbetball, disbetball = self.angleCalculaor(self.__football_position, self.__robot_position,
                                                     self.__robot_orientation)
        print(f"angbetsta is {angbetsta}, distbetsta is {distbetsta}")
        print(f"angbetball is {angbetball}, disbetball is {disbetball}")


        if self.dribbling_status == DRIBBLE.INITIAL:
            print("DRIBBLE INITIAL")
            self.setMotorPosition('LShoulderPitch', 1.49)
            self.setMotorPosition('RShoulderPitch', 1.49)
            self.setMotorPosition('LShoulderRoll', 0.000000086)
            self.setMotorPosition('RShoulderRoll', -0.000000086)
            self.setMotorPosition('LElbowRoll', -0.49)
            self.setMotorPosition('RElbowRoll', 0.49)
            self.setMotorPosition('LElbowYaw', 0.000000049)
            self.setMotorPosition('RElbowYaw', -0.000000049)
            if (self.getMoveStage('LShoulderPitch') is move_status.END and
                    self.getMoveStage('RShoulderPitch') is move_status.END and
                    self.getMoveStage('LShoulderRoll') is move_status.END and
                    self.getMoveStage('RShoulderRoll') is move_status.END and
                    self.getMoveStage('LElbowRoll') is move_status.END and
                    self.getMoveStage('RElbowRoll') is move_status.END and
                    self.getMoveStage('LElbowYaw') is move_status.END and
                    self.getMoveStage('RElbowYaw') is move_status.END):
                self.dribbling_status = DRIBBLE.PREPARE
                return
            else:
                return
        elif self.dribbling_status == DRIBBLE.PREPARE:
            print("DRIBBLE PREPARE")
            if self.__isonline(self.__robot_position, self.__stadiumgoal_blue_position, self.__football_position) and np.isclose(disbetball, 0.2, atol=0.1):
                self.dribbling_status = DRIBBLE.ADJUSTING_ANGLE
                return
            else:
                self.dribbling_status = DRIBBLE.BALLFINDING
                return
        elif self.dribbling_status == DRIBBLE.BALLFINDING:
            print("DRIBBLE BALLFINDING")
            if np.abs(angbetball) <= 5.0:
                self.startMotion(self.forwards)
                if np.isclose(disbetball, 0.2, atol=0.1):
                    if (self.__isonline(self.__robot_position, self.__stadiumgoal_blue_position, self.__football_position)
                        and np.abs(angbetsta) <= 30):
                        ''' If striker is facing to the opposite stadium goal and the ball is online, it can dribble the ball'''
                        self.dribbling_status = DRIBBLE.DRIBBLING
                    else:
                        self.__temp_angle = angbetsta
                        self.dribbling_status = DRIBBLE.ADJUSTING_ANGLE
                    return
                else:
                    return
            else:
                if 180.0 >= angbetball > 5.0 and self.is_balanced():
                    self.startMotion(self.turnleft40)
                elif -180 <= angbetball < -5.0 and self.is_balanced():
                    self.startMotion(self.turnright40)
                return

        elif self.dribbling_status == DRIBBLE.ADJUSTING_ANGLE:
            print("DRIBBLE ADJUSTING_ANGLE")
            if self.__temp_angle <= 30:
                if 0 <= np.abs(angbetsta) < 5:
                    self.__temp_pos = self.__robot_position
                    self.dribbling_status = DRIBBLE.ADJUSTING_COORDINATE
                    return
                else:
                    if isinstance(self.currentlyPlaying, bool):
                        self.startMotion(self.turnright40)
                    if angbetsta < 0 and self.currentlyPlaying.isOver() and self.is_balanced():
                        self.startMotion(self.turnright40)
                    else:
                        if self.currentlyPlaying.isOver() and self.is_balanced():
                            self.startMotion(self.turnleft40)
            else:
                if 0 <= np.abs(angbetsta) < 5:
                    self.dribbling_status = DRIBBLE.DRIBBLING
                    return
                else:
                    self.counterclockwise_winding(disbetball)

        elif self.dribbling_status == DRIBBLE.ADJUSTING_COORDINATE:
            print("DRIBBLE ADJUSTING_COORDINATE")
            if not self.__isonline(self.__football_position, self.__temp_pos, self.__robot_position):
                if self.__isonline(self.__robot_position, self.__stadiumgoal_blue_position, self.__football_position):
                    self.stopMotion()
                    self.dribbling_status = DRIBBLE.DRIBBLING
                    return
                else:
                    if angbetball < 0:
                        self.startMotion(self.sidestepright)
                        return
                    elif angbetball > 0:
                        self.startMotion(self.sidestepleft)
                        return
            else:
                self.startMotion(self.backwards)
                return

        elif self.dribbling_status == DRIBBLE.DRIBBLING:
            print("DRIBBLE DRIBBLING")
            if np.isclose(distbetsta, 1.60, atol=0.1):
                self.dribbling_status = DRIBBLE.CHECK_SHOOT
                return
            else:
                if np.abs(angbetball) > 90:
                    pass
                    # self.__pre_dribble_stage = self.dribbling_status
                    self.dribbling_status = DRIBBLE.BALLFINDING
                    return
                else:
                    if np.abs(angbetball) > 15:
                        if 180.0 >= angbetball > 15.0 and self.currentlyPlaying.isOver() and self.is_balanced():
                            self.startMotion(self.sidestepleft)
                        elif -180 <= angbetball < -15.0 and self.currentlyPlaying.isOver() and self.is_balanced():
                            self.startMotion(self.sidestepright)
                        # self.dribbling_status = DRIBBLE.BALLFINDING
                        return
                    else:
                        if np.abs(angbetsta) > 5:
                            pass
                            if angbetsta < -5 and self.currentlyPlaying.isOver() and self.is_balanced():
                                self.startMotion(self.turnright40)
                            elif angbetsta > 5 and self.currentlyPlaying.isOver() and self.is_balanced():
                                self.startMotion(self.turnleft40)
                        else:
                            self.startMotion(self.forwards)
                        return
        elif self.dribbling_status == DRIBBLE.CHECK_SHOOT:
            print("DRIBBLING CHECK_SHOOT")
            if 15 <= angbetball <= 20:
                self.startMotion(self.forwards)
                if np.isclose(disbetball, 0.2, atol=0.01):
                    self.stopMotion()
                    self.dribbling_status = DRIBBLE.FINISH
                    return
                else:
                    return
            else:
                if angbetball < 0 or 0 <= angbetball < 15:
                    self.startMotion(self.sidestepright)
                elif angbetball > 20:
                    self.startMotion(self.sidestepleft)
                return
        elif self.dribbling_status == DRIBBLE.FINISH:
            print("DRIBBLE FINISH")
            self.dribbling_status = DRIBBLE.END
            return
        elif self.dribbling_status == DRIBBLE.END:
            print("DRIBBLE END")
            return True
        else:
            print("UNKNOWN DRIBBLE STATUS!")

    def backtomiddlepoint(self):
        limitationofdistance = 0.1
        bitsOfRound = 2
        if self.__robot_position is None or self.__robot_orientation is None or self.__football_position is None:
            print("robot_position or robot_orientation or football_position is None!")
            return

        angle, distance = self.angleCalculaor([0.0, 0.0 ,0.0], self.__robot_position, self.__robot_orientation)
        if angle is None or distance is None:
            print("angle is None or distance is None!")
            return
        print(f"original point angle: {angle}, distance: {distance}")

        # if self.__football_position[0] >= 0:
        #     return
        if self.b2mp_stage == BACK_MIDDLE.INITIAL:
            print("Back to middlepoint Initial!")
            self.setMotorPosition('LShoulderPitch', 1.49)
            self.setMotorPosition('RShoulderPitch', 1.49)
            self.setMotorPosition('LShoulderRoll', 0.000000086)
            self.setMotorPosition('RShoulderRoll', -0.000000086)
            self.setMotorPosition('LElbowRoll', -0.49)
            self.setMotorPosition('RElbowRoll', 0.49)
            self.setMotorPosition('LElbowYaw', 0.000000049)
            self.setMotorPosition('RElbowYaw', -0.000000049)
            # print(f"L Shoulder Roll is {self.sensors['LShoulderRoll'].getValue()}, LElbowRoll is {self.sensors['LElbowRoll'].getValue()}, "
            #       f"LElbowYaw is {self.sensors['LElbowYaw'].getValue()}")
            # print(
            #     f"R Shoulder Roll is {self.sensors['RShoulderRoll'].getValue()}, RElbowRoll is {self.sensors['RElbowRoll'].getValue()}, "
            #     f"RElbowYaw is {self.sensors['RElbowYaw'].getValue()}")
            if (self.getMoveStage('LShoulderPitch') is move_status.END and
                    self.getMoveStage('RShoulderPitch') is move_status.END and
                    self.getMoveStage('LShoulderRoll') is move_status.END and
                    self.getMoveStage('RShoulderRoll') is move_status.END and
                    self.getMoveStage('LElbowRoll') is move_status.END and
                    self.getMoveStage('RElbowRoll') is move_status.END and
                    self.getMoveStage('LElbowYaw') is move_status.END and
                    self.getMoveStage('RElbowYaw') is move_status.END):
                self.b2mp_stage = BACK_MIDDLE.PREPARE
                return
            else:
                return
        elif self.b2mp_stage == BACK_MIDDLE.PREPARE:
            print("Back to middlepoint Prepare!")
            if np.round(distance, bitsOfRound) >= 0.2 and (180.0 >= angle >= 15.0 or -15.0 >= angle >= -180.0):
                self.__temp_angle = angle
                self.b2mp_stage = BACK_MIDDLE.ANGLE_ADJUSTING
                return
            else:
                self.b2mp_stage = BACK_MIDDLE.MOVING
                return
        elif self.b2mp_stage == BACK_MIDDLE.ANGLE_ADJUSTING:
            print("Back to middlepoint ANGLE_ADJUSTING!")
            if 180.0 >= self.__temp_angle > 15.0:
                self.startMotion(self.turnright40)
            elif -15.0 >= self.__temp_angle > -180.0:
                self.startMotion(self.turnleft40)

            if np.abs(angle) <= 15.0:
                self.stopMotion()
                self.b2mp_stage = BACK_MIDDLE.MOVING
                return
            else:
                return
        elif self.b2mp_stage == BACK_MIDDLE.MOVING:
            print("Back to middlepoint Moving!")
            print(f"distance: {distance}")
            # print(np.round(distance, bitsOfRound) >= 0.2)
            # print(self.b2mp_stage)
            if np.round(distance, bitsOfRound) >= 0.2:
                # print("Distance is over 0.2!")
                if 180.0 >= angle > 15.0 or -15.0 > angle >= -180.0:
                    self.__temp_angle = angle
                    self.b2mp_stage = BACK_MIDDLE.ANGLE_ADJUSTING
                    return
                else:
                    # print("Is moving forwards!")
                    self.startMotion(self.forwards)
                    # print("startMotion has been done!")
                    return
            else:
                print("Has arrived to the middlepoint!")
                # self.stopMotion()
                self.__temp_angle = angle
                self.b2mp_stage = BACK_MIDDLE.FINISH
                return
        elif self.b2mp_stage == BACK_MIDDLE.FINISH:
            print("Back to middlepoint FINISH!")
            # if np.round(distance, bitsOfRound) >= 0.2:
            #     if angle < 0:
            #         self.startMotion(self.forwards)
            #     else:
            #         self.startMotion(self.backwards)
            #     return
            # else:
            self.b2mp_stage = BACK_MIDDLE.END
            return
        elif self.b2mp_stage == BACK_MIDDLE.END:
            print("Back to middlepoint END!")
            return
        else:
            print("Unknown Stage!")
            return

    def counterclockwise_winding(self, distance):
        if self.currentlyPlaying.isOver() and distance <= 0.2:
            self.startMotion(self.backwards)
            return
        else:
            if isinstance(self.currentlyPlaying, bool):
                self.startMotion(self.turnleft40)
                self.__side_count = 0
                return
            else:
                if self.currentlyPlaying.isOver() and self.__side_count < 2:
                    self.startMotion(self.sidestepright)
                    self.__side_count += 1
                    return
                if self.currentlyPlaying.isOver() and self.__side_count >= 2:
                    self.startMotion(self.turnleft40)
                    self.__side_count = 0
                    return

    def clockwise_winding(self, distance):
        if self.currentlyPlaying.isOver() and distance <= 0.2:
            self.startMotion(self.backwards)
            return
        else:
            if isinstance(self.currentlyPlaying, bool):
                self.startMotion(self.turnright40)
                self.__side_count = 0
                return
            else:
                if self.currentlyPlaying.isOver() and self.__side_count < 2:
                    self.startMotion(self.sidestepleft)
                    self.__side_count += 1
                    return
                if self.currentlyPlaying.isOver() and self.__side_count >= 2:
                    self.startMotion(self.turnright40)
                    self.__side_count = 0
                    return

    def run(self):
        if self.striker_stage == STRIKER.INITIAL:
            print("STRIKER INITIAL")
            if self.standupIfnecessary():
                self.__pre_run_stage = self.striker_stage
                self.set_stage(STAND_UP.INITIAL)
                self.striker_stage = STRIKER.STAND_UP
                return
            else:
                self.striker_stage = STRIKER.PREPARE
            return
        elif self.striker_stage == STRIKER.PREPARE:
            print("STRIKER PREPARE")
            if self.standupIfnecessary():
                self.__pre_run_stage = self.striker_stage
                self.set_stage(STAND_UP.INITIAL)
                self.striker_stage = STRIKER.STAND_UP
                return
            self.striker_stage = STRIKER.DRIBBLE
            return
        elif self.striker_stage == STRIKER.DRIBBLE:
            print("STRIKER DRIBBLE")
            if self.standupIfnecessary():
                self.__pre_run_stage = self.striker_stage
                self.set_stage(STAND_UP.INITIAL)
                self.striker_stage = STRIKER.STAND_UP
                return
            self.dribble2stadium()
            if self.dribbling_status == DRIBBLE.END and self.is_balanced():
                self.striker_stage = STRIKER.KICK
                return
            else:
                return
        elif self.striker_stage == STRIKER.KICK:
            print("STRIKER KICK")
            if self.standupIfnecessary():
                self.__pre_run_stage = self.striker_stage
                self.set_stage(STAND_UP.INITIAL)
                self.striker_stage = STRIKER.STAND_UP
                return
            self.kick_motion()
            if self.kick_stage == KICK_STAGE.END:
                self.striker_stage = STRIKER.BACK2MIDLINE
                return
            else:
                return
        elif self.striker_stage == STRIKER.STAND_UP:
            print("STRIKER STAND_UP")
            self.is_standup()
            if self.__standup_stage == STAND_UP.END and self.is_balanced():
                self.striker_stage = STRIKER.DRIBBLE
                self.set_stage(DRIBBLE.INITIAL)
                return
            else:
                return
        elif self.striker_stage == STRIKER.BACK2MIDLINE:
            print("STRIKER BACK2MIDLINE")
            self.backtomiddlepoint()
            if self.b2mp_stage == BACK_MIDDLE.END:
                self.striker_stage = STRIKER.FINISH
        elif self.striker_stage == STRIKER.FINISH:
            print("STRIKER FINISH")
            self.striker_stage = STRIKER.END
            return
        elif self.striker_stage == STRIKER.END:
            print("STRIKER END")
            return
        else:
            print("UNKNOWN STRIKER STATUS!")
    # def test_module(self):
    #     robot_position = self.__shared_info['striker_red']['position']
    #     robot_orientation = self.__shared_info['striker_red']['orientation']
    #     football_position = self.__shared_info['football']['position']
    #     stadium_position = self.__shared_info['stadiumgoal_red']['position']
    #     angbetsta, distbetsta = self.angleCalculaor(stadium_position, robot_position, robot_orientation)
    #     angbetball, disbetball = self.angleCalculaor(football_position, robot_position, robot_orientation)
    #     print(f"angbetsta is {angbetsta}, distbetsta is {distbetsta}")
    #     print(f"angbetball is {angbetball}, disbetball is {disbetball}")

redteam_striker = NAO_RedTeam_Striker()
while redteam_striker.step(redteam_striker.timeStep) != -1:
    pass
    redteam_striker.position_refresh()
    redteam_striker.run()