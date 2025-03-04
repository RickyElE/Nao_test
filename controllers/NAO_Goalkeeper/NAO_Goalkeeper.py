from controller import Supervisor, Robot, Motion, motion
import json
from enum import Enum,auto,unique
import numpy as np
import os

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

class DEFEND_STAGE(Enum):
    INITIAL = auto()
    PREPARE = auto()
    X_AXIS_ADJUST = auto()
    ADJUSTING_ANGLE = auto()
    SIDE_STEP_ADJUST = auto()
    BOUNDARY_ADJUST = auto()
    BACK_TO_REGISTERED = auto()
    HUSTLE = auto()
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

class HUSTLE(Enum):
    INITIAL = auto()
    PREPARE = auto()
    HUSTLE_LEFT = auto()
    HUSTLE_RIGHT = auto()
    WAITING = auto()
    FINISH = auto()
    END = auto()

class GOAL_KEEPER(Enum):
    INITIAL = auto()
    PREPARE = auto()
    DEFEND = auto()
    HUSTLE = auto()
    STAND_UP = auto()
    KICK_OUT = auto()
    BACK_TO_MIDDLE = auto()
    FINISH = auto()
    END = auto()

class STAND_UP(Enum):
    INITIAL = auto()
    PREPARE = auto()
    FROM_FRONT = auto()
    FROM_BACK = auto()
    FINISH = auto()
    END = auto()

class GOAL_KEEPER_ROLE(Enum):
    REDTEAM_GOALKEEPER = "RedTeam_Goalkeeper"
    BLUETEAM_GOALKEEPER = "BlueTeam_Goalkeeper"

class Nao_Goalkeeper(Robot):
    PHALANX_MAX = 8

    def loadMotionFiles(self):
        '''
        This Function mainly loads the motion files from libraries
        '''
        current_path = os.path.abspath(__file__)
        current_folder_path = os.path.dirname(current_path)
        pre_folder_path = os.path.dirname(current_folder_path)
        pre_pre_folder_path = os.path.dirname(pre_folder_path)
        self.forwards = Motion(os.path.join(pre_pre_folder_path, 'libraries/Forwards.motion'))
        self.backwards = Motion(os.path.join(pre_pre_folder_path, 'libraries/Backwards.motion'))
        self.shoot = Motion(os.path.join(pre_pre_folder_path, 'libraries/Shoot.motion'))
        self.turnleft40 = Motion(os.path.join(pre_pre_folder_path, 'libraries/TurnLeft40.motion'))
        self.turnright40 = Motion(os.path.join(pre_pre_folder_path, 'libraries/TurnRight40.motion'))
        self.sidestepleft = Motion(os.path.join(pre_pre_folder_path, 'libraries/SideStepLeft.motion'))
        self.sidestepright = Motion(os.path.join(pre_pre_folder_path, 'libraries/SideStepRight.motion'))
        self.KICK = Motion(os.path.join(pre_pre_folder_path, 'libraries/KICK.motion'))
        self.StandUpFromFront = Motion(os.path.join(pre_pre_folder_path, 'libraries/StandUpFromFront.motion'))
        self.StandUpFromBack = Motion(os.path.join(pre_pre_folder_path, 'libraries/StandUpFromBack.motion'))
        self.ReturnFromSide = Motion(os.path.join(pre_pre_folder_path, 'libraries/ReturnFromSide.motion'))

    def startMotion(self, motion):
        # interrupt current motion
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()

        # start new motion
        motion.play()
        self.currentlyPlaying = motion

    def stopMotion(self):
        if self.currentlyPlaying:
            duration = self.currentlyPlaying.getDuration()
            print("duration is: ", duration)
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

    def position_is_arrive(self, joints="", targets=None):
        '''
        This function checks if the target position is arrived.

        :param joints:
        :param targets:
        :return: if the target position is arrived, return True, else return False
        '''
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
        '''
        This function gets the current movestage.
        :param joints:
        :return: The joints current movestage
        '''
        if joints == "":
            print("You do not set any joints")
            return

        if joints in self.motor_names:
            return self.movestage[joints]
        else:
            print(f"{joints} cannot not found")
            return

    def setMotorPosition(self, joints="", targets=None):
        '''
        This function sets the joints desired motor positions.
        :param joints:
        :param targets:
        :return: If the joint is "" or the target is None, return False
        '''
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
        '''
        This is the constructor, only be used once by creating the object.
        It will initialize all the motors and the sensors.
        If you want to initialize some parameters you need, you can do it in here.
        '''
        Robot.__init__(self)
        print('NAO_GoalKeeper has initialized')
        self.currentlyPlaying = False
        self.wait_frames = 0
        self.isTurningRight = None
        self.countOfXAxisRetryTime = 0
        self.maxRetryTimes = 5
        # initialize stuff
        self.findAndEnableDevices()
        self.loadMotionFiles()
        self.emitter = self.getDevice("emitter")
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

        self.gk_stage = DEFEND_STAGE.INITIAL
        self.b2mp_stage = BACK_MIDDLE.INITIAL
        self.kick_stage = KICK_STAGE.INITIAL
        self.__robot_position = None
        self.__football_position = None
        self.__robot_orientation = None
        self.__mate_stadium_position = None
        self.__mate_stadium_orientation = None
        self.__mate_defender_left_position = None
        self.__mate_defender_right_position = None
        self.__mate_striker_position = None
        self.__mate_defender_left_orientation = None
        self.__mate_defender_right_orientation = None
        self.__mate_striker_orientation = None

        self.__has_turn2correct_direction = False
        self.hustle_status = HUSTLE.INITIAL
        self.__is_hustle = False
        self.__is_standup = False
        self.run_stage = GOAL_KEEPER.INITIAL
        self.__standup_stage = STAND_UP.INITIAL

        self.__temp_time = 0

        self.__goalkeeper_name = self.getName()
        self.__goalkeeper_list = ["RedTeam_GoalKeeper", "BlueTeam_GoalKeeper"]
        self.__count_time = 0

        self.__redteam_goal_area = [[3.9, 4.5],
                                    [-1.05, 1.05]]

        self.__blueteam_goal_area = [[-3.9, -4.5],
                                     [-1.05, 1.05]]

        self.__ball_position_history = []
        self.__max_position_history  = 5

        self.__acc_history = []
        self.__max_acc_history = 5

        """Previous Stage"""
        self.__pre_kick_stage = KICK_STAGE.INITIAL
        self.__pre_gk_stage = DEFEND_STAGE.INITIAL
        self.__pre_b2mp_stage = BACK_MIDDLE.INITIAL
        self.__pre_hustle_stage = HUSTLE.INITIAL
        self.__pre_run_stage = GOAL_KEEPER.INITIAL
        self.__pre_standup_stage = STAND_UP.INITIAL

    def set_stage(self, stage=None):
        '''
        This function sets the current stage.
        It will check the stage belongs to which one.
        :param stage:
        :return:
        '''
        if stage is None:
            print("You do not set any stage")
            return

        if isinstance(stage, MOTION_PLAY):
            self.tracking_stage = stage
        elif isinstance(stage, KICK_STAGE):
            self.__pre_kick_stage = self.kick_stage
            self.kick_stage = stage
        elif isinstance(stage, HUSTLE):
            self.__pre_hustle_stage = self.hustle_status
            self.hustle_status = stage
        elif isinstance(stage, DEFEND_STAGE):
            self.__pre_gk_stage = self.gk_stage
            self.gk_stage = stage
        elif isinstance(stage, BACK_MIDDLE):
            self.__pre_b2mp_stage = BACK_MIDDLE
            self.b2mp_stage = stage
        elif isinstance(stage, GOAL_KEEPER):
            self.__pre_run_stage = self.run_stage
            self.run_stage = stage
        elif isinstance(stage, STAND_UP):
            self.__pre_standup_stage = self.__standup_stage
            self.__standup_stage = stage
        else:
            print(f"Stage {stage} not supported")
        return

    def trackingBall(self):
        '''
        This function tracks the ball. If the distance between the ball and robot is less than the epsilon_.

        INITIAL stage: Initialize the joints for ready to walk.

        PREPARE stage: Check the angle, if the angle is not suitable, then adjust it, Otherwise walk to the ball.

        :return:
        '''
        epsilon_ = 0.2
        if self.__robot_position is None or self.__robot_orientation is None or self.__football_position is None:
            print("robot_position or robot_orientation or football_position is None!")
            return

        angle, distance = self.angleCalculaor(self.__football_position, self.__robot_position, self.__robot_orientation)
        if angle is None or distance is None:
            print("angle is None or distance is None!")
            return

        if self.tracking_stage == MOTION_PLAY.INITIAL:
            print("MOTION PLAY INITIAL")
            # L_pitch = self.sensors['LShoulderPitch'].getValue()
            # R_pitch = self.sensors['RShoulderPitch'].getValue()
            # print("L_pitch", L_pitch)
            # print("R_pitch", R_pitch)
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

    def is_balanced(self):
        vel = self.gyro.getValues()
        # print('angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1]))
        # print(vel[0], vel[1])
        # all_in_balance = np.round(np.float64(vel[0])) == 0.0 and np.round(np.float64(vel[1])) == 0.0
        all_in_balance = np.isclose(np.float64(vel[0]),0.0, atol=0.1) and np.isclose(np.float64(vel[1]),0.0, atol=0.1)
        # print(f"all_in_balance: {all_in_balance}")
        return all_in_balance

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

    def position_refresh(self):
        if self.receiver.getQueueLength() > 0:
            data = self.receiver.getString()
            shared_info = json.loads(data)
            robot_position = robot_orientation = mate_defender_left_position = mate_defender_right_position\
            = mate_striker_position\
            = mate_defender_left_orientation\
            = mate_defender_right_orientation\
            = mate_striker_orientation       \
            = mate_stadium_position          \
            = mate_stadium_orientation = None
            if self.__goalkeeper_name == GOAL_KEEPER_ROLE.REDTEAM_GOALKEEPER.value:
                robot_position = np.float64(shared_info["goalkeeper_red"]["position"])
                robot_orientation = np.float64(shared_info["goalkeeper_red"]["orientation"])
                mate_defender_left_position      = np.float64(shared_info["RedTeam_DefenderLeft"]["position"])
                mate_defender_right_position     = np.float64(shared_info["RedTeam_DefenderRight"]["position"])
                mate_striker_position            = np.float64(shared_info["striker_red"]["position"])
                mate_defender_left_orientation   = np.float64(shared_info["RedTeam_DefenderLeft"]["orientation"])
                mate_defender_right_orientation  = np.float64(shared_info["RedTeam_DefenderRight"]["orientation"])
                mate_striker_orientation         = np.float64(shared_info["striker_red"]["orientation"])
                mate_stadium_position            = np.float64(shared_info["stadiumgoal_red"]["position"])
                mate_stadium_orientation         = np.float64(shared_info["stadiumgoal_red"]["orientation"])
            elif self.__goalkeeper_name == GOAL_KEEPER_ROLE.BLUETEAM_GOALKEEPER.value:
                robot_position = np.float64(shared_info["goalkeeper_blue"]["position"])
                robot_orientation = np.float64(shared_info["goalkeeper_blue"]["orientation"])
                mate_defender_left_position      = np.float64(shared_info["BlueTeam_DefenderLeft"]["position"])
                mate_defender_right_position     = np.float64(shared_info["BlueTeam_DefenderRight"]["position"])
                mate_striker_position            = np.float64(shared_info["striker_blue"]["position"])
                mate_defender_left_orientation   = np.float64(shared_info["BlueTeam_DefenderLeft"]["orientation"])
                mate_defender_right_orientation  = np.float64(shared_info["BlueTeam_DefenderRight"]["orientation"])
                mate_striker_orientation         = np.float64(shared_info["striker_blue"]["orientation"])
                mate_stadium_position            = np.float64(shared_info["stadiumgoal_blue"]["position"])
                mate_stadium_orientation         = np.float64(shared_info["stadiumgoal_blue"]["orientation"])
            else:
                print("Unknown goalkeeper")

            football_position = np.float64(shared_info["football"]["position"])

            self.receiver.nextPacket()
            self.__robot_position = robot_position
            self.__robot_orientation = robot_orientation
            self.__football_position = football_position
            self.__mate_defender_left_position      = mate_defender_left_position
            self.__mate_defender_right_position     = mate_defender_right_position
            self.__mate_striker_position            = mate_striker_position
            self.__mate_defender_left_orientation   = mate_defender_left_orientation
            self.__mate_defender_right_orientation  = mate_defender_right_orientation
            self.__mate_striker_orientation         = mate_striker_orientation
            self.__mate_stadium_position            = mate_stadium_position
            self.__mate_stadium_orientation         = mate_stadium_orientation
            self.__ball_position_history.append(self.__football_position)
            if len(self.__ball_position_history) > self.__max_position_history:
                self.__ball_position_history.pop(0)

            self.__acc_history.append(self.accelerometer.getValues())
            if len(self.__acc_history) > self.__max_acc_history:
                self.__acc_history.pop(0)
            return True
            # return robot_position, robot_orientation, football_position
        else:
            # return None, None, None
            self.__robot_position = None
            self.__robot_orientation = None
            self.__football_position = None
            self.__mate_defender_left_position  = None
            self.__mate_defender_right_position = None
            self.__mate_striker_position        = None
            self.__mate_defender_left_orientation = None
            self.__mate_defender_right_orientation = None
            self.__mate_striker_orientation = None
            return False

    def __check_teammate_near_goal_area(self):
        '''
            This function will check the positions of each teammate.
            If they are in the goal area,
            the function will calculate the minimization of distance between goalkeeper and other teammates.
            Then return the nearest teammate position.
            Else, return None.
        '''
        position = {"mate_left_defender"    :  self.__mate_defender_left_position,
                    "mate_right_defender"   :  self.__mate_defender_right_position,
                    "mate_striker"          :  self.__mate_striker_position}
        def check_min_dist(position):
            dist = []
            for key,value in position.items():
                _, distance = self.angleCalculaor(value, self.__robot_position, self.__robot_orientation)
                dist.append(distance)
            if len(dist) == 0:
                return None
            min_dist = min(dist)
            pointer = dist.index(min_dist)
            key = list(position.keys())[pointer]
            value = position[key]
            return value
        def is_in_goal_area(position, goal_area):
            x, y = position
            x_min, x_max = goal_area[0]
            y_min, y_max = goal_area[1]
            return x_min <= x <= x_max and y_min <= y <= y_max
        need2consider = {}
        for key, value in position.items():
            print(f"value: {value[0:2]}")
            if self.__goalkeeper_name == GOAL_KEEPER_ROLE.BLUETEAM_GOALKEEPER.value:
                if is_in_goal_area(value[0:2], self.__blueteam_goal_area):
                    need2consider[key] = value
                else:
                    continue
            else:
                if is_in_goal_area(value[0:2], self.__redteam_goal_area):
                    need2consider[key] = value
                else:
                    continue
        if need2consider:
            return check_min_dist(need2consider)
        else:
            return None


    # Calculate the angle
    def angleCalculaor(self, football_position, robot_position, orientation = None):
        dx = football_position[0] - robot_position[0]
        dy = football_position[1] - robot_position[1]
        target_magnitude = np.sqrt(dx ** 2 + dy ** 2)
        target_vector_normalized = [dx / target_magnitude, dy / target_magnitude]
        if orientation is not None:
            front_magnitude = np.sqrt(orientation[0] ** 2 + orientation[3] ** 2)
            front_vector_normalized = [orientation[0] / front_magnitude, orientation[3] / front_magnitude]
            dot_product = sum(f * t for f, t in zip(front_vector_normalized, target_vector_normalized))
            angle = np.rad2deg(np.arccos(dot_product))
            cross_product = front_vector_normalized[0] * target_vector_normalized[1] - \
                        front_vector_normalized[1] * target_vector_normalized[0]
        # print(f"cross_product: {cross_product}")
            if cross_product < 0:
                angle = -angle
        else:
            angle = np.rad2deg(np.arctan2(dy, dx))
        distance = np.sqrt(dx ** 2 + dy ** 2)
        return angle, distance

    def goalkeeper_registered_check(self):
        bitsOfRound = 2
        limitationofdistance = 1.0
        if (np.round(self.__robot_position[1],2) <= 1.05 + 0.25
                and np.round(self.__robot_position[1],2) >= - (1.05 + 0.25)
                and np.round(self.__robot_position[0],2) <= 3.90 + limitationofdistance
                and np.round(self.__robot_position[0],2) >= 3.90 - limitationofdistance):
            return True
        else:
            return False

    def __ball_has_moved(self):
        if len(self.__ball_position_history) < 2:
            return False
        position_changes = [
            np.linalg.norm(np.array(self.__ball_position_history[i]) - np.array(self.__ball_position_history[i - 1]))
            for i in range(1, len(self.__ball_position_history))
        ]
        max_change = max(position_changes)
        if max_change > 0.05:
            return True
        else:
            return False

    def defendingBall(self):
        limitationofdistance = 0.1
        bitsOfRound = 2
        # robot_position, robot_orientation, football_position = self.position_refresh()
        if self.__robot_position is None or self.__robot_orientation is None or self.__football_position is None:
            print("robot_position or robot_orientation or football_position is None!")
            return

        angle, distance = self.angleCalculaor(self.__football_position, self.__robot_position, self.__robot_orientation)
        if angle is None or distance is None:
            print("angle is None or distance is None!")
            return
        print(f"angle is {angle}")
        print(f"distance is {distance}")
        # if ((self.__goalkeeper_name == GOAL_KEEPER_ROLE.REDTEAM_GOALKEEPER.value and self.__football_position[0] < 0)
        #         or (self.__goalkeeper_name == GOAL_KEEPER_ROLE.BLUETEAM_GOALKEEPER.value and self.__football_position[0] > 0)):
        #     # print("football position is negative")
        #     return

        if self.gk_stage == DEFEND_STAGE.INITIAL:
            print("DEFEND INITIAL")
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
                self.previous_stage = self.gk_stage
                self.gk_stage = DEFEND_STAGE.PREPARE
                return
            else:
                return
        elif self.gk_stage == DEFEND_STAGE.PREPARE:
            print("DEFEND PREPARE")
            front_angle, front_distance = self.angleCalculaor([0.0, self.__robot_position[1], 0.0], self.__robot_position,
                                                              self.__robot_orientation)
            print("front_angle:", front_angle)
            print("front_distance:", front_distance)

            if np.abs(np.round(front_angle,1)) > 15.0:
                print("Need to adjust front_angle!")
                self.stopMotion()
                self.previous_stage = self.gk_stage
                self.gk_stage = DEFEND_STAGE.ADJUSTING_ANGLE
                return

            if self.__goalkeeper_name == GOAL_KEEPER_ROLE.REDTEAM_GOALKEEPER.value and ((np.round(np.round(front_distance,bitsOfRound),1) > 3.90 + limitationofdistance)
                    or (np.round(np.round(front_distance,bitsOfRound),1) < 3.90 - limitationofdistance)):
                print("Need to adjust X_Axis!")
                self.stopMotion()
                self.previous_stage = self.gk_stage
                self.gk_stage = DEFEND_STAGE.X_AXIS_ADJUST
                return
            elif self.__goalkeeper_name == GOAL_KEEPER_ROLE.BLUETEAM_GOALKEEPER.value and ((np.round(np.round(front_distance,bitsOfRound),1) > (3.90 + limitationofdistance))
                    or (np.round(np.round(front_distance,bitsOfRound),1) < (3.90 - limitationofdistance))):
                print("Need to adjust X_Axis!")
                self.stopMotion()
                self.previous_stage = self.gk_stage
                self.gk_stage = DEFEND_STAGE.X_AXIS_ADJUST
                return

            if self.__goalkeeper_name == GOAL_KEEPER_ROLE.REDTEAM_GOALKEEPER.value:
                if ((angle >= 15.0 and self.__robot_position[1] >= -1.05) or (
                        angle <= -15.0 and self.__robot_position[1] <= 1.05)
                ):
                    self.__temp_angle = angle
                    self.__temp_position = self.__football_position
                    self.previous_stage = self.gk_stage
                    self.gk_stage = DEFEND_STAGE.ADJUSTING_ANGLE
                    return
                else:
                    self.previous_stage = self.gk_stage
                    self.gk_stage = DEFEND_STAGE.FINISH
                    return
            elif self.__goalkeeper_name == GOAL_KEEPER_ROLE.BLUETEAM_GOALKEEPER.value:
                if ((angle >= 15.0 and self.__robot_position[1] >= -1.05) or (
                        angle <= -15.0 and self.__robot_position[1] <= 1.05)
                ):
                    self.__temp_angle = angle
                    self.__temp_position = self.__football_position
                    self.previous_stage = self.gk_stage
                    self.gk_stage = DEFEND_STAGE.ADJUSTING_ANGLE
                    return
                else:
                    self.previous_stage = self.gk_stage
                    self.gk_stage = DEFEND_STAGE.FINISH
                    return
            else:
                print("Goalkeeper ROLE NOT FOUND")

        elif self.gk_stage == DEFEND_STAGE.ADJUSTING_ANGLE:
            print("DEFEND ADJUSTING_ANGLE")
            front_angle, front_distance = self.angleCalculaor([0.0, self.__robot_position[1], 0.0], self.__robot_position, self.__robot_orientation)
            print(front_angle)
            if self.__goalkeeper_name == GOAL_KEEPER_ROLE.REDTEAM_GOALKEEPER.value:
                if np.round(np.abs(front_angle), 1) >= 15.0:
                    if ((180.0 >= np.round(front_angle, 1) >= 15.0)
                            or (180.0 >= np.round(front_angle, 1) >= 15.0 and self.isTurningRight is None)
                            or self.isTurningRight):
                        self.startMotion(self.turnright40)
                        self.isTurningRight = True
                    elif ((-180.0 <= np.round(front_angle, 1) <= -15.0)
                          or (-180.0 <= np.round(front_angle, 1) <= -15.0 and self.isTurningRight is None)
                          or not self.isTurningRight):
                        self.startMotion(self.turnleft40)
                        self.isTurningRight = False
                if np.round(np.abs(front_angle), 1) < 15.0:
                    self.stopMotion()
                    self.isTurningRight = None
                    if self.previous_stage == DEFEND_STAGE.FINISH:
                        self.previous_stage = self.gk_stage
                        if (np.round(np.round(front_distance, bitsOfRound), 1) > 3.90 + limitationofdistance
                                or np.round(np.round(front_distance, bitsOfRound), 1) < 3.90 - limitationofdistance):
                            self.gk_stage = DEFEND_STAGE.X_AXIS_ADJUST
                    else:
                        self.__temp_angle = angle
                        self.__temp_position = self.__football_position
                        self.previous_stage = self.gk_stage
                        self.gk_stage = DEFEND_STAGE.SIDE_STEP_ADJUST
                    return
                else:
                    return
            elif self.__goalkeeper_name == GOAL_KEEPER_ROLE.BLUETEAM_GOALKEEPER.value:
                if np.round(np.abs(front_angle), 1) >= 15.0:
                    if ((180.0 >= np.round(front_angle, 1) >= 15.0)
                            or (180.0 >= np.round(front_angle, 1) >= 15.0 and self.isTurningRight is None)
                            or self.isTurningRight):
                        self.startMotion(self.turnright40)
                        self.isTurningRight = True
                    elif ((-180.0 <= np.round(front_angle, 1) <= -15.0)
                          or (-180.0 <= np.round(front_angle, 1) <= -15.0 and self.isTurningRight is None)
                          or not self.isTurningRight):
                        self.startMotion(self.turnleft40)
                        self.isTurningRight = False
                if np.round(np.abs(front_angle), 1) < 15.0:
                    self.stopMotion()
                    self.isTurningRight = None
                    if self.previous_stage == DEFEND_STAGE.FINISH:
                        self.previous_stage = self.gk_stage
                        if (np.round(np.round(front_distance, bitsOfRound), 1) > 3.90 + limitationofdistance
                                or np.round(np.round(front_distance, bitsOfRound), 1) < 3.90 - limitationofdistance):
                            self.gk_stage = DEFEND_STAGE.X_AXIS_ADJUST
                    else:
                        self.__temp_angle = angle
                        self.__temp_position = self.__football_position
                        self.previous_stage = self.gk_stage
                        self.gk_stage = DEFEND_STAGE.SIDE_STEP_ADJUST
                    return
                else:
                    return


        elif self.gk_stage == DEFEND_STAGE.SIDE_STEP_ADJUST:
            print("DEFEND SIDE_STEP_ADJUST")
            print("__temp_angle", self.__temp_angle)
            if self.__temp_angle >= 15.0 and self.__robot_position[1] >= -1.05:
                self.startMotion(self.sidestepleft)

            elif self.__temp_angle <= -15.0 and self.__robot_position[1] <= 1.05:
                self.startMotion(self.sidestepright)

            self.__temp_angle, judge_distance = self.angleCalculaor(self.__football_position, self.__robot_position,
                                                                    self.__robot_orientation)
            front_angle, front_distance = self.angleCalculaor([0.0, self.__robot_position[1], 0.0], self.__robot_position,self.__robot_orientation)
            print("front_angle:", front_angle)
            print("front_distance:", front_distance)
            lim_pos = self.__check_teammate_near_goal_area()
            if (np.round(np.abs(front_angle),1) >= 15.0
                    # and self.previous_stage != DEFEND_STAGE.BOUNDARY_ADJUST
            ):
                print("Need to adjust front_angle!")
                self.stopMotion()
                self.previous_stage = self.gk_stage
                self.gk_stage = DEFEND_STAGE.ADJUSTING_ANGLE
                return

            if self.__goalkeeper_name == GOAL_KEEPER_ROLE.REDTEAM_GOALKEEPER.value and ((np.round(np.round(front_distance,bitsOfRound),1) > (3.90 + limitationofdistance))
                    or (np.round(np.round(front_distance,bitsOfRound),1) < (3.90 - limitationofdistance))):
                print("Need to adjust X_Axis!")
                # self.stopMotion()
                self.previous_stage = self.gk_stage
                self.gk_stage = DEFEND_STAGE.X_AXIS_ADJUST
                return
            elif self.__goalkeeper_name == GOAL_KEEPER_ROLE.BLUETEAM_GOALKEEPER.value and ((np.round(np.round(front_distance,bitsOfRound),1) > (3.90 + limitationofdistance))
                    or (np.round(np.round(front_distance,bitsOfRound),1) < (3.90 - limitationofdistance))):
                print("Need to adjust X_Axis!")
                # self.stopMotion()
                self.previous_stage = self.gk_stage
                self.gk_stage = DEFEND_STAGE.X_AXIS_ADJUST
                return

            if lim_pos is None:
                if (not (self.__temp_angle >= 15.0 and self.__robot_position[1] >= -1.05)
                        and not (self.__temp_angle <= -15.0 and self.__robot_position[1] <= 1.05)
                ):
                    self.previous_stage = self.gk_stage
                    self.gk_stage = DEFEND_STAGE.FINISH
                    return
                else:
                    return
            else:
                ang2Teammate, dist2Teammate = self.angleCalculaor(lim_pos, self.__robot_position, self.__robot_orientation)
                if (ang2Teammate/np.abs(ang2Teammate)) * (self.__temp_angle/np.abs(self.__temp_angle)) > 0:
                    if np.round(np.abs(dist2Teammate),1) <= 0.5:
                        self.previous_stage = self.gk_stage
                        self.gk_stage = DEFEND_STAGE.FINISH
                        return
                    else:
                        return
                else:
                    if (not (self.__temp_angle >= 15.0 and self.__robot_position[1] >= -1.05)
                            and not (self.__temp_angle <= -15.0 and self.__robot_position[1] <= 1.05)
                    ):
                        self.previous_stage = self.gk_stage
                        self.gk_stage = DEFEND_STAGE.FINISH
                        return
                    else:
                        return

        elif self.gk_stage == DEFEND_STAGE.X_AXIS_ADJUST:
            print("DEFEND X_AXIS_ADJUST")
            front_angle, front_distance = self.angleCalculaor([0.0, self.__robot_position[1], 0.0], self.__robot_position,self.__robot_orientation)
            print("front_distance:", np.round(np.round(front_distance,bitsOfRound),2))
            if (np.round(np.abs(front_angle),1) >= 15.0
            ):
                print("Need to adjust front_angle!")
                self.stopMotion()
                self.previous_stage = self.gk_stage
                self.gk_stage = DEFEND_STAGE.ADJUSTING_ANGLE
                return
            if self.__goalkeeper_name == GOAL_KEEPER_ROLE.REDTEAM_GOALKEEPER.value:
                if np.round(np.round(front_distance, bitsOfRound), 1) > 3.90 + limitationofdistance:
                    print("Is adjusting forwards!")
                    self.startMotion(self.forwards)
                    self.countOfXAxisRetryTime += 1

                elif np.round(np.round(front_distance, bitsOfRound), 1) < 3.90 - limitationofdistance:
                    print("Is adjusting backwards!")
                    self.startMotion(self.backwards)
                    self.countOfXAxisRetryTime += 1

                if (3.90 + limitationofdistance >= np.round(np.round(front_distance, bitsOfRound),
                                                            1) >= 3.90 - limitationofdistance
                        # or self.countOfXAxisRetryTime > self.maxRetryTimes
                ):
                    # self.stopMotion()
                    temp = self.previous_stage
                    self.previous_stage = self.gk_stage
                    self.gk_stage = temp
                    # if self.previous_stage == DEFEND_STAGE.PREPARE:
                    #     self.previous_stage = self.gk_stage
                    #     self.gk_stage = DEFEND_STAGE.PREPARE
                    # else:
                    #     self.previous_stage = self.gk_stage
                    #     self.gk_stage = DEFEND_STAGE.FINISH
                    return
                else:
                    return
            elif self.__goalkeeper_name == GOAL_KEEPER_ROLE.BLUETEAM_GOALKEEPER.value:
                if np.round(np.round(front_distance, bitsOfRound), 1) > (3.90 + limitationofdistance):
                    print("Is adjusting forwards!")
                    self.startMotion(self.forwards)
                    self.countOfXAxisRetryTime += 1

                elif np.round(np.round(front_distance, bitsOfRound), 1) < (3.90 - limitationofdistance):
                    print("Is adjusting backwards!")
                    self.startMotion(self.backwards)
                    self.countOfXAxisRetryTime += 1

                if ((3.90 + limitationofdistance) >= np.round(np.round(front_distance, bitsOfRound),
                                                            1) >= (3.90 - limitationofdistance)
                        # or self.countOfXAxisRetryTime > self.maxRetryTimes
                ):
                    # self.stopMotion()
                    temp = self.previous_stage
                    self.previous_stage = self.gk_stage
                    self.gk_stage = temp
                    # if self.previous_stage == DEFEND_STAGE.PREPARE:
                    #     self.previous_stage = self.gk_stage
                    #     self.gk_stage = DEFEND_STAGE.PREPARE
                    # else:
                    #     self.previous_stage = self.gk_stage
                    #     self.gk_stage = DEFEND_STAGE.FINISH
                    return
                else:
                    return
        elif self.gk_stage == DEFEND_STAGE.FINISH:
            print("DEFEND FINISH")
            front_angle, front_distance = self.angleCalculaor([0.0, self.__robot_position[1], 0.0], self.__robot_position, self.__robot_orientation)
            if self.is_balanced():
                if (np.round(np.abs(front_angle), 1) >= 15.0
                        # and self.previous_stage != DEFEND_STAGE.BOUNDARY_ADJUST
                ):
                    self.previous_stage = self.gk_stage
                    self.gk_stage = DEFEND_STAGE.ADJUSTING_ANGLE
                else:
                    self.previous_stage = self.gk_stage
                    self.gk_stage = DEFEND_STAGE.END
            return
        elif self.gk_stage == DEFEND_STAGE.END:
            print("DEFEND END")
            def restart():
                if (
                        (angle > 15.0 and np.round(self.__robot_position[1], bitsOfRound) < -1.05)
                        or (angle < -15.0 and np.round(self.__robot_position[1], bitsOfRound) > 1.05)
                        or ( 1.05 >= np.round(self.__robot_position[1],bitsOfRound) >= -1.05 and (angle > 15.0 or angle < -15.0))
                        # and ((self.__goalkeeper_name == GOAL_KEEPER_ROLE.REDTEAM_GOALKEEPER.value and self.__football_position[0] >= 0)
                        #      or (self.__goalkeeper_name == GOAL_KEEPER_ROLE.BLUETEAM_GOALKEEPER.value and self.__football_position[0] <= 0))
                )and self.__ball_has_moved():
                    return True
                else:
                    return False
            if self.is_balanced() and restart():
                    self.previous_stage = self.gk_stage
                    self.gk_stage = DEFEND_STAGE.INITIAL
                    # self.__has_turn2correct_direction = False
            return
        else:
            print("Unknown stage")
            return False

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

    def is_standup(self):
        pass
        if self.__standup_stage == STAND_UP.INITIAL:
            print("Stand-Up INITIAL")
            self.__standup_stage = STAND_UP.PREPARE
            return
        elif self.__standup_stage == STAND_UP.PREPARE:
            print("Stand-Up PREPARE")
            self.__standup_stage = STAND_UP.FROM_BACK
            self.__count_time = self.getTime()
            return
        elif self.__standup_stage == STAND_UP.FROM_FRONT:
            print("Stand-Up From FRONT")
            return
        elif self.__standup_stage == STAND_UP.FROM_BACK:
            print("Stand-Up From BACK")
            if (self.currentlyPlaying is not self.StandUpFromBack or self.currentlyPlaying.isOver()):
                self.startMotion(self.StandUpFromBack)
            if self.is_balanced():
                self.__standup_stage = STAND_UP.FINISH
            return
        elif self.__standup_stage == STAND_UP.FINISH:
            print("Stand-Up FINISH")
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
                    self.getMoveStage('RElbowYaw') is move_status.END
                    and self.is_balanced()
            ):
                self.__standup_stage = STAND_UP.END
            return
        elif self.__standup_stage == STAND_UP.END:
            print("Stand-Up END")
            return
        else:
            print("Unknown stand-up stage")

    def backtomiddlepoint(self):
        limitationofdistance = 0.1
        bitsOfRound = 2
        if self.__robot_position is None or self.__robot_orientation is None or self.__football_position is None:
            print("robot_position or robot_orientation or football_position is None!")
            return

        angle = distance = None
        if self.__goalkeeper_name == GOAL_KEEPER_ROLE.REDTEAM_GOALKEEPER.value:
            angle, distance = self.angleCalculaor([3.9, 0.0 ,0.0], self.__robot_position, self.__robot_orientation)
            if angle is None or distance is None:
                print("angle is None or distance is None!")
                return
        elif self.__goalkeeper_name == GOAL_KEEPER_ROLE.BLUETEAM_GOALKEEPER.value:
            angle, distance = self.angleCalculaor([-3.9, 0.0, 0.0], self.__robot_position, self.__robot_orientation)
            if angle is None or distance is None:
                print("angle is None or distance is None!")
                return

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
                self.b2mp_stage = BACK_MIDDLE.FACING_TO_MIDDLE
                return
        elif self.b2mp_stage == BACK_MIDDLE.FACING_TO_MIDDLE:
            print("Back to middlepoint FACING_TO_MIDDLE!")
            front_angle, front_distance = self.angleCalculaor([0.0, self.__robot_position[1], 0.0], self.__robot_position,
                                                              self.__robot_orientation)
            print(front_angle)
            if np.round(np.abs(front_angle), 1) >= 15.0:
                if ((180.0 >= np.round(front_angle, 1) >= 15.0)
                        or (180.0 >= np.round(front_angle, 1) >= 15.0 and self.isTurningRight is None)
                        or self.isTurningRight):
                    self.startMotion(self.turnright40)
                    self.isTurningRight = True
                elif ((-180.0 <= np.round(front_angle, 1) <= -15.0)
                      or (-180.0 <= np.round(front_angle, 1) <= -15.0 and self.isTurningRight is None)
                      or not self.isTurningRight):
                    self.startMotion(self.turnleft40)
                    self.isTurningRight = False

            if np.round(np.abs(front_angle), 1) < 15.0:
                self.isTurningRight = None
                self.b2mp_stage = BACK_MIDDLE.FINISH
                return
            else:
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
            print(f"distance: {distance}")
            front_angle, front_distance = self.angleCalculaor([0.0, self.__robot_position[1], 0.0],
                                                              self.__robot_position,
                                                              self.__robot_orientation)
            if self.is_balanced():
                if (self.__football_position[0] < 0 and
                        # np.round(distance, bitsOfRound) > 0.2
                    not np.isclose(distance, 0.2, atol=1e-1)
                ):
                    self.b2mp_stage = BACK_MIDDLE.INITIAL
            return
        else:
            print("Unknown Stage!")
            return

    def __initialize_hustle(self):
        print("Initializing Hustle!")
        motor_names = [
            'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
            'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll',
            'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
            'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'
        ]
        for name in motor_names:
            if name in self.motors:
                self.setMotorPosition(name, 0.0)
        all_in_end = all(self.getMoveStage(n) is move_status.END for n in motor_names)
        return all_in_end

    def __pre_hustle(self):
        pass
        print("Prepare to hustle!")
        hus_position = {
            #hand
            'LShoulderPitch': 1.49,
            'LShoulderRoll': 0.000000086,
            'LElbowYaw': 0.000000049,
            'LElbowRoll': -0.49,
            'RShoulderPitch': 1.49,
            'RShoulderRoll': -0.000000086,
            'RElbowYaw': 0.000000049,
            'RElbowRoll': 0.49
        }
        for name, position in hus_position.items():
            if name in self.motors:
                self.setMotorPosition(name, position)

        all_in_end = all(self.getMoveStage(n) is move_status.END for n in self.motor_names)
        if all_in_end and self.is_balanced():
            return True
        else:
            return False

    def hustle(self):
        pass

        if self.__robot_position is None or self.__robot_orientation is None or self.__football_position is None:
            print("robot_position or robot_orientation or football_position is None!")
            return

        angle, distance = self.angleCalculaor(self.__football_position, self.__robot_position, self.__robot_orientation)
        if angle is None or distance is None:
            print("angle is None or distance is None!")
            return
        angbetballstadium, distbetballstadium = self.angleCalculaor(self.__football_position, self.__mate_stadium_position,
                                              self.__mate_stadium_orientation)
        if angbetballstadium is None and distbetballstadium is None:
            print("angle is None or distance is None!")
            return
        if self.hustle_status == HUSTLE.INITIAL:
            print("Hustle initiated!")
            # self.setMotorPosition('LShoulderPitch', 1.49)
            # self.setMotorPosition('RShoulderPitch', 1.49)
            # self.setMotorPosition('LShoulderRoll', 0.000000086)
            # self.setMotorPosition('RShoulderRoll', -0.000000086)
            # self.setMotorPosition('LElbowRoll', -0.49)
            # self.setMotorPosition('RElbowRoll', 0.49)
            # self.setMotorPosition('LElbowYaw', 0.000000049)
            # self.setMotorPosition('RElbowYaw', -0.000000049)
            # self.setMotorPosition('LKneePitch', 0.0000020595)
            # self.setMotorPosition('RKneePitch', 0.0000023045)
            # if (self.getMoveStage('LShoulderPitch') is move_status.END and
            #         self.getMoveStage('RShoulderPitch') is move_status.END and
            #         self.getMoveStage('LShoulderRoll') is move_status.END and
            #         self.getMoveStage('RShoulderRoll') is move_status.END and
            #         self.getMoveStage('LElbowRoll') is move_status.END and
            #         self.getMoveStage('RElbowRoll') is move_status.END and
            #         self.getMoveStage('LElbowYaw') is move_status.END and
            #         self.getMoveStage('RElbowYaw') is move_status.END and
            #         self.getMoveStage('LKneePitch') is move_status.END and
            #         self.getMoveStage('RKneePitch') is move_status.END
            #         ):
            #     self.hustle_status = HUSTLE.PREPARE
            #     return
            # else:
            #     return
            if self.__initialize_hustle() :
                self.__is_hustle = True
                self.hustle_status = HUSTLE.PREPARE
                return
            else:
                return
        elif self.hustle_status == HUSTLE.PREPARE:
            print("Hustle prepared!")
            if self.__pre_hustle():
                if angbetballstadium < 0:
                    if angle < 0:
                        self.hustle_status = HUSTLE.HUSTLE_RIGHT
                        return
                    else:
                        self.hustle_status = HUSTLE.HUSTLE_LEFT
                        return
                else:
                    if angle < 0:
                        self.hustle_status = HUSTLE.HUSTLE_RIGHT
                        return
                    else:
                        self.hustle_status = HUSTLE.HUSTLE_LEFT
                        return
            return
        elif self.hustle_status == HUSTLE.HUSTLE_RIGHT:
            print("Hustle hustle_right!")
            self.setMotorPosition("RShoulderRoll", -1.326)
            self.setMotorPosition("RShoulderPitch", -2.08)
            self.setMotorPosition("RElbowYaw", 0.0)
            self.setMotorPosition("RElbowRoll", 0.0)
            # self.setMotorPosition("RWristYaw", 1.82)
            self.setMotorPosition("RHipRoll", 0.379)
            self.setMotorPosition("LHipRoll", 0.379)
            if (self.getMoveStage('RShoulderRoll') is move_status.END
                and self.getMoveStage("RShoulderPitch") is move_status.END
                and self.getMoveStage('RElbowYaw') is move_status.END
                and self.getMoveStage("RElbowRoll") is move_status.END
                and self.getMoveStage("LHipRoll") is move_status.END
            ):
                self.hustle_status = HUSTLE.FINISH
                return
            else:
                return
        elif self.hustle_status == HUSTLE.HUSTLE_LEFT:
            print("Hustle hustle_left!")
            # Hustle Left
            self.setMotorPosition("LShoulderRoll", 1.326)
            self.setMotorPosition("LShoulderPitch", 2.08)
            self.setMotorPosition("LElbowYaw", 0.0)
            self.setMotorPosition("LElbowRoll", 0.0)
            self.setMotorPosition("LHipRoll", -0.379)
            self.setMotorPosition("RHipRoll", -0.379)
            if (self.getMoveStage('LShoulderRoll') is move_status.END
                and self.getMoveStage("LShoulderPitch") is move_status.END
                and self.getMoveStage('LElbowYaw') is move_status.END
                and self.getMoveStage("LElbowRoll") is move_status.END
                and self.getMoveStage("LHipRoll") is move_status.END
                and self.getMoveStage("RHipRoll") is move_status.END
                ):
                self.__temp_time = self.getTime()
                self.hustle_status = HUSTLE.WAITING
                return
            else:
                return
        elif self.hustle_status == HUSTLE.WAITING:
            print("Hustle waiting!")
            time = self.getTime()
            if time - self.__temp_time > 20:
                self.hustle_status = HUSTLE.FINISH
                return
            else:
                return
        elif self.hustle_status == HUSTLE.FINISH:
            print("Hustle finished!")
            self.hustle_status = HUSTLE.END
            return
        elif self.hustle_status == HUSTLE.END:
            print("Hustle ended!")
            # self.__is_hustle = False
            # self.hustle_status = HUSTLE.INITIAL
            return
        else:
            print("Unknown Hustle status!")

    def __check_ball_has_passed_teammate(self):
        position = {"mate_left_defender": self.__mate_defender_left_position,
                    "mate_right_defender": self.__mate_defender_right_position,
                    "mate_striker": self.__mate_striker_position}
        orientation = {"mate_left_defender": self.__mate_defender_left_orientation,
                       "mate_right_defender": self.__mate_defender_right_orientation,
                       "mate_striker": self.__mate_striker_orientation}
        ang = []
        dist = []
        has_passed = 0
        for key, value in position.items():
            # print(f"{key}: {value}")
            # print(f"{key}: {orientation[key]}")
            angle, distance = self.angleCalculaor(self.__football_position, position[key], orientation[key])
            print(f"{key}: {angle}, {distance}")
            ang.append(angle)
            dist.append(distance)
        print(f"the angle of ball between to each robot is {ang}")
        for value in ang:
            if value >= 90 or value <= -90:
                has_passed += 1

        if has_passed == 3:
            return True
        else:
            return False

    def run(self):
        if self.run_stage == GOAL_KEEPER.INITIAL:
            print("Run Initial!")
            if self.standupIfnecessary():
                self.set_stage(STAND_UP.INITIAL)
                self.set_stage(GOAL_KEEPER.STAND_UP)
            else:
                self.set_stage(GOAL_KEEPER.PREPARE)
            return
        elif self.run_stage == GOAL_KEEPER.PREPARE:
            print("Run Prepare!")
            if self.standupIfnecessary():
                self.set_stage(STAND_UP.INITIAL)
                self.set_stage(GOAL_KEEPER.STAND_UP)
            else:
                self.set_stage(GOAL_KEEPER.DEFEND)
            return
        elif self.run_stage == GOAL_KEEPER.DEFEND:
            print("Run Defend!")
            result = self.__check_ball_has_passed_teammate()
            if self.standupIfnecessary():
                self.set_stage(STAND_UP.INITIAL)
                self.set_stage(GOAL_KEEPER.STAND_UP)
                return
            elif ((self.__goalkeeper_name == GOAL_KEEPER_ROLE.REDTEAM_GOALKEEPER.value
                  and (3.90 - self.__football_position[0]) <= 0.5
                  and result)
                or (self.__goalkeeper_name == GOAL_KEEPER_ROLE.BLUETEAM_GOALKEEPER.value
                  and (-3.90 - self.__football_position[0]) >= -0.5)
                  and result):
                self.set_stage(GOAL_KEEPER.HUSTLE)
                return
            else:
                self.defendingBall()

        elif self.run_stage == GOAL_KEEPER.HUSTLE:
            print("Run Hustle!")
            self.hustle()
            if self.hustle_status == HUSTLE.END:
                self.set_stage(STAND_UP.INITIAL)
                self.set_stage(GOAL_KEEPER.STAND_UP)
            return
        elif self.run_stage == GOAL_KEEPER.STAND_UP:
            print("Run Stand Up!")
            self.is_standup()
            if self.__standup_stage == STAND_UP.END:
                self.set_stage(GOAL_KEEPER.BACK_TO_MIDDLE)
                # if self.__pre_run_stage == GOAL_KEEPER.HUSTLE:
                #     self.set_stage(GOAL_KEEPER.KICK_OUT)
                # else:
                #     self.set_stage(GOAL_KEEPER.DEFEND)
                #     self.set_stage(DEFEND_STAGE.INITIAL)
            return
        elif self.run_stage == GOAL_KEEPER.KICK_OUT:
            print("Run Kick Out!")
            if self.standupIfnecessary():
                self.set_stage(STAND_UP.INITIAL)
                self.set_stage(GOAL_KEEPER.STAND_UP)
                return
            if self.trackingBall():
                if self.kick_ball():
                    self.set_stage(GOAL_KEEPER.BACK_TO_MIDDLE)
                    return
            return
        elif self.run_stage == GOAL_KEEPER.BACK_TO_MIDDLE:
            print("Run Back to Middle!")
            if self.standupIfnecessary():
                self.set_stage(STAND_UP.INITIAL)
                self.set_stage(GOAL_KEEPER.STAND_UP)
                return
            self.backtomiddlepoint()
            if self.b2mp_stage == BACK_MIDDLE.END:
                self.set_stage(GOAL_KEEPER.DEFEND)
                return
            else:
                return
        elif self.run_stage == GOAL_KEEPER.FINISH:
            print("Run Finish!")
            self.set_stage(GOAL_KEEPER.END)
            return
        elif self.run_stage == GOAL_KEEPER.END:
            print("Run End!")
            self.set_stage(GOAL_KEEPER.INITIAL)
            return
        else:
            print("Unknown run stage!")

    def test_module(self):
        pos = self.__check_teammate_near_goal_area()
        print(f"pos = {pos}")
        result = self.__ball_has_moved()
        print(f"result = {result}")
        result = self.__check_ball_has_passed_teammate()
        print(f"result = {result}")
        angle, distance = self.angleCalculaor(self.__football_position, self.__mate_defender_left_position, self.__mate_defender_left_orientation)
        print(f"angle = {angle}")
        self.standupIfnecessary()

goal_keeper = Nao_Goalkeeper()
while goal_keeper.step(goal_keeper.timeStep) != -1:
    pass
    if goal_keeper.position_refresh():
        goal_keeper.run()
        # goal_keeper.test_module()

    # goal_keeper.startMotion(goal_keeper.StandUpFromBack)
    # # # goal_keeper.hustle()
    # if not goal_keeper.standupIfnecessary():
    #     # goal_keeper.hustle()
    #     goal_keeper.defendingBall()
    #     goal_keeper.backtomiddlepoint()
    # print(f"prediction is {goal_keeper.ball_dir_calculator()}")
    # print("L Knee pitch is ",format(goal_keeper.sensors["LKneePitch"].getValue(),'.10f'))
    # print("R Knee pitch is ",format(goal_keeper.sensors["RKneePitch"].getValue(),'.10f'))
    # goal_keeper.setMotorPosition("RElbowYaw", 2.08)
    # print(goal_keeper.gyro.getValues())
    # roll, pitch, _ = goal_keeper.inertialUnit.getRollPitchYaw()
    # print(np.rad2deg(roll), np.rad2deg(pitch))