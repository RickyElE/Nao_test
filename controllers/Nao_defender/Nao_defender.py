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
    FINISH = auto()
    END = auto()

class DEFENDER_STAGE(Enum):
    INITIAL = auto()
    INTERCEPT = auto()
    ADJUSTING_ANGLE = auto()
    APPROCH = auto()
    SIDE_STEP_ADJUST = auto()
    SIDE_STEP = auto()
    FINISH = auto()
    WAIT = auto()
    VICE_DEFEND = auto()
    STAND_UP = auto()

class STAND_UP(Enum):
    INITIAL = auto()
    PREPARE = auto()
    FROM_FRONT = auto()
    FROM_BACK = auto()
    FINISH = auto()
    END = auto()

class Nao_Goalkeeper(Robot):
    PHALANX_MAX = 8
    kick_stage = KICK_STAGE.INITIAL

    def loadMotionFiles(self):
        '''
        This Function mainly loads the motion files from libraries
        '''
        self.forwards = Motion('F:/Mrobotic/TDP/Nao_test_keeperAndDefender/libraries/Forwards.motion')
        self.backwards = Motion('F:/Mrobotic/TDP/Nao_test_keeperAndDefender/libraries/Backwards.motion')
        self.shoot = Motion('F:/Mrobotic/TDP/Nao_test_keeperAndDefender/libraries/Shoot.motion')
        self.turnleft40 = Motion('F:/Mrobotic/TDP/Nao_test_keeperAndDefender/libraries/TurnLeft40.motion')
        self.turnright40 = Motion('F:/Mrobotic/TDP/Nao_test_keeperAndDefender/libraries/TurnRight40.motion')
        self.sidestepleft = Motion('F:/Mrobotic/TDP/Nao_test_keeperAndDefender/libraries/SideStepLeft.motion')
        self.sidestepright = Motion('F:/Mrobotic/TDP/Nao_test_keeperAndDefender/libraries/SideStepRight.motion')
        self.KICK = Motion('F:/Mrobotic/TDP/Nao_test_keeperAndDefender/libraries/KICK.motion')

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
            if passing <= self.__threshold:
                return True
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
            self.kick_stage = stage
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
        robot_position = None
        football_position = None
        orientation = None
        if self.receiver.getQueueLength() > 0:
            data = self.receiver.getString()
            shared_info = json.loads(data)
            # print("shared_info:", shared_info)
            robot_position = float(shared_info["goalkeeper_red"]["position"])
            football_position = float(shared_info["football"]["position"])
            orientation = float(shared_info["goalkeeper_red"]["orientation"])
            print(robot_position)
            print(orientation)
            self.receiver.nextPacket()
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
        cross_product = front_vector_normalized[0] * target_vector_normalized[0] - \
                        front_vector_normalized[1] * target_vector_normalized[1]
        if cross_product < 0:
            angle = -angle
        distance = np.sqrt(dx ** 2 + dy ** 2)
        print(f"distance is {distance}")
        print(f"angle is {angle}")
        print(f"cross_product is {cross_product}")

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
            print("MOTION PLAY PLAYING")
            print(f"distance is {distance}")
            print(f"distance in x : {dx} y: {dy}")
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

    def is_balanced(self):
        vel = self.gyro.getValues()
        # print('angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1]))
        # print(vel[0], vel[1])
        all_in_balance = float(vel[0]) == 0.0 and float(vel[1]) == 0.0

        print(f"all_in_balance: {all_in_balance}")
        return all_in_balance

    def position_refresh(self):
        if self.receiver.getQueueLength() > 0:
            data = self.receiver.getString()
            shared_info = json.loads(data)

            robot_position = np.float64(shared_info["ReadTeam_Goalkeeper"]["position"])
            robot_orientation = np.float64(shared_info["ReadTeam_Goalkeeper"]["orientation"])
            football_position = np.float64(shared_info["SoccerBall"]["position"])

            self.receiver.nextPacket()
            return robot_position, robot_orientation, football_position
        else:
            return None, None, None

    # Calculate the angle
    def angleCalculaor(self, football_position, robot_position, orientation):
        dx = football_position[0] - robot_position[0]
        dy = football_position[1] - robot_position[1]
        target_magnitude = np.sqrt(dx ** 2 + dy ** 2)
        target_vector_normalized = [dx / target_magnitude, dy / target_magnitude]
        front_magnitude = np.sqrt(orientation[0] ** 2 + orientation[3] ** 2)
        front_vector_normalized = [orientation[0] / front_magnitude, orientation[3] / front_magnitude]
        dot_product = sum(f * t for f, t in zip(front_vector_normalized, target_vector_normalized))
        angle = np.rad2deg(np.arccos(dot_product))
        cross_product = front_vector_normalized[0] * target_vector_normalized[1] - \
                        front_vector_normalized[1] * target_vector_normalized[0]
        # print(f"cross_product: {cross_product}")
        if cross_product < 0:
            angle = -angle
        distance = np.sqrt(dx ** 2 + dy ** 2)
        return angle, distance

    def defendingBall(self):
        limitationofdistance = 0.1
        bitsOfRound = 2
        robot_position, robot_orientation, football_position = self.position_refresh()
        if robot_position is None or robot_orientation is None or football_position is None:
            print("robot_position or robot_orientation or football_position is None!")
            return

        angle, distance = self.angleCalculaor(football_position, robot_position, robot_orientation)
        if angle is None or distance is None:
            print("angle is None or distance is None!")
            return
        # print(f"angle is {angle}")
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
            front_angle, front_distance = self.angleCalculaor([0.0, robot_position[1], 0.0], robot_position,
                                                              robot_orientation)
            print("front_angle:", front_angle)
            print("front_distance:", front_distance)

            if np.abs(np.round(front_angle,1)) > 15.0:
                print("Need to adjust front_angle!")
                self.stopMotion()
                self.previous_stage = self.gk_stage
                self.gk_stage = DEFEND_STAGE.ADJUSTING_ANGLE
                return

            if ((np.round(np.round(front_distance,bitsOfRound),1) > 3.90 + limitationofdistance)
                    or (np.round(np.round(front_distance,bitsOfRound),1) < 3.90 - limitationofdistance)):
                print("Need to adjust X_Axis!")
                self.stopMotion()
                self.previous_stage = self.gk_stage
                self.gk_stage = DEFEND_STAGE.X_AXIS_ADJUST
                return

            if football_position[0] >= 0:
                if (angle >= 15.0 and robot_position[1] >= -1.05) or (angle <= -15.0 and robot_position[1] <= 1.05):
                    self.__temp_angle = angle
                    self.__temp_position = football_position
                    self.previous_stage = self.gk_stage
                    self.gk_stage = DEFEND_STAGE.ADJUSTING_ANGLE
                    return
                else:
                    self.previous_stage = self.gk_stage
                    self.gk_stage = DEFEND_STAGE.FINISH
                    return
            else:
                self.previous_stage = self.gk_stage
                self.gk_stage = DEFEND_STAGE.FINISH
                return
        elif self.gk_stage == DEFEND_STAGE.ADJUSTING_ANGLE:
            print("DEFEND ADJUSTING_ANGLE")
            front_angle, front_distance = self.angleCalculaor([0.0, robot_position[1], 0.0], robot_position, robot_orientation)
            print(front_angle)
            if np.round(np.abs(front_angle),1) >= 15.0:
                if ((180.0 >= np.round(front_angle,1) >= 15.0)
                        or (180.0 >= np.round(front_angle,1) >= 15.0 and self.isTurningRight is None)
                        or self.isTurningRight):
                    self.startMotion(self.turnright40)
                    self.isTurningRight = True
                elif ((-180.0 <= np.round(front_angle,1) <= -15.0)
                        or (-180.0 <= np.round(front_angle,1) <= -15.0 and self.isTurningRight is None)
                        or not self.isTurningRight):
                    self.startMotion(self.turnleft40)
                    self.isTurningRight = False

            if np.round(np.abs(front_angle),1) < 15.0:
                self.stopMotion()
                self.isTurningRight = None
                if self.previous_stage == DEFEND_STAGE.FINISH:
                    self.previous_stage = self.gk_stage
                    if (np.round(np.round(front_distance,bitsOfRound),1) > 3.90 + limitationofdistance
                            or np.round(np.round(front_distance,bitsOfRound),1) < 3.90 - limitationofdistance):
                        self.gk_stage = DEFEND_STAGE.X_AXIS_ADJUST
                else:
                    self.__temp_angle = angle
                    self.__temp_position = football_position
                    self.previous_stage = self.gk_stage
                    self.gk_stage = DEFEND_STAGE.SIDE_STEP_ADJUST
                return
            else:
                return
        elif self.gk_stage == DEFEND_STAGE.SIDE_STEP_ADJUST:
            print("DEFEND SIDE_STEP_ADJUST")
            print("__temp_angle", self.__temp_angle)
            if self.__temp_angle >= 15.0 and robot_position[1] >= -1.05:
                self.startMotion(self.sidestepleft)

            elif self.__temp_angle <= -15.0 and robot_position[1] <= 1.05:
                self.startMotion(self.sidestepright)

            self.__temp_angle, judge_distance = self.angleCalculaor(football_position, robot_position,
                                                                    robot_orientation)
            front_angle, front_distance = self.angleCalculaor([0.0, robot_position[1], 0.0], robot_position,robot_orientation)
            print("front_angle:", front_angle)
            print("front_distance:", front_distance)
            if (np.round(np.abs(front_angle),1) >= 15.0
                    # and self.previous_stage != DEFEND_STAGE.BOUNDARY_ADJUST
            ):
                print("Need to adjust front_angle!")
                self.stopMotion()
                self.previous_stage = self.gk_stage
                self.gk_stage = DEFEND_STAGE.ADJUSTING_ANGLE
                return

            if ((np.round(np.round(front_distance,bitsOfRound),1) > (3.90 + limitationofdistance))
                    or (np.round(np.round(front_distance,bitsOfRound),1) < (3.90 - limitationofdistance))):
                print("Need to adjust X_Axis!")
                # self.stopMotion()
                self.previous_stage = self.gk_stage
                self.gk_stage = DEFEND_STAGE.X_AXIS_ADJUST
                return

            if (not (self.__temp_angle >= 15.0 and robot_position[1] >= -1.05)
                    and not (self.__temp_angle <= -15.0 and robot_position[1] <= 1.05)):
                # self.stopMotion()
                # if (np.abs(self.__temp_angle) >= 15.0
                #         and (robot_position[1] <= -1.05 or robot_position[1] >= 1.05)):
                #     self.previous_stage = self.gk_stage
                #     self.gk_stage = DEFEND_STAGE.BOUNDARY_ADJUST
                #     return
                # else:
                    self.previous_stage = self.gk_stage
                    self.gk_stage = DEFEND_STAGE.FINISH
                    return
            # elif (self.previous_stage == DEFEND_STAGE.BOUNDARY_ADJUST and
            #              (robot_position[1] <= -1.05 or robot_position[1] >= 1.05 or np.abs(self.__temp_angle) <= 15.0)):
            #     self.previous_stage = self.gk_stage
            #     self.gk_stage = DEFEND_STAGE.FINISH
            #     return
            else:
                return
        # elif self.gk_stage == DEFEND_STAGE.BOUNDARY_ADJUST:
        #     print("DEFEND BOUNDARY_ADJUST")
        #     front_angle, front_distance = self.angleCalculaor([0.0, robot_position[1], 0.0], robot_position,
        #                                                       robot_orientation)
        #     if self.__temp_position[1] <= -1.05 or self.__temp_position[1] >= 1.05 or np.abs(front_angle) < 90.0:
        #         if self.__temp_position[1] <= -1.05:
        #             self.startMotion(self.turnleft40)
        #
        #         elif self.__temp_position[1] >= 1.05:
        #             self.startMotion(self.turnright40)
        #
        #     if np.abs(front_angle) >= 90.0:
        #         if self.__temp_angle >= 15.0:
        #             self.startMotion(self.sidestepleft)
        #
        #         elif self.__temp_angle <= -15.0:
        #             self.startMotion(self.sidestepright)
        #         self.__temp_angle, judge_distance = self.angleCalculaor(football_position, robot_position,
        #                                                                       robot_orientation)
        #         print(f"temp_angle: {self.__temp_angle}, temp_position: {self.__temp_position}")
        #         print(f"front_angle: {front_angle}, front_distance: {front_distance}")
        #         if np.abs(self.__temp_angle) < 15.0:
        #             self.previous_stage = self.gk_stage
        #             self.gk_stage = DEFEND_STAGE.FINISH
        #             return
        #         else:
        #             return
        #     else:
        #         return
        elif self.gk_stage == DEFEND_STAGE.X_AXIS_ADJUST:
            print("DEFEND X_AXIS_ADJUST")
            front_angle, front_distance = self.angleCalculaor([0.0, robot_position[1], 0.0], robot_position,robot_orientation)
            print("front_distance:", np.round(np.round(front_distance,bitsOfRound),2))
            if np.round(np.round(front_distance,bitsOfRound),1) > 3.90 + limitationofdistance:
                print("Is adjusting forwards!")
                self.startMotion(self.forwards)
                self.countOfXAxisRetryTime+=1

            elif np.round(np.round(front_distance,bitsOfRound),1) < 3.90 - limitationofdistance:
                print("Is adjusting backwards!")
                self.startMotion(self.backwards)
                self.countOfXAxisRetryTime += 1

            if (3.90 + limitationofdistance >= np.round(np.round(front_distance,bitsOfRound),1) >= 3.90 - limitationofdistance
                # or self.countOfXAxisRetryTime > self.maxRetryTimes
            ):
                # self.stopMotion()
                if self.previous_stage == DEFEND_STAGE.PREPARE:
                    self.previous_stage = self.gk_stage
                    self.gk_stage = DEFEND_STAGE.PREPARE
                else:
                    self.previous_stage = self.gk_stage
                    self.gk_stage = DEFEND_STAGE.FINISH
                return
            else:
                return
        elif self.gk_stage == DEFEND_STAGE.FINISH:
            print("DEFEND FINISH")
            front_angle, front_distance = self.angleCalculaor([0.0, robot_position[1], 0.0], robot_position, robot_orientation)
            if self.is_balanced():
                if (np.round(np.abs(front_angle),1) >= 15.0
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
            if ((angle >= 15.0 and robot_position[1] >= -1.05)
                    or (angle <= -15.0 and robot_position[1] <= 1.05) and football_position[0] >= 0):
                self.previous_stage = self.gk_stage
                self.gk_stage = DEFEND_STAGE.INITIAL
            return
        else:
            print("Unknown stage")
            return False

class Nao_Defender(Robot):
    PHALANX_MAX = 8
    kick_stage = KICK_STAGE.INITIAL

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
            if passing <= self.__threshold:
                return True
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

    def angleCalculaor(self, football_position, robot_position, orientation):
        dx = football_position[0] - robot_position[0]
        dy = football_position[1] - robot_position[1]
        target_magnitude = np.sqrt(dx ** 2 + dy ** 2)
        target_vector_normalized = [dx / target_magnitude, dy / target_magnitude]
        front_magnitude = np.sqrt(orientation[0] ** 2 + orientation[3] ** 2)
        front_vector_normalized = [orientation[0] / front_magnitude, orientation[3] / front_magnitude]
        dot_product = sum(f * t for f, t in zip(front_vector_normalized, target_vector_normalized))
        angle = np.rad2deg(np.arccos(dot_product))
        cross_product = front_vector_normalized[0] * target_vector_normalized[1] - \
                        front_vector_normalized[1] * target_vector_normalized[0]
        # print(f"cross_product: {cross_product}")
        if cross_product > 0:
            angle = -angle
        distance = np.sqrt(dx ** 2 + dy ** 2)
        return angle, distance

    def __init__(self):
        '''
        This is the constructor, only be used once by creating the object.
        It will initialize all the motors and the sensors.
        If you want to initialize some parameters you need, you can do it in here.
        '''
        Robot.__init__(self)
        print('NAO_Defender has initialized')
        self.currentlyPlaying = False
        self.wait_frames = 0
        self.isTurningRight = None
        self.countOfXAxisRetryTime = 0
        self.maxRetryTimes = 5
        # initialize stuff
        self.findAndEnableDevices()
        self.loadMotionFiles()
        self.myName = Robot.getName(self)
        if "Red" in self.myName:
            self.striker = "striker_red"
            self.oppo_striker = "striker_blue"
            self.oppo_defender_l = "BlueTeam_DefenderLeft"
            self.oppo_defender_r = "BlueTeam_DefenderRight"
            self.oppo_goal = "stadiumgoal_blue"
            self.goal = "stadiumgoal_red"
            if  "Left" in self.myName:
                self.mateName = "RedTeam_DefenderRight"
            else:
                self.mateName = "RedTeam_DefenderLeft"
        elif "Blue" in self.myName:
            self.striker = "striker_blue"
            self.oppo_striker = "striker_red"
            self.oppo_defender_l = "RedTeam_DefenderLeft"
            self.oppo_defender_r = "RedTeam_DefenderRight"
            self.oppo_goal = "stadiumgoal_red"
            self.goal = "stadiumgoal_blue"
            if "Left" in self.myName:
                self.mateName = "BlueTeam_DefenderRight"
            else:
                self.mateName = "BlueTeam_DefenderLeft"

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

        self.df_stage = DEFENDER_STAGE.INITIAL
        self.heading = None

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
            self.kick_stage = stage
        elif isinstance(stage, STAND_UP):
            self.__standup_stage = stage
        else:
            print(f"Stage {stage} not supported")
        return

    def is_balanced(self):
        vel = self.gyro.getValues()
        # print('angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1]))
        # print(vel[0], vel[1])
        all_in_balance = np.round(float(vel[0]),2) == 0.0 and np.round(float(vel[1]),2) == 0.0

        print(f"all_in_balance: {all_in_balance}")
        return all_in_balance

    def position_refresh(self):
        if self.receiver.getQueueLength() > 0:
            self.data = self.receiver.getString()
            shared_info = json.loads(self.data)

            robot_position = np.float64(shared_info[self.myName]["position"])
            robot_orientation = np.float64(shared_info[self.myName]["orientation"])
            football_position = np.float64(shared_info["football"]["position"])
            self.mate_position = np.float64(shared_info[self.mateName]["position"])
            self.mate_orientation = np.float64(shared_info[self.mateName]["orientation"])
            self.striker_position = np.float64(shared_info[self.striker]["position"])
            self.goal_position = np.float64(shared_info[self.goal]["position"])

            self.oppo_striker_position = np.float64(shared_info[self.oppo_striker]["position"])
            self.oppo_defender_l_position = np.float64(shared_info[self.oppo_defender_l]["position"])
            self.oppo_defender_r_position = np.float64(shared_info[self.oppo_defender_r]["position"])
            self.oppo_goal_position = np.float64(shared_info[self.oppo_goal]["position"])
            # print(f"Available keys in shared_info: {shared_info.keys()}")

            self.receiver.nextPacket()
            return robot_position, robot_orientation, football_position
        else:
            return None, None, None

    def over_range_detect(self,position):
        """计算禁区边缘和地方半场"""
        """constrain of panalty zone and oppo zone"""
        penalty_bounds = np.array([
            self.goal_position[0] - 1.4,  # x_min
            self.goal_position[0] + 1.4,  # x_max
            self.goal_position[1] - 1,  # y_min
            self.goal_position[1] + 1  # y_max
        ])
        # 检查点是否在禁区内
        if penalty_bounds[0] <= position[0] <= penalty_bounds[1] and \
                penalty_bounds[2] <= position[1] <= penalty_bounds[3]:

            distances = np.abs(np.array([
                position[0] - penalty_bounds[0],  # 距离 x_min
                position[0] - penalty_bounds[1],  # 距离 x_max
                position[1] - penalty_bounds[2],  # 距离 y_min
                position[1] - penalty_bounds[3]  # 距离 y_max
            ]))
            # 计算最近的边
            closest_edge_idx = np.argmin(distances)
            # 更新点位置
            if closest_edge_idx < 2:
                position[0] = penalty_bounds[closest_edge_idx]
            else:
                position[1] = penalty_bounds[closest_edge_idx]
        if "Red" in self.myName:
            if position[0] < 1.3:
                position[0] = 1.3
        elif "Blue" in self.myName:
            if position[0] > -1.3:
                position[0] = -1.3
        return position

    def calculate_navigation_vector(self,defender_pos, intercept_pos, avoid_objects, avoid_radius=0.8):
        """
        计算带避让功能的导航向量

        """
        repulse_factor = 2

        # 计算吸引力（指向拦截点）
        vector_to_intercept = intercept_pos - defender_pos[0:2]
        distance_to_intercept = np.linalg.norm(vector_to_intercept)
        force_attract = vector_to_intercept / (distance_to_intercept + 1e-6)  # 避免除零

        # 计算斥力（避让所有障碍物）
        total_repulse_force = np.array([0.0, 0.0])

        def compute_repulsive_force(obstacle_pos):
            vector_to_obstacle = defender_pos[0:2] - obstacle_pos[0:2]
            distance_to_obstacle = np.linalg.norm(vector_to_obstacle)

            if distance_to_obstacle < avoid_radius:
                # 斥力 = repulse_factor * log(1 + (R - d) / R)，避免剧烈跳变
                repulse_strength = repulse_factor * np.log(1 + (avoid_radius - distance_to_obstacle) / avoid_radius)
                force = repulse_strength * (vector_to_obstacle / distance_to_obstacle)
            else:
                force = np.array([0.0, 0.0])  # 超过避让范围不施加力

            return force

        for obstacle in avoid_objects:
            total_repulse_force += compute_repulsive_force(obstacle[0:2])

        # 合力 = 吸引力 + 总斥力
        navigation_vector = force_attract + total_repulse_force
        navigation_vector = navigation_vector / (np.linalg.norm(navigation_vector) + 1e-6)  # 归一化
        # print(f"navigation_vector:{navigation_vector}")
        return navigation_vector

    def intercept_solving(self, football_position, robot_position, goal_position, orientation,avoid_objects):
        """calculate the vector football to goal, and the distance between robot and best intercept position"""
        """用向量的方法求球到球门的连线的法线方向，然后求出机器人到这个连线的垂点的直线距离，返回这个距离和连线的指向"""
        """判断这个拦截点是否已经超过自己的防守深度了，超了那就往球指向门的向量上，球前方确定距离的点位赶路"""
        #计算球到球门的向量 & 机器人到球门的向量
        vector_ball2goal = goal_position[0:2] - football_position[0:2]
        vector_robot2goal = goal_position[0:2] - robot_position[0:2]
        #计算拦截点向量(defender指向拦截点)
        projection_length = np.dot(vector_ball2goal, vector_robot2goal)/np.dot(vector_ball2goal,vector_ball2goal)
        vector_projection = projection_length*vector_ball2goal
        vector_intercept = vector_robot2goal - vector_projection
        distance = np.linalg.norm(vector_intercept) #here is the distance
        intercept_position = robot_position[0:2] + vector_intercept

        """计算禁区边缘和地方半场"""
        """constrain of panalty zone and oppo zone"""
        intercept_position = self.over_range_detect(intercept_position)

        """refresh intercept vector"""
        vector_intercept = intercept_position - robot_position[0:2]
        distance = np.linalg.norm(vector_intercept)

        """calculate the intercept point, to see if it is too late to intercept"""
        """if impossible,choose the point at the front of the ball to intercept"""
        intercept_distance = 0.6
        amplitude_vector_ball2goal = np.linalg.norm(vector_ball2goal)
        vector_ball2goal_normalized = vector_ball2goal/amplitude_vector_ball2goal

        vector_ball2intercept = intercept_position - football_position[0:2]
        dot_product = np.dot(vector_ball2goal_normalized, vector_ball2intercept)
        if dot_product <=0.8:
            new_intercept_position = football_position + np.append(intercept_distance*vector_ball2goal_normalized,0)
                #penalty zone check and correct
            # 检查点是否在禁区内
            new_intercept_position = self.over_range_detect(new_intercept_position)
            # print(new_intercept_position)
            # print(football_position)
            angle, distance = self.angleCalculaor(new_intercept_position, robot_position, orientation)

            return angle,distance

        def compute_intercept_angle(vector_intercept,orientation):
            """below is calculating angle need to turn"""
            vector_intercept_normalized = vector_intercept/np.linalg.norm(vector_intercept)
            amplitude_orientation = np.sqrt(orientation[0] ** 2 + orientation[3] ** 2)
            vector_heading_normalized = [orientation[0]/amplitude_orientation, orientation[3]/amplitude_orientation]
            dot_product = np.dot(vector_intercept_normalized, vector_heading_normalized)
            intercept_angle = np.rad2deg(np.arccos(dot_product))
            cross_product = vector_intercept_normalized[0] * vector_heading_normalized[1] - \
                            vector_intercept_normalized[1] * vector_heading_normalized[0]
            if cross_product < 0:
                intercept_angle = -intercept_angle
            # print(f"intercept position:{intercept_position}")
            return intercept_angle

        vector_intercept = self.calculate_navigation_vector(robot_position, intercept_position, avoid_objects)
        intercept_angle = compute_intercept_angle(vector_intercept,orientation)

        return intercept_angle,distance

    def ball_clear_judge(self,football_position,intercepter_position,tolerance_angle,torlerance_distance):
        """this function is built to figure out if there is a clear way between robot and oppo goal"""
        """搞清楚地方球门和球之间是否干净，干净就赶紧冲锋"""
        vector_ball2goal = self.oppo_goal_position[0:2]-football_position[0:2]
        vector_ball2intercepter = intercepter_position[0:2]-football_position[0:2]
        vector_ball2goal_normalized = vector_ball2goal/np.linalg.norm(vector_ball2goal)
        distance_judge = np.linalg.norm(vector_ball2intercepter)
        vector_ball2intercepter_normalized =  vector_ball2intercepter/distance_judge
        dot_product = np.dot(vector_ball2goal_normalized, vector_ball2intercepter_normalized)
        angle = np.rad2deg(np.arccos(dot_product))
        # print(angle)
        # print(distance_judge)
        if np.abs(angle) < tolerance_angle and distance_judge < torlerance_distance:
            return False
        else:
            return True

    def vice_solving(self,robot_position, orientation):
        """将待命位置放在主防守者后侧方"""
        """calculate the waiting point"""
        if "Red" in self.myName:
            vice_position_x = 0.7
        else:
            vice_position_x = -0.7
        new_intercept_position_l = self.mate_position[0:2] + [vice_position_x,-0.3]
        new_intercept_position_r = self.mate_position[0:2] + [vice_position_x,+0.3]
        distance2pl = np.linalg.norm(robot_position[0:2] - new_intercept_position_l)
        distance2pr = np.linalg.norm(robot_position[0:2] - new_intercept_position_r)
        if distance2pl <= distance2pr:
            new_intercept_position = new_intercept_position_l
        else:
            new_intercept_position = new_intercept_position_r
        # print(f"mate{self.mate_position[0:2]}")
        # print(new_intercept_position)
        avoid_objects = [self.striker_position,self.mate_position]
        wait_vector = self.calculate_navigation_vector(robot_position,new_intercept_position,avoid_objects)
        def compute_wait_angle(vector_intercept,orientation):
            """below is calculating angle need to turn"""
            vector_intercept_normalized = vector_intercept/np.linalg.norm(vector_intercept)
            amplitude_orientation = np.sqrt(orientation[0] ** 2 + orientation[3] ** 2)
            vector_heading_normalized = [orientation[0]/amplitude_orientation, orientation[3]/amplitude_orientation]
            dot_product = np.dot(vector_intercept_normalized, vector_heading_normalized)
            intercept_angle = np.rad2deg(np.arccos(dot_product))
            cross_product = vector_intercept_normalized[0] * vector_heading_normalized[1] - \
                            vector_intercept_normalized[1] * vector_heading_normalized[0]
            if cross_product < 0:
                intercept_angle = -intercept_angle
            # print(f"intercept position:{intercept_position}")
            return intercept_angle
        wait_angle = compute_wait_angle(wait_vector,orientation)
        wait_distance = np.linalg.norm(new_intercept_position - robot_position[0:2])
        return wait_angle, wait_distance

    def standupIfnecessary(self):
        Acc = self.accelerometer.getValues()
        # print(f"Acc is {Acc}")
        if (
            (np.abs(Acc[2]) < 5.0 and np.abs(Acc[0]) > 4.0)
            or (np.abs(Acc[2]) < 5.0 and np.abs(Acc[1]) > 4.0)
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
            # print("Stand-Up INITIAL")
            self.__standup_stage = STAND_UP.PREPARE
            return
        elif self.__standup_stage == STAND_UP.PREPARE:
            # print("Stand-Up PREPARE")
            self.__standup_stage = STAND_UP.FROM_BACK
            return
        elif self.__standup_stage == STAND_UP.FROM_FRONT:
            # print("Stand-Up From FRONT")
            return
        elif self.__standup_stage == STAND_UP.FROM_BACK:
            # print("Stand-Up From BACK")
            self.startMotion(self.StandUpFromBack)
            if self.standupIfnecessary():
                self.__standup_stage = STAND_UP.FINISH
            return
        elif self.__standup_stage == STAND_UP.FINISH:
            # print("Stand-Up FINISH")
            self.__standup_stage = STAND_UP.END
            return
        elif self.__standup_stage == STAND_UP.END:
            print("Stand-Up END")
            return
        else:
            print("Unknown stand-up stage")

    def intercepting(self):
        """
        this version of defender can work, but still has many logical details to fill up
        :return:
        """
        limitationofdistance = 0.2
        limitationofdistance2 = 0.13
        alert_range_mate = 1
        bitsOfRound = 2
        """get this robot position,direction, and football position"""
        """in every time step"""
        robot_position, robot_orientation, football_position = self.position_refresh()
        if robot_position is None or robot_orientation is None or football_position is None:
            print("robot_position or robot_orientation or football_position is None!")
            return

        avoid_objects_me = [self.striker_position,self.mate_position]
        intercept_angle, intercept_distance = self.intercept_solving(football_position,\
                            robot_position, self.goal_position,robot_orientation,avoid_objects_me)
        avoid_objects_mate = [self.striker_position, robot_position]
        intercept_angle_mate, intercept_distance_mate = self.intercept_solving(football_position,\
                           self.mate_position, self.goal_position,self.mate_orientation,avoid_objects_mate)

        angle2mate, distance2mate = self.angleCalculaor(self.mate_position, robot_position, robot_orientation)
        angle, distance2ball = self.angleCalculaor(football_position, robot_position, robot_orientation)
        angle_mate, distance2ball_mate = self.angleCalculaor(football_position, self.mate_position, robot_orientation)
        angle_oppo_striker,distance2oppo_striker = self.angleCalculaor(self.oppo_striker_position, robot_position, robot_orientation)

        # print(f"intercept_distance: {intercept_distance}")
        # print(f"intercept_distance_mate:{intercept_distance_mate}")
        """initialize joints"""
        if self.df_stage == DEFENDER_STAGE.INITIAL:
            if self.standupIfnecessary():
                self.previous_stage = self.df_stage
                self.set_stage(STAND_UP.INITIAL)
                self.df_stage = DEFENDER_STAGE.STAND_UP
                return
            print("DEFEND INITIAL")
            self.setMotorPosition('LShoulderPitch', 1.2)
            self.setMotorPosition('RShoulderPitch', 1.2)
            self.setMotorPosition('LShoulderRoll', 0.000000086)
            self.setMotorPosition('RShoulderRoll', -0.000000086)
            self.setMotorPosition('LElbowRoll', -0.49)
            self.setMotorPosition('RElbowRoll', 0.49)
            self.setMotorPosition('LElbowYaw', 0.000000049)
            self.setMotorPosition('RElbowYaw', -0.000000049)
            self.setMotorPosition('LHipYawPitch', 0)
            self.setMotorPosition('RHipYawPitch', 0)
            self.setMotorPosition('LHipRoll', 0)
            self.setMotorPosition('RHipRoll', 0)
            self.setMotorPosition('LHipPitch', 0)
            self.setMotorPosition('RHipPitch', 0)
            self.setMotorPosition('LKneePitch', 0.0000020595)
            self.setMotorPosition('RKneePitch', 0.0000023045)
            self.setMotorPosition('LAnklePitch', 0)
            self.setMotorPosition('RAnklePitch', 0)
            self.setMotorPosition('LAnkleRoll', 0)
            self.setMotorPosition('RAnkleRoll', 0)


            if (self.getMoveStage('LShoulderPitch') is move_status.END and
                    self.getMoveStage('RShoulderPitch') is move_status.END and
                    self.getMoveStage('LShoulderRoll') is move_status.END and
                    self.getMoveStage('RShoulderRoll') is move_status.END and
                    self.getMoveStage('LElbowRoll') is move_status.END and
                    self.getMoveStage('RElbowRoll') is move_status.END and
                    self.getMoveStage('LElbowYaw') is move_status.END and
                    self.getMoveStage('RElbowYaw') is move_status.END and
                    self.getMoveStage('LHipYawPitch') is move_status.END and
                    self.getMoveStage('RHipYawPitch') is move_status.END and
                    self.getMoveStage('LHipRoll') is move_status.END and
                    self.getMoveStage('RHipRoll') is move_status.END and
                    self.getMoveStage('LHipPitch') is move_status.END and
                    self.getMoveStage('RHipPitch') is move_status.END and
                    self.getMoveStage('LKneePitch') is move_status.END and
                    self.getMoveStage('RKneePitch') is move_status.END and
                    self.getMoveStage('LAnklePitch') is move_status.END and
                    self.getMoveStage('RAnklePitch') is move_status.END and
                    self.getMoveStage('LAnkleRoll') is move_status.END and
                    self.getMoveStage('RAnkleRoll') is move_status.END
                    ):
                self.previous_stage = self.df_stage
                self.df_stage = DEFENDER_STAGE.INTERCEPT
            else:
                return

            """ decide to turn or directly to intercept"""
        elif self.df_stage == DEFENDER_STAGE.INTERCEPT:
            if self.standupIfnecessary():
                self.previous_stage = self.df_stage
                self.set_stage(STAND_UP.INITIAL)
                self.df_stage = DEFENDER_STAGE.STAND_UP
                return
            print("DEFENDER INTERCEPT")
            # print("intercept_angle, intercept_distance: ", intercept_angle, intercept_distance)
            if intercept_distance <= limitationofdistance:
                self.df_stage = DEFENDER_STAGE.SIDE_STEP_ADJUST
                return
            elif 180.0 >= intercept_angle >= 15.0 or -15.0 >= intercept_angle >= -180.0:
                self.df_stage = DEFENDER_STAGE.ADJUSTING_ANGLE
                return
            else:
                self.df_stage = DEFENDER_STAGE.APPROCH
                return

            # logic to change to vice defender
        if distance2mate < alert_range_mate:
            if "Red" in self.myName:
                if intercept_distance > intercept_distance_mate and football_position[0] < self.mate_position[0]:
                    self.df_stage = DEFENDER_STAGE.VICE_DEFEND
            elif "Blue" in self.myName:
                if intercept_distance > intercept_distance_mate and football_position[0] > self.mate_position[0]:
                    self.df_stage = DEFENDER_STAGE.VICE_DEFEND

            '''turn to intercept direction'''
        if self.df_stage == DEFENDER_STAGE.ADJUSTING_ANGLE:
            if self.standupIfnecessary():
                self.previous_stage = self.df_stage
                self.set_stage(STAND_UP.INITIAL)
                self.df_stage = DEFENDER_STAGE.STAND_UP
                return
            print("DEFENDER ADJUSTING_ANGLE")
            if np.round(np.abs(intercept_angle),1) >= 20.0:
                if ((180.0 >= intercept_angle >= 20.0)
                        or (180.0 >= intercept_angle >= 20.0 and self.isTurningRight is None)
                        or self.isTurningRight):
                    self.startMotion(self.turnright40)
                    self.isTurningRight = True
                if ((-180.0 <= intercept_angle <= -20.0)
                        or (-180.0 <= intercept_angle <= -20.0 and self.isTurningRight is None)
                        or not self.isTurningRight):
                    self.startMotion(self.turnleft40)
                    self.isTurningRight = False
                # print(f"{self.isTurningRight}")
                # print(f"intercept_angle: {intercept_angle}")
            if np.abs(intercept_angle) <= 21.0:
                # self.stopMotion()
                if self.is_balanced():
                    self.df_stage = DEFENDER_STAGE.APPROCH
                return
            else:
                return

            '''go to intercept position and turn to next stage when close enough'''
        elif self.df_stage == DEFENDER_STAGE.APPROCH:
            if self.standupIfnecessary():
                self.previous_stage = self.df_stage
                self.set_stage(STAND_UP.INITIAL)
                self.df_stage = DEFENDER_STAGE.STAND_UP
                return
            print("DEFENDER APPROCH")
            # print(f"intercept_distance is {intercept_distance}")
            # print(f"intercept angle is {intercept_angle}")
            if intercept_distance >= limitationofdistance:
                if ((90.0 >= intercept_angle >= 15.0)
                        or (90.0 >= intercept_angle >= 15.0 and self.isTurningRight is None)
                        or self.isTurningRight):
                    self.startMotion(self.turnright40)
                    self.isTurningRight = True
                if ((-90.0 <= intercept_angle <= -15.0)
                        or (-90.0 <= intercept_angle <= -15.0 and self.isTurningRight is None)
                        or not self.isTurningRight):
                    self.startMotion(self.turnleft40)
                    self.isTurningRight = False
                if 180.0 >= intercept_angle >= 21.0 or -21.0 >= intercept_angle >= -180.0:
                    if self.is_balanced():
                        self.df_stage = DEFENDER_STAGE.ADJUSTING_ANGLE
                    return
                else:
                    self.startMotion(self.forwards)
                    return
            elif limitationofdistance2 < intercept_distance <= limitationofdistance :
                self.startMotion(self.forwards)
                return
            else:
                # self.stopMotion()
                if self.is_balanced():
                    self.df_stage = DEFENDER_STAGE.SIDE_STEP_ADJUST
                return

             # turn towards to ball, side step to block the ball, to next stage when close enough
        elif self.df_stage == DEFENDER_STAGE.SIDE_STEP_ADJUST:
            if self.standupIfnecessary():
                self.previous_stage = self.df_stage
                self.set_stage(STAND_UP.INITIAL)
                self.df_stage = DEFENDER_STAGE.STAND_UP
                return
            print("DEFENDER SIDE_STEP_ADJUST")
            # print(f"intercept_distance is {intercept_distance}")
            angle, distance_2_ball = self.angleCalculaor(football_position, robot_position, robot_orientation)
            # print(f"angle is {angle}")
            self.__temp_angle = angle
            if intercept_distance > limitationofdistance+0.5:
                self.df_stage = DEFENDER_STAGE.INTERCEPT
                return
            if np.round(np.abs(self.__temp_angle),1) >= 25.0:
                if ((180.0 >= self.__temp_angle >= 25.0)
                        or (180.0 >= self.__temp_angle >= 25.0 and self.isTurningRight is None)
                        or self.isTurningRight):
                    self.startMotion(self.turnright40)
                    self.isTurningRight = True
                if ((-180.0 <= self.__temp_angle <= -25.0)
                        or (-180.0 <= self.__temp_angle <= -25.0 and self.isTurningRight is None)
                        or not self.isTurningRight):
                    self.startMotion(self.turnleft40)
                    self.isTurningRight = False
            elif np.round(np.abs(self.__temp_angle),1) <= 25.0:
                if self.is_balanced():
                    self.isTurningRight = None
                    self.df_stage = DEFENDER_STAGE.SIDE_STEP
                    self.isStepRight = None
                    return
                return
        elif self.df_stage == DEFENDER_STAGE.SIDE_STEP:
            if self.standupIfnecessary():
                self.previous_stage = self.df_stage
                self.set_stage(STAND_UP.INITIAL)
                self.df_stage = DEFENDER_STAGE.STAND_UP
                return
            print("DEFENDER SIDE_STEP")
            # print(f"intercept_distance is {intercept_distance}")
            angle, distance_2_ball = self.angleCalculaor(football_position, robot_position, robot_orientation)
            self.__temp_angle = angle
            # print(f"intercept_angle is {intercept_angle}")
            if np.round(intercept_distance,bitsOfRound) > limitationofdistance+0.1:
                if self.is_balanced():
                    self.df_stage = DEFENDER_STAGE.INTERCEPT
                return
            elif np.round(np.abs(self.__temp_angle),1) > 60.0:
                if self.is_balanced():
                    self.df_stage = DEFENDER_STAGE.SIDE_STEP_ADJUST
                return
            """下面是一个滞回比较器，（大概是叫这个名字吧）"""
            if self.isStepRight is None:
                if intercept_distance <= limitationofdistance2:
                    self.isStepRight = None
                    self.df_stage = DEFENDER_STAGE.FINISH
                elif intercept_distance >= limitationofdistance2 and intercept_angle > 0:
                    self.isStepRight = True
                    self.startMotion(self.sidestepright)
                    return
                elif intercept_distance >= limitationofdistance2 and intercept_angle < 0:
                    self.isStepRight = False
                    self.startMotion(self.sidestepleft)
                    return
            elif self.isStepRight:
                if intercept_angle > 0:
                    self.startMotion(self.sidestepright)
                    return
                elif intercept_distance <= limitationofdistance:
                    if self.is_balanced():
                        self.isStepRight = None
                        self.df_stage = DEFENDER_STAGE.FINISH
                    return
                else:
                    self.isStepRight = False
                    return
            elif not self.isStepRight:
                if intercept_angle < 0:
                    self.startMotion(self.sidestepleft)
                    return
                elif intercept_distance <= limitationofdistance:
                    if self.is_balanced():
                        self.isStepRight = None
                        self.df_stage = DEFENDER_STAGE.FINISH
                    return
                else:
                    self.isStepRight = True
                    return
            return
        elif self.df_stage == DEFENDER_STAGE.FINISH:
            if self.standupIfnecessary():
                self.previous_stage = self.df_stage
                self.set_stage(STAND_UP.INITIAL)
                self.df_stage = DEFENDER_STAGE.STAND_UP
                return
            print("INTERCEPT FINISH")
            if intercept_distance >= limitationofdistance2:
                self.df_stage = DEFENDER_STAGE.INTERCEPT
            if self.is_balanced():
                self.df_stage = DEFENDER_STAGE.WAIT
            return
        elif self.df_stage == DEFENDER_STAGE.WAIT:
            # print(f"goal position:{self.goal_position}")
            # print(f"oppo goal:{self.oppo_goal_position}")
            if self.standupIfnecessary():
                self.previous_stage = self.df_stage
                self.set_stage(STAND_UP.INITIAL)
                self.df_stage = DEFENDER_STAGE.STAND_UP
                return
            print("WAIT")
            if intercept_distance >= limitationofdistance+0.1:
                self.df_stage = DEFENDER_STAGE.INTERCEPT
                return True
            elif distance2ball <= limitationofdistance and np.abs(angle)<=22:
                print("KICK")
                return True

        elif self.df_stage == DEFENDER_STAGE.VICE_DEFEND:
            if self.standupIfnecessary():
                self.previous_stage = self.df_stage
                self.set_stage(STAND_UP.INITIAL)
                self.df_stage = DEFENDER_STAGE.STAND_UP
                return
            print("VICE DEFEND")
            # print(f"intercept distance: {intercept_distance}")
            wait_circle = 0.4
            if distance2mate > alert_range_mate+0.2 or intercept_distance < intercept_distance_mate:
                if "Red" in self.myName and football_position[0] > self.mate_position[0] :
                    # print("to intercept")
                    self.df_stage = DEFENDER_STAGE.INTERCEPT
                elif "Blue" in self.myName and football_position[0] < self.mate_position[0] :
                    # print("to intercept")
                    self.df_stage = DEFENDER_STAGE.INTERCEPT
                return
            elif distance2ball <= limitationofdistance+0.03 and self.ball_clear_judge(football_position,self.oppo_striker_position,30,1):
                print("STRIKER HENSHIN!!!!")
                return "STRIKER HENSHIN!!!!"

            wait_angle, wait_distance = self.vice_solving(robot_position, robot_orientation)
            if self.heading is True:
                print("Heading True")
                if 22.0 <= wait_angle <= 180.0 and wait_distance >= limitationofdistance:
                    # print("r")
                    if self.is_balanced():
                        self.startMotion(self.turnright40)
                    return
                elif -180.0 <= wait_angle <= -22.0 and wait_distance >= limitationofdistance:
                    # print("l")
                    if self.is_balanced():
                        self.startMotion(self.turnleft40)
                    return
                elif wait_distance >= limitationofdistance:
                    # print("go")
                    if self.is_balanced():
                      self.startMotion(self.forwards)
                    return
                elif limitationofdistance2 < wait_distance <= limitationofdistance:
                    # print("go close")
                    if self.is_balanced():
                        self.startMotion(self.forwards)
                    return
                else:
                    # print("stay")
                    self.heading = False
                    return
            else:
                print("Heading False")
                if wait_distance >= wait_circle:
                    # print("go to wait point")
                    self.heading = True
                    return
                elif 21.0 <= angle <= 180.0:
                    # print("turn right")
                    if self.is_balanced():
                        self.startMotion(self.turnright40)
                elif -180.0 <= angle <= -21.0:
                    # print("turn left")
                    if self.is_balanced():
                        self.startMotion(self.turnleft40)
                elif self.is_balanced():
                    self.df_stage = DEFENDER_STAGE.INTERCEPT
                return
        elif self.df_stage == DEFENDER_STAGE.STAND_UP:
            print("Stand Up!")
            self.is_standup()
            if self.__standup_stage == STAND_UP.END and self.is_balanced():
                self.df_stage = self.previous_stage
            return
        else:
            print("Unknown stage")
            return False






defender = Nao_Defender()
while defender.step(defender.timeStep) != -1:
    defender.intercepting()