from controller import Supervisor, Robot, Motion, motion
import json
from enum import Enum,auto,unique
import numpy as np

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

class Nao_Goalkeeper(Robot):
    PHALANX_MAX = 8
    kick_stage = KICK_STAGE.INITIAL

    def loadMotionFiles(self):
        '''
        This Function mainly loads the motion files from libraries
        '''
        self.forwards = Motion('libraries/Forwards.motion')
        self.backwards = Motion('libraries/Backwards.motion')
        self.shoot = Motion('libraries/Shoot.motion')
        self.turnleft40 = Motion('libraries/TurnLeft40.motion')
        self.turnright40 = Motion('libraries/TurnRight40.motion')
        self.sidestepleft = Motion('libraries/SideStepLeft.motion')
        self.sidestepright = Motion('libraries/SideStepRight.motion')
        self.KICK = Motion('libraries/KICK.motion')

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

            robot_position = np.float64(shared_info["goalkeeper_red"]["position"])
            robot_orientation = np.float64(shared_info["goalkeeper_red"]["orientation"])
            football_position = np.float64(shared_info["football"]["position"])

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

goal_keeper = Nao_Goalkeeper()
while goal_keeper.step(goal_keeper.timeStep) != -1:
    pass
    goal_keeper.defendingBall()