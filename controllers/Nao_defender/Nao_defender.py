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
    PREPARE = auto()
    INTERCEPT = auto()
    ADJUSTING_ANGLE = auto()
    APPROCH = auto()
    SIDE_STEP_ADJUST = auto()
    FINISH = auto()
    END = auto()

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
        if cross_product < 0:
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

            robot_position = np.float64(shared_info["defender_red_1"]["position"])
            robot_orientation = np.float64(shared_info["defender_red_1"]["orientation"])
            football_position = np.float64(shared_info["SoccerBall"]["position"])

            self.receiver.nextPacket()
            return robot_position, robot_orientation, football_position
        else:
            return None, None, None

    def intercept_solving(self, football_position, robot_position, goal_position, orientation):
        """calculate the vector football to goal, and the distance between robot and best intercept position"""
        """用向量的方法求球到球门的连线的法线方向，然后求出机器人到这个连线的垂点的直线距离，返回这个距离和连线的指向"""
        vector_ball2goal = goal_position[0:2] - football_position[0:2]
        vector_robot2goal = goal_position[0:2] - robot_position[0:2]
        projection_length = np.dot(vector_ball2goal, vector_robot2goal)/np.dot(vector_ball2goal,vector_ball2goal)
        vector_projection = projection_length*vector_ball2goal
        vector_intercept = vector_robot2goal - vector_projection
        distance = np.linalg.norm(vector_intercept) #here is the distance

        # below is calculating angle need to turn

        vector_heading_normalized = np.array([0.8,0.6])

        vector_intercept_normalized = vector_intercept/distance
        amplitude_orientation = np.sqrt(orientation[0] ** 2 + orientation[3] ** 2)
        vector_heading_normalized = [orientation[0]/amplitude_orientation, orientation[3]/amplitude_orientation]
        dot_product = np.dot(vector_intercept_normalized, vector_heading_normalized)
        intercept_angle = np.rad2deg(np.arccos(dot_product))
        cross_product = vector_intercept_normalized[0] * vector_heading_normalized[1] - \
                        vector_intercept_normalized[1] * vector_heading_normalized[0]
        if cross_product < 0:
            intercept_angle = -intercept_angle

        return intercept_angle,distance

    def intercepting(self):
        """
        this version can work, but still has many logical details to fill up
        :return:
        """
        limitationofdistance = 0.2
        limitationofdistance2 = 0.1
        bitsOfRound = 2
        goal_red = [4.5,0]
        """get this robot position,direction, and football position"""
        """in every time step"""
        robot_position, robot_orientation, football_position = self.position_refresh()
        if robot_position is None or robot_orientation is None or football_position is None:
            print("robot_position or robot_orientation or football_position is None!")
            return
        # robot_position_round = [float(round(k,bitsOfRound)) for k in robot_position]
        # print(f"robot_position: {robot_position_round}")
        # angle, distance = self.angleCalculaor(football_position, robot_position, robot_orientation)
        # if angle is None or distance is None:
        #     print("angle is None or distance is None!")
        #     return
        # print(f"angle: {angle}, distance: {distance}")
        intercept_angle, distance = self.intercept_solving(football_position, robot_position, goal_red,robot_orientation)
        # print("intercept_angle, distance: ", intercept_angle, distance)
        """initialize joints"""
        if self.df_stage == DEFENDER_STAGE.INITIAL:
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
                self.previous_stage = self.df_stage
                self.df_stage = DEFENDER_STAGE.INTERCEPT
                return
            else:
                return

            """ decide to turn or directly to intercept"""
        elif self.df_stage == DEFENDER_STAGE.INTERCEPT:
            print("DEFENDER INTERCEPT")
            # print("intercept_angle, distance: ", intercept_angle, distance)
            if distance <= limitationofdistance:
                self.df_stage = DEFENDER_STAGE.SIDE_STEP_ADJUST
                return
            elif 180.0 >= intercept_angle >= 15.0 or -15.0 >= intercept_angle >= -180.0:
                self.__temp_angle = intercept_angle
                self.df_stage = DEFENDER_STAGE.ADJUSTING_ANGLE
                return
            else:
                self.df_stage = DEFENDER_STAGE.APPROCH
                return

            '''turn to intercept direction'''
        elif self.df_stage == DEFENDER_STAGE.ADJUSTING_ANGLE:
            print("DEFENDER ADJUSTING_ANGLE")
            if np.round(np.abs(intercept_angle),1) >= 15.0:
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
            if np.abs(intercept_angle) <= 15.0:
                # self.stopMotion()
                self.df_stage = DEFENDER_STAGE.APPROCH
                return
            else:
                return

            '''go to intercept position and turn to next stage when close enough'''
        elif self.df_stage == DEFENDER_STAGE.APPROCH:
            print("DEFENDER APPROCH")
            print(f"distance is {distance}")
            print(f"intercept angle is {intercept_angle}")
            if distance >= limitationofdistance:
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
                if 180.0 >= intercept_angle >= 15.0 or -15.0 >= intercept_angle >= -180.0:
                    self.__temp_angle = intercept_angle
                    self.df_stage = DEFENDER_STAGE.ADJUSTING_ANGLE
                else:
                    self.startMotion(self.forwards)
                    return
            else:
                # self.stopMotion()
                self.__temp_angle = intercept_angle
                self.df_stage = DEFENDER_STAGE.SIDE_STEP_ADJUST
                return

             # turn towards to ball, side step to block the ball, to next stage when close enough
        elif self.df_stage == DEFENDER_STAGE.SIDE_STEP_ADJUST:
            print("DEFENDER SIDE_STEP_ADJUST")
            print(f"distance is {distance}")
            angle, distance_2_ball = self.angleCalculaor(football_position, robot_position, robot_orientation)
            print(f"angle is {angle}")
            self.__temp_angle = angle
            if np.round(np.abs(self.__temp_angle),1) >= 15.0:
                if ((-120.0 <= self.__temp_angle <= -15.0)
                        or (-90.0 <= self.__temp_angle <= -15.0 and self.isTurningRight is None)
                        or self.isTurningRight):
                    self.startMotion(self.turnright40)
                    self.isTurningRight = True
                if ((120.0 >= self.__temp_angle >= 15.0)
                        or (90.0 >= self.__temp_angle >= 15.0 and self.isTurningRight is None)
                        or not self.isTurningRight):
                    self.startMotion(self.turnleft40)
                    self.isTurningRight = False
                else:
                    print("over angle! line 1255")
            elif np.round(np.abs(self.__temp_angle),1) < 15.0:
                self.isTurningRight = None
                if np.round(np.round(distance,bitsOfRound),1) > limitationofdistance2:
                    if intercept_angle < limitationofdistance2:
                        self.startMotion(self.sidestepleft)
                    if intercept_angle > limitationofdistance2:
                        self.startMotion(self.sidestepright)
                else:
                    # self.stopMotion()
                    self.df_stage = DEFENDER_STAGE.FINISH
            elif distance >= limitationofdistance:
                self.df_stage = DEFENDER_STAGE.INTERCEPT
            else:
                print("Something went wrong in side step angle judge line 1269")
                return

        elif self.df_stage == DEFENDER_STAGE.FINISH:
            print("INTERCEPT FINISH")
            if distance >= limitationofdistance2:
                self.df_stage = DEFENDER_STAGE.INTERCEPT
            if self.is_balanced():
                self.df_stage = DEFENDER_STAGE.END
            return
        elif self.df_stage == DEFENDER_STAGE.END:
            print("INTERCEPT END")
            if distance >= limitationofdistance2:
                self.df_stage = DEFENDER_STAGE.INTERCEPT
            return True
        else:
            print("Unknown stage")
            return False



defender = Nao_Defender()
while defender.step(defender.timeStep) != -1:
    pass
    # defender.intercepting()
    defender.startMotion(defender.turnright40)