from fontTools.misc.cython import returns

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

class K2OPPO_STAGE(Enum):
    INITIAL = auto()
    ANALYSE = auto()
    WAIT = auto()
    APPROCH = auto()
    ADJUST_ANGLE = auto()
    KICK = auto()
    POWER_KICK = auto()
    FINISH = auto()
    SWITCH = auto()
    STAND_UP = auto()

class K2MATE_STAGE(Enum):
    INITIAL = auto()
    ANALYSE = auto()
    STAND_BY = auto()
    APPROCH = auto()
    ADJUST_ANGLE = auto()
    KICK = auto()
    POWER_KICK = auto()
    FINISH = auto()
    SWITCH = auto()
    STAND_UP = auto()

class DEFENDER_STAGE(Enum):
    INITIAL = auto()
    ANALYSE = auto()
    ADJUSTING_ANGLE = auto()
    APPROCH = auto()
    SIDE_STEP_ADJUST = auto()
    SIDE_STEP = auto()
    FINISH = auto()
    ADVANCE = auto()
    KICK = auto()
    VICE_DEFEND = auto()
    STAND_UP = auto()

class STAND_UP(Enum):
    INITIAL = auto()
    PREPARE = auto()
    FROM_FRONT = auto()
    FROM_BACK = auto()
    FINISH = auto()
    END = auto()

class DEFENDER_ROLE(Enum):
    INTERCEPT = auto()  # 防守
    KICK2OPPO = auto()  # 传球给对方半场
    KICK2MATE = auto()  # 传球给队友
    HOLD_BALL = auto()  # 拿着球等


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
        self.k2o_stage = K2OPPO_STAGE.INITIAL
        self.k2mate_stage = K2MATE_STAGE.INITIAL
        self.defender_role = DEFENDER_ROLE.INTERCEPT
        self.heading = None
        self.__side_count = 0
        self.__winding = True

    def set_stage(self, stage=None):
        '''
        This function sets the current stage.
        It will check the stage belongs to which one.
        :param stage:
        :return:
        '''
        if not isinstance(stage, Enum):
            print(f"Stage {stage} not supported")
            return
        if type(stage) in [MOTION_PLAY, KICK_STAGE, STAND_UP, DEFENDER_STAGE]:
            setattr(self, f"{type(stage).__name__.lower()}_stage", stage)
        else:
            print(f"Stage {stage} not supported")
        return

    def is_balanced(self):
        vel = self.gyro.getValues()
        # print('angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1]))
        # print(vel[0], vel[1])
        all_in_balance = np.round(float(vel[0]),2) == 0.0 and np.round(float(vel[1]),2) == 0.0

        # print(f"all_in_balance: {all_in_balance}")
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
        """input:[0:3]   output:[0:3]"""
        """input:[0:2]   output:[0:2]"""
        output = position.copy() #避免修改输入的数组，影响dot product
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
                output[0] = penalty_bounds[closest_edge_idx]
            else:
                output[1] = penalty_bounds[closest_edge_idx]
        if "Red" in self.myName:
            if position[0] < 1.3:
                output[0] = 1.3
        elif "Blue" in self.myName:
            if position[0] > -1.3:
                output[0] = -1.3
        return output

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

    def powerful_kick(self):
        if self.kick_stage is KICK_STAGE.INITIAL:
            print("KICK INITIAL")
            for name in self.motor_names:
                if name in self.motors:
                    self.setMotorPosition(name, 0.0)
            all_in_end = all(self.getMoveStage(n) is move_status.END for n in self.motor_names)
            if all_in_end and self.is_balanced():
                self.kick_stage = KICK_STAGE.PREPARE
            return

        elif self.kick_stage is KICK_STAGE.PREPARE:
            print('Stage 0: PREPARE', flush=True)
            if self.prepare_kick():
                self.kick_stage = KICK_STAGE.WEIGHT_SHIFT
            return
        elif self.kick_stage is KICK_STAGE.WEIGHT_SHIFT:
            print('Stage 1: Weight shift', flush=True)
            self.setMotorPosition('LHipRoll', 0.3)  # 8.59
            self.setMotorPosition('RHipRoll', 0.2)
            self.setMotorPosition('LAnkleRoll', -0.2)
            self.setMotorPosition('RAnkleRoll', -0.2)

            if (self.getMoveStage('LHipRoll') is move_status.END and
                    self.getMoveStage('RHipRoll') is move_status.END and
                    self.getMoveStage('LAnkleRoll') is move_status.END and
                    self.getMoveStage('RAnkleRoll') is move_status.END):
                self.kick_stage = KICK_STAGE.BEND_LEFT_LEG
            return
        elif self.kick_stage is KICK_STAGE.BEND_LEFT_LEG:
            print('Stage 2: bend the left leg', flush=True)
            self.setMotorPosition('LKneePitch', 2.1)
            self.setMotorPosition('LHipPitch', 0.17)  # 9.75
            self.setMotorPosition('LAnklePitch', -1.0)
            self.setMotorPosition('RShoulderRoll', -0.26)
            self.setMotorPosition('LShoulderRoll', -0.26)

            if (self.getMoveStage('LKneePitch') is move_status.END and
                    self.getMoveStage('LHipPitch') is move_status.END and
                    self.getMoveStage('LAnklePitch') is move_status.END and
                    self.getMoveStage('RShoulderRoll') is move_status.END and
                    self.getMoveStage('LShoulderRoll') is move_status.END):
                self.kick_stage = KICK_STAGE.KICK
            return
        elif self.kick_stage is KICK_STAGE.KICK:
            print('Stage 3: Kick', flush=True)
            self.setMotorPosition('LHipPitch', -1.22)
            self.setMotorPosition('LAnklePitch', -0.3)
            if (self.getMoveStage('LHipPitch') is move_status.END and
                    self.getMoveStage('LAnklePitch') is move_status.END):
                self.kick_stage = KICK_STAGE.LEG_IN
            return
        elif self.kick_stage is KICK_STAGE.LEG_IN:
            print('Stage 4: LEG_IN', flush=True)
            self.setMotorPosition('LKneePitch', -0.09)
            if (self.getMoveStage('LKneePitch') is move_status.END):
                self.kick_stage = KICK_STAGE.COMPLETE
            return
        elif self.kick_stage is KICK_STAGE.COMPLETE:
            print('Stage 5: Kick complete', flush=True)
            for name in self.motor_names:
                if name in self.motors:
                    self.setMotorPosition(name, 0.0)

            all_in_end = all(self.getMoveStage(n) is move_status.END for n in self.motor_names)
            if all_in_end:
                self.kick_stage = KICK_STAGE.IS_BALANCE
            return
        elif self.kick_stage is KICK_STAGE.IS_BALANCE:
            print('Stage 6: Is balance', flush=True)
            if self.is_balanced():
                self.kick_stage = KICK_STAGE.END
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

    def calculate_navigation_vector(self,defender_pos, intercept_pos, avoid_objects, avoid_radius=1):
        """
        计算带避让功能的导航向量

        """
        repulse_factor = 1.5

        # 计算吸引力（指向拦截点）
        vector_to_intercept = intercept_pos[0:2] - defender_pos[0:2]
        distance_to_intercept = np.linalg.norm(vector_to_intercept)
        force_attract = vector_to_intercept / (distance_to_intercept + 1e-6)  # 避免除零

        # 计算斥力（避让所有障碍物）
        total_repulse_force = np.array([0.0, 0.0])

        def compute_repulsive_force(obstacle_pos):
            vector_to_obstacle = obstacle_pos[0:2] - defender_pos[0:2]
            distance_to_obstacle = np.linalg.norm(vector_to_obstacle)

            if distance_to_obstacle < avoid_radius:
                # 斥力 = repulse_factor * log(1 + (R - d) / R)，避免剧烈跳变
                repulse_strength = -repulse_factor * np.log(1 + (avoid_radius - distance_to_obstacle) / avoid_radius)
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

    def calculate_perpendicular_intercept(self, football_position, robot_position, goal_position):
        # 计算球到球门的向量 & 机器人到球门的向量
        vector_ball2goal = goal_position[0:2] - football_position[0:2]
        vector_robot2goal = goal_position[0:2] - robot_position[0:2]
        # 计算拦截点向量(defender指向拦截点)
        projection_length = np.dot(vector_ball2goal, vector_robot2goal) / np.dot(vector_ball2goal, vector_ball2goal)
        vector_projection = projection_length * vector_ball2goal
        vector_intercept = vector_robot2goal - vector_projection
        distance = np.linalg.norm(vector_intercept)  # here is the distance
        ori_intercept_position = robot_position[0:2] + vector_intercept
        return ori_intercept_position

    def calculate_equilateral_intercept(self, football_position, robot_position, goal_position):
        """
        计算位于球门-球连线上的拦截点，该点到球和机器人的距离相等。
        :param football_position: (x_f, y_f) 球的位置
        :param robot_position: (x_r, y_r) 机器人的位置
        :param goal_position: (x_g, y_g) 球门的位置
        :return: (x_i, y_i) 拦截点坐标
        """
        # 计算 球门 -> 球 的向量和单位向量
        V_ball2goal = football_position[0:2] - goal_position[0:2]
        unit_V_ball2goal = V_ball2goal / np.linalg.norm(V_ball2goal)  # 归一化
        # 计算 机器人 -> 球 的向量
        V_robot2ball = football_position[0:2] - robot_position[0:2]
        # 计算 t
        numerator = np.linalg.norm(V_robot2ball) ** 2
        denominator = -2 * np.dot(V_robot2ball, unit_V_ball2goal)
        if abs(denominator) < 1e-6:  # 避免除零错误
            return football_position  # 如果计算失败，返回球的位置
        t = numerator / denominator
        # 计算拦截点 P_i
        P_i = football_position[0:2] + t * unit_V_ball2goal
        return P_i

    def compute_intercept_angle(self,vector_intercept, orientation):
        """below is calculating angle need to turn"""
        vector_intercept_normalized = vector_intercept / np.linalg.norm(vector_intercept)
        amplitude_orientation = np.sqrt(orientation[0] ** 2 + orientation[3] ** 2)
        vector_heading_normalized = [orientation[0] / amplitude_orientation, orientation[3] / amplitude_orientation]
        dot_product = np.dot(vector_intercept_normalized, vector_heading_normalized)
        intercept_angle = np.rad2deg(np.arccos(dot_product))
        cross_product = vector_intercept_normalized[0] * vector_heading_normalized[1] - \
                        vector_intercept_normalized[1] * vector_heading_normalized[0]
        if cross_product < 0:
            intercept_angle = -intercept_angle
        # print(f"intercept position:{intercept_position}")
        return intercept_angle

    def intercept_solving(self, football_position, robot_position, goal_position, orientation,avoid_objects):
        """calculate the vector football to goal, and the distance between robot and best intercept position"""
        """用向量的方法求球到球门的连线的法线方向，然后求出机器人到这个连线的垂点的直线距离，返回这个距离和连线的指向"""
        """判断这个拦截点是否已经超过自己的防守深度了，超了那就往球指向门的向量上，球前方确定距离的点位赶路"""
        """在上面基础上继续判断球是否越过了自己，已经越过了那就朝绕过足球的位置走"""
        vector_ball2goal = goal_position[0:2] - football_position[0:2]
        # 计算垂点拦截点
        ori_intercept_position = self.calculate_perpendicular_intercept(football_position, robot_position, goal_position)

        """计算禁区边缘和地方半场"""
        """constrain of panalty zone and oppo zone"""
        intercept_position = self.over_range_detect(ori_intercept_position)
        """refresh intercept vector"""
        vector_intercept = intercept_position - robot_position[0:2]
        distance = np.linalg.norm(vector_intercept)

        """calculate the intercept point, to see if it is too late to intercept"""
        """if impossible,choose the point at the front of the ball to intercept"""
        unit_vector_ball2goal = vector_ball2goal/np.abs(np.linalg.norm(vector_ball2goal))

        vector_ball2intercept = ori_intercept_position - football_position[0:2]
        dot_product = np.dot(unit_vector_ball2goal, vector_ball2intercept)
        # print(f"vector_ball2intercept:{vector_ball2intercept}")
        # print(f"orig intercept position: {ori_intercept_position}")
        # print(f"football_position:{football_position[0:2]}")


        if dot_product <=0.6: #球越过了defender
            if dot_product <= 0:
                print("dot<0")
                # 计算球的法向量方向（球-球门连线的垂直方向）
                perp_vector = np.array([-unit_vector_ball2goal[1], unit_vector_ball2goal[0]])
                # 选取正确的绕行方向（确保不往球门方向绕）
                if np.dot(perp_vector, robot_position[:2] - football_position[:2]) < 0:
                    perp_vector = -perp_vector
                d = 0.4       # 设定绕行距离 d
                new_intercept_position = football_position[:2] + d * perp_vector           # **绕行点范围约束，避免超出边界**
                new_intercept_position = self.over_range_detect(new_intercept_position)
                # 计算机器人到绕行点的向量
                vector_new = self.calculate_navigation_vector(robot_position, new_intercept_position, avoid_objects)
                angle = self.compute_intercept_angle(vector_new, orientation)               # 计算角度
                return angle, np.linalg.norm(new_intercept_position - robot_position[:2])
            else:
                new_intercept_position = self.calculate_equilateral_intercept(football_position, robot_position, goal_position)
                #penalty zone check and correct
                # 检查点是否在禁区内
                new_intercept_position = self.over_range_detect(new_intercept_position)
                distance = np.linalg.norm(new_intercept_position - robot_position[0:2])
                # print(new_intercept_position)
                # print(football_position)
                vector_new = self.calculate_navigation_vector(robot_position, new_intercept_position, avoid_objects)
                angle = self.compute_intercept_angle(vector_new, orientation)
                return angle,distance

        vector_intercept = self.calculate_navigation_vector(robot_position, intercept_position, avoid_objects)
        intercept_angle = self.compute_intercept_angle(vector_intercept,orientation)

        return intercept_angle,distance

    def ball_clear_judge(self,football_position,intercepter_position,tolerance_angle=30,torlerance_distance=1):
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
        new_intercept_position = self.over_range_detect(new_intercept_position)
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

    def face_and_go(self,robot_position,target_position,orientation, avoid_objects, angle_tor = 20, go = True):
        """turn to target, and (optional) move forward"""
        """calculate first"""
        navigation_vector = self.calculate_navigation_vector(robot_position,target_position,avoid_objects)
        angle = self.compute_intercept_angle(navigation_vector, orientation)
        """then move"""
        if np.round(np.abs(angle), 1) >= angle_tor:
            if ((180.0 >= angle >= angle_tor)
                    or (180.0 >= angle >= angle_tor and self.isTurningRight is None)
                    or self.isTurningRight):
                if self.is_balanced():
                    self.startMotion(self.turnright40)
                    self.isTurningRight = True
            if ((-180.0 <= angle <= -angle_tor)
                    or (-180.0 <= angle <= -angle_tor and self.isTurningRight is None)
                    or not self.isTurningRight):
                if self.is_balanced():
                    self.startMotion(self.turnleft40)
                    self.isTurningRight = False
            # print(f"{self.isTurningRight}")
            # print(f"angle: {angle}")
        if np.abs(angle) <= angle_tor + 1:
            # self.stopMotion()
            if self.is_balanced() and go:
                self.startMotion(self.forwards)

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

    def intercepting(self):
        """
        this version of defender can work
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

        # print(f"intercept_angle: {intercept_angle}")
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
                self.df_stage = DEFENDER_STAGE.ANALYSE
            else:
                return

            """ decide to turn or directly to intercept"""
        elif self.df_stage == DEFENDER_STAGE.ANALYSE:
            print("INTERCEPT")
            if self.standupIfnecessary():
                self.previous_stage = self.df_stage
                self.set_stage(STAND_UP.INITIAL)
                self.df_stage = DEFENDER_STAGE.STAND_UP
                return
            if distance2mate < alert_range_mate:
                if "Red" in self.myName:
                    if intercept_distance > intercept_distance_mate and football_position[0] < self.mate_position[0]:
                        self.df_stage = DEFENDER_STAGE.VICE_DEFEND
                        return
                elif "Blue" in self.myName:
                    if intercept_distance > intercept_distance_mate and football_position[0] > self.mate_position[0]:
                        self.df_stage = DEFENDER_STAGE.VICE_DEFEND
                        return

                '''turn to intercept direction'''
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
        elif self.df_stage == DEFENDER_STAGE.ADJUSTING_ANGLE:
            if self.standupIfnecessary():
                self.previous_stage = self.df_stage
                self.set_stage(STAND_UP.INITIAL)
                self.df_stage = DEFENDER_STAGE.STAND_UP
                return
            if distance2mate < alert_range_mate:
                if "Red" in self.myName:
                    if intercept_distance > intercept_distance_mate and football_position[0] < self.mate_position[0]:
                        self.df_stage = DEFENDER_STAGE.VICE_DEFEND
                    return
                elif "Blue" in self.myName:
                    if intercept_distance > intercept_distance_mate and football_position[0] > self.mate_position[0]:
                        self.df_stage = DEFENDER_STAGE.VICE_DEFEND
                    return

                '''turn to intercept direction'''
            print("DEFENDER ADJUSTING_ANGLE")
            if intercept_distance <= limitationofdistance:
                self.df_stage = DEFENDER_STAGE.APPROCH
                return
            if np.round(np.abs(intercept_angle),1) >= 20.0:
                if ((180.0 >= intercept_angle >= 20.0)
                        or (180.0 >= intercept_angle >= 20.0 and self.isTurningRight is None)
                        or self.isTurningRight):
                    if self.is_balanced():
                        self.startMotion(self.turnright40)
                        self.isTurningRight = True
                if ((-180.0 <= intercept_angle <= -20.0)
                        or (-180.0 <= intercept_angle <= -20.0 and self.isTurningRight is None)
                        or not self.isTurningRight):
                    if self.is_balanced():
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
            if distance2mate < alert_range_mate:
                if "Red" in self.myName:
                    if intercept_distance > intercept_distance_mate and football_position[0] < self.mate_position[0]:
                        self.df_stage = DEFENDER_STAGE.VICE_DEFEND
                    return
                elif "Blue" in self.myName:
                    if intercept_distance > intercept_distance_mate and football_position[0] > self.mate_position[0]:
                        self.df_stage = DEFENDER_STAGE.VICE_DEFEND
                    return

                '''turn to intercept direction'''
            print("DEFENDER APPROCH")
            # print(f"intercept_distance is {intercept_distance}")
            # print(f"intercept angle is {intercept_angle}")
            if intercept_distance >= limitationofdistance:
                if ((90.0 >= intercept_angle >= 25.0)
                        or (90.0 >= intercept_angle >= 25.0 and self.isTurningRight is None)
                        or self.isTurningRight):
                    if self.is_balanced():
                        self.startMotion(self.turnright40)
                        self.isTurningRight = True
                if ((-90.0 <= intercept_angle <= -25.0)
                        or (-90.0 <= intercept_angle <= -25.0 and self.isTurningRight is None)
                        or not self.isTurningRight):
                    if self.is_balanced():
                        self.startMotion(self.turnleft40)
                        self.isTurningRight = False
                if 180.0 >= intercept_angle >= 25.0 or -25.0 >= intercept_angle >= -180.0:
                    if self.is_balanced():
                        self.df_stage = DEFENDER_STAGE.ADJUSTING_ANGLE
                    return
                else:
                    self.startMotion(self.forwards)
                    return
            # elif limitationofdistance2 < intercept_distance <= limitationofdistance :
            #     self.startMotion(self.forwards)
            #     return
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
                self.df_stage = DEFENDER_STAGE.ANALYSE
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
                    self.df_stage = DEFENDER_STAGE.ANALYSE
                return
            elif np.round(np.abs(self.__temp_angle),1) > 30.0:
                if self.is_balanced():
                    self.df_stage = DEFENDER_STAGE.SIDE_STEP_ADJUST
                return
            elif distance2ball <= limitationofdistance and np.abs(angle)<=22:
                self.df_stage = DEFENDER_STAGE.KICK
                return
            """下面是一个滞回比较器，（大概是叫这个名字吧）"""
            if self.isStepRight is None:        #判断左走还是右走
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
            elif self.isStepRight:     #右走
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
            elif not self.isStepRight:   #左走
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
                self.df_stage = DEFENDER_STAGE.ANALYSE
            if self.is_balanced():
                self.df_stage = DEFENDER_STAGE.ADVANCE
            return
        elif self.df_stage == DEFENDER_STAGE.ADVANCE:
            # print(f"goal position:{self.goal_position}")
            # print(f"oppo goal:{self.oppo_goal_position}")
            if self.standupIfnecessary():
                self.previous_stage = self.df_stage
                self.set_stage(STAND_UP.INITIAL)
                self.df_stage = DEFENDER_STAGE.STAND_UP
                return
            print("ADVANCE")
            if intercept_distance >= limitationofdistance+0.1:
                self.df_stage = DEFENDER_STAGE.ANALYSE
                return True
            elif distance2ball <= limitationofdistance and np.abs(angle)<=22:
                self.df_stage = DEFENDER_STAGE.KICK
                return
            elif np.array_equal(self.over_range_detect(robot_position), robot_position):
                if self.is_balanced():
                    self.startMotion(self.forwards)
            return
        elif self.df_stage == DEFENDER_STAGE.KICK:
            if self.standupIfnecessary():
                self.previous_stage = self.df_stage
                self.set_stage(STAND_UP.INITIAL)
                self.df_stage = DEFENDER_STAGE.STAND_UP
                return
            if self.ball_clear_judge(football_position,self.oppo_striker_position,30,1):
                self.powerful_kick()
                if self.kick_stage == KICK_STAGE.END:
                    self.df_stage = DEFENDER_STAGE.INITIAL
            else:
                self.kick_motion()
                if self.kick_stage == KICK_STAGE.END:
                    self.df_stage = DEFENDER_STAGE.INITIAL
            return

        elif self.df_stage == DEFENDER_STAGE.VICE_DEFEND:
            if self.standupIfnecessary():
                self.previous_stage = self.df_stage
                self.set_stage(STAND_UP.INITIAL)
                self.df_stage = DEFENDER_STAGE.STAND_UP
                return
            print("VICE DEFEND")
            # print(f"intercept distance: {intercept_distance}")
            wait_circle = 0.4
            if distance2mate > alert_range_mate+0.2 or intercept_distance < intercept_distance_mate+0.2:
                # print("to intercept")
                self.df_stage = DEFENDER_STAGE.ANALYSE
                return
            if "Red" in self.myName and football_position[0] > self.mate_position[0]:
                # print("to intercept")
                self.df_stage = DEFENDER_STAGE.ANALYSE
                return
            elif "Blue" in self.myName and football_position[0] < self.mate_position[0]:
                # print("to intercept")
                self.df_stage = DEFENDER_STAGE.ANALYSE
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
                # print(f"angle:{angle}")
                if wait_distance >= wait_circle:
                    # print("go to wait point")
                    self.heading = True
                    return
                elif 22.0 <= angle <= 180.0:
                    # print("turn right")
                    if self.is_balanced():
                        self.startMotion(self.turnright40)
                        return
                elif -180.0 <= angle <= -22.0:
                    # print("turn left")
                    if self.is_balanced():
                        self.startMotion(self.turnleft40)
                        return
                elif self.is_balanced():
                    self.df_stage = DEFENDER_STAGE.ANALYSE
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

    def kick2oppo(self):
        """
        in this state, defender will wait for the ball at a point. When the ball reaches its half field,
        defender will kick it to oppo field
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

        avoid_objects_mate = [self.striker_position, robot_position]


        angle2mate, distance2mate = self.angleCalculaor(self.mate_position, robot_position, robot_orientation)
        angle, distance2ball = self.angleCalculaor(football_position, robot_position, robot_orientation)
        angle_mate, distance2ball_mate = self.angleCalculaor(football_position, self.mate_position, robot_orientation)
        angle_oppo_striker,distance2oppo_striker = self.angleCalculaor(self.oppo_striker_position, robot_position, robot_orientation)


        """initialize joints"""
        if self.k2o_stage == K2OPPO_STAGE.INITIAL:
            if self.standupIfnecessary():
                self.previous_stage = self.k2o_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2o_stage = K2OPPO_STAGE.STAND_UP
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
                self.previous_stage = self.k2o_stage
                self.k2o_stage = K2OPPO_STAGE.ANALYSE
            else:
                return

            """ decide to turn or directly to intercept"""
        elif self.k2o_stage == K2OPPO_STAGE.ANALYSE:
            print("ANALYSE")
            if self.standupIfnecessary():
                self.previous_stage = self.k2o_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2o_stage = K2OPPO_STAGE.STAND_UP
                return

                return
            self.waiting_point = np.zeros(2)
            if "Blue" in self.myName:
                self.waiting_point[0] = -1.5
            else:
                self.waiting_point[0] = 1.5
            if football_position[1] > 0:
                self.waiting_point[1] = -1.5
            else:
                self.waiting_point[1] = 1.5
            self.k2o_stage = K2OPPO_STAGE.WAIT
            return
        elif self.k2o_stage == K2OPPO_STAGE.WAIT:
            print("DEFENDER WAITING")
            if self.standupIfnecessary():
                self.previous_stage = self.k2o_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2o_stage = K2OPPO_STAGE.STAND_UP
                return
                '''turn to wait direction'''

            _, distance2wait_point = self.angleCalculaor(self.waiting_point, robot_position, robot_orientation)
            if distance2wait_point > limitationofdistance:         #没到wait point范围内那就走过去
                self.face_and_go(robot_position,self.waiting_point,robot_orientation,avoid_objects_me)
            else:                                                  #到了就面向球
                self.face_and_go(robot_position, football_position, robot_orientation, avoid_objects_me, go = False)
            if distance2ball < 1 :           #球靠近了就过去
                self.k2o_stage = K2OPPO_STAGE.APPROCH
            if "Blue" in self.myName and football_position[1]*robot_position[1] > 1 and football_position[0] < 0 :
                self.k2o_stage = K2OPPO_STAGE.APPROCH
            elif football_position[1]*robot_position[1] > 1 and football_position[0] > 0 :
                self.k2o_stage = K2OPPO_STAGE.APPROCH
            else:
                print("Ball is elsewhere")
            return
            '''go to waiting position and turn to ball, wait until ball is close enough'''
        elif self.k2o_stage == K2OPPO_STAGE.APPROCH:
            if self.standupIfnecessary():
                self.previous_stage = self.k2o_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2o_stage = K2OPPO_STAGE.STAND_UP
                return

                '''turn to intercept direction'''
            print("DEFENDER APPROCH")
            if distance2ball > limitationofdistance:
                self.face_and_go(robot_position,football_position,robot_orientation,avoid_objects_me,25)
            else:
                self.k2o_stage = K2OPPO_STAGE.ADJUST_ANGLE
            return

             # turn towards to ball, side step to block the ball, to next stage when close enough
        elif self.k2o_stage == K2OPPO_STAGE.ADJUST_ANGLE:
            """circle around the ball until on the right direction"""
            if self.standupIfnecessary():
                self.previous_stage = self.k2o_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2o_stage = K2OPPO_STAGE.STAND_UP
                return
            print("ADJUST_ANGLE")

            target_position = self.waiting_point*[-1,1]
            if distance2ball > limitationofdistance+0.3:
                self.k2o_stage = K2OPPO_STAGE.APPROCH
                return
            # 计算角度
            fake_vec_def2ball = [football_position[0]-robot_position[0],0,0,football_position[1]-robot_position[1]]
            angbetball, _ = self.angleCalculaor(target_position, robot_position, fake_vec_def2ball)
            angle2target, _ = self.angleCalculaor(target_position, robot_position, robot_orientation)

            # 1. 检查是否三点一线
            if self.__winding:
                if not self.__isonline(football_position, target_position, robot_position):
                    print("not in line, circle the ball")

                    # 根据机器人相对球的位置选择绕行方向
                    if angbetball > 0:
                        self.clockwise_winding(distance2ball)  # 顺时针绕球
                    else:
                        self.counterclockwise_winding(distance2ball)  # 逆时针绕球
                    return  # 继续调整
                else:
                    self.__winding = False

            if not self.__winding:
                if np.abs(angle2target) > 20 or distance2ball > 0.2:
                    self.face_and_go(robot_position, target_position, robot_orientation, [[300,300,0]], go=False)
                # 根据 angle 调整站位，使球位于 -15° 角度
                elif angle > -12:  # 球偏右，需要向左调整
                    print("球偏右，机器人向左跨步")
                    self.startMotion(self.sidestepleft)
                    return
                elif angle < -16:  # 球偏左，需要向右调整
                    print("球偏左，机器人向右跨步")
                    self.startMotion(self.sidestepright)
                    return

            # 3. 位置调整完成，根据敌人远近判断如何踢球
            _,distance2oppo_defender1 = self.angleCalculaor (self.oppo_defender_l_position, robot_position, robot_orientation)
            _,distance2oppo_defender2 = self.angleCalculaor (self.oppo_defender_r_position, robot_position, robot_orientation)
            min_dis2oppo = min(distance2oppo_striker,distance2oppo_defender1, distance2oppo_defender2)
            if min_dis2oppo < 1:
                self.k2o_stage = K2OPPO_STAGE.KICK
                self.__winding = True
            else:
                self.k2o_stage = K2OPPO_STAGE.KICK
                self.__winding = True

        elif self.k2o_stage == K2OPPO_STAGE.POWER_KICK:
            if self.standupIfnecessary():
                self.previous_stage = self.k2o_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2o_stage = K2OPPO_STAGE.STAND_UP
                return
            print("DEFENDER POWER_KICK")
            self.powerful_kick()
            if self.kick_stage == KICK_STAGE.END:
                self.k2o_stage = K2OPPO_STAGE.FINISH
            return
        elif self.k2o_stage == K2OPPO_STAGE.KICK:
            if self.standupIfnecessary():
                self.previous_stage = self.k2o_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2o_stage = K2OPPO_STAGE.STAND_UP
                return
            print("DEFENDER KICK 2 OPPO")
            self.kick_motion()
            if self.kick_stage == KICK_STAGE.END:
                self.k2o_stage = K2OPPO_STAGE.FINISH
            return
        elif self.k2o_stage == K2OPPO_STAGE.FINISH:
            # print(f"goal position:{self.goal_position}")
            # print(f"oppo goal:{self.oppo_goal_position}")
            if self.standupIfnecessary():
                self.previous_stage = self.k2o_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2o_stage = K2OPPO_STAGE.STAND_UP
                return
            print("DEFENDER FINISH")
            self.k2o_stage = K2OPPO_STAGE.SWITCH
            return
        elif self.k2o_stage == K2OPPO_STAGE.SWITCH:
            if self.standupIfnecessary():
                self.previous_stage = self.k2o_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2o_stage = K2OPPO_STAGE.STAND_UP
                return
            self.defender_role = DEFENDER_ROLE.INTERCEPT
            return

        elif self.k2o_stage == K2OPPO_STAGE.STAND_UP:
            print("Stand Up!")
            self.is_standup()
            if self.__standup_stage == STAND_UP.END and self.is_balanced():
                self.df_stage = self.previous_stage
            return
        else:
            print("Unknown stage")
            return False

    def kick2mate(self):
        """
         in this state, defender will get the ball from striker.
         When enemy leave another half, defender will kick the ball to another field
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

        avoid_objects_me = [self.striker_position, self.mate_position]

        avoid_objects_mate = [self.striker_position, robot_position]

        angle2mate, distance2mate = self.angleCalculaor(self.mate_position, robot_position, robot_orientation)
        angle, distance2ball = self.angleCalculaor(football_position, robot_position, robot_orientation)
        angle_mate, distance2ball_mate = self.angleCalculaor(football_position, self.mate_position, robot_orientation)
        angle_oppo_striker, distance2oppo_striker = self.angleCalculaor(self.oppo_striker_position, robot_position,
                                                                        robot_orientation)
        stand_by_point = np.zeros(2)     #是否改为走到striker前面，反过身来准备将球踢到自己家里？？
        if "Blue" in self.myName:
            stand_by_point[0] = self.striker_position[0] - 0.5
        else:
            stand_by_point[0] = self.striker_position[0] + 0.5
        if robot_position[1] > 0:
            stand_by_point[1] = self.striker_position[1] + 0.5
        else:
            stand_by_point[1] = self.striker_position[1] - 0.5
        """initialize joints"""
        if self.k2mate_stage == K2MATE_STAGE.INITIAL:
            if self.standupIfnecessary():
                self.previous_stage = self.k2mate_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2mate_stage = K2MATE_STAGE.STAND_UP
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
                self.previous_stage = self.k2mate_stage
                self.k2mate_stage = K2MATE_STAGE.ANALYSE
            else:
                return

            """ decide to turn or directly to intercept"""
        elif self.k2mate_stage == K2MATE_STAGE.ANALYSE:
            print("K2MATE ANALYSE")
            if self.standupIfnecessary():
                self.previous_stage = self.k2mate_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2mate_stage = K2MATE_STAGE.STAND_UP
                return

                return
            self.k2mate_stage = K2MATE_STAGE.STAND_BY
            return
        elif self.k2mate_stage == K2MATE_STAGE.STAND_BY:         #走到striker身后侧面，等球
            if self.standupIfnecessary():
                self.previous_stage = self.k2mate_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2mate_stage = K2MATE_STAGE.STAND_UP
                return
            print("K2MATE GO TO STAND BY")
            _, distance2stand_by = self.angleCalculaor(stand_by_point, robot_position, robot_orientation)
            _, oppo_striker2ball = self.angleCalculaor(football_position, self.oppo_striker_position, robot_orientation)
            if distance2stand_by > limitationofdistance:
                self.face_and_go(robot_position, stand_by_point, robot_orientation, avoid_objects_me, 20)
            else:
                self.face_and_go(robot_position, football_position, robot_orientation,[[300,300,0]],go = False)
            if oppo_striker2ball > distance2ball:
                self.k2mate_stage = K2MATE_STAGE.APPROCH
            return
        elif self.k2mate_stage == K2MATE_STAGE.APPROCH:
            if self.standupIfnecessary():
                self.previous_stage = self.k2mate_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2mate_stage = K2MATE_STAGE.STAND_UP
                return
            print("K2MATE APPROCH")
            if distance2ball > limitationofdistance:
                self.face_and_go(robot_position, football_position, robot_orientation, avoid_objects_me, 22)
            else:
                self.k2mate_stage = K2MATE_STAGE.ADJUST_ANGLE
            return

            # turn towards to ball, side step to block the ball, to next stage when close enough
        elif self.k2mate_stage == K2MATE_STAGE.ADJUST_ANGLE:
            """circle around the ball until on the right direction"""
            if self.standupIfnecessary():
                self.previous_stage = self.k2mate_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2mate_stage = K2MATE_STAGE.STAND_UP
                return
            print("K2MATE ADJUST_ANGLE")

            target_position = np.array(self.mate_position) * [0.8,1,1]
            if distance2ball > limitationofdistance +0.2:
                self.k2mate_stage = K2MATE_STAGE.APPROCH
                return
            # 计算角度
            fake_vec_def2ball = [football_position[0] - robot_position[0], 0, 0,
                                 football_position[1] - robot_position[1]]
            angbetball, _ = self.angleCalculaor(target_position, robot_position, fake_vec_def2ball)
            angle2target, _ = self.angleCalculaor(target_position, robot_position, robot_orientation)

            # 1. 检查是否三点一线
            if self.__winding:
                if not self.__isonline(football_position, robot_position, target_position):
                    print("not in line, circle the ball")
                    # 根据机器人相对球的位置选择绕行方向
                    if angbetball > 0:
                        self.clockwise_winding(distance2ball)  # 顺时针绕球
                    else:
                        self.counterclockwise_winding(distance2ball)  # 逆时针绕球
                    return  # 继续调整
                else:
                    self.__winding = False

            if not self.__winding:
                if np.abs(angle2target) > 20 or distance2ball > limitationofdistance:
                    self.face_and_go(robot_position, target_position, robot_orientation, [[300, 300, 0]], go=False)
                # 根据 angle 调整站位，使球位于 -15° 角度
                elif angle > -9:  # 球偏右，需要向左调整
                    print("球偏右，机器人向左跨步")
                    self.startMotion(self.sidestepleft)
                    return
                elif angle < -16:  # 球偏左，需要向右调整
                    print("球偏左，机器人向右跨步")
                    self.startMotion(self.sidestepright)
                    return

            # 3. 位置调整完成，根据敌人远近判断如何踢球
            _, distance2oppo_defender1 = self.angleCalculaor(self.oppo_defender_l_position, robot_position,
                                                             robot_orientation)
            _, distance2oppo_defender2 = self.angleCalculaor(self.oppo_defender_r_position, robot_position,
                                                             robot_orientation)
            min_dis2oppo = min(distance2oppo_striker, distance2oppo_defender1, distance2oppo_defender2)
            if min_dis2oppo < 1:
                self.k2mate_stage = K2MATE_STAGE.KICK
                self.__winding = True
            else:
                self.k2mate_stage = K2MATE_STAGE.KICK
                self.__winding = True

        elif self.k2mate_stage == K2MATE_STAGE.POWER_KICK:
            if self.standupIfnecessary():
                self.previous_stage = self.k2mate_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2mate_stage = K2MATE_STAGE.STAND_UP
                return
            print("K2MATE POWER_KICK")
            self.powerful_kick()
            if self.kick_stage == KICK_STAGE.END:
                self.k2mate_stage = K2MATE_STAGE.FINISH
            return
        elif self.k2mate_stage == K2MATE_STAGE.KICK:
            if self.standupIfnecessary():
                self.previous_stage = self.k2mate_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2mate_stage = K2MATE_STAGE.STAND_UP
                return
            print("K2MATE KICK 2 OPPO")
            self.kick_motion()
            if self.kick_stage == KICK_STAGE.END:
                self.k2mate_stage = K2MATE_STAGE.FINISH
            return
        elif self.k2mate_stage == K2MATE_STAGE.FINISH:
            if self.standupIfnecessary():
                self.previous_stage = self.k2mate_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2mate_stage = K2MATE_STAGE.STAND_UP
                return
            print("K2MATE FINISH")
            self.k2mate_stage = K2MATE_STAGE.SWITCH
            return
        elif self.k2mate_stage == K2MATE_STAGE.SWITCH:
            if self.standupIfnecessary():
                self.previous_stage = self.k2mate_stage
                self.set_stage(STAND_UP.INITIAL)
                self.k2mate_stage = K2MATE_STAGE.STAND_UP
                return
            print("K2MATE SWITCH")
            self.defender_role = DEFENDER_ROLE.INTERCEPT
            return

        elif self.k2mate_stage == K2MATE_STAGE.STAND_UP:
            print("Stand Up!")
            self.is_standup()
            if self.__standup_stage == STAND_UP.END and self.is_balanced():
                self.df_stage = self.previous_stage
            return
        else:
            print("Unknown stage")
            return False

    def upper_state(self):
        robot_position, robot_orientation, football_position = self.position_refresh()
        if robot_position is None or robot_orientation is None or football_position is None:
            print("robot_position or robot_orientation or football_position is None!")
            return
        def ball_oppo(football_position, robot_position):
            if football_position[0] * robot_position[0] < 0:
                return True
            else:
                return False

        manha_dis2oppo_defender1 = np.array(np.abs(self.oppo_defender_l_position - robot_position))
        manha_dis2oppo_defender2 = np.array(np.abs(self.oppo_defender_r_position - robot_position))
        manha_dis2oppo_striker = np.array(np.abs(self.oppo_striker_position - robot_position))
        angle, distance2ball = self.angleCalculaor(football_position, robot_position, robot_orientation)
        angle_mate, distance2ball_mate = self.angleCalculaor(football_position, self.mate_position, robot_orientation)

        manha_dis2oppo = np.array([manha_dis2oppo_defender1,manha_dis2oppo_defender2,manha_dis2oppo_striker])
        oppo_position = np.array([self.oppo_defender_l_position,self.oppo_defender_r_position,self.oppo_striker_position])

        y_lim = 1
        if np.min(manha_dis2oppo[:,0]) < 1:         #敌在本能寺
            if np.min(manha_dis2oppo[:,1]) > 2:     #但是在对半边
                self.defender_role = DEFENDER_ROLE.KICK2OPPO
            else:
                self.defender_role = DEFENDER_ROLE.INTERCEPT
        elif np.max(oppo_position[:,1]) < y_lim:           #敌无2,不在我边
            if ball_oppo(football_position,robot_position):
                self.defender_role = DEFENDER_ROLE.INTERCEPT
            elif not ball_oppo(football_position,robot_position):
                if distance2ball < distance2ball_mate:          #离得近的去kick to mate
                    self.defender_role = DEFENDER_ROLE.KICK2MATE
                else:
                    self.defender_role = DEFENDER_ROLE.KICK2OPPO
        elif np.min(oppo_position[:,1]) > -y_lim:                           #敌无3，不在我边
            if ball_oppo(football_position,robot_position):
                self.defender_role = DEFENDER_ROLE.INTERCEPT
            elif not ball_oppo(football_position,robot_position):
                if distance2ball < distance2ball_mate:          # 离得近的去kick to mate
                    self.defender_role = DEFENDER_ROLE.KICK2MATE
                else:
                    self.defender_role = DEFENDER_ROLE.KICK2OPPO
        else:
            "敌在对面散开，不在我边"
            if ball_oppo(football_position,robot_position):
                self.defender_role = DEFENDER_ROLE.INTERCEPT
            elif not ball_oppo(football_position,robot_position):
                "拿着球等"
        if self.defender_role == DEFENDER_ROLE.INTERCEPT:
            self.intercepting()
        elif self.defender_role == DEFENDER_ROLE.KICK2MATE:
            self.kick2mate()
        elif self.defender_role == DEFENDER_ROLE.KICK2OPPO:
            self.kick2oppo()
        return


defender = Nao_Defender()
while defender.step(defender.timeStep) != -1:
    defender.kick2mate()


