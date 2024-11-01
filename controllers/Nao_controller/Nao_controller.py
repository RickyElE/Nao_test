"""NaoController controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from enum import Enum, unique, auto
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
# initial NAO robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# initial each joints
# left leg and sensor
l_hip_roll = robot.getDevice('LHipRoll')
l_hip_pitch = robot.getDevice('LHipPitch')
l_knee_pitch = robot.getDevice('LKneePitch')
l_ankle_pitch = robot.getDevice('LAnklePitch')
l_ankle_roll = robot.getDevice('LAnkleRoll')

l_hip_roll_sensor = l_hip_roll.getPositionSensor()
l_hip_pitch_sensor = l_hip_pitch.getPositionSensor()
l_knee_pitch_sensor = l_knee_pitch.getPositionSensor()
l_ankle_pitch_sensor = l_ankle_pitch.getPositionSensor()
l_ankle_roll_sensor = l_ankle_roll.getPositionSensor()

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
l_shoulder_roll_v = l_shoulder_roll.getVelocity()

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
            print(f"\n {joints} is not finding!")
            return None
    def position_is_arrive(self, joints="", targets=None):
        if joints == "":
            print("\nYou do not set any joints")
            return

        if targets is None:
            print("\nYou do not set any positions")
            return

        if joints == "LShoulderPitch":
            if targets is self.__pitch_previous_target:
                position = l_shoulder_pitch_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                # print("\nabs is", passing)
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
                # print("\nabs is", passing)
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        else:
            print("\nCan not find any joints, please set again in position_is_arrive")
            return

    def set_position(self, joints="", positions=None):
        if joints == "":
            print("\nYou do not set any joints")
            return

        if positions is None:
            print("\nYou do not set any positions")
            return
        # print(f"\njoints: {joints}, positions: {positions}, moved: {self.moved}")
        if joints == "LShoulderPitch":
            if positions > self.__max_shoulder_pitch_radian or positions < self.__min_shoulder_pitch_radian:
                print(f"\nThe position you set is {positions}, is out of range in [{self.__max_shoulder_pitch_radian}, {self.__min_shoulder_pitch_radian}]")
                return
            # if self.__pitch_previous_target is positions and self.__pitch_status is move_status.INITIAL:
            #     return
            if self.__pitch_status is move_status.INITIAL:
                print(f"\n {joints} is in Initial")
                self.__pitch_previous_target = positions
                self.__pitch_status = move_status.PREPARE
            elif self.__pitch_status is move_status.PREPARE:
                if positions is self.__pitch_previous_target:
                    print(f"\n {joints} is in Prepare")
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
                    print(f"\n {joints} is in Moving")
                    l_shoulder_pitch.setPosition(positions)
                    print(f"\n {joints}'s velocity is {self.get_velocity(joints)}")
                    if self.position_is_arrive(joints, positions):
                        self.__pitch_status = move_status.FINISH
                else:
                    return
            elif self.__pitch_status is move_status.FINISH:
                if positions is self.__pitch_previous_target:
                    print(f"\n {joints} is in Finish")
                    if self.position_is_arrive(joints, positions):
                        self.__pitch_status = move_status.END
                else:
                    return
            elif self.__pitch_status is move_status.END:
                print(f"\n {joints} is in End")
                if positions is not self.__pitch_previous_target:
                    self.__pitch_status = move_status.INITIAL
                else:
                    return
            else:
                print(f"\n {joints} is in error status!")
                return
        elif joints == "LShoulderRoll":
             if positions > self.__max_shoulder_roll_radian or positions < self.__min_shoulder_roll_radian:
                    print(f"\nThe position you set is {positions}, is out of range in [{self.__max_shoulder_roll_radian}, {self.__min_shoulder_roll_radian}]")
                    return
             # if self.__roll_previous_target is positions and self.__roll_status is move_status.INITIAL:
             #    return
             if self.__roll_status is move_status.INITIAL:
                    print(f"\n {joints} is in Initial")
                    self.__roll_previous_target = positions
                    self.__roll_status = move_status.PREPARE
             elif self.__roll_status is move_status.PREPARE:
                 if positions is self.__roll_previous_target:
                    print(f"\n {joints} is in Prepare")
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
                        print(f"\n {joints} is in Moving")
                        l_shoulder_roll.setPosition(positions)
                        print(f"\n {joints}'s velocity is {self.get_velocity(joints)}")
                        if self.position_is_arrive(joints, positions):
                            self.__roll_status = move_status.FINISH
                    else:
                        return
             elif self.__roll_status is move_status.FINISH:
                    if positions is self.__roll_previous_target:
                        print(f"\n {joints} is in Finish")
                        if self.position_is_arrive(joints, positions):
                            self.__roll_status = move_status.END
                    else:
                        return
             elif self.__roll_status is move_status.END:
                    print(f"\n {joints} is in End")
                    if positions is not self.__roll_previous_target:
                        self.__roll_status = move_status.INITIAL
                    else:
                        return
             else:
                    print(f"\n {joints} is in error status!")
                    return
        else:
            print("\nCan not find any joints, please set again in set_position")
            return
        # print("End moved is", self.moved)

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
            print(f"\n {joints} is not finding!")
            return None

    def position_is_arrive(self, joints="", targets=None):
        if joints == "":
            print("\nYou do not set any joints")
            return

        if targets is None:
            print("\nYou do not set any positions")
            return

        if joints == "LElbowYaw":
            if targets is self.__yaw_previous_target:
                position = l_elbow_yaw_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                # print("\nabs is", passing)
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
                # print("\nabs is", passing)
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        else:
            print("\nCan not find any joints, please set again in position_is_arrive")
            return

    def set_position(self, joints="", positions=None):
        if joints == "":
            print("\nYou do not set any joints")
            return

        if positions is None:
            print("\nYou do not set any positions")
            return

        if joints == "LElbowYaw":
            if positions > self.__max_elbow_yaw_radian or positions < self.__min_elbow_yaw_radian:
                print(f"\nThe position you set is {positions}, is out of range in [{self.__max_elbow_yaw_radian}, {self.__min_elbow_yaw_radian}]")
                return
            # if self.__yaw_previous_target is positions and self.__yaw_status is move_status.INITIAL:
            #     return
            if self.__yaw_status is move_status.INITIAL:
                print(f"\n {joints} is in Initial")
                self.__yaw_previous_target = positions
                self.__yaw_status = move_status.MOVING
            elif self.__yaw_status is move_status.MOVING:
                if positions is self.__yaw_previous_target:
                    print(f"\n {joints} is in Moving")
                    l_elbow_yaw.setPosition(positions)
                    if self.position_is_arrive(joints, positions):
                        self.__yaw_status = move_status.FINISH
                else:
                    return
            elif self.__yaw_status is move_status.FINISH:
                if positions is self.__yaw_previous_target:
                    print(f"\n {joints} is in Finish")
                    if self.position_is_arrive(joints, positions):
                        self.__yaw_status = move_status.END
                else:
                    return
            elif self.__yaw_status is move_status.END:
                print(f"\n {joints} is in End")
                if positions is not self.__yaw_previous_target:
                    self.__yaw_status = move_status.INITIAL
                    return
                else:
                    return
            else:
                print(f"\n {joints} is in error status!")
                return
        elif joints == 'LElbowRoll':
            if positions > self.__max_elbow_roll_radian or positions < self.__min_elbow_roll_radian:
                print(f"\nThe position you set is {positions}, is out of range in [{self.__max_elbow_roll_radian}, {self.__min_elbow_roll_radian}]")
                return
            # if self.__roll_previous_target is positions and self.__roll_status is move_status.INITIAL:
            #     return
            if self.__roll_status is move_status.INITIAL:
                print(f"\n {joints} is in Initial")
                self.__roll_previous_target = positions
                self.__roll_status = move_status.PREPARE
            elif self.__roll_status is move_status.PREPARE:
                print(f"\n {joints} is in Prepare")
                self.__roll_status = move_status.MOVING
            elif self.__roll_status is move_status.MOVING:
                if positions is self.__roll_previous_target:
                    print(f"\n {joints} is in Moving")
                    l_elbow_roll.setPosition(positions)
                    if self.position_is_arrive(joints, positions):
                        self.__roll_status = move_status.FINISH
                else:
                    return
            elif self.__roll_status is move_status.FINISH:
                if positions is self.__roll_previous_target:
                    print(f"\n {joints} is in Finish")
                    if self.position_is_arrive(joints, positions):
                        self.__roll_status = move_status.END
                else:
                    return
            elif self.__roll_status is move_status.END:
                if positions is not self.__roll_previous_target:
                    print(f"\n {joints} is in End")
                    self.__roll_status = move_status.INITIAL
                    return
                else:
                    return
            else:
                print(f"\n {joints} is in error status!")
                return
        else:
            print("\nCan not find any joints, please set again in set_position")
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
            print(f"\n {joints} is not finding!")
            return None
    def position_is_arrive(self, joints="", targets=None):
        if joints == "":
            print("\nYou do not set any joints")
            return

        if targets is None:
            print("\nYou do not set any positions")
            return

        if joints == "RShoulderPitch":
            if targets is self.__pitch_previous_target:
                position = r_shoulder_pitch_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                # print("\nabs is", passing)
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
                # print("\nabs is", passing)
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        else:
            print("\nCan not find any joints, please set again in position_is_arrive")
            return

    def set_position(self, joints="", positions=None):
        if joints == "":
            print("\nYou do not set any joints")
            return

        if positions is None:
            print("\nYou do not set any positions")
            return
        # print(f"\njoints: {joints}, positions: {positions}, moved: {self.moved}")
        if joints == "RShoulderPitch":
            if positions > self.__max_shoulder_pitch_radian or positions < self.__min_shoulder_pitch_radian:
                print(f"\nThe position you set is {positions}, is out of range in [{self.__max_shoulder_pitch_radian}, {self.__min_shoulder_pitch_radian}]")
                return
            # if self.__pitch_previous_target is positions and self.__pitch_status is move_status.INITIAL:
            #     return
            if self.__pitch_status is move_status.INITIAL:
                print(f"\n {joints} is in Initial")
                self.__pitch_previous_target = positions
                self.__pitch_status = move_status.PREPARE
            elif self.__pitch_status is move_status.PREPARE:
                if positions is self.__pitch_previous_target:
                    print(f"\n {joints} is in Prepare")
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
                    print(f"\n {joints} is in Moving")
                    r_shoulder_pitch.setPosition(positions)
                    if self.position_is_arrive(joints, positions):
                        self.__pitch_status = move_status.FINISH
                else:
                    return
            elif self.__pitch_status is move_status.FINISH:
                if positions is self.__pitch_previous_target:
                    print(f"\n {joints} is in Finish")
                    if self.position_is_arrive(joints, positions):
                        self.__pitch_status = move_status.END
                else:
                    return
            elif self.__pitch_status is move_status.END:
                print(f"\n {joints} is in End")
                if positions is not self.__pitch_previous_target:
                    self.__pitch_status = move_status.INITIAL
                else:
                    return
            else:
                print(f"\n {joints} is in error status!")
                return
        elif joints == "RShoulderRoll":
             if positions > self.__max_shoulder_roll_radian or positions < self.__min_shoulder_roll_radian:
                    print(f"\nThe position you set is {positions}, is out of range in [{self.__max_shoulder_roll_radian}, {self.__min_shoulder_roll_radian}]")
                    return
             # if self.__roll_previous_target is positions and self.__roll_status is move_status.INITIAL:
             #    return
             if self.__roll_status is move_status.INITIAL:
                    print(f"\n {joints} is in Initial")
                    self.__roll_previous_target = positions
                    self.__roll_status = move_status.PREPARE
             elif self.__roll_status is move_status.PREPARE:
                 if positions is self.__roll_previous_target:
                    print(f"\n {joints} is in Prepare")
                    self.__roll_status = move_status.MOVING
                 else:
                     return
             elif self.__roll_status is move_status.MOVING:
                    if positions is self.__roll_previous_target:
                        print(f"\n {joints} is in Moving")
                        r_shoulder_roll.setPosition(positions)
                        if self.position_is_arrive(joints, positions):
                            self.__roll_status = move_status.FINISH
                    else:
                        return
             elif self.__roll_status is move_status.FINISH:
                    if positions is self.__roll_previous_target:
                        print(f"\n {joints} is in Finish")
                        if self.position_is_arrive(joints, positions):
                            self.__roll_status = move_status.END
                    else:
                        return
             elif self.__roll_status is move_status.END:
                    print(f"\n {joints} is in End")
                    if positions is not self.__roll_previous_target:
                        self.__roll_status = move_status.INITIAL
                    else:
                        return
             else:
                    print(f"\n {joints} is in error status!")
                    return
        else:
            print("\nCan not find any joints, please set again in set_position")
            return
        # print("End moved is", self.moved)
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
            print(f"\n {joints} is not finding!")
            return None

    def position_is_arrive(self, joints="", targets=None):
        if joints == "":
            print("\nYou do not set any joints")
            return

        if targets is None:
            print("\nYou do not set any positions")
            return

        if joints == "RElbowYaw":
            if targets is self.__yaw_previous_target:
                position = r_elbow_yaw_sensor.getValue()
                passing = abs(abs(targets) - abs(position))
                # print("\nabs is", passing)
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
                # print("\nabs is", passing)
                if passing <= self.__limitation :
                    # self.moved = False
                    return True
                else:
                    return False
            else:
                return True
        else:
            print("\nCan not find any joints, please set again in position_is_arrive")
            return

    def set_position(self, joints="", positions=None):
        if joints == "":
            print("\nYou do not set any joints")
            return

        if positions is None:
            print("\nYou do not set any positions")
            return

        if joints == "RElbowYaw":
            if positions > self.__max_elbow_yaw_radian or positions < self.__min_elbow_yaw_radian:
                print(f"\nThe position you set is {positions}, is out of range in [{self.__max_elbow_yaw_radian}, {self.__min_elbow_yaw_radian}]")
                return
            # if self.__yaw_previous_target is positions and self.__yaw_status is move_status.INITIAL:
            #     return
            if self.__yaw_status is move_status.INITIAL:
                print(f"\n {joints} is in Initial")
                self.__yaw_previous_target = positions
                self.__yaw_status = move_status.MOVING
            elif self.__yaw_status is move_status.MOVING:
                if positions is self.__yaw_previous_target:
                    print(f"\n {joints} is in Moving")
                    r_elbow_yaw.setPosition(positions)
                    if self.position_is_arrive(joints, positions):
                        self.__yaw_status = move_status.FINISH
                else:
                    return
            elif self.__yaw_status is move_status.FINISH:
                if positions is self.__yaw_previous_target:
                    print(f"\n {joints} is in Finish")
                    if self.position_is_arrive(joints, positions):
                        self.__yaw_status = move_status.END
                else:
                    return
            elif self.__yaw_status is move_status.END:
                print(f"\n {joints} is in End")
                if positions is not self.__yaw_previous_target:
                    self.__yaw_status = move_status.INITIAL
                    return
                else:
                    return
            else:
                print(f"\n {joints} is in error status!")
                return
        elif joints == 'RElbowRoll':
            if positions > self.__max_elbow_roll_radian or positions < self.__min_elbow_roll_radian:
                print(f"\nThe position you set is {positions}, is out of range in [{self.__max_elbow_roll_radian}, {self.__min_elbow_roll_radian}]")
                return
            # if self.__roll_previous_target is positions and self.__roll_status is move_status.INITIAL:
            #     return
            if self.__roll_status is move_status.INITIAL:
                print(f"\n {joints} is in Initial")
                self.__roll_previous_target = positions
                self.__roll_status = move_status.PREPARE
            elif self.__roll_status is move_status.PREPARE:
                print(f"\n {joints} is in Prepare")
                self.__roll_status = move_status.MOVING
            elif self.__roll_status is move_status.MOVING:
                if positions is self.__roll_previous_target:
                    print(f"\n {joints} is in Moving")
                    r_elbow_roll.setPosition(positions)
                    if self.position_is_arrive(joints, positions):
                        self.__roll_status = move_status.FINISH
                else:
                    return
            elif self.__roll_status is move_status.FINISH:
                if positions is self.__roll_previous_target:
                    print(f"\n {joints} is in Finish")
                    if self.position_is_arrive(joints, positions):
                        self.__roll_status = move_status.END
                else:
                    return
            elif self.__roll_status is move_status.END:
                if positions is not self.__roll_previous_target:
                    print(f"\n {joints} is in End")
                    self.__roll_status = move_status.INITIAL
                    return
                else:
                    return
            else:
                print(f"\n {joints} is in error status!")
                return
        else:
            print("\nCan not find any joints, please set again in set_position")
            return

class NAO_MOTION:
    def __init__(self):
        print("Init NAO_MOTION!")

    __l_shoulder = left_shoulder()
    __l_elbow = left_elbow()
    __r_shoulder = right_shoulder()
    __r_elbow = right_elbow()

    __salute_status = SALUTE_MOTION.INITIAL

    def salute_motion(self):
        if self.__salute_status is SALUTE_MOTION.INITIAL:
            print("\nIn SALUTE_MOTION.INITIAL")
            self.__salute_status = SALUTE_MOTION.PREPARE
        elif self.__salute_status is SALUTE_MOTION.PREPARE:
            print("\nIn SALUTE_MOTION.PREPARE")
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
            print("\nIn SALUTE_MOTION.PREPARE")
            self.__l_shoulder.set_position("LShoulderPitch", -1)
            self.__l_shoulder.set_position("LShoulderRoll", 1)
            if (self.__l_shoulder.getJointsStatus('LShoulderPitch') is move_status.END and
                self.__l_shoulder.getJointsStatus('LShoulderRoll') is move_status.END):
                self.__salute_status = SALUTE_MOTION.SALUTE
            else:
                return
        elif self.__salute_status is SALUTE_MOTION.SALUTE:
            print("\nIn SALUTE_MOTION.SALUTE")
            self.__l_elbow.set_position('LElbowRoll', -1.5)
            if self.__l_elbow.getJointsStatus('LElbowRoll') is move_status.END:
                self.__salute_status = SALUTE_MOTION.RESET
            else:
                return
        elif self.__salute_status is SALUTE_MOTION.RESET:
            print("\nIn SALUTE_MOTION.RESET")
            self.__l_elbow.set_position('LElbowRoll', -0.035)
            if self.__l_elbow.getJointsStatus('LElbowRoll') is move_status.END:
                self.__salute_status = SALUTE_MOTION.FINISH
            else:
                return
        elif self.__salute_status is SALUTE_MOTION.FINISH:
            print("\nIn SALUTE_MOTION.FINISH")
            self.__salute_status = SALUTE_MOTION.END
        elif self.__salute_status is SALUTE_MOTION.END:
            print("\nIn SALUTE_MOTION.END")
            self.__salute_status = SALUTE_MOTION.INITIAL
        else:
            print("\nsalute motion in error status")
            return

nao_motion = NAO_MOTION()

while robot.step(timestep) != -1:
    nao_motion.salute_motion()
    pass
# Enter here exit cleanup code.
