from controller import Robot, Keyboard
import numpy as np

class NaoPowerKick:
    def __init__(self):
        self.robot = Robot()
        self.timeStep = int(self.robot.getBasicTimeStep())
        
        # 等待一段時間讓鍵盤初始化
        for _ in range(10):
            self.robot.step(self.timeStep)
            
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timeStep)
        print("Keyboard enabled. Press P to start kick sequence.", flush=True)
        
        self.motors = {}
        self.sensors = {}
        self.motor_names = [
            'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
            'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll',
            'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
            'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'
        ]
        
        # 初始化馬達和感測器
        for name in self.motor_names:
            self.motors[name] = self.robot.getDevice(name)
            if 'Shoulder' in name or 'Elbow' in name:
                self.motors[name].setVelocity(0.7)
            else:
                self.motors[name].setVelocity(0.6)
                
            sensor = self.robot.getDevice(name + 'S')
            if sensor:
                sensor.enable(self.timeStep)
                self.sensors[name] = sensor
        
        self.gyro = self.robot.getDevice('gyro')
        self.gyro.enable(self.timeStep)
        self.accelerometer = self.robot.getDevice('accelerometer')
        self.accelerometer.enable(self.timeStep)
        
        self.is_kicking = False
        self.kick_stage = 0
        self.shift_counter = 0
        self.stability_counter = 0
        
        # 調整PID參數
        self.pid = {
            'kp': 0.4,    # 增加比例增益
            'ki': 0.05,   # 保持積分增益
            'kd': 0.15,   # 增加微分增益
            'prev_error': 0,
            'integral': 0
        }

    def get_motor_position(self, name):
        """安全地獲取馬達位置"""
        if name in self.sensors:
            return self.sensors[name].getValue()
        return 0.0

    def get_com_position(self):
        """計算質心位置"""
        accel_values = self.accelerometer.getValues()
        x_pos = accel_values[0]
        y_pos = accel_values[1]
        print(f"Accelerometer values: x={x_pos:.3f}, y={y_pos:.3f}", flush=True)
        return x_pos, y_pos

    def prepare_kick(self):
        """設定初始位置，所有關節同時平順運動"""
        if not hasattr(self, 'prep_counter'):
            self.prep_counter = 0
            self.start_positions = {}
            for name in self.motor_names:
                self.start_positions[name] = self.get_motor_position(name)
            
        transition_time = 100
        progress = min(1.0, self.prep_counter / transition_time)
        smooth_progress = (1 - np.cos(progress * np.pi)) / 2
        
        target_positions = {
            'LHipYawPitch': 0.0,
            'LHipRoll': 0.1,
            'LHipPitch': -0.4,
            'LKneePitch': 0.7,
            'LAnklePitch': -0.3,
            'LAnkleRoll': -0.1,
            'RHipYawPitch': 0.0,
            'RHipRoll': -0.1,
            'RHipPitch': -0.4,
            'RKneePitch': 0.7,
            'RAnklePitch': -0.3,
            'RAnkleRoll': 0.1,
            'LShoulderPitch': 1.57,
            'LShoulderRoll': 0.3,
            'LElbowYaw': -1.0,
            'LElbowRoll': -0.5,
            'RShoulderPitch': 1.57,
            'RShoulderRoll': -0.3,
            'RElbowYaw': 1.0,
            'RElbowRoll': 0.5
        }
        
        for name, target_pos in target_positions.items():
            if name in self.motors:
                start_pos = self.start_positions[name]
                current_pos = start_pos + (target_pos - start_pos) * smooth_progress
                self.motors[name].setPosition(current_pos)
        
        self.prep_counter += 1
        if self.prep_counter > transition_time:
            self.prep_counter = 0
            return True
        return False

    def shift_weight_to_left(self):
        
        if not hasattr(self, 'shift_start_time'):
            self.shift_start_time = 0
            print("Starting weight shift", flush=True)
            
            # 記錄所有關節的初始位置
            self.initial_positions = {}
            
            for name in self.motor_names:
                if name in self.sensors:
                    self.initial_positions[name] = self.sensors[name].getValue()
                else:
                    self.initial_positions[name] = 0.0
                print(f"Initial position for {name}: {self.initial_positions[name]}", flush=True)
                
            self.pid['integral'] = 0
            self.pid['prev_error'] = 0
        
        self.shift_counter += 1
        max_shift = 0.7  # 最大移動範圍
        shift_progress = min(1.0, self.shift_counter / 100.0)
        current_shift = max_shift * shift_progress
        print(f"Shift progress: {shift_progress:.2%}, current_shift: {current_shift:.3f}", flush=True)
        
        # 修改腳部動作，從各自的初始位置開始
        # HipRoll
        hip_roll_change = -current_shift  # 目標變化量
        left_hip_roll_target = self.initial_positions['LHipRoll'] + (hip_roll_change *1.2)
        right_hip_roll_target = self.initial_positions['RHipRoll'] + hip_roll_change
        self.motors['LHipRoll'].setPosition(left_hip_roll_target)
        self.motors['RHipRoll'].setPosition(right_hip_roll_target)
        
        # AnkleRoll
        ankle_roll_change = -current_shift * 0.7
        left_ankle_roll_target = self.initial_positions['LAnkleRoll'] + (-ankle_roll_change )
        right_ankle_roll_target = self.initial_positions['RAnkleRoll'] + ankle_roll_change
        right_hip_roll_target = self.initial_positions['RHipRoll'] + ankle_roll_change
        self.motors['LAnkleRoll'].setPosition(left_ankle_roll_target)
        self.motors['RAnkleRoll'].setPosition(right_ankle_roll_target)
        #self.motors['RHipRoll'].setPosition(right_hip_roll_target*1.5)
        
        # ShoulderRoll
        shoulder_roll_change = -current_shift * 0.8
        left_shoulder_roll_target = self.initial_positions['LShoulderRoll'] + ((shoulder_roll_change)*1.5)
        right_shoulder_roll_target = self.initial_positions['RShoulderRoll'] + shoulder_roll_change
        self.motors['LShoulderRoll'].setPosition(-left_shoulder_roll_target)
        self.motors['RShoulderRoll'].setPosition(right_shoulder_roll_target)
        
        # Pitch adjustments
        pitch_change = 0.05 * shift_progress
        
        # AnklePitch
        left_ankle_pitch_target = self.initial_positions['LAnklePitch'] + pitch_change
        right_ankle_pitch_target = self.initial_positions['RAnklePitch'] + pitch_change
        self.motors['LAnklePitch'].setPosition(left_ankle_pitch_target)
        self.motors['RAnklePitch'].setPosition(right_ankle_pitch_target)
        
        # HipPitch
        left_hip_pitch_target = self.initial_positions['LHipPitch'] - pitch_change
        right_hip_pitch_target = self.initial_positions['RHipPitch'] - (pitch_change * 0.3)
        self.motors['LHipPitch'].setPosition(left_hip_pitch_target)
        self.motors['RHipPitch'].setPosition(right_hip_pitch_target)
        
        # Debug output
        print(f"Current positions - LHipRoll: {left_hip_roll_target:.3f}, RHipRoll: {right_hip_roll_target:.3f}", flush=True)
        
        # 取得目前的質心位置
        com_x, com_y = self.get_com_position()
        print(f"COM position: x={com_x:.3f}, y={com_y:.3f}", flush=True)
        
        target_x = -0.3
        target_y = 0.0
        
        error_x = target_x - com_x
        error_y = target_y - com_y
        total_error = np.sqrt(error_x**2 + error_y**2)
        print(f"Total error: {total_error:.3f}", flush=True)
        
        self.pid['integral'] = np.clip(self.pid['integral'] + total_error, -1.0, 1.0)
        derivative = total_error - self.pid['prev_error']
        
        control_signal = (
            0.3 * error_x +
            self.pid['kp'] * total_error +
            self.pid['ki'] * self.pid['integral'] +
            self.pid['kd'] * derivative
        )
        
        self.pid['prev_error'] = total_error
        
        is_stable = abs(error_x) < 0.13 and abs(error_y) < 0.1
        if is_stable:
            self.stability_counter += 1
            print(f"Stability counter: {self.stability_counter}", flush=True)
        else:
            self.stability_counter = 0
        
        if self.stability_counter > 30:
            print("COM centered on left foot!", flush=True)
            self.shift_counter = 0
            self.stability_counter = 0
            return True
        
        if self.shift_counter > 150:
            print("Weight shift timeout - evaluating stability")
            if abs(com_x) < 0.15:
                return True
            self.shift_counter = 0
        
        return False

    def execute_kick(self):
        """執行踢球動作"""
        if not hasattr(self, 'kick_counter'):
            self.kick_counter = 0
            self.motors['RHipPitch'].setVelocity(2.0)
            self.motors['RKneePitch'].setVelocity(3.0)
            self.motors['RAnklePitch'].setVelocity(2.0)
            self.motors['LShoulderPitch'].setVelocity(1.5)
            self.motors['RHipPitch'].setPosition(0.4)
            self.motors['RKneePitch'].setPosition(0)
            self.motors['RAnklePitch'].setPosition(0)
            self.motors['LHipYawPitch'].setPosition(-1)
            self.motors['RKneePitch'].setPosition(1.7)
            self.motors['RHipYawPitch'].setPosition(0.3)
            self.motors['RHipRoll'].setPosition(-10)
            self.motors['RAnkleRoll'].setPosition(0)
            self.motors['LShoulderPitch'].setPosition(-2.0)

        self.kick_counter += 1
        if self.kick_counter > 40:
            self.kick_counter = 0
            return True
        return False
        
    def front_kick(self):
        """定義前踢動作"""
        if not hasattr(self, 'front_kick_phase'):
            self.front_kick_phase = 0
            self.front_kick_counter = 0
            # 設定較高的速度以實現快速踢球
            self.motors['RHipPitch'].setVelocity(10.0)
            self.motors['LHipYawPitch'].setVelocity(4.0)
            self.motors['RKneePitch'].setVelocity(1.5)
            self.motors['RAnklePitch'].setVelocity(12)
            
        self.front_kick_counter += 1
        
        if self.front_kick_phase == 0:  # 快速前踢
            # 髖關節向前甩動
            self.motors['RHipPitch'].setPosition(-1.2)
            # 膝蓋快速伸直
            self.motors['RKneePitch'].setPosition(0.5)
            # 腳踝配合動作
            self.motors['RAnklePitch'].setPosition(-0.5)
            self.motors['LHipYawPitch'].setPosition(3.0)
            self.motors['LShoulderPitch'].setPosition(0)
            
            if self.front_kick_counter >= 50:
                self.front_kick_phase = 1
                self.front_kick_counter = 0
                
        elif self.front_kick_phase == 1:  # 收回腿回到初始姿勢
            print('return')
            # #降低速度以平穩回歸
            target_positions = {
            
                'LHipYawPitch': 0.0,
                'LHipRoll': 0.1,
                'LHipPitch': -1.0,
                'LKneePitch': 0.7,
                'LAnklePitch': -0.3,
                'LAnkleRoll': -0.1,
                'RHipYawPitch': 0.0,
                'RHipRoll': -0.1,
                'RHipPitch': -1.0,
                'RKneePitch': 1.1,
                'RAnklePitch': -0.3,
                'RAnkleRoll': 0.1,
                'LShoulderPitch': 1.57,
                'LShoulderRoll': 0.3,
                'LElbowYaw': -1.0,
                'LElbowRoll': -0.5,
                'RShoulderPitch': 1.57,
                'RShoulderRoll': -0.3,
                'RElbowYaw': 1.0,
                'RElbowRoll': 0.5
            }
        
            velocities = {
                # 腿部關節更慢
                'LHipYawPitch': 0.2,
                'LHipRoll': 0.5,
                'LHipPitch': 1.0,
                'LKneePitch': 0.7,
                'LAnklePitch': 0.5,
                'LAnkleRoll': 0.5,
                'RHipYawPitch': 0.5,
                'RHipRoll': 0.5,
                'RHipPitch': 1.5,
                'RKneePitch': 1.5,
                'RAnklePitch': 0.5,
                'RAnkleRoll': 0.5,
                # 手臂關節可以稍快
                'LShoulderPitch': 0.8,
                'LShoulderRoll': 0.8,
                'LElbowYaw': 0.8,
                'LElbowRoll': 0.8,
                'RShoulderPitch': 0.8,
                'RShoulderRoll': 0.8,
                'RElbowYaw': 0.8,
                'RElbowRoll': 0.8
            }
        
            # 設定速度
            for name, velocity in velocities.items():
                if name in self.motors:
                    self.motors[name].setVelocity(velocity)
        
            # 設定目標位置
            for name, target_pos in target_positions.items():
                if name in self.motors:
                    self.motors[name].setPosition(target_pos)
                    
            #如果已經到達位置，進入下一階段
            if self.front_kick_counter >= 50:
                self.front_kick_phase = 2
                self.front_kick_counter = 0
            
        elif self.front_kick_phase == 2:  # 調整平衡
            # #微調其他關節以保持平衡
            self.motors['RKneePitch'].setVelocity(2.0)
            
            self.motors['LHipPitch'].setVelocity(0.85)
            self.motors['RHipPitch'].setVelocity(0.85)
            
            self.motors['RKneePitch'].setPosition(0.7)
            
            self.motors['LHipPitch'].setPosition(-0.6)
            self.motors['RHipPitch'].setPosition(-0.6)

            
            if self.front_kick_counter >= 20:
                
                self.front_kick_phase = 0
                self.front_kick_counter = 0
                delattr(self, 'front_kick_phase')
                return True
                
        return False
            
    def handle_keyboard(self):
        key = self.keyboard.getKey()
        if key != -1:
            print(f"Key pressed: {key}", flush=True)
            if key == ord('P') or key == ord('p'):
                if not self.is_kicking:
                    print("Starting power kick sequence...", flush=True)
                    self.is_kicking = True
                    self.kick_stage = 1
                    return True
        return False
    
    def update(self):
        key_pressed = self.handle_keyboard()
        
        if key_pressed or self.is_kicking:
            if self.kick_stage == 1:
                if self.prepare_kick():
                    print("Initial position set.", flush=True)
                    self.kick_stage = 2
            elif self.kick_stage == 2:
                print('Stage 2: Weight shifting', flush=True)
                if self.shift_weight_to_left():
                    print("Weight shifted successfully.", flush=True)
                    self.kick_stage = 3
            elif self.kick_stage == 3:
                print('Stage 3: Executing kick', flush=True)
                if self.execute_kick():
                    print("Kick executed.", flush=True)
                    self.kick_stage = 4
            elif self.kick_stage == 4:
                print('Stage 4: Performing front kick', flush=True)
                if self.front_kick():
                    print("Front kick executed.", flush=True)
                    self.kick_stage = 5  # 完成所有動作
    
        return True

def main():
    controller = NaoPowerKick()
    print("Controller initialized. Press P to start.", flush=True)
    while controller.robot.step(controller.timeStep) != -1:
        try:
            if not controller.update():
                break
        except Exception as e:
            print(f"Error during execution: {e}", flush=True)
            break

if __name__ == "__main__":
    main()