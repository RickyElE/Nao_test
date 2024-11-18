from controller import Supervisor
import numpy as np

class LIPM3D:
    def __init__(self,
                 dt=0.001,
                 T_sup=1.0,
                 support_leg='left_leg'):
        self.dt = dt
        self.t = 0
        self.T_sup = T_sup # support time

        self.p_x = 0  # desired foot location x
        self.p_y = 0  # desired foot location y

        self.p_x_star = 0 # modified foot location x
        self.p_y_star = 0 # modified foot location y

        # Initialize the gait parameters
        self.s_x = 0.0
        self.s_y = 0.0

        # COM initial state
        self.x_0 = 0
        self.vx_0 = 0
        self.y_0 = 0
        self.vy_0 = 0

        # COM real-time state
        self.x_t = 0
        self.vx_t = 0
        self.y_t = 0
        self.vy_t = 0

        # COM desired state
        self.x_d = 0
        self.vx_d = 0
        self.y_d = 0
        self.vy_d = 0

        # final state for one gait unit
        self.bar_x = 0.0
        self.bar_y = 0.0
        self.bar_vx = 0.0
        self.bar_vy = 0.0

        self.support_leg = support_leg
        self.left_foot_pos = [0.0, 0.0, 0.0]
        self.right_foot_pos = [0.0, 0.0, 0.0]
        self.COM_pos = [0.0, 0.0, 0.0]

    def initializeModel(self, COM_pos, left_foot_pos, right_foot_pos):
        self.COM_pos = COM_pos
        self.left_foot_pos = left_foot_pos
        self.right_foot_pos = right_foot_pos

        self.zc = self.COM_pos[2]
        self.T_c = np.sqrt(self.zc / 9.81)  # set gravity parameter as 9.81
        self.C = np.cosh(self.T_sup / self.T_c)
        self.S = np.sinh(self.T_sup / self.T_c)

    def updateParameters(self, T_sup):
        self.T_sup = T_sup
        self.C = np.cosh(self.T_sup / self.T_c)
        self.S = np.sinh(self.T_sup / self.T_c)

    def step(self):
        self.t += self.dt
        t = self.t
        T_c = self.T_c

        self.x_t = self.x_0 * np.cosh(t / T_c) + T_c * self.vx_0 * np.sinh(t / T_c)
        self.vx_t = self.x_0 / T_c * np.sinh(t / T_c) + self.vx_0 * np.cosh(t / T_c)

        self.y_t = self.y_0 * np.cosh(t / T_c) + T_c * self.vy_0 * np.sinh(t / T_c)
        self.vy_t = self.y_0 / T_c * np.sinh(t / T_c) + self.vy_0 * np.cosh(t / T_c)

    def calculateXtVt(self, t):
        T_c = self.T_c

        x_t = self.x_0 * np.cosh(t / T_c) + T_c * self.vx_0 * np.sinh(t / T_c)
        vx_t = self.x_0 / T_c * np.sinh(t / T_c) + self.vx_0 * np.cosh(t / T_c)

        y_t = self.y_0 * np.cosh(t / T_c) + T_c * self.vy_0 * np.sinh(t / T_c)
        vy_t = self.y_0 / T_c * np.sinh(t / T_c) + self.vy_0 * np.cosh(t / T_c)

        return x_t, vx_t, y_t, vy_t

    def nextReferenceFootLocation(self, s_x, s_y, theta=0):
        if self.support_leg is 'left_leg':  # then the next support leg is the right leg
            p_x_new = self.p_x + np.cos(theta) * s_x - np.sin(theta) * s_y
            p_y_new = self.p_y + np.sin(theta) * s_x + np.cos(theta) * s_y
        elif self.support_leg is 'right_leg':  # then the next support leg is the left leg
            p_x_new = self.p_x + np.cos(theta) * s_x + np.sin(theta) * s_y
            p_y_new = self.p_y + np.sin(theta) * s_x - np.cos(theta) * s_y

        return p_x_new, p_y_new

    def nextState(self, s_x, s_y, theta=0):
        '''
        Calculate next final state at T_sup
        '''
        if self.support_leg is 'left_leg':
            bar_x_new = np.cos(theta) * s_x / 2.0 - np.sin(theta) * s_y / 2.0
            bar_y_new = np.sin(theta) * s_x / 2.0 + np.cos(theta) * s_y / 2.0
        elif self.support_leg is 'right_leg':
            bar_x_new = np.cos(theta) * s_x / 2.0 + np.sin(theta) * s_y / 2.0
            bar_y_new = np.sin(theta) * s_x / 2.0 - np.cos(theta) * s_y / 2.0
        return bar_x_new, bar_y_new

    def nextVel(self, bar_x=0, bar_y=0, theta=0):
        C = self.C
        S = self.S
        T_c = self.T_c

        bar_vx_new = np.cos(theta) * (1 + C) / (T_c * S) * bar_x - np.sin(theta) * (C - 1) / (T_c * S) * bar_y
        bar_vy_new = np.sin(theta) * (1 + C) / (T_c * S) * bar_x + np.cos(theta) * (C - 1) / (T_c * S) * bar_y

        return bar_vx_new, bar_vy_new

    def targetState(self, p_x, bar_x, bar_vx):
        x_d = p_x + bar_x
        vx_d = bar_vx

        return x_d, vx_d

    def modifiedFootLocation(self, a=1.0, b=1.0, x_d=0, vx_d=0, x_0=0, vx_0=0):
        C = self.C
        S = self.S
        T_c = self.T_c
        D = a * (C - 1) ** 2 + b * (S / T_c) ** 2

        p_x_star = -a * (C - 1) * (x_d - C * x_0 - T_c * S * vx_0) / D - b * S * (vx_d - S * x_0 / T_c - C * vx_0) / (
                    T_c * D)

        return p_x_star

    def calculateFootLocationForNextStep(self, s_x=0.0, s_y=0.0, a=1.0, b=1.0, theta=0.0, x_0=0.0, vx_0=0.0, y_0=0.0,
                                         vy_0=0.0):
        self.s_x = s_x
        self.s_y = s_y

        # ----------------------------- calculate desired COM states and foot locations for the given s_x, s_y and theta
        # calculate desired foot locations
        print(self.p_x, self.p_y)
        p_x_new, p_y_new = self.nextReferenceFootLocation(s_x, s_y, theta)
        # print('-- p_x_new=%.3f'%p_x_new, ', p_y_new=%.3f'%p_y_new)

        # calculate desired COM states
        bar_x, bar_y = self.nextState(s_x, s_y, theta)
        bar_vx, bar_vy = self.nextVel(bar_x, bar_y, theta)
        # print('-- bar_x=%.3f'%bar_x, ', bar_y=%.3f'%bar_y)
        # print('-- bar_vx=%.3f'%bar_vx, ', bar_vy=%.3f'%bar_vy)

        # calculate target COM state in the next step
        self.x_d, self.vx_d = self.targetState(p_x_new, bar_x, bar_vx)
        self.y_d, self.vy_d = self.targetState(p_y_new, bar_y, bar_vy)
        # print('-- x_d=%.3f'%self.x_d, ', vx_d=%.3f'%self.vx_d)
        # print('-- y_d=%.3f'%self.y_d, ', vy_d=%.3f'%self.vy_d)

        # ----------------------------- calculate modified foot locations based on the current actual COM states
        # correct the modified foot locations to minimize the errors
        self.p_x_star = self.modifiedFootLocation(a, b, self.x_d, self.vx_d, x_0, vx_0)
        self.p_y_star = self.modifiedFootLocation(a, b, self.y_d, self.vy_d, y_0, vy_0)
        # print('-- p_x_star=%.3f'%self.p_x_star, ', p_y_star=%.3f'%self.p_y_star)

    def switchSupportLeg(self):
        if self.support_leg is 'left_leg':
            print('\n---- switch the support leg to the right leg')
            self.support_leg = 'right_leg'
            COM_pos_x = self.x_t + self.left_foot_pos[0]
            COM_pos_y = self.y_t + self.left_foot_pos[1]
            self.x_0 = COM_pos_x - self.right_foot_pos[0]
            self.y_0 = COM_pos_y - self.right_foot_pos[1]
        elif self.support_leg is 'right_leg':
            print('\n---- switch the support leg to the left leg')
            self.support_leg = 'left_leg'
            COM_pos_x = self.x_t + self.right_foot_pos[0]
            COM_pos_y = self.y_t + self.right_foot_pos[1]
            self.x_0 = COM_pos_x - self.left_foot_pos[0]
            self.y_0 = COM_pos_y - self.left_foot_pos[1]

        self.t = 0
        self.vx_0 = self.vx_t
        self.vy_0 = self.vy_t

# 初始化 Supervisor
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
emitter = supervisor.getDevice("emitter")

# 获取根节点
root = supervisor.getRoot()
children = root.getField("children")

# 遍历所有子节点，查找 NAO 机器人
nao = None
for i in range(children.getCount()):
    node = children.getMFNode(i)
    if node.getTypeName() == "Nao":  # 检查是否是 NAO 类型
        nao = node
        break

# 获取质心
if nao:
    COM_pos_x = list()
    COM_pos_y = list()
    left_foot_pos_x = list()
    left_foot_pos_y = list()
    left_foot_pos_z = list()
    right_foot_pos_x = list()
    right_foot_pos_y = list()
    right_foot_pos_z = list()


    # COM_pos_0 = [0.00227531, -1.06243e-09, -0.0559439]
    # COM_v0 = nao.getVelocity()[:3]
    #
    # left_foot_pos = [0.0285024, 0.05332, -0.317548]
    # right_foot_pos = [0.0285024, -0.0533201, -0.317548]

    # COM_pos_0 = [-0.4, 0.2, 1.0]
    # COM_v0 = [1.0, -0.1]
    #
    # left_foot_pos = [-0.2, 0.3, 0]
    # right_foot_pos = [0.2, -0.3, 0]

    COM_pos_0 = [0.0256073, 0.000179269, 0.275234]
    COM_v0 = nao.getVelocity()[:3]
    nao_position_world = nao.getPosition()
    print("position is: ", nao_position_world)
    nao_rotation_world = nao.getOrientation()
    print(nao.getOrientation())
    rotation_matrix_b = np.array([
        [nao_rotation_world[0], nao_rotation_world[1], nao_rotation_world[2]],
        [nao_rotation_world[3], nao_rotation_world[4], nao_rotation_world[5]],
        [nao_rotation_world[6], nao_rotation_world[7], nao_rotation_world[8]]
    ])
    print(rotation_matrix_b)
    nao_position_world_np = np.array(nao_position_world)

    # relative
    # left_foot_pos = [0.0285024, 0.05332, -0.317548]
    # right_foot_pos = [0.0285024, -0.0533201, -0.317548]

    left_foot_pos = [0.0274684, 0.0534722, 0.0124141]
    right_foot_pos = [0.0231228, -0.0530794, 0.0124121]


    delta_t = 0.02

    s_x = 0.5
    s_y = 0.4
    a = 1.0
    b = 1.0
    theta = 0.0

    LIPM_model = LIPM3D(dt=delta_t, T_sup=0.5)
    LIPM_model.initializeModel(COM_pos_0, left_foot_pos, right_foot_pos)

    LIPM_model.support_leg = 'left_leg'  # set the support leg to right leg in next step
    if LIPM_model.support_leg is 'left_leg':
        support_foot_pos = LIPM_model.left_foot_pos
        LIPM_model.p_x = LIPM_model.left_foot_pos[0]
        LIPM_model.p_y = LIPM_model.left_foot_pos[1]
    else:
        support_foot_pos = LIPM_model.right_foot_pos
        LIPM_model.p_x = LIPM_model.right_foot_pos[0]
        LIPM_model.p_y = LIPM_model.right_foot_pos[1]

    LIPM_model.x_0 = LIPM_model.COM_pos[0] - support_foot_pos[0]
    LIPM_model.y_0 = LIPM_model.COM_pos[1] - support_foot_pos[1]
    LIPM_model.vx_0 = COM_v0[0]
    LIPM_model.vy_0 = COM_v0[1]

    step_num = 0
    total_time = 30  # seconds
    global_time = 0

    swing_data_len = int(LIPM_model.T_sup / delta_t)
    swing_foot_pos = np.zeros((swing_data_len, 3))
    j = 0

    switch_index = swing_data_len

    for i in range(int(total_time / delta_t)):
        global_time += delta_t

        LIPM_model.step()

        if step_num >= 1:
            if LIPM_model.support_leg is 'left_leg':
                LIPM_model.right_foot_pos = [swing_foot_pos[j, 0], swing_foot_pos[j, 1], swing_foot_pos[j, 2]]
            else:
                LIPM_model.left_foot_pos = [swing_foot_pos[j, 0], swing_foot_pos[j, 1], swing_foot_pos[j, 2]]
            j += 1

        # record data
        COM_pos_x.append(LIPM_model.x_t + support_foot_pos[0])
        COM_pos_y.append(LIPM_model.y_t + support_foot_pos[1])
        left_foot_pos_x.append(LIPM_model.left_foot_pos[0])
        left_foot_pos_y.append(LIPM_model.left_foot_pos[1])
        left_foot_pos_z.append(LIPM_model.left_foot_pos[2])
        right_foot_pos_x.append(LIPM_model.right_foot_pos[0])
        right_foot_pos_y.append(LIPM_model.right_foot_pos[1])
        right_foot_pos_z.append(LIPM_model.right_foot_pos[2])

        # switch the support leg
        if (i > 0) and (i % switch_index == 0):
            j = 0

            LIPM_model.switchSupportLeg()  # switch the support leg
            step_num += 1

            # theta -= 0.04 # set zero for walking forward, set non-zero for turn left and right

            if step_num >= 5:  # stop forward after 5 steps
                s_x = 0.0

            if step_num >= 10:
                s_y = 0.0

            if LIPM_model.support_leg is 'left_leg':
                support_foot_pos = LIPM_model.left_foot_pos
                LIPM_model.p_x = LIPM_model.left_foot_pos[0]
                LIPM_model.p_y = LIPM_model.left_foot_pos[1]
            else:
                support_foot_pos = LIPM_model.right_foot_pos
                LIPM_model.p_x = LIPM_model.right_foot_pos[0]
                LIPM_model.p_y = LIPM_model.right_foot_pos[1]

            # calculate the next foot locations, with modification, stable
            x_0, vx_0, y_0, vy_0 = LIPM_model.calculateXtVt(
                LIPM_model.T_sup)  # calculate the xt and yt as the initial state for next step

            if LIPM_model.support_leg is 'left_leg':
                x_0 = x_0 + LIPM_model.left_foot_pos[0]  # need the absolute position for next step
                y_0 = y_0 + LIPM_model.left_foot_pos[1]  # need the absolute position for next step
            else:
                x_0 = x_0 + LIPM_model.right_foot_pos[0]  # need the absolute position for next step
                y_0 = y_0 + LIPM_model.right_foot_pos[1]  # need the absolute position for next step

            LIPM_model.calculateFootLocationForNextStep(s_x, s_y, a, b, theta, x_0, vx_0, y_0, vy_0)
            # print('p_star=', LIPM_model.p_x_star, LIPM_model.p_y_star)

            # calculate the foot positions for swing phase
            if LIPM_model.support_leg is 'left_leg':
                right_foot_target_pos = [LIPM_model.p_x_star, LIPM_model.p_y_star, 0]
                swing_foot_pos[:, 0] = np.linspace(LIPM_model.right_foot_pos[0], right_foot_target_pos[0],
                                                   swing_data_len)
                swing_foot_pos[:, 1] = np.linspace(LIPM_model.right_foot_pos[1], right_foot_target_pos[1],
                                                   swing_data_len)
                swing_foot_pos[1:swing_data_len - 1, 2] = 0.1
            else:
                left_foot_target_pos = [LIPM_model.p_x_star, LIPM_model.p_y_star, 0]
                swing_foot_pos[:, 0] = np.linspace(LIPM_model.left_foot_pos[0], left_foot_target_pos[0], swing_data_len)
                swing_foot_pos[:, 1] = np.linspace(LIPM_model.left_foot_pos[1], left_foot_target_pos[1], swing_data_len)
                swing_foot_pos[1:swing_data_len - 1, 2] = 0.1

        # print(f"x_0 is {x_0}, y_0 is {y_0}")
    print(f"swing_foot_pos is {swing_foot_pos[1]}")








    # while supervisor.step(timestep) != -1:
    #     CoM = nao.getCenterOfMass()
    #     print(CoM)
    # com_field = nao.getField("CoM")
    # if com_field:
    #     com_position = com_field.getSFVec3f()
    #     print("NAO 机器人的质心位置（相对于本地坐标系）:", com_position)
    # else:
    #     print("无法获取质心位置")
    # nao_children = nao.getField("bodySlot")
    # solid = nao_children.getMFNode(0)
    # print(solid.getTypeName())
    # if mass:
    #     print(mass)
    # else:
    #     print("无法获取质心位置")
else:
    print("未找到 NAO 机器人节点")