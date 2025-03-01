import mujoco
import numpy as np
from scipy.interpolate import CubicSpline

#IK导入
import kinpy as kp
import math

#ros通信导入

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class asset_init:
    def __init__(self) :
        self.model = mujoco.MjModel.from_xml_path('src/asset/MI_arm/MI_robot.xml')
        self.data = mujoco.MjData(self.model) 
        

class robot_arm():
    """
        定义了机械臂的关节空间、正运动学、逆运动学、轨迹规划
        全部以关节角度的方式进行传输
    """
    def __init__(self) -> None:
        """
            DH参数初始化
        """

        pass

def quaternion_multiply(q1, q2):
    """
    四元数乘法
    :param q1: 四元数 1，格式为 [w1, x1, y1, z1]
    :param q2: 四元数 2，格式为 [w2, x2, y2, z2]
    :return: 相乘后的四元数，格式为 [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1
    x = w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1
    y = w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1
    z = w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1
    return [w, x, y, z]
class arm_init(asset_init):

    """
        机械臂初始化函数，定义各关节信息，定义获取各关节角度方法
    """
    def __init__(self) -> None:
        super().__init__()
        model = self.model
        self.motor_1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint1')
        self.motor_2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint2')
        self.motor_3 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint3')
        self.motor_4 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint4')
        self.motor_5 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint5')
        self.motor_6 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint6')
        self.motor_7 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint7')
        self.motor_claw1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'DH_finger_joint1')
        self.motor_claw2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'DH_finger_joint2')
        self.end_effector_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'DH_end_center')
        self.box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'box')
        self.motor_matrix = [self.motor_1,self.motor_2,self.motor_3,self.motor_4,self.motor_5,self.motor_6,self.motor_7]
        self.claw_matrix = [self.motor_claw1,self.motor_claw2]
        #所有关节的PID参数  总共9个，前7个是机械臂，后面2个是夹爪
        self.Kp = np.array([40,20,20,20,20,20,20,100,100])
        # self.Ki = np.array([0,0,0,0,0,0,0,1,1])
        self.Ki = np.array([4,4,4,4,4,4,4,1,1])
        self.Kd = np.array([20,50,30,30,30,20,20,0.1,0.1])
        self.current = [0,0,0,0,0,0,0,0,0]
        self.integral_error = 0
        self.prev_error = 0
        self.dt = 0.01  # 控制周期，需要与模拟步长相匹配
    def get_joint_state(self,data):
        self.current[0] = data.qpos[self.motor_1]
        self.current[1] = data.qpos[self.motor_2]
        self.current[2] = data.qpos[self.motor_3]
        self.current[3] = data.qpos[self.motor_4]
        self.current[4] = data.qpos[self.motor_5]
        self.current[5] = data.qpos[self.motor_6]
        self.current[6] = data.qpos[self.motor_7]
        self.current[7] = data.qpos[self.motor_claw1]
        self.current[8] = data.qpos[self.motor_claw2]
        return self.current
    def get_quat(self,data,id):
        quat = data.xquat[id]
        return quat
    def get_pos(self,data,id):
        pos = data.xpos[id]
        return pos
    def get_end_effector(self,data):
        """
            获取到末端执行器的四元数数据
        """
        end_effector_quat = data.xquat[self.end_effector_id]
        
        # print("End effector quaternion:", end_effector_quat)
        return end_effector_quat
    def PID_calculate_control(self,data):
        """
            Arg:
                传入numpy形式的一维数组，总共9个关节角度，后两个关节保持数据相同
        """
        self.get_joint_state(data)
        error = self.target - self.current
        self.integral_error += error * self.dt#累积误差
        derivative = (error - self.prev_error) / self.dt
        output = (self.Kp * error) + (self.Ki * self.integral_error) + (self.Kd * derivative)
        self.prev_error = error
        return output   
    def move2target(self,data):
        """
            外部直接调用此接口

        """
        
        self.ctrl_signal_send(data)

    def ctrl_signal_send(self,data):
        """
            下发角度，并且更新mujoco中data
        """
        #获取末端执行器+box四元数
        end_effector_quat = self.get_quat(data,self.end_effector_id)#id11
        target_box_quat = self.get_quat(data,self.box_id)#id13
        end_effector_pos = self.get_pos(data,self.end_effector_id)
        target_box_pos = self.get_pos(data,self.box_id)
        self.target  = np.array(self.compute_ik(target_box_pos,target_box_quat))

        
        ctr_signal = self.PID_calculate_control(data)
        for i in range(9):
            data.ctrl[i] = ctr_signal[i]
        return data
    
    def generate_trajectory(self,points, duration):
        """
            points: 机械臂末端需要经过的关键点列表，每个点为(x, y, z)
            duration: 每段轨迹的持续时间
        """
        num_points = len(points)
        time_steps = np.linspace(0, (num_points - 1) * duration, 1000 * (num_points - 1))  # 生成1000个时间点
        cs_x = CubicSpline(np.arange(num_points), [point[0] for point in points])
        cs_y = CubicSpline(np.arange(num_points), [point[1] for point in points])
        cs_z = CubicSpline(np.arange(num_points), [point[2] for point in points])
        trajectory = np.array([cs_x(time_steps), cs_y(time_steps), cs_z(time_steps)]).T
        return trajectory
    
    def compute_ik(self,target_pos,tar_quat):
        """
            计算逆运动学，返回关节角度
            model: MuJoCo模型
            sim: MuJoCo仿真数据
            target_pos: 目标位置 (3,)
            target_quat: 目标姿态 (四元数, 4,)
        """

        chain = kp.build_serial_chain_from_urdf(open("mirobot_description/urdf/arm.urdf"), "panda_fingertip_centered")
        th = [0.0, -math.pi / 4.0, 0.0, math.pi / 2.0, 0.0, math.pi / 4.0, 0.0]
        ret = chain.forward_kinematics(th, end_only=True)

        ret.pos = target_pos#目标抓取物的位置
        ret.rot = quaternion_multiply([0,0,1,0],tar_quat)#旋转目标的四元数

        arm_ag = chain.inverse_kinematics(ret)#机械臂逆解，需要提供目标的pos+quat
        claw_ag = [0.025, 0.025]
        totol_ag = np.append(arm_ag,claw_ag)
 
        # 返回机械臂关节角度
        # return data.data.qpos.copy()
        return totol_ag
    
    def move_idx(self):
        """
            机械臂的有限状态机，执行对应的动作序列
            不能进行实时反解，也就是说不能跟随
        """



        pass


class MujocoSim(Node):
    def __init__(self):
        super().__init__('mujoco_sim')
        # 加载MuJoCo模型
        self.model = mujoco.MjModel.from_xml_path("asset/MI_arm/MI_robot.xml")
        self.data = mujoco.MjData(self.model)
        
        # 订阅控制指令
        self.command_sub = self.create_subscription(
            JointState, 
            '/isaac_joint_commands', 
            self.command_callback, 
            10
        )
    
    def command_callback(self, msg):
        # 将ROS消息中的关节位置指令应用到MuJoCo
        for i, name in enumerate(msg.name):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.data.ctrl[joint_id] = msg.position[i]
    
    def run_simulation(self):
        joint_pub = self.create_publisher(JointState, '/isaac_joint_states', 10)
        rate = self.create_rate(1000)  # 1kHz
        while rclpy.ok():
            mujoco.mj_step(self.model, self.data)
            
            # 构造JointState消息
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['joint1', 'joint2', 'joint3','joint4','joint5','joint6','joint7','panda_finger_joint1','panda_finger_joint2']  # 按实际关节名填写
            msg.position = self.data.qpos.tolist()
            msg.velocity = self.data.qvel.tolist()
            
            joint_pub.publish(msg)
            rate.sleep()