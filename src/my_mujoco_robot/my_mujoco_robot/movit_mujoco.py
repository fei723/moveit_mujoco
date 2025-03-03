import mujoco
import mujoco.viewer
import numpy as np
from scipy.interpolate import CubicSpline
import threading
import time

#IK导入
import kinpy as kp
import math

#ros通信导入

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class MujocoSim(Node):
    def __init__(self):
        super().__init__('mujoco_sim')
        # 加载MuJoCo模型
        self.model = mujoco.MjModel.from_xml_path('src/asset/MI_arm/MI_robot.xml')
        self.data = mujoco.MjData(self.model) 
        # 订阅控制指令
        self.command_sub = self.create_subscription(
            JointState, 
            '/isaac_joint_commands', 
            self.command_callback, 
            10
        )
        
        self.motor_1 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint1')
        self.motor_2 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint2')
        self.motor_3 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint3')
        self.motor_4 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint4')
        self.motor_5 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint5')
        self.motor_6 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint6')
        self.motor_7 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint7')
        self.motor_claw1 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'DH_finger_joint1')
        self.motor_claw2 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'DH_finger_joint2')
        self.end_effector_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'DH_end_center')
        self.box_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'box')
        self.motor_matrix = [self.motor_1,self.motor_2,self.motor_3,self.motor_4,self.motor_5,self.motor_6,self.motor_7]
        self.claw_matrix = [self.motor_claw1,self.motor_claw2]
    
    
    def command_callback(self, msg):
        # 将ROS消息中的关节位置指令应用到MuJoCo

        for i in range(9):
            self.data.qpos[i] = msg.position[i]
            # print(msg.position)
        
        
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


def main():
  # 初始化 ROS 2
    rclpy.init()
    asset = MujocoSim()
    # rclpy.spin(asset)
    # spin_thread = threading.Thread(target=spin_node, args=(asset))
    # spin_thread.start()
    with mujoco.viewer.launch_passive(asset.model, asset.data) as viewer:
        start = time.time()
        while 1:
            step_start = time.time()
            rclpy.spin_once(asset)
            # 进行一步仿真
            mujoco.mj_step(asset.model, asset.data)
            # asset.run_simulation()

            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(asset.data.time % 2)
                # viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_AXES] = True  # 显示坐标轴
            # 同步查看器
            viewer.sync()

            # 时间控制
            time_until_next_step = asset.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    rclpy.shutdown()

if __name__ == "__main__" :
    main()