import mujoco
import time
import mujoco.viewer



#ros通信导入

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class MujocoControl(Node):
    def __init__(self):
        super().__init__('mujoco_sim')
        # 加载MuJoCo模型
        self.model = mujoco.MjModel.from_xml_path('src/work_mujoco/xml/tasks/m92uw_wing_upload_scene.xml')
        self.data = mujoco.MjData(self.model) 
        # 订阅控制指令
        self.command_sub = self.create_subscription(
            JointState, 
            '/m92uw_joint_commands', 
            self.command_callback, 
            10
        )

        # 获取躯干关节 ID
        self.motor_ankle_p_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ankle_p_joint')
        self.motor_knee_p_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'knee_p_joint')
        self.motor_waist1_p_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'waist1_p_joint')
        self.motor_waist2_y_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'waist2_y_joint')

        # 获取头部关节 ID
        self.motor_head1_y_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'head1_y_joint')
        self.motor_head2_p_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'head2_p_joint')

        # 获取左臂关节 ID
        self.motor_l_arm1_shoulder_p_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'l_arm1_shoulder_p_joint')
        self.motor_l_arm2_shoulder_r_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'l_arm2_shoulder_r_joint')
        self.motor_l_arm3_shoulder_y_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'l_arm3_shoulder_y_joint')
        self.motor_l_arm4_elbow_p_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'l_arm4_elbow_p_joint')
        self.motor_l_arm5_wrist_y_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'l_arm5_wrist_y_joint')
        self.motor_l_arm6_wrist_p_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'l_arm6_wrist_p_joint')
        self.motor_l_arm7_wrist_r_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'l_arm7_wrist_r_joint')

        # 获取右臂关节 ID
        self.motor_r_arm1_shoulder_p_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'r_arm1_shoulder_p_joint')
        self.motor_r_arm2_shoulder_r_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'r_arm2_shoulder_r_joint')
        self.motor_r_arm3_shoulder_y_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'r_arm3_shoulder_y_joint')
        self.motor_r_arm4_elbow_p_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'r_arm4_elbow_p_joint')
        self.motor_r_arm5_wrist_y_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'r_arm5_wrist_y_joint')
        self.motor_r_arm6_wrist_p_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'r_arm6_wrist_p_joint')
        self.motor_r_arm7_wrist_r_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'r_arm7_wrist_r_joint')

        # 获取左手指关节 ID
        self.motor_l_index_mcp_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'l_index_mcp_joint')
        self.motor_l_middle_mcp_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'l_middle_mcp_joint')
        self.motor_l_pinky_mcp_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'l_pinky_mcp_joint')
        self.motor_l_ring_mcp_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'l_ring_mcp_joint')
        self.motor_l_thumb_cmc_y_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'l_thumb_cmc_y_joint')
        self.motor_l_thumb_cmc_p_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'l_thumb_cmc_p_joint')

        # 获取右手指关节 ID
        self.motor_r_index_mcp_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'r_index_mcp_joint')
        self.motor_r_middle_mcp_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'r_middle_mcp_joint')
        self.motor_r_pinky_mcp_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'r_pinky_mcp_joint')
        self.motor_r_ring_mcp_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'r_ring_mcp_joint')
        self.motor_r_thumb_cmc_y_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'r_thumb_cmc_y_joint')
        self.motor_r_thumb_cmc_p_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'r_thumb_cmc_p_joint')

    def command_callback(self, msg):
        # 将ROS消息中的关节位置指令应用到MuJoCo
        pass
        # for i in range(9):
        #     self.data.qpos[i] = msg.position[i]
        #     # print(msg.position)
        


def main():
    rclpy.init()
    asset = MujocoControl()
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