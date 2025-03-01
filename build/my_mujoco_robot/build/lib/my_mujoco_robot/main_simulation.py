import time
import mujoco
import mujoco.viewer
import numpy as np
import asset_init
from ros_publisher_init import *
from asset_init import *

def main():
  # 初始化 ROS 2

    rclpy.init()
    
    # 创建发布者节点实例
    publisher = CtrSignalPublisher()
    state_publisher = StatePublisher()


    asset = asset_init()  # 加载资产
    MI_robot = arm_init()

    with mujoco.viewer.launch_passive(asset.model, asset.data) as viewer:
        start = time.time()
        while 1:
            step_start = time.time()
            # #之后ros2 topic要用
            # state = MI_robot.get_joint_state(asset.data)
            #完成一次力矩控制    
            # asset.data = MI_robot.ctrl_signal_send(asset.data)

            # 广播控制信号
            # publisher.publish_ctr_signal(ctr_signal)
            # 广播关节状态
            # state_publisher.publish_state(np.array(state))
            # 进行一步仿真
            mujoco.mj_step(asset.model, asset.data)

            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(asset.data.time % 2)
                # viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_AXES] = True  # 显示坐标轴
            # 同步查看器
            viewer.sync()

            # 时间控制
            time_until_next_step = asset.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    # 销毁节点并关闭 ROS 2
    publisher.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()













# data.ctrl[motor_claw1] = -0.1
# data.ctrl[motor_claw2] = -0.1 


# with mujoco.viewer.launch_passive(asset.model, asset.data) as viewer:
#   # Close the viewer automatically after 30 wall-seconds.
#   start = time.time()
#   # while viewer.is_running() and time.time() - start < 100:
#   while 1:
#     step_start = time.time()
#     # displacement1 = data.qpos[MI_robot.motor_claw1]
#     # displacement2 = data.qpos[MI_robot.motor_claw2]
#     ctr_signal = MI_robot.PID_calculate_control(asset.data)
#     # data.ctrl[MI_robot.motor_claw1] = claw_control.calculate(0.031,displacement1)
#     # data.ctrl[MI_robot.motor_claw1] = claw_control.calculate(0.031,displacement1)
#     for i in range (9):
#       asset.data.ctrl[i] = ctr_signal[i]
    
    
#     # mj_step can be replaced with code that also evaluates
#     # a policy and applies a control signal before stepping the physics.
    
    
#     mujoco.mj_step(asset.model, asset.data)

#     # Example modification of a viewer option: toggle contact points every two seconds.
#     with viewer.lock():
#       viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(asset.data.time % 2)

#     # Pick up changes to the physics state, apply perturbations, update options from GUI.
#     viewer.sync()

#     # Rudimentary time keeping, will drift relative to wall clock.
#     time_until_next_step = asset.model.opt.timestep - (time.time() - step_start)
#     if time_until_next_step > 0:
#       time.sleep(time_until_next_step)
