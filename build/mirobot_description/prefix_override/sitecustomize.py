import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shifei/Downloads/Isaac_sim_assets/moveit_des/install/mirobot_description'
