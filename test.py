from building_blocks import Building_Blocks
from kinematics import UR5e_PARAMS, Transform
from environment import Environment
from visualizer import Visualize_UR

import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np


ur_params = UR5e_PARAMS(inflation_factor=1)
env = Environment(env_idx=1)
transform = Transform(ur_params)

bb = Building_Blocks(transform=transform, 
                    ur_params=ur_params, 
                    env=env,
                    resolution=0.1, 
                    p_bias=0.05,)

visualizer = Visualize_UR(ur_params, env=env, transform=transform, bb=bb)


while True:
    conf = bb.sample(None)
    if conf is None:
        continue
    if not bb.is_in_collision(conf):
        continue
    print(f"configuration: {conf}")
    visualizer.show_conf(conf)
    break
    
# for i in range(50):
#     conf = bb.sample(None)
#     if conf is not None and any(abs(val) > np.pi for val in conf):
#         print("value out of range!!!")
#     print("============================================")
#     print("configuration: ", conf)
#     if conf is not None:
#         print(bb.is_in_collision(conf))
    
    # sns.scatterplot(x=range(len(conf)), y=conf)
    # plt.show()