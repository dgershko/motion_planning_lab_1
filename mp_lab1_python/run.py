import numpy as np
from environment import Environment
from kinematics import UR5e_PARAMS, Transform
from planners import RRT_STAR
from building_blocks import Building_Blocks
from visualizer import Visualize_UR


def main():
    ur_params = UR5e_PARAMS(inflation_factor=1)
    env = Environment(env_idx=1)
    transform = Transform(ur_params)
    
    bb = Building_Blocks(transform=transform, 
                        ur_params=ur_params, 
                        env=env,
                        resolution=0.1, 
                        p_bias=0.05,)
    
    visualizer = Visualize_UR(ur_params, env=env, transform=transform, bb=bb)

    # --------- configurations-------------
    home = np.deg2rad([0, -90, 0, -90, 0,0 ])
    
    # ---------------------------------------
    
    visualizer.show_conf(home)
    
   

if __name__ == '__main__':
    main()



