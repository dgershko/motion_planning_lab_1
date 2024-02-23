import numpy as np
from kinematics import UR5e_PARAMS as UR_PARAMS
from scipy.spatial.distance import cdist

class Building_Blocks(object):
    '''
    @param resolution determines the resolution of the local planner(how many intermidiate configurations to check)
    @param p_bias determines the probability of the sample function to return the goal configuration
    '''
    def __init__(self, transform, ur_params, env, resolution=0.1, p_bias=0.05):
        self.transform = transform
        self.ur_params = ur_params # type: UR_PARAMS
        self.env = env
        self.resolution = resolution
        self.p_bias = p_bias
        self.cost_weights = np.array([0.4, 0.3 ,0.2 ,0.1 ,0.07 ,0.05])
        
    def sample(self, goal_conf) -> np.array:
        """
        sample random configuration
        @param goal_conf - the goal configuration
        """
        if np.random.uniform(0, 1) < self.p_bias:
            return goal_conf
        constraints = np.array(list(self.ur_params.mechamical_limits.values()))
        conf = np.random.uniform(constraints[:, 0], constraints[:, 1])
        return np.array(conf)
        

    def is_in_collision(self, conf) -> bool:
        """check for collision in given configuration, arm-arm and arm-obstacle
        return True if in collision
        @param conf - some configuration 
        """
        # TODO 
        # hint: use self.transform.conf2sphere_coords(), self.ur_params.sphere_radius, self.env.obstacles
        # global sphere coords: {link name: list of spheres}, s.t. list of spheres = [(x, y, z, [SOMETHING??])]
        global_sphere_coords = self.transform.conf2sphere_coords(conf)

        # arm - arm collision
        parts = list(global_sphere_coords.keys())
        arm_part_combinations = []
        # get list of combination of robot parts, ignoring parts that are adjacent as they always collide at their connection point
        for i in range(len(parts)):
            for j in range(i + 2, len(parts)):
                arm_part_combinations.append((parts[i], parts[j]))
        for part_1, part_2 in arm_part_combinations:
            spheres_1 = np.array(global_sphere_coords[part_1])
            spheres_2 = np.array(global_sphere_coords[part_2])
            distances = cdist(spheres_1[:, :-1], spheres_2[:, :-1])
            differences = distances - (self.ur_params.sphere_radius[part_1] + self.ur_params.sphere_radius[part_2])
            if np.any(differences < 0):
                print(f"collision detected between {part_1} and {part_2}")
                return True

        # arm - obstacle collision
        obstacle_spheres = self.env.obstacles
        robot_spheres = np.concatenate(list(global_sphere_coords.values()), axis=0)
        distances = cdist(robot_spheres[:, :-1], obstacle_spheres)
        sphere_radii = np.concatenate([np.repeat(self.ur_params.sphere_radius[part], len(global_sphere_coords[part])) for part in parts])[:, None]
        differences = distances - (sphere_radii + self.env.radius)
        if np.any(differences < 0):
            print("Collision detected with obstacle!")
            return True
        
        # arm - floor collision
        spheres_without_shoulder = {key: value for key, value in global_sphere_coords.items() if key != "shoulder_link"}
        parts = list(spheres_without_shoulder.keys())
        robot_spheres = np.concatenate(list(spheres_without_shoulder.values()), axis = 0)
        distances = robot_spheres[:, 2]
        differences = distances - np.concatenate([np.repeat(self.ur_params.sphere_radius[part], len(spheres_without_shoulder[part])) for part in parts])
        if np.any(differences < 0):
            print("Collision with floor detected!")
            return True
        return False
    
    def local_planner(self, prev_conf ,current_conf) -> bool:
        '''check for collisions between two configurations - return True if trasition is valid
        @param prev_conf - some configuration
        @param current_conf - current configuration
        '''
        num_configs = int(np.max(np.abs(current_conf - prev_conf)) / self.resolution)
        if num_configs < 3:
            num_configs = 3
        
        configs = np.linspace(prev_conf, current_conf, num_configs, True)
        return not any(self.is_in_collision(config) for config in configs)
        
        # if (self.is_in_collision(prev_conf) or self.is_in_collision(current_conf)):
        #     # if self.is_in_collision(prev_conf):
        #     #     print(f"collision in initial config!")
        #     # else:
        #     #     print(f"collision in final config!")
        #     return False
        # i_conf = prev_conf
        # # i_conf stands for intermediate configuration
        # # while loop runs while there's a point in the intermediate config that's further than self.res from the goal config
        # # by the way these comments were written by dgershko, not chatgpt
        # iteration = 0
        # while not np.allclose(i_conf, current_conf, atol=self.resolution):
        #     iteration += 1
        #     # mask of points that are close enough to the target conf
        #     close_mask = np.isclose(i_conf, current_conf, atol=self.resolution)
        #     # mask of points that are larger or smaller than the target conf
        #     far_mask = i_conf < current_conf
        #     far_mask_conf = np.where(far_mask, i_conf + self.resolution, i_conf - self.resolution)
        #     # make far points closer, while not touching close enough points
        #     i_conf = np.where(close_mask, i_conf, far_mask_conf)
        #     # rounding to 4 decimals (the number 4 came to me in a dream)
        #     i_conf = np.around(i_conf, 4)
        #     if self.is_in_collision(i_conf):
        #         # print(f"collision in iteration: {iteration}")
        #         return False
        # return True

    
    def edge_cost(self, conf1, conf2):
        '''
        Returns the Edge cost- the cost of transition from configuration 1 to configuration 2
        @param conf1 - configuration 1
        @param conf2 - configuration 2
        '''
        return np.dot(self.cost_weights, np.power(conf1-conf2,2)) ** 0.5
    
    

    
    
    