


from legged_gym.envs import AnymalCRoughCfg, AnymalCRoughCfgPPO


class UnitreeGo1FlatCfg(AnymalCRoughCfg):
    class env(AnymalCRoughCfg.env):

        num_observations = 48
        episode_length_s = 4.

        num_envs = 4096

    class terrain(AnymalCRoughCfg.terrain):
        mesh_type = 'plane'
        measure_heights = False

    
    class commands(AnymalCRoughCfg.commands): 

        heading_command = False
        num_commands = 5
        resampling_time = 2.
        
        base_height_command = True

        class ranges:
            lin_vel_x = [0., 0.]
            lin_vel_y = [0., 0.]
            ang_vel_yaw = [0., 0.]
            heading = [0., 0.]
            base_height = [0.2, 0.6]
            # base_height_adjustment = [0., 0.]
            # base_height_target = 0.6
    

    class init_state(AnymalCRoughCfg.init_state):
        
        robot_upside_down = True
        pos = [0.0, 0.0, 0.3]

        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.1,   # [rad] #0.1 radian is 5.7 degrees
            'RL_hip_joint': 0.1,   # [rad] #0.1 radian is 5.7 degrees
            'FR_hip_joint': -0.1 ,  # [rad] #-0.1 radian is -5.7 degrees
            'RR_hip_joint': -0.1,   # [rad] #-0.1 radian is -5.7 degrees

            'FL_thigh_joint': 0.8,     # [rad] #0.8 radian is 45.9 degrees
            'RL_thigh_joint': 1.,   # [rad] #1 radian is 57.3 degrees
            'FR_thigh_joint': 0.8,     # [rad] #0.8 radian is 45.9 degrees
            'RR_thigh_joint': 1.,   # [rad] #1 radian is 57.3 degrees

            'FL_calf_joint': -1.5,   # [rad] #-1.5 radian is -85.9 degrees
            'RL_calf_joint': -1.5,    # [rad] #-1.5 radian is -85.9 degrees
            'FR_calf_joint': -1.5,  # [rad] #-1.5 radian is -85.9 degrees
            'RR_calf_joint': -1.5,    # [rad] #-1.5 radian is -85.9 degrees 

        }

        #Joint limitations: for referencen 
        #Hip joint: -60 ~ 60 degrees, in radians: -1.0472 ~ 1.0472
        #Thigh joint: -38 ~ 170 degrees, in radians: -0.663225115 ~ 2.96705973
        #Calf joint: -156 ~ -48 degrees, in radians: -2.70526034 ~ -0.837758041


        # namespace UNITREE_LEGGED_SDK 
        # {
        #     constexpr double go1_Hip_max   = 1.047;    // unit:radian ( = 60   degree)
        #     constexpr double go1_Hip_min   = -1.047;   // unit:radian ( = -60  degree)
        #     constexpr double go1_Thigh_max = 2.966;    // unit:radian ( = 170  degree)
        #     constexpr double go1_Thigh_min = -0.663;   // unit:radian ( = -38  degree)
        #     constexpr double go1_Calf_max  = -0.837;   // unit:radian ( = -48  degree)
        #     constexpr double go1_Calf_min  = -2.721;   // unit:radian ( = -156 degree)
        # }

        #Enum difinition for the four legs in Unitree Go1 
        # Leg0 FR = right front leg
        # Leg1 FL = left front leg
        # Leg2 RR = right rear leg
        # Leg3 RL = left rear leg

        

    class control(AnymalCRoughCfg.control):

        control_type = 'P'
        stiffness = {'joint': 20.}  # [N*m/rad]
        damping = {'joint': 0.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

        use_actuator_network = False



    class asset(AnymalCRoughCfg.asset):
        self_collisions = 0

        file = "{LEGGED_GYM_ROOT_DIR}/resources/robots/go1/urdf/Xinhua_go1.urdf"

        name = "go1"
        foot_name = "foot"
        penalize_contacts_on = ["base", "thigh", "calf"]
        
        terminate_after_contacts_on = []

        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter

    
    class rewards( AnymalCRoughCfg.rewards ):
        max_contact_force = 350.
        base_height_target = 0.3
        class scales ( AnymalCRoughCfg.rewards.scales ):

            
            termination = -0.0
            tracking_lin_vel = 0.
            tracking_ang_vel = 0.
            lin_vel_z = -0.
            ang_vel_xy = 0. 
            orientation = -1.0
            torques = -0.00001
            dof_vel = -0.
            dof_acc = -2.5e-7
            base_height = -5.
            feet_air_time =  0.
            collision = -0.5
            feet_stumble = -0.0 
            action_rate = -0.01
            stand_still = -0.

            base_uprightness = 1.
            foot_contact = 1. 
            dof_power = -0.0001
            dof_pos_limits = -0.1

            dof_position = -1.

        
    
    class domain_rand( AnymalCRoughCfg.domain_rand ):
        friction_range = [0., 1.5] # on ground planes the friction combination mode is averaging, i.e total friction = (foot_friction + 1.)/2.

class UnitreeGo1FlatCfgPPO( AnymalCRoughCfgPPO ):
    # class policy( AnymalCRoughCfgPPO.policy ):
    #     actor_hidden_dims = [128, 64, 32]
    #     critic_hidden_dims = [128, 64, 32]
    #     activation = 'elu' # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid

    # class algorithm( AnymalCRoughCfgPPO.algorithm):
    #     entropy_coef = 0.01

    class runner ( AnymalCRoughCfgPPO.runner):
        run_name = 'go1_flat'
        experiment_name = 'flat_unitree_go1'
        # load_run = -1
        load_run = r"/home/bridge/Desktop/legged_gym/logs/flat_unitree_go1/Jun19_17-10-47_go1_flat" 
        max_iterations = 1501

        num_steps_per_env = 30 # 30 steps per env

