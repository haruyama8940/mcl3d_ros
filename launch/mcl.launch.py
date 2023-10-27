import launch
import launch_ros.actions

def generate_launch_description():
    # Localizer
    mcl_3d_ros_node = launch_ros.actions.Node(
        package='mcl3d_ros',
        executable='mcl',
        name='mcl3d_ros_node',
        parameters=[
            # {'map_yaml_file':'/home/haru/bags/ros2bag_tsudanuma_gaisyu/0811/range120/tsudanuma_dist_map.yaml'},
            # {'map_yaml_file':'/home/haru/bags/tsukuba/2023/0817/down001_0817_tsukuba.yaml'},
            {'map_yaml_file':'/home/haru/bags/tsukuba/2023/0817/0817_tsukuba_dist_map.yaml'},
            {'sensor_points_name':'surestar_points'},
            {'odom_name':'/odom'},
            {'imu_name':'/imu/data'},
            {'map_points_name':'map_cloud'},

            {'particles_name':'/particles'},
            {'opt_particles_name':'/optimized_particles'},
            {'pose_name':'/mcl_pose'},
            {'opt_pose_name"':'/opt_pose'},
            {'map_frame"':'map'},
            {'odom_frame"':'odom'},
            {'base_link_frame':'base_footprint'},
            {'laser_frame':'surestar'},
            {'opt_pose_frame':'opt_pose'},

            {'use_odom':True},
            {'use_imu"':False},

            {'broadcast_tf':True},
            {'use_odom_tf':True},
            {'use_initial_pose_cb':True},

            {'use_linear_interpolation':True},

            {'imu_sample_freq':'100'},
            # <!-- 0: measurement model optimization (similar to ICP scan matching) -->
            # <!-- 1: particle filter (it does not work usually if odometry and/or IMU are not available) -->
            # <!-- 2: fusion of particle-filter and optimization-based localizations -->
            # <!-- 3: extended kalman filter -->
            {'localization_mode':2},
            {'measurement_model_type':3},

            {'particle_num':100},

            {'sensor_points_num':500},

            {'random_particle_rate':0.1},
            {'resample_threshold':0.5},

            {'z_hit':0.9},
            {'z_rand':0.05},
            {'z_max"':0.05},
            {'var_hit':0.4},
            {'range_reso"':0.1},
            {'range_max':150.0},
            {'unknown_lambda"':0.001},

            # {'voxel_leaf_size"':1.0},
            {'voxel_leaf_size"':2.0},

            {'opt_max_iter_num':30},
            {'opt_max_error':0.5},
            {'convergence_threshold':0.02},

            {'optimized_particle_num':100},
            {'optimized_pose_cov_coef':1.0},
            {'gmm_postion_var':0.3},
            {'gmm_angle_var':0.1},

            {'ahrs_filter_kp':0.0},
            {'ahrs_filter_ki':0.0},

            {'log_file':'/tmp/mcl3d_log.txt'},
            {'write_log':False},

        ]
    )
    return launch.LaunchDescription([
        mcl_3d_ros_node
    ])
if __name__ == '__main__':
    generate_launch_description()