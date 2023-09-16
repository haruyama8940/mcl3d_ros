import launch
import launch_ros.actions

def generate_launch_description():
    pc_to_df_node = launch_ros.actions.Node(
        package='',
        executable='',
        name='',
        parameters=[
            # #<!-- if from_pcd_file is true, pcd_file must be appropriately set. -->
            # {'map_points_name':'/map_points'},
            # <!-- if from_pcd_file is true, pcd_file must be appropriately set. -->
            {'pcd_file':'/home/haru/bags/tsukuba/2023/0817/0817_tsukuba.pcd'},
            #  <!-- map name -->
            {'map_file_name':'tsukuba_con.bin'},
            #<!-- map resolutions. sub_map_resolution is actual resolution for the distance field -->
            {'resolution':'5.0'},
            {'sub_map_resolution':'0.1'},
            #<!-- map margin -->
            {'map_margin':'1.0'},
            # <!-- yaml file -->
            {'yaml_file_path':'/home/haru/bags/tsukuba/2023/0817/tsukuba_dist_map.yaml'},
        ]
    )
    return launch.LaunchDescription([
        pc_to_df_node
    ])
if __name__ == '__main__':
    generate_launch_description()