import os
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 
from ament_index_python.packages import get_package_share_directory
from launch.actions.timer_action import TimerAction
from launch.conditions import IfCondition

def generate_turtlebot_description():
    ld_list = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'turtlebot3_bringup'), 'launch/robot.launch.py')
            )
        )
    ]

    return ld_list


def generate_realsense_description():
    realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'realsense2_camera'), 'launch/rs_launch.py')
            ),
            launch_arguments={
                'align_depth': LaunchConfiguration('align_depth'),
                'device_type': LaunchConfiguration('device_type'),
                'initial_reset': LaunchConfiguration('initial_reset'),
                'log_level': 'info'
            }.items()
        )

    ld_list = [
        DeclareLaunchArgument(
            name='align_depth',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='device_type',
            default_value='d455'
        ),
        DeclareLaunchArgument(
            name='initial_reset',
            default_value='true'
        ),
        TimerAction(period=7.0, actions=[realsense_launch])
    ]

    return ld_list
    

def generate_server_description():
    server_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'univloc_server'), 'launch/server.launch.py')
            ),
            launch_arguments={
                'fix_scale': LaunchConfiguration('fix_scale'),
                'rviz': LaunchConfiguration('server_rviz'),
                'save_map_path': LaunchConfiguration('save_map_path'),
                'save_traj_folder': LaunchConfiguration('save_traj_folder'),
                'load_map_path': LaunchConfiguration('load_map_path'),
                'server_mode': LaunchConfiguration('server_mode'),
            }.items()
        )

    ld_list = [
        DeclareLaunchArgument(
            name='fix_scale',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='server_rviz',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='save_map_path',
            default_value='""'
        ),
        DeclareLaunchArgument(
            name='save_traj_folder',
            default_value='""'
        ),
        DeclareLaunchArgument(
            name='load_map_path',
            default_value='""'
        ),
        DeclareLaunchArgument(
            name='server_mode',
            default_value='mapping'
        ),
        TimerAction(period=17.0, actions=[server_launch])
    ]

    return ld_list

def generate_tracker_description():
    tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'univloc_tracker'), 'launch/tracker.launch.py')
        ),
        launch_arguments={
	    'use_odom': LaunchConfiguration('use_odom'),
	    'odom_tf_query_timeout': LaunchConfiguration('odom_tf_query_timeout'),
	    'odom_buffer_query_timeout': LaunchConfiguration('odom_buffer_query_timeout'),
            'baselink_frame': LaunchConfiguration('baselink_frame'),
	    'image_frame': LaunchConfiguration('image_frame'),
            'camera': LaunchConfiguration('camera'),
            'camera_setup': LaunchConfiguration('camera_setup'),
            'get_camera_extrin_from_tf': LaunchConfiguration('get_camera_extrin_from_tf'),
            'camera_extrin_query_timeout': LaunchConfiguration('camera_extrin_query_timeout'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'map_frame': LaunchConfiguration('map_frame'),
            'pub_tf_child_frame': LaunchConfiguration('pub_tf_child_frame'),
            'queue_size': LaunchConfiguration('queue_size'),
            'rviz': LaunchConfiguration('tracker_rviz'),
            'gui': LaunchConfiguration('gui'),
            'log_level': LaunchConfiguration('tracker_log_level'),
	    'traj_store_path': LaunchConfiguration('traj_store_path'),
	    'octree_store_path': LaunchConfiguration('octree_store_path'),
	    'enable_fast_mapping': LaunchConfiguration('enable_fast_mapping'),
	    'zmin': LaunchConfiguration('zmin'),
	    'zmax': LaunchConfiguration('zmax'),
	    'projection_min': LaunchConfiguration('projection_min'),
	    'projection_max': LaunchConfiguration('projection_max'),
            'depth_max_range': LaunchConfiguration('depth_max_range'),
            'octree_load_path': LaunchConfiguration('octree_load_path'),
            'odom_frame': LaunchConfiguration('odom_frame'),
              
        }.items()
    )
    ld_list = [
        DeclareLaunchArgument(
            name='use_odom',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='odom_tf_query_timeout',
            default_value='50.0'
        ),
        DeclareLaunchArgument(
            name='odom_buffer_query_timeout',
            default_value='25.0'
        ),
        DeclareLaunchArgument(
            name='baselink_frame',
            default_value='base_link'
        ),
        DeclareLaunchArgument(
            name='image_frame',
            default_value='camera_color_optical_frame'
        ),
        DeclareLaunchArgument(
            name='camera',
            default_value='camera'
        ),
        DeclareLaunchArgument(
            name='camera_setup',
            default_value='RGBD'
        ),
        DeclareLaunchArgument(
            name='get_camera_extrin_from_tf',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='camera_extrin_query_timeout',
            default_value='10.0'
        ),
        DeclareLaunchArgument(
            name='publish_tf',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='map_frame',
            default_value='map'
        ),
        DeclareLaunchArgument(
            name='pub_tf_child_frame',
            default_value='odom'
        ),
        DeclareLaunchArgument(
            name='queue_size',
            default_value='1'
        ),
        DeclareLaunchArgument(
            name='tracker_rviz',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='tracker_log_level',
            default_value='warning'
        ),
        DeclareLaunchArgument(
            name='enable_fast_mapping',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='zmin',
            default_value='0.3'
        ),
        DeclareLaunchArgument(
            name='zmax',
            default_value='0.6'
        ),
        DeclareLaunchArgument(
            name='projection_min',
            default_value='0.3'
        ),
        DeclareLaunchArgument(
            name='projection_max',
            default_value='0.6'
        ),
        DeclareLaunchArgument(
            name='depth_max_range',
            default_value='2.5'
        ),
        DeclareLaunchArgument(
            name='octree_store_path',
            default_value='""'
        ),
        DeclareLaunchArgument(
            name='traj_store_path',
            default_value='""'
        ),
        DeclareLaunchArgument(
            name='octree_load_path',
            default_value='""'
        ),
        DeclareLaunchArgument(
            name='odom_frame',
            default_value='odom'
        ),
        DeclareLaunchArgument(
            name='slam_node',
            default_value='mapping'
        ),
        # Delay 5 seconds to launch tracker node after realsense node and server node
        TimerAction(period=20.0, actions=[tracker_launch])
    ]

    return ld_list
    
    
        

def generate_navigation_description():
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.getcwd(), 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('nav_params_file'),
            'remap_map_id': LaunchConfiguration('remap_map_id')
        }.items(),
        condition=IfCondition(
            LaunchConfiguration('start_navigation_node')
        )
    )

    ld_list = [
        DeclareLaunchArgument(
            name='nav_params_file',
            # Specify the value to your own navigation configuration file
            default_value='/home/zy/ws/collaborative_slam/script/waffle_pi.yaml'
        ),
        DeclareLaunchArgument(
            name='start_navigation_node',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='remap_map_id',
            default_value='/univloc_tracker_0/map'
        ),
        # Delay 15 seconds to launch nav2_bringup after fast_mapping node
        TimerAction(period=23.0, actions=[navigation_launch])
    ]

    return ld_list

def generate_static_tf_description():
    static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            output='screen',
            # x y z yaw pitch roll frame_id child_frame_id
            arguments=[
                LaunchConfiguration('static_tf_x'),
                LaunchConfiguration('static_tf_y'),
                LaunchConfiguration('static_tf_z'),
                LaunchConfiguration('static_tf_yaw'),
                LaunchConfiguration('static_tf_pitch'),
                LaunchConfiguration('static_tf_roll'),
                'base_link',
                'camera_link'
            ],
            parameters=[{
                'use_sim_time': LaunchConfiguration('static_tf_use_sim_time')
            }]
        )
    ld_list = [
        DeclareLaunchArgument(
            name='static_tf_x',
            default_value='0.064'
        ),
        DeclareLaunchArgument(
            name='static_tf_y',
            default_value='0.045'
        ),
        DeclareLaunchArgument(
            name='static_tf_z',
            default_value='0.11'
        ),
        DeclareLaunchArgument(
            name='static_tf_yaw',
            default_value='0'
        ),
        DeclareLaunchArgument(
            name='static_tf_pitch',
            default_value='0'
        ),
        DeclareLaunchArgument(
            name='static_tf_roll',
            default_value='0'
        ),
        DeclareLaunchArgument(
            name='static_tf_use_sim_time',
            default_value='False'
        ),
        TimerAction(period=5.0, actions=[static_tf_node])
    ]

    return ld_list


def generate_launch_description():
    turtlebot3 = generate_turtlebot_description()
    tracker = generate_tracker_description()
    server = generate_server_description()
    realsense = generate_realsense_description()
    navigation = generate_navigation_description()
    static_tf = generate_static_tf_description()

    # Need to carefully specify the action delay to ensure the below startup order
    # 1. TurtleBot3 2. Static TF 3. Realsense 4. Collaborative SLAM 5. Navigation
    launch_description = turtlebot3+static_tf+realsense+server+tracker+navigation
    
    return launch.LaunchDescription(launch_description)

if __name__ == '__main__':
    generate_launch_description()
