from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mocap_vicon',
            executable='mocap_vicon_node',
            name='vicon',
            output='screen',
            parameters=[
                {'server_address': 'mocap.perch'},
                {'frame_rate': 100},
                {'max_accel': 10.0},
                {'publish_tf': False},
                {'publish_pts': False},
                {'fixed_frame_id': 'mocap'},
                {'model_list': ['']},
            ],
            remappings=[
                # Uncomment and modify the remapping if needed
                # ('vicon/model_name/odom', '/model_name/odom'),
            ]
        )
    ])
