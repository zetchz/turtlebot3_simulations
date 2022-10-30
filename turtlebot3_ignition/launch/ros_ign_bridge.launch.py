# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
          '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # cmd_vel bridge
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
          '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # cmd_vel bridge
    cmd_vel_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                          name='cmd_vel_bridge',
                          output='screen',
                          parameters=[{
                              'use_sim_time': use_sim_time
                          }],
                          arguments=[
                              '/cmd_vel' + '@geometry_msgs/msg/Twist' + '[ignition.msgs.Twist',
                          ])
    # transform
    tf_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                          name='transform_bridge',
                          output='screen',
                          parameters=[{
                              'use_sim_time': use_sim_time
                          }],
                          arguments=[
                              '/model/turtlebot3_burger/pose' + '@geometry_msgs/msg/TransformStamped' + '@gz.msgs.Pose',
                          ])

    return LaunchDescription([
        imu_bridge,
        scan_bridge,
        cmd_vel_bridge,
        tf_bridge,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
    ])
