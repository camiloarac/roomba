from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_param_builder import load_xacro
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join 
from launch.substitutions import Command
from pathlib import Path

def generate_launch_description():

    # Start a simulation with the cafe world
    cafe_world_uri = join(get_package_share_directory("krytn"), "models", "gamecity_world.sdf")
    # cafe_world_uri = "empty.sdf"
    path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    
    gazebo_sim = IncludeLaunchDescription(path,
                                          launch_arguments=[("gz_args", '-r ' + cafe_world_uri)])

    # Create a robot in the world.
    # Steps: 
    # 1. Process a file using the xacro tool to get an xml file containing the robot description.
    # 2. Publish this robot description using a ros topic so all nodes can know about the joints of the robot. 
    # 3. Spawn a simulated robot in the gazebo simulation using the published robot description topic. 

    # Step 1. Process robot file. 
    robot_file = join(get_package_share_directory("roomba"), "robot_description","roomba","roomba.urdf.xacro")
    robot_xml = load_xacro(Path(robot_file))

    #Step 2. Publish robot file to ros topic /robot_description & static joint positions to /tf
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description':robot_xml, 
                     'use_sim_time':True}],
        namespace="/roomba"
    )

    # Step 3. Spawn a robot in gazebo by listening to the published topic.
    robot = Node(
        package='ros_gz_sim',
        executable="create",
        arguments=[
            "-topic", "/roomba/robot_description", 
            "-z", "0.5",
        ],
        name="spawn_robot",
        output="both"
    )

    # Gazebo Bridge: This brings data (sensors/clock) out of gazebo into ROS.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                #    '/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image',
                #    '/realsense/depth@sensor_msgs/msg/Image[gz.msgs.Image',
                #    '/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'
                   ],
        output='screen'
        )

    # The controller is setup to put everything in the /roomba frame. We need to construct a static transform to bring it back into the un-namespaced frames. 
    static_pub = Node(package="tf2_ros", 
                      executable="static_transform_publisher",
                      arguments=["0","0","0","0","0","0", "/roomba/body_link", "body_link"])

    # A gui tool for easy tele-operation.
    robot_steering = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
    )

    # Step 5: Enable the ros2 controllers
    start_controllers  = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["-c","/roomba/controller_manager",
                             'joint_state_broadcaster', 'diff_drive_base_controller'],
                output="screen",
            )
    
    return LaunchDescription([gazebo_sim, bridge, static_pub, robot,
                              robot_steering, robot_state_publisher,
                              start_controllers])