import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.conditions import IfCondition, LaunchConfigurationEquals,LaunchConfigurationNotEquals,evaluate_condition_expression
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,PythonExpression
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static"),("/aft_mapped","/aft_mapped")]
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'
    
    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("rviz")
    imu = LaunchConfiguration("imu")
    lidar_type = LaunchConfiguration("lidar_type")
    
    declare_lidar_type = DeclareLaunchArgument(
        "lidar_type",
        default_value="mid360",
        choices=["airy", "mid360","temp"],
        description="Type of LiDAR to use: airy or mid360",
    )
    point_lio_dir = get_package_share_directory("point_lio")
    
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )
    
    declare_imu = DeclareLaunchArgument(
        "imu",
        default_value="inner",
        choices=["inner","outer","renni"],
        description="inner or outer imu to use",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz", 
        default_value="True", 
        description="Flag to launch RViz."
    )

    usd_rename_lidar_link=Node(
        package="tf2_ros",
        condition=LaunchConfigurationEquals("imu", "renni"),
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "usd_aft_mapped", "lidar_link"],
        output="log",
        ros_arguments=['--log-level', "info"]
    )

    # normal_rename_lidar_link=Node(
    #     package="tf2_ros",
    #     # condition=LaunchConfigurationNotEquals("imu", "renni"),
    #     condition=IfCondition(
    #         PythonExpression([
    #             '"', LaunchConfiguration('lidar_type'), '" != "airy" and ', # airy使用airy_tf_corrector发布lidar_link
    #             '"', LaunchConfiguration('imu'), '" != "renni"'
    #         ])
    #     ),
    #     executable="static_transform_publisher",
    #     arguments=["0", "0", "0", "0", "0", "0", "aft_mapped", "lidar_link"],
    #     output="log",
    #     ros_arguments=['--log-level', "error"]
    # )


    # rename_lidar_odom=Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["0", "0", "0", "0", "0", "0", "camera_init", "lidar_odom"],
    #     output="log",
    #     ros_arguments=['--log-level', "error"]
    # )

    
    # 定义两种配置文件路径
    inner_config = PathJoinSubstitution([point_lio_dir, "config", "mid360_inner.yaml"])
    outer_config = PathJoinSubstitution([point_lio_dir, "config", "mid360_outer.yaml"])
    renni_config = PathJoinSubstitution([point_lio_dir,"config","mid360_renni.yaml"])
    airy_config = PathJoinSubstitution([point_lio_dir, "config", "airy.yaml"])
    temp_config = PathJoinSubstitution([point_lio_dir, "config", "temp.yaml"])
    start_point_lio_node_inner = Node(
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('lidar_type'), '" == "mid360" and ',
                '"', LaunchConfiguration('imu'), '" == "inner"'
            ])
        ),
        package="point_lio",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=[inner_config],
        remappings=remappings,
        output="screen",
    )

    start_point_lio_node_outer = Node(
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('lidar_type'), '" == "mid360" and ',
                '"', LaunchConfiguration('imu'), '" == "outer"'
            ])
        ),
        package="point_lio",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=[outer_config],
        remappings=remappings,
        output="screen",
    )

    start_point_lio_node_renni = Node(
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('lidar_type'), '" == "mid360" and ',
                '"', LaunchConfiguration('imu'), '" == "renni"'
            ])
        ),
        package="point_lio",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=[renni_config],
        remappings=remappings,
        output="screen",
    )

    start_point_lio_node_airy = Node(
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration('lidar_type'), '" == "airy"'])
        ),
        package="point_lio",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=[airy_config],
        remappings=remappings,
        output="screen",
    )
    start_point_lio_node_temp = Node(
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration('lidar_type'), '" == "temp"'])
        ),
        package="point_lio",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=[temp_config],
        remappings=remappings,
        output="screen",
    )

    start_rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        name="rviz",
        remappings=remappings,
        arguments=[
            "-d",
            PathJoinSubstitution([point_lio_dir, "rviz_cfg", "loam_livox.rviz"]),
        ],
        ros_arguments=['--log-level', "info"]
    )

    start_usd_tf_node = Node(
        condition=LaunchConfigurationEquals("imu", "renni"),
        package="usd_tf",          
        executable="usd_tf_node",  
        name="usd_tf",             
        output="screen",           
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_rviz)
    ld.add_action(declare_lidar_type)
    ld.add_action(declare_imu)
    # ld.add_action(rename_lidar_odom)
    ld.add_action(usd_rename_lidar_link)
    # ld.add_action(normal_rename_lidar_link)
    ld.add_action(start_point_lio_node_inner)
    ld.add_action(start_point_lio_node_outer)
    ld.add_action(start_point_lio_node_renni)
    ld.add_action(start_point_lio_node_airy)
    ld.add_action(start_point_lio_node_temp)
    ld.add_action(start_rviz_node)
    ld.add_action(start_usd_tf_node)

    return ld
