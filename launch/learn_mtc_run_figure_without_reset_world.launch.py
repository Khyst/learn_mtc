import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as f:
        return yaml.safe_load(f)

def get_fr3_robot_description(description_pkg_name):
    description_pkg_path = get_package_share_directory(description_pkg_name)
    franka_xacro_file = os.path.join(description_pkg_path, 'robots', 'fr3', 'fr3.urdf.xacro')
    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'robot_type': 'fr3',
            'hand': 'true',
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': 'franka_hand'
        }
    )
    return {'robot_description': robot_description_config.toxml()}

def get_fr3_robot_description_semantic(description_pkg_name):
    description_pkg_path = get_package_share_directory(description_pkg_name)
    srdf_xacro_file = os.path.join(description_pkg_path, 'robots', 'fr3', 'fr3.srdf.xacro')
    robot_description_semantic_config = xacro.process_file(
        srdf_xacro_file,
        mappings={'hand': 'true', 'ee_id': 'franka_hand'}
    )
    return {'robot_description_semantic': robot_description_semantic_config.toxml()}

def get_fr3_ompl_planning_pipeline_config(config_pkg_name, config_file_name):
    ompl_planning_yaml = load_yaml(config_pkg_name, f'config/{config_file_name}')
    ompl_planning_pipeline_config = {
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ' '.join([
                'default_planner_request_adapters/AddTimeOptimalParameterization',
                'default_planner_request_adapters/ResolveConstraintFrames',
                'default_planner_request_adapters/FixWorkspaceBounds',
                'default_planner_request_adapters/FixStartStateBounds',
                'default_planner_request_adapters/FixStartStateCollision',
                'default_planner_request_adapters/FixStartStatePathConstraints',
            ]),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)
    return ompl_planning_pipeline_config

def generate_launch_description():
    # 1. 기존 파라미터 로드 설정
    robot_description = get_fr3_robot_description('franka_fr3_description')
    robot_description_semantic = get_fr3_robot_description_semantic('franka_fr3_description')
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    ompl_planning_pipeline_config = get_fr3_ompl_planning_pipeline_config('franka_fr3_moveit_config', 'ompl_planning.yaml')

    moveit_controllers = {
        'moveit_simple_controller_manager': load_yaml('franka_fr3_moveit_config', 'config/moveit_controllers.yaml'),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # 2. 명령어 인자(Launch Argument) 선언
    # 여기에 적힌 default_value보다 명령어로 입력한 값이 우선됩니다.
    parameter_arguments = [
        DeclareLaunchArgument('obj_x', default_value='0.6'),
        DeclareLaunchArgument('obj_y', default_value='-0.25'),
        DeclareLaunchArgument('obj_z', default_value='0.1'),
        DeclareLaunchArgument('place_x', default_value='0.6'),
        DeclareLaunchArgument('place_y', default_value='0.3'),
        DeclareLaunchArgument('place_z', default_value='0.2'),
        DeclareLaunchArgument('gripper_left_joint', default_value='0.1'),
        DeclareLaunchArgument('gripper_right_joint', default_value='0.1'),
    ]

    # 3. YAML 파일 경로
    learn_mtc_params_file = os.path.join(
        get_package_share_directory('learn_mtc'), 'config', 'learn_mtc_run_figure_config.yaml'
    )

    # 4. MTC Demo node 설정
    pick_place_demo = Node(
        package="learn_mtc",
        executable="learn_mtc",
        name="mtc_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            # (중요) 먼저 YAML 파일을 로드합니다.
            learn_mtc_params_file,
            # (중요) 그 다음 명령어 인자를 맵핑합니다. 
            # 리스트 뒷부분에 있으므로 YAML의 동일한 변수명을 덮어씁니다.
            {
                'obj_x': LaunchConfiguration('obj_x'),
                'obj_y': LaunchConfiguration('obj_y'),
                'obj_z': LaunchConfiguration('obj_z'),
                'place_x': LaunchConfiguration('place_x'),
                'place_y': LaunchConfiguration('place_y'),
                'place_z': LaunchConfiguration('place_z'),
                'gripper_left_joint': LaunchConfiguration('gripper_left_joint'),
                'gripper_right_joint': LaunchConfiguration('gripper_right_joint'),
                'use_sim_time': True,
            },
        ],
    )

    # 선언된 인자들과 노드를 함께 리턴
    return LaunchDescription(parameter_arguments + [pick_place_demo])