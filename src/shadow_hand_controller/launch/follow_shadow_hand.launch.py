from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # 공통 인자
    use_keyboard = LaunchConfiguration('use_keyboard')

    # 이동 릴레이 인자
    in_tr   = LaunchConfiguration('input_topic_translate')
    out_tr  = LaunchConfiguration('output_topic_translate')
    en_tr   = LaunchConfiguration('enable_topic_translate')
    scale   = LaunchConfiguration('pos_scale')
    frm_tr  = LaunchConfiguration('frame_id_translate')
    start_tr= LaunchConfiguration('enabled_translate')

    # 회전 릴레이 인자
    in_or   = LaunchConfiguration('input_topic_orient')
    out_or  = LaunchConfiguration('output_topic_orient')
    en_or   = LaunchConfiguration('enable_topic_orient')
    frm_or  = LaunchConfiguration('frame_id_orient')
    start_or= LaunchConfiguration('enabled_orient')

    # --- Nodes ---
    relay_translate = Node(
        package='shadow_hand_controller',
        executable='shadow_hand_relay',
        name='shadow_hand_relay_translate',
        output='screen',
        parameters=[{
            'input_topic': in_tr,
            'output_topic': out_tr,
            'enable_topic': en_tr,
            'enabled': start_tr,
            'pos_scale': scale,
            'frame_id_override': frm_tr,
            'position_only': True,
            'orientation_only': False,
        }]
    )

    relay_orient = Node(
        package='shadow_hand_controller',
        executable='shadow_hand_relay',
        name='shadow_hand_relay_orient',
        output='screen',
        parameters=[{
            'input_topic': in_or,
            'output_topic': out_or,
            'enable_topic': en_or,
            'enabled': start_or,
            'pos_scale': 1.0,            # 회전 전용이라 의미 없음
            'frame_id_override': frm_or,
            'position_only': False,
            'orientation_only': True,
        }]
    )

    keyctl = Node(
        package='shadow_hand_controller',
        executable='shadow_hand_keyctl',
        name='shadow_hand_keyctl',
        output='screen',
        condition=IfCondition(use_keyboard),
        parameters=[{
            'translate_enable_topic': en_tr,
            'orient_enable_topic': en_or,
        }]
    )

    return LaunchDescription([
        # ---- Declare Args ----
        DeclareLaunchArgument('use_keyboard', default_value='false'),

        DeclareLaunchArgument('input_topic_translate',  default_value='/right_controller'),
        DeclareLaunchArgument('output_topic_translate', default_value='/controller_translate'),
        DeclareLaunchArgument('enable_topic_translate', default_value='/shadow_hand_enable_translate'),
        DeclareLaunchArgument('frame_id_translate',     default_value=''),
        DeclareLaunchArgument('enabled_translate',      default_value='true'),
        DeclareLaunchArgument('pos_scale',              default_value='1.0'),

        DeclareLaunchArgument('input_topic_orient',     default_value='/right_controller'),
        DeclareLaunchArgument('output_topic_orient',    default_value='/controller_orient'),
        DeclareLaunchArgument('enable_topic_orient',    default_value='/shadow_hand_enable_orient'),
        DeclareLaunchArgument('frame_id_orient',        default_value=''),
        DeclareLaunchArgument('enabled_orient',         default_value='true'),

        # ---- Nodes ----
        relay_translate,
        relay_orient,
        keyctl,
    ])
