from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chappie_control',
            executable='obstacle_guard_node',
            name='obstacle_guard',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'stop_distance_m':         0.30,  # Parada de emergencia en seco (360°)
                'slowdown_distance_m':     0.80,  # Inicio de frenado progresivo y esquiva
                'arc_half_width_deg':      60.0,  # Cono frontal de visión (±60° = 120° total)
                'front_angle_offset_rad':  0.0,   # Ajuste por si el 0° del láser no apunta al frente
                'k_avoid_rad_s':           0.6,   # Agresividad del giro al esquivar
                'clear_scans_required':    3,     # Scans limpios antes de soltar el freno (~0.3s)
                'max_linear_x_m_s':        0.30,  # Límite absoluto de velocidad lineal
                'min_valid_range_m':       0.05,  # Filtro para ignorar rebotes del propio chasis
            }],
        ),
    ])
