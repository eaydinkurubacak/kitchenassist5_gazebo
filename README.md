KitchenAssist5 — 5-DoF Robot Arm
ROS 2 Lyrical + Gazebo Jetty simülasyonu.

Çalıştırma:
colcon build --packages-select kitchenassist5_gazebo
source install/setup.bash
ros2 launch kitchenassist5_gazebo gazebo_spawn.launch.py

Node'lar:
arm_move_sequence.py — 3 konfigürasyon + başlangıca dönüş
visualize_state.py — eklem grafiği + kamera penceresi
fk_validation.py — 10 konfigürasyon FK doğrulama
end_effector_publisher.py — uç efektör konumu (/end_effector_position)

Topic'ler:
/joint_states
/end_effector_position
/kitchen_camera/image_raw
/arm_controller/joint_trajectory
