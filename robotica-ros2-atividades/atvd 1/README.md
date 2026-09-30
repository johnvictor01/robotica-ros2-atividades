# Atividade 1 — Descrição do robô

Workspace ROS 2 Humble independente para descrever e visualizar a versão inicial do robô no Gazebo.

## Compilar e executar

Com ROS 2 Humble e os pacotes `gazebo_ros` e `robot_state_publisher` instalados:

```bash
cd ~/dev/ros2_ws/robotica-ros2-atividades/"atvd 1"
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
ros2 launch model_description robot.launch.py
```

O modelo URDF está em `src/model_description/urdf/robot.urdf`; o launch inicia Gazebo, publica a descrição do robô e o insere no mundo padrão.
