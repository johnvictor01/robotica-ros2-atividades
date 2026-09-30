# Atividade 2 — Controle de velocidade e teleoperação

Workspace ROS 2 Humble independente. O robô TRI-X tem três rodas, controle diferencial nas duas rodas dianteiras e um mundo com piso de asfalto. O launch publica a odometria no tópico `/odom`.

## Dependências

```bash
sudo apt update
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-teleop-twist-keyboard \
  ros-humble-topic-tools \
  python3-numpy \
  gnome-terminal
```

PlotJuggler é opcional; para abri-lo a partir do launch, instale também `ros-humble-plotjuggler` e `ros-humble-plotjuggler-ros`.

## Compilar e executar

```bash
cd ~/dev/ros2_ws/robotica-ros2-atividades/"atvd 2"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch model_description robot.launch.py
```

O launch abre um terminal separado para teleoperação. Use `i` para avançar, `,` para ré, `j` e `l` para girar e `k` para parar. Os limites padrão são `0.5 m/s` e `1.5 rad/s`. Para escolher outro mundo, passe `world:=/caminho/para/arquivo.world`.

## Odometria e PlotJuggler

O controlador publica odometria em `/diff_drive_controller/odom`; o launch encaminha a mensagem para `/odom`. Para iniciar também o PlotJuggler:

```bash
ros2 launch model_description robot.launch.py plot:=true
```

No PlotJuggler, selecione **ROS2 Topic Subscriber**, clique em **Start** e adicione `/odom` (`nav_msgs/msg/Odometry`). Observe `twist.twist.linear.x` e `twist.twist.angular.z`. Para iniciar Gazebo sem interface gráfica, use `gui:=false`.
