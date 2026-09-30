# Atividade 2 — Controle de velocidade e teleoperação

Esta atividade é um workspace ROS 2 Humble independente da Atividade 1. O modelo TRI-X tem carroceria esportiva azul, cabine escura e três rodas visíveis. As duas rodas dianteiras são acionadas por `ros2_control` e `diff_drive_controller`; a roda traseira é um caster de baixo atrito. O launch inicia um mundo com piso texturizado de asfalto, teclado e publicação de odometria em `/odom`.

## Dependências

Instale os pacotes usados pela simulação (o PlotJuggler é opcional):

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-teleop-twist-keyboard \
  ros-humble-topic-tools python3-numpy gnome-terminal
```

Para abrir PlotJuggler com o launch, instale também `ros-humble-plotjuggler-ros`.

## Compilar e executar

```bash
cd ~/dev/ros2_ws/"atvd 2"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch model_description robot.launch.py
```

O launch abre uma janela de terminal separada para o teclado. Use `i` para avançar, `,` para ré, `j`/`l` para girar e `k` para parar. Os limites configurados são 0,5 m/s e 1,5 rad/s. Para iniciar outro mundo, passe `world:=/caminho/para/arquivo.world`.

Para iniciar PlotJuggler junto com Gazebo e o teclado (após instalar `ros-humble-plotjuggler-ros`):

```bash
ros2 launch model_description robot.launch.py plot:=true
```

No PlotJuggler, adicione `/odom` (`nav_msgs/msg/Odometry`) e observe `twist.twist.linear.x` e `twist.twist.angular.z`. O controlador publica em `/diff_drive_controller/odom`; o launch encaminha essa mensagem para `/odom` conforme o enunciado. `gui:=false` inicia Gazebo sem interface gráfica.
