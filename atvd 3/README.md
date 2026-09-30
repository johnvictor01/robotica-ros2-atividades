# Atividade 3 — Controle de pose e trajetória

Workspace ROS 2 Humble com o robô TRI-X no Gazebo Classic, controle diferencial e controladores de pose, missão por waypoints e trajetória em oito. O controlador C++ usa `control_toolbox::Pid`; parâmetros e waypoints ficam em YAML.

## Requisitos

Ubuntu com ROS 2 Humble instalado. Instale os pacotes usados pelo workspace:

```bash
sudo apt update
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-control-toolbox \
  ros-humble-plotjuggler \
  ros-humble-plotjuggler-ros
```

## Compilar

Abra um terminal e rode:

```bash
cd ~/dev/ros2_ws/"atvd 3"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Sempre que abrir um terminal novo para executar os nós, carregue os ambientes ROS e do workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/dev/ros2_ws/"atvd 3"/install/setup.bash
```

## Missão pelos waypoints

Execute a missão no Terminal 1:

```bash
cd ~/dev/ros2_ws/"atvd 3"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch trajectory_control trajectory.launch.py mode:=mission plot:=false
```

O robô usa controle contínuo e percorre os waypoints em sequência. Os valores iniciais são `(5, 0)`, `(5, 5)` e `(0, 5)` metros, configurados em `src/trajectory_control/config/control.yaml`. No Gazebo, cada ponto tem um disco amarelo no centro e um anel verde pontilhado com raio de chegada de `0.10 m`. Ao entrar nesse raio, o controlador troca para o próximo ponto sem uma parada programada. Ao terminar os três pontos, volta ao primeiro porque `loop_mission` está habilitado.

Para executar a missão uma vez e parar no fim:

```bash
ros2 launch trajectory_control trajectory.launch.py mode:=mission loop_mission:=false plot:=false
```

Os centros amarelos e anéis verdes do cenário correspondem aos waypoints padrão. Se você alterar as coordenadas no YAML ou o raio, atualize também os marcadores em `src/model_description/worlds/asphalt.world`.

## Trajetória em oito

Encerre a simulação anterior com `Ctrl+C` e execute no Terminal 1:

```bash
cd ~/dev/ros2_ws/"atvd 3"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch trajectory_control trajectory.launch.py mode:=figure8 plot:=false
```

A trajetória segue as equações `x = A sin(Ωt)` e `y = B sin(2Ωt)`, centrada na origem do mundo e repetida continuamente. Os valores padrão são `A = 2 m`, `B = 4 m` e `Ω = 0.09 rad/s`. O seguimento padrão usa realimentação. Para comparar os modos:

```bash
# Malha aberta
ros2 launch trajectory_control trajectory.launch.py mode:=figure8 tracking_mode:=open_loop plot:=false

# Realimentação com feedforward
ros2 launch trajectory_control trajectory.launch.py mode:=figure8 tracking_mode:=feedback kff:=1.0 plot:=false
```

## PlotJuggler ao vivo

O lançamento da simulação usa `plot:=false` nos comandos acima. Com Gazebo rodando, abra um Terminal 2 e inicie o PlotJuggler:

```bash
source /opt/ros/humble/setup.bash
source ~/dev/ros2_ws/"atvd 3"/install/setup.bash
ros2 run plotjuggler plotjuggler
```

No PlotJuggler:

1. Em **Streaming**, selecione **ROS2 Topic Subscriber** e clique em **Start**.
2. Aguarde `/trajectory/desired_pose` e `/trajectory/actual_pose` aparecerem na lista.
3. Selecione a visualização **XY Plot** e arraste `x` e `y` de cada tópico para comparar o caminho desejado com o real.

As mensagens são `geometry_msgs/msg/Pose2D`. Para visualizar os valores ao longo do tempo, use um gráfico temporal e adicione os campos `x`, `y` ou `theta`.

## Gravar dados para analisar depois

Para salvar a trajetória desejada, a real e a odometria em um rosbag, acrescente `record:=true` ao comando de lançamento. Exemplo:

```bash
ros2 launch trajectory_control trajectory.launch.py mode:=mission plot:=false record:=true
```

O rosbag é salvo em um diretório `rosbag2_*` no diretório atual e finalizado ao encerrar a simulação com `Ctrl+C`. Esses arquivos de dados locais ficam fora do Git pelo `.gitignore`.

## Controle de uma pose

Inicie o modo de pose:

```bash
ros2 launch trajectory_control trajectory.launch.py mode:=pose plot:=false
```

Em outro terminal, envie uma pose `(x, y, theta)` em metros e radianos:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/Pose2D "{x: 1.0, y: 0.5, theta: 1.57}"
```

Para usar a estratégia de três manobras (rotação, translação e rotação), passe `pose_method:=three_maneuvers`. A estratégia contínua é o padrão.

## Tópicos ROS 2

| Tópico | Tipo | Uso |
| --- | --- | --- |
| `/goal_pose` | `geometry_msgs/msg/Pose2D` | Pose desejada no modo `pose` |
| `/odom` | `nav_msgs/msg/Odometry` | Odometria do robô |
| `/diff_drive_controller/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | Comando para o controlador diferencial |
| `/trajectory/desired_pose` | `geometry_msgs/msg/Pose2D` | Pose planejada para PlotJuggler |
| `/trajectory/actual_pose` | `geometry_msgs/msg/Pose2D` | Pose real estimada pela odometria |
| `/mission/status` | `std_msgs/msg/String` | Estado e progresso da missão |

## Organização do workspace

- `src/model_description`: URDF/Xacro, malhas, mundo asfaltado e lançamento do Gazebo.
- `src/trajectory_control`: controlador C++, parâmetros, lançamento da missão e da trajetória.
- `src/trajectory_control/config/control.yaml`: ganhos PID, waypoints e parâmetros do oito.

O workspace não declara uma licença de redistribuição. Consulte o autor antes de reutilizar ou redistribuir o código.
