# Atividades de Robótica — ROS 2 Humble

Repositório geral das atividades práticas de robótica. Cada diretório `atvd N` é um workspace ROS 2 independente, com seus próprios pacotes, instruções e comandos de compilação. Compile e execute dentro da pasta da atividade escolhida.

## Atividades

| Pasta | Conteúdo |
| --- | --- |
| [`atvd 1`](atvd%201/README.md) | Descrição e visualização básica do robô no Gazebo. |
| [`atvd 2`](atvd%202/README.md) | Robô TRI-X de três rodas, controle diferencial, teleoperação e mundo asfaltado. |
| [`atvd 3`](atvd%203/README.md) | Controle de pose, missão contínua por waypoints, trajetória em oito e visualização/gravação de dados. |

## Requisitos gerais

É necessário ter Ubuntu e ROS 2 Humble instalados. As atividades usam Gazebo Classic. Cada README lista os pacotes adicionais necessários para aquela prática.

## Como começar

Entre na pasta da atividade desejada, carregue ROS 2, compile e carregue o workspace. Exemplo para a Atividade 3, considerando o clone neste caminho:

```bash
cd ~/dev/ros2_ws/robotica-ros2-atividades/"atvd 3"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Depois siga o README dentro da atividade para iniciar a simulação e os nós correspondentes. Encerre uma simulação antes de iniciar outra.

## Estrutura

```text
robotica-ros2-atividades/
├── atvd 1/    # workspace independente
├── atvd 2/    # workspace independente
└── atvd 3/    # workspace independente
```

Builds, logs, arquivos temporários e gravações locais de rosbag são ignorados pelo Git. O repositório não declara uma licença de redistribuição; consulte o autor antes de reutilizar ou redistribuir o código.
