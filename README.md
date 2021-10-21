# MetalBot

## Описание
Репозиторий содержит основные модули для работы MetalBot как в симуляции так и на реальном роботе.
Используемый навигационный стек - Navigation2.

<!-- vim-markdown-toc GitLab -->

* [User](#user)
  * [Docker Guide](#docker-guide)
  * [Настройка окружения](#Настройка-окружения)
    * [ROS2 workspace](#Building-ros2-workspace)
    * [ROS1 workspace (don't source r2)](#Building-ros1-workspace-dont-source-r2)
    * [ros1_bridge](#building-ros1_bridge)
    * [Micro-ROS](#Building-Micro-ROS)
  * [Сборка tycmd](#Сборка-tycmd)
  * [Запуск основных модулей](#Запуск-основных-модулей)
    * [BringUp.](#bringup)
    * [Navigation2](#navigation2)
    * [RViz](#rviz)
    * [Groot](#groot)
    * [Slam](#slam)
    * [Teleop](#teleop)
    * [Oakd Camera](#oakd-camera)
    * [Micro-ROS](#Micro-ROS)

<!-- vim-markdown-toc -->



## User

Для начала необходимо создать воркспейс и склонировать репозиторий

```bash
git clone --recurse-submodules -j4 https://github.com/FastSense/metalbot/
```

Далее рекомендуется использовать Docker для того чтобы не устанавливать все необходимые зависимости вручную

### Docker Guide

```bash
cd docker

# Если предполагается собирать образ с использованием CUDA
./nvidia-deps.sh

# Установка имен, необходимых параметров при сборки образа и контейнера (выбрать версию с gazebo, для робота или универсальную)

# Select one of the following
source env-gazebo.sh
source env-robot.sh
source env-universal.sh

# Создание образа
./drun.sh build

# Создание контейнера
./drun.sh run

# Загрузка образа с dockerhub
./drun.sh pull

# Загрузка образа на dockerhub
./drun.sh push

# Работа с уже созданным контейнером
docker start $container
docker attach $container
```

### Настройка окружения

#### ROS2
**First terminal**
Build ROS2 basic packages (**don't source r1**)
```bash
cd ros2_ws
# source ros2
r2
# build basic packages, no Gazebo, Groot, sensors, grid_map
cb_basic
r2

# Optionaly
cb_gazebo
cd ros2_ws
cb_selected package_name1 package_name2 ... # Build selected packages
cb_realsense
cb_oakd
cb_rplidar
```

#### ROS1 (don't source r2)
**Second terminal**
```bash
cd ros1_ws
r1
catkin_make -j4
r1
```

#### ros1_bridge
**Third terminal**
Note that you must build and source all required interfaces first (msg, srv)
```bash
cd ros2_ws
r1
r2
cb_bridge
```
#### Micro-ROS
**Fourth terminal**
```bash
cd microros_ws
# Source ROS2
r2
# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y
# Build micro-ROS tools and source ROS2 & Micro-ROS
colcon build
r2
# Download micro-ROS agent packages
ros2 run micro_ros_setup create_agent_ws.sh
# Build step
ros2 run micro_ros_setup build_agent.sh
```

### Запуск основных модулей

Для запуска модулей MetalBot существует набор launch фаилов.  
Набор модулей для симуляции представлены в пакете rosbot_gazebo.
Модули для реального робота, а так же общие модули представлены в пакете rosbot.

#### BringUp.
```bash
# Запуск основных нод для работы с роботом (в симуляции спаунит робота)
ros2 launch rosbot[_gazebo] bringup.launch.py
```

```bash
# Запуск основных нод для работы с роботом в симуляции, но без статичных tf
ros2 launch rosbot_gazebo bringup_no_static_tf.launch.py
```

#### Navigation2
```bash
ros2 launch rosbot[_gazebo] nav2.launch.py
```

#### RViz
```bash
ros2 launch rosbot rviz.launch.py
```

#### Groot
```bash
# Запуск программы для визуализации Behavior Tree
ros2 run groot Groot
```

#### Slam
```bash
# BringUp + Navigation2 + SlamToolBox + RViz[optional]
ros2 launch rosbot[_gazebo] slam.launch.py
```

#### Teleop
```bash
# Запуск ноды телеуправления с клавиатуры (wasd)  (Запускается на хост машине)
ros2 launch rosbot_controller rosbot_sim_keyboard_teleop.launch.py

# Альтернативно можно воспользоваться встроенным телеопом
sudo apt update
sudo apt install ros-foxy-teleop-tools
ros2 run teleop_twist_keyboard teleop_twist_keyboard   
```

#### Oakd Camera
```bash
# Запуск камеры Oakd
ros2 run oakd oakd_node

# (В случае ошибки связанной с не найденным устройством посмотреть в интерпретаторе питона возможные id камеры и поменять в ноде oakd_node - device_id)
python3
import depthai as dai
dev = dai.Device()
dev.getAllAvailableDevices()[0].getMxId()
dev.getAllAvailableDevices()[1].getMxId()
dev.getAllAvailableDevices()[2].getMxId()

Один из айди должен подойти.
```

#### Micro-ROS
```bash
r2
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```
