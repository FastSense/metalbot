# MetalBot

## Описание
Репозиторий содержит основные модули для работы MetalBot как в симуляции так и на реальном роботе.
Используемый навигационный стек - Navigation2.

## Оглавление

- [MetalBot](#metalbot)
  - [Описание](#описание)
  - [Оглавление](#оглавление)
  - [Настройка среды](#настройка-среды)
    - [Setup Docker](#setup-docker)
      - [Утилита drun.sh](#утилита-drunsh)
      - [Работа с уже созданным контейнером](#работа-с-уже-созданным-контейнером)
    - [Setup workspaces](#setup-workspaces)
      - [Build ROS2 workspace](#build-ros2-workspace)
      - [Build ROS1 workspace](#build-ros1-workspace)
      - [Build ros1_bridge](#build-ros1_bridge)
      - [Build Micro-ROS](#build-micro-ros)
    - [Build tycmd](#build-tycmd)
  - [Запуск основных модулей](#запуск-основных-модулей)
      - [Bring Up](#bring-up)
      - [Navigation2](#navigation2)
      - [RViz](#rviz)
      - [Groot](#groot)
      - [Teleop](#teleop)
      - [Realsense Camera](#realsense-camera)
      - [Micro-ROS](#micro-ros)
      - [State Estimation 2D](#state-estimation-2d)
      - [ros1_bridge](#ros1_bridge)



## Настройка среды

Для начала необходимо создать воркспейс и склонировать репозиторий

```bash
git clone --recurse-submodules -j4 https://github.com/FastSense/metalbot/
```

Если предполагается собирать образ с использованием CUDA
```bash
./nvidia-deps.sh
```


### Setup Docker

```bash
cd docker

# Выбрать версию докер-контейнера
source env-gazebo.sh
source env-robot.sh
source env-universal.sh
```

#### Утилита drun.sh
```bash
# Создание образа
./drun.sh build

# Создание контейнера
./drun.sh run

# Загрузка образа с dockerhub
./drun.sh pull

# Загрузка образа на dockerhub
./drun.sh push


```

#### Работа с уже созданным контейнером
```bash
docker start $container
docker attach $container
```

### Setup workspaces
Докер содержит пространства ros1, ros2 и micro_ros

> При сборке ros1 не должны быть экспортированы переменные окружения ros2, и наоборот.

Для экспорта ros1_ws существует alias ```r1```  
Для экспорта ros2_ws и micro_ros_ws существует alias ```r2```


#### Build ROS2 workspace
> Переменные окружения ROS1 не должны быть экспортированы
```bash
# Basic packages (no gazebo, groot, sensors, grid_map)
cd ros2_ws
r2
cb_basic

# Build selected packages
cb_selected package_name1 package_name2 ...

# Optional packages
cb_gazebo
cb_realsense
```

#### Build ROS1 workspace
> Переменные окружения ROS2 не должны быть экспортированы
```bash
cd ros1_ws
r1
catkin_make -j4
```

#### Build ros1_bridge

> Перед началом сборки ros1_bridge следует собрать все имеющиеся интерфейсы (сообщения, сервисы) ros1 и ros2 и экспортировать переменные окружения в порядке: ros1, ros2
```bash
# .. build all interfaces
cd ros2_ws
r1
r2
cb_bridge
```
#### Build Micro-ROS
```bash
cd micro_ros_ws
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

### Build tycmd
> Установливать утилиту следует в систему бортового компьютера (не в докер)

Утилита используется для программного перезапуска прошивки Teensy 4.1. Например, в случае разрыва связи с агентом Micro-ROS.

  **Сборка**
```bash
git clone https://github.com/Koromix/tytools.git
cd tytools
mkdir -p build/linux && cd build/linux
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ../..
make 
make install
```

  **Базовые Команды**

```bash
# Отображение информации о доступных платах Teensy
tycmd list -w

# Перезапустить выбранный контроллер
# Micro-ROS агент должен быть выключен
sudo tycmd reset
```
## Запуск основных модулей
#### Bring Up
Запуск основных нод для работы с роботом
```bash
ros2 launch metalbot[_gazebo] bringup.launch.py
```
#### Navigation2
```bash
ros2 launch metalbot[_gazebo] nav2.launch.py
```

#### RViz
```bash
ros2 launch metalbot rviz.launch.py
```

#### Groot
```bash
# Запуск программы для визуализации Behavior Tree
ros2 run groot Groot
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

#### Realsense Camera
```bash
ros2 launch fs-realsense rs_launch.py
```

#### Micro-ROS
```bash
# Запускается в скрипте bringup
ros2 launch metalbot micro_ros.launch.py
```
#### State Estimation 2D
Запуск на metalbot:
```bash
ros2 launch state_estimation_2d state_estimation_metalbot.launch.py 
```

#### ros1_bridge
> Статические tf не пробрасываются. Нужно запускать bringup с Параметром use_tf_static = false
```bash
# Показать доступные mapping'и интерфейсов
ros2 run ros1_bridge dynamic_bridge --print-pairs

# Запуск ros2_bridge
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
