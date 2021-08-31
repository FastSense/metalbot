# ROSbot on ROS2

## Описание
Репозиторий содержит основные модули для работы ROSbot как в симуляции так и на реальном роботе.
Используемый навигационный стек - Navigation2.

<!-- vim-markdown-toc GitLab -->

* [User Guide](#user-guide)
  * [Docker Guide](#docker-guide)
  * [Настройка окружения](#Настройка-окружения)
  * [Запуск основных модулей](#Запуск-основных-модулей)

<!-- vim-markdown-toc -->

## User Guide

Для начала необходимо создать воркспейс и склонировать репозиторий

```
mkdir rosbot_ws
cd rosbot_ws
git clone --recurse-submodules -j4 https://github.com/FastSense/rosbot-ros2/ src
```

Далее рекомендуется использовать Docker для того чтобы не устанавливать все необходимые зависимости вручную

### Docker Guide

Комманды необходимые для установки зависимостей и сборки Docker образа и контейнера:
```
cd src/docker-gazebo
./dependencies.sh		        # Установка зависимостей на локальную машину
./build.sh			        # Установка Docker образа 
./run.sh				# Создать Docker контейнер
```

Для начала работы с созданным контейнером небходимо стартовать контейнер и подсоеденится к нему:
```
docker start rosbot2 
docker attach rosbot2 
```

### Настройка окружения 

Внутри конейнера нужно собрать рабочее пространство ROS
```
cd ros2_ws
colcon build --symlink-install
. install/setup.zsh
```

### Запуск основных модулей

Для запуска модулей ROSbot существует набор launch фаилов.  
Набор модулей для симуляции представлены в пакете rosbot_gazebo. 
Модули для реального робота, а так же общие модули представлены в пакете rosbot.

- BringUp. 
```bash
# Запуск основных нод для работы с роботом (в симуляции спаунит робота)
ros2 launch rosbot[_gazebo] bringup.launch.py
```

- Navigation2
```bash
ros2 launch rosbot[_gazebo] nav2.launch.py
```

- RViz
```bash
ros2 launch rosbot rviz.launch.py
```

- Groot
```bash
# Запуск программы для визуализации Behavior Tree
ros2 run groot Groot
```

- Slam
```bash
# BringUp + Navigation2 + SlamToolBox + RViz[optional] 
ros2 launch rosbot[_gazebo] slam.launch.py
```

- Teleop
```bash
# Запуск ноды телеуправления с клавиатуры (wasd) 
ros2 launch rosbot_controller rosbot_sim_keyboard_teleop.launch.py
```
