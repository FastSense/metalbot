# ROSbot on ROS2

## Описание
Репозиторий содержит основные модули для работы ROSbot как в симуляции так и на реальном роботе.

<!-- vim-markdown-toc GitLab -->

* [Docker](#docker)
    * [Сборка Docker Image и создание контейнера](#Сборка-docker-image-и-создание-контейнера)
    * [Использование Docker контейнера](#Использование-docker-контейнера)
* [Запуск ROSbot](#Запуск-rosbot)
    * [BringUp](#bringup)
    * [Navigation2](#navigation2)
    * [RViz](#rviz)
    * [Slam](#slam)

<!-- vim-markdown-toc -->

## Docker 
Рекомендуется работать через Docker. 

### Сборка Docker Image и создание контейнера

```
./docker_gazebo/dependencies.sh		        # установка зависимостей на локальную машину
./docker_gazebo/build.sh			# Build image 
./docker_gazebo/run.sh				# Create & Run container
```

### Использование Docker контейнера
```
docker start rosbot2 
docker attach rosbot2 
```

## Запуск ROSbot
Для запуска основных модулей робота существует набор launch фаилов.  
Набор модулей для симуляции представлены в пакете rosbot_gazebo. 
Общие для симуляции и реального робота модули представлены в пакете rosbot.

### BringUp 
Данный лаунч запускае основные ноды для работы с роботом (в симуляции спаунит робота)

```bash
ros2 launch rosbot[_gazebo] bringup.launch.py
```

### Navigation2
```bash
ros2 launch rosbot[_gazebo] nav2.launch.py
```

### RViz
```bash
ros2 launch rosbot rviz.launch.py
```

### Slam
BringUp + Navigation2 + SlamToolBox + RViz[optional] 
```bash
ros2 launch rosbot[_gazebo] slam.launch.py
```

