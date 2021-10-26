# Hdf5 data publisher

## Описание

Этот пакет предназначен для публикации данных, сохраненных в формате hdf5. Публикация выполняется скриптом `hdf5_data_publisher.py`. Cкрипт имеет следующие параметры запуска:

* `path_to_hdf5` (default "default.hdf5") - путь к hgf5 файлу, в котором хранятся данные
* `publish_depth` (default True) - публиковать глубину или нет
* `publish_rgb` (default True) - публиковать картинку или нет
* `publish_pcd` (default True) - публиковать облако точек или нет
* `publish_pose` (default True) - публиковать позицию или нет
* `publish_tf` (default True) - публиковать трансформ odom->base\_link или нет. Работает только при `publish_pose=true`
* `publish_camera_info` (default True) - публиковать параметры камеры или нет
* `depth_topic` (default /depth) - топик, в который публикуется глубина
* `rgb_topic` (default /image) - топик, в который публикуется картинка
* `pcd_topic` (default /points) - топик, в который публикуется облако точек
* `pose_topic` (default /pose) - топик, в который публикуется позиция
* `camera_info_topic` (default /camera_info) - топик, в который публикуется CameraInfo
* `camera_info_file` (default camera_info.yaml) - файл с записанными параметрами камеры (нужно поместить в директорию `config`)
* `fps` (default 30) - частота публикации. Данный скрипт не смотрит на частоту публикации исходных данных, а публикует их с фиксированным fps, заданным этим параметром

Запустить скрипт можно с помощью команды `rosrun`:

`rosrun hdf5_data_publisher hdf5_data_publisher.py _path_to_hdf5:=/path/to/save.hdf5 ...`

Или с помощью команды `roslaunch`:

`roslaunch hdf5_data_publisher oakd.launch/realsense.launch/rosbot_gazebo.launch`

## Лаунчфайлы

**oakd.launch**

Этот файл предназначен для публикации данных из старых росбэгов, записанных на росботе с oakd (https://drive.google.com/drive/folders/1oLtTfYzAlP5XjEvNv6on0yXTZBv4Ypip?usp=sharing). Публикуются картинка с правой камеры, глубина и позиция с трансформом `odom->base_link`

**realsense.launch**

Предназначен для публикации данных из росбэгов, записанных на D455. Публикуются картинка, глубина и облако точек

**rosbot_gazebo.launch**

Предназначен для публикации данных, записанных с модели росбота в Gazebo. Публикуются только облако точек и позиция
