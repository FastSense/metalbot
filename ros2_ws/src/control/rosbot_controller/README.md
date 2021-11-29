# rosbot_controller package
Пакет содержащий ноды для управления росботом, а также скрипты для сбора автоматиированного сбора данных.

## control_generator node

Нода котоая генерирует последовательность управления по заданным параметрам. 
Пример Запуска:
```
ros2 launch rosbot_controller control_gen.launch.py
```
Аргументы
* mode - режим работы: periodic-генерирует управление, from_file-публикует управление из файла
* num_of_subs - количество требуемых подписчиков
* control_topic - имя топика для команд управлени
* pub_rate - частота публикации управления (Гц)

При mode == periodic
* Tmax - Длительность проезда
* period_lin - период изменения линейной скорости
* period_ang - период изменения угловой скорости
* v_min - минимальная линейная скорость
* v_max - макисмальная линейная скорость
* w_min - минимальная угловая скорость
* w_max - макисмальная угловая скорость
* a_lin - линейное ускорение
* a_ang - угловое ускорение

При mode == from_file
* file_path - путь до файла с управлением

## rosbot_teleop node
Нода которая интерпретирует команды от клавиатуры (топик /keyboard) в команды управления для Rosbot.
Клавиши для управления: W, A, S, D, Shift, Space
W - вперед
S - назад
A - влево
D - вправо
Shift - ускоение увеличивается в два раза
Space - моментальная остановка

Пример запуска:
```
ros2 launch rosbot_controller rosbot_teleop.launch.py
```
Aргументы
* update_rate - частота публикации управления

## script collect_data.py
Скрипт который дважды запускает control_generator с рандмными параметрами для генерации управления и ноду для логирования logger. Первый проезд параметры генерируется с нормальным распределением. После первого проезда  угловое ускорение меняет знак (ang_acc = -ang_acc) и запускается второй проезд. Таким образом получается две зеркальные траектории.
Пример запуска:

```
python3 collect_data.py
```


## script collect_data.zsh
Данный скрипт запускает collect_data.py в цикле, для автоматизированного сбора данных.

Пример запуска:
```
cd scripts 
./collect_data.zsh
```

## simple_controller
Данный лаунчер запускает два лаунчера (path_publisher и path_follower): path_publisher публикуют заданный путь в топик /path, path_follower читает этот путь из /path, затем публикует управление росботом по этому пути.

Пример запуска:
```
ros2 launch rosbot_controller simple_controller.launch.py traj_type:=@param1 move_plan:=@param2 vel_coeff:=@param3 ang_vel_coeff:=@param3 reverse=param4
```
Aргументы:
* traj_type --- (Str) тип траектории (2.0sin2.0, 4.0spiral, polygon, from_file)
* move_plan --- (Str) путь к файлу, если traj_type=from_file
* vel_coeff --- (Float) множитель линейной скорости робота (1.0 2.0 1.5)
* ang_vel_coeff --- (Float) множитель угловой скорости робота (1.0 2.0 1.5)
* reverse --- (Bool) если True траектория будет обратной
* use_odom --- (Bool)если True поза робота берется из топика одометрии, если False - из топика TF


## model_runner
Данный лаунчер запускает на выбора две модели: кинематическую и нейросетевую.
Визуализация нейросетевой:
```
ros2 launch rosbot_controller model_runner.launch.py model_type:=nn_model nn_model_path:=@PARAM
```
(tf - nn_model_link)

Визуализация кинематической:
```
ros2 launch rosbot_controller model_runner.launch.py model_type:=kinematic
```
(tf - kinematic_model_link)

Аргументы:
* model_type - тип модели: а) Кинематическая (kinematic), б) Нейросетевая (nn_model)
* nn_model_path - путь к весам нейронной сети (.onnx)
* cmd_freq - частота публикации информации о положении (default = 60)
* parent_frame - фрейм, из которого формируется сообщение в tf (default="odom")
* control_topic - топик, из которого читаются скорости