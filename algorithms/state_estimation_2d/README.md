# 2d State Estiamtion

Оценка состояния росбота на плоскости с использованием Extended Kalman Filter.

https://homes.cs.washington.edu/~todorov/courses/cseP590/readings/tutorialEKF.pdf

## Пример запуска

launch файл: `launch/state_estimation.launch.py`

1. Запустить докер, tmux, собрать workspace, сделать source.
2. Запустить `ros2 launch state_estimation_2d state_estimation.launch.py`

Откроется 2 окна: газебо и rviz2. В rviz будет показываться оценка текущего состояния робота, и ground truth состояние. Управление с клавиатуры с помощью WASD (не важно, какое окно открыто). В терминал будет выводиться ATE (absolute translation error) за время с начала запуска.
