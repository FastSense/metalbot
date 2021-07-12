
# **Запуск симулятора с rosbot**

```bash
ros2 launch rosbot_controller rosbot_sim.launch.py
```


# **state_estimation_2d**
ROS2 нода для оценки состояния колесной платформы. 

### **Предустановка**

Запускается внутри докера rosbot2.

(!) В каждом новом терминале докера необходимо прописывать:
```
export ROS_DOMAIN_ID='уникальный id'
```
Занятые ID:
- 10 : Эд
- 11 : Костя
- 12 : Руслан
- 20 : Rosbot 

1. Установить питоновские библиотеки:
```
sudo pip uninstall onnxruntime-gpu
pip install onnxruntime
```

2. Перейти в папку ros2_ws и сбилдить.
```
cd ros2_ws/
colcon build
```
3. ```source install/setup.zsh ```

### **Запуск**
```ros2 launch state_estimation_2d state_estimation.launch.py```
