# **state_estimation_2d**
ROS2 нода для оценки состояния колесной платформы. 

### **Предустановка**

Запускается внутри докера rosbot2.

1. Установить питоновские библиотеки:
```
pip install filterpy
sudo pip uninstall onnxruntime-gpu
pip install onnxruntime
```

2. Перейти в папку ros2_ws и сбилдить.
    ```
    cd ros2_ws/
    colcon build
    ```
3.    ```source install/setup.zsh ```

### **Запуск**
```ros2 launch state_estimation_2d state_estimation.launch.py```
