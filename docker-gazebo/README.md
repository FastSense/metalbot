# Docker

How to work in docker:

Build docker:
```bash
./build.sh
```

Run docker:
```bash
./run.sh
```

Attach to this docker:
```bash
docker attach rosbot2
```

Recommended to use tmux inside docker:
```bash
tmux
```

Take a look at [tmux shortcuts & cheatsheet](https://gist.github.com/MohamedAlaa/2961058).

Run RosBot2.0
```bash
docker attach $GZ_DOCKER
tmux
cd ros2_ws
colcon build
source ~/ros2_ws/install/setup.zsh
ros2 launch rosbot_description rosbot_sim.launch.py
```
