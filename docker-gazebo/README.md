# Docker

How to work in docker:

Build docker:
```bash
./build.sh
```

Run docker:
```bash
GZ_DOCKER=$(./run.sh)
```

Attach to this docker:
```bash
docker attach $GZ_DOCKER
```

Recommended to use tmux inside docker:
```bash
tmux
```

Take a look at [tmux shortcuts & cheatsheet](https://gist.github.com/MohamedAlaa/2961058).
