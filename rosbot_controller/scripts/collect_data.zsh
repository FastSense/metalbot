NUM=4
for i in $(seq 1 $NUM)
do
    # ros2 launch rosbot_description rosbot_sim.launch.py world:=/home/user/empty gui:=false rosbot_update_rate:=30 &
    python3 collect_data.py
    kill $(pidof gzserver)
done
