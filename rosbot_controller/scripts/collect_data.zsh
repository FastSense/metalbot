NUM=4
for i in $(seq 1 $NUM)
do
    python3 collect_data.py
    kill $(pidof gzserver)
done
