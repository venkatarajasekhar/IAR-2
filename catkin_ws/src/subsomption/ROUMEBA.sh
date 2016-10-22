roslaunch fastsim fastsim.launch&
wait 0.1
python spirale_left.py& # channel 0
wait 0.1
python reculer.py& # channel 8
wait 0.1
python avoid_walls.py& # channel 2
wait 0.1
python random_angle.py& # channel 1
wait 0.1
python subsomption_architecture.py 10 &