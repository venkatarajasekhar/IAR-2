roslaunch fastsim fastsim.launch&
wait 0.1
python spirale_left.py&
wait 0.1
python reculer.py&
wait 0.1
python avoid_walls.py&
wait 0.1
python subsomption_architecture.py 10 &