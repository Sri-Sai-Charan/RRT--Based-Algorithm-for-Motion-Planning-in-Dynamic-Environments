Steps to run:

2D RRT*FN:
cd into scripts folder
execute `python3 RRTStarFN.py`

2D RRT*FN-Dynamic:
execute `python3 RRTStarFN_Dynamic.py`

3D RRT*FND:
1. place the folder inside a catkin_ws.
2. catkin_make from root
3. execute `roslaunch astar_turtlebot3 demo.launch`.
4. cd into scripts
5. execute `python3 controller.py`
6. to plot robot path, execute `python3 plot_robot_trajectory.py`
