# drinking-buddy
An automated drink mixing and delivery system for iRobot's Stretch2 robot

## How To Run Our ArUco Demo (Edit once we add other)

1. Run `ros2 launch stretch_core stretch_driver.launch.py mode:=navigation` on Stretch.
2. Run `ros2 launch stretch_core d435i_high_resolution.launch.py` in a seperate terminal.
3. Run `ros2 launch stretch_core stretch_aruco.launch.py` in a seperate terminal.
4. In a seperate terminal, run the command `python3 drinky_buddies/arucoApproach.py`. Ensure that Stretch can see the ArUco marker when you do so!

For debugging, you may find it useful to have RViz show what ArUco markers it can see. To do so, run the command `ros2 run rviz2 rviz2 -d /home/hello-robot/ament_ws/src/stretch_tutorials/rviz/aruco_detector_example.rviz` in (yet another) seperate terminal!
