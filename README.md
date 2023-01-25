# Pick and Place with Franka Emika Panda

- Pyramid.py is a script that places 6 small boxes into a pyramid using the Franka Emika Panda manipulator with the MoveIt python interface.

- The script uses some of the functions from the [MoveIt python interface tutorial](https://github.com/ros-planning/moveit_tutorials/blob/melodic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py)

- The script provides the option to sequentially choose which box to place and an option to place all remaining boxes. 

- The Panda is mounted on top of a Clearpath Ridgeback omnidirectional base. The base is stationary for this project.

### To run the script in Rviz

- First, `roslaunch panda_moveit_config demo.launch`

- Then, in another shell `rosrun pyramid pyramid.py`

<p align = "center">
 <img src="https://user-images.githubusercontent.com/20496918/184697670-c8b0e584-da03-41b7-ae52-ba7c8c33ecdf.png" alt="Picture of Panda" class="center" width="500" height="600"> 
 <p/>

