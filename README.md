# lbot_ws
lbot for llm vlm navigation
The initial implementation uses YOLO based detection module(trained on COCO data set) in lieu of VLM to detection and segmentation.
This is WIP and functionalty is not complete - i.e. there are erros in mapping pose in the camera frame into world map frame, etc.
Segmentation is not marked in th occupancy map  in real time - it just keeps dictionary of name and the coordinats of teh objects detected.


Here is brief explanation of whta it does
1. The simulation starts in a house environment in a gazebo simulation
2. tb3 needs to explore the entire house manually using nav2goal gui on rviz
3. Once tb3 finish exploring, detecteed objects are add to dictionay with there coordinates and saved into my_file.npy
4. User can give the name of the object to which tb3 needs to navigate to.
5. lbot_misc_node loads my_file.npy and query dictionary if the object exist in it.
6. Once it finds the object, it gets its coordinates and send it to \goal_pose topic
7. tb3 navigates to the goal pose

Here is how to run it - it is assumed that ROS2 Humble is installed in the default directory. NAV2 pacakge and RTAP SLAM was installed in the default directory.

1. Download the code
2. Run colcon build, run . install/setup.bash in the workspace with every terminal 
3. it needs 3 terminal windows.
   a. open the first window and type "ros2 launch lbot_control two_launchy.py" - this starts tb3 in gazebo, navigatio2 package and rtap slam
   b. open the second window and type "ros2 launch lbot_control lbot_control_launch.py" - this starts detector module
   c. open the third window and type "ros2 run lbot_misc_node name_of_object - this will make the tb3 navigate to the object if it was detected before

   

