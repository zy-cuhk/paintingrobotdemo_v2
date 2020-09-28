git clone https://github.com.cnpmjs.org/zy-cuhk/paintingrobotdemo_v2.git


the z limit should be -0.20 instead of -0.17, 3cm must be given 
self.parameterz=rospy.get_param('mat_parameterz')#0.028625
<param name="min_holding_distance" value="-0.15" />
<param name="max_holding_distance" value="0.03" />
<param name="min_climbing_distance" value="-0.17" />
<param name="max_climbing_distance" value="0.90" />
<param name="min_rotation_distance" value="-5.50" />
<param name="max_rotation_distance" value="2.36" />

the modification part includes:
1. z limit should be changed and tested 
2. planing dict about the manipulator planing part should be modified 
(1) the planning result should be tested 
(2) the painting gun control part should be added 


## the experiment steps are shown as follows:
##  for the second version algorithm of paintingrobot_demo, the packages are listed as follows:
1. paintingrobot_description 
2. paintingrobot_moveit_config
3. paintingrobot_plan
4. paintingrobot_control
5. paintingrobot_visualization
6. paintingrobot_navigation
7. scanned_result

# the modified tutorial is shown as follows:

1. Input scan results, which includes:
1.1. 2D map: tif and txt document, which should be similiar to the 2D map generated by 2D laser
1.2. 3D document: xyz document, which should be under 10MB and used for visualization
1.3. STL document, which is used for painting robot planning, stl document only contains target painting surface 

2. Using 2D map for mobile platform navigation,
2.1:  put 2D map into the mobile platform navigation package

3. planning process 
3.1 (execute main_global_paintingpaths_generation_part1.m) in MATLAB
3.2 execute main_global_paintingpaths_generation_part2.m in MATLAB
3.3 execute main_mobileplatform_paintingcells_generation.m in MATLAB
3.3 python painting_climbing_and_manipulator_planner.py in VSCODE 


4. Visualization of planning results, painting scnene, painting robot model with some painting results motion in RVIZ
roslaunch paintingrobot_visualization paintingrobotmotion_simulation.launch



5. Verification of painting robot motion state  
5.0 chmod 777 /tty/USB*
5.1 roslaunch paintingrobot_control usb_port_search.launch, see if usb ports are ok or not
5.2 roslaunch paintingrobot_control paintingrobot_rosdriver.launch, see if paintingrobot ros drivers are ok or not
5.3 roslaunch paintingrobot_control paintingrobot_test.launch
--check the contained python document: "paintingrobot_motion_test.py"



6. Triggerring the motion of painting robot
6.1 roslaunch paintingrobot_control coverage_painting_demo_farubim.launch
6.3 roslaunch paintingrobot_visualization paintingrobotmotion_visualizaiton.launch


the problems of our proposed algorithm 
1. mobile base sequence planning 
2. rod climbing mechanism problem: the planning result is lower than the minimum position of climbing mechanism
3. manipulator path plannning problem should be solved firstly.

step 1: after the manipulator path planning problem is solved out, then test the performance of manipulator and climbing mechanism

step 2: 
