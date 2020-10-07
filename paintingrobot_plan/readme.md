# The applied planning algorithm before 202007 is shown as follows:
1. The generation and visualization of planning result: roslaunch painting_robot_demo paintingrobot_states_visualization
2. The applied python files include:
2.1. coverage_planning_offline_farubim.py for generating planning result
2.2. paintingrobot_planningresult_visualization.py for visualizing planning result 


# workspace based planning for painting robot aims to generate below variables:
1. mobile platform positions based on the defined effective workspace
2. rod climbing mechanism positions and manipulator trajectories for coverage painting planning inside each defined effective workspace 
for point 2, the algorithm is a novel rtsp algorithm, and
input: renovation_cell_waypaths, renovation_cell_mobileplatform_positions
output: climbing_mechanism and manipulator_joints   



# the intial framework for our problem  
# while(1):
    # 1. sample climbing mechanism joints 
    # 2. for each climbing mechanism joints
        # 2.1 obtain the corresponding sampled manipulator base positions based on the kinematic model
        # 2.2 obtain the waypaths inside the workspace of sampled manipulator base positions with octotree
        # 2.3 connect these waypaths with Cartesian-space tsp
        # 2.4 obtain the joint-space tsp solver
    # 3. compute the motion cost combining climbing mechanism and manipulator 
    # 4. compute the painting awards
    # 5. based on step 3 and step 4, the joint list is computed.
    # the painting cost is the motion cost 
    # the painting award is the net painting area 

# the modified framework for our problem  
# while(1):
    # 1. sample climbing mechanism joints 
    # 2. for each climbing mechanism joints
        # 2.1 obtain the corresponding sampled manipulator base positions based on the kinematic model
        # 2.2 select the waypaths inside the workspace of sampled manipulator base positions 
        # 2.3 select again to obtain the waypaths on which waypoints has colision-free inverse kinematic solutions [] 
        # 2.3 connect these selected waypaths with Cartesian-space tsp solver
        # 2.4 obtain the joint-space tsp solver
    # 3. compute the motion cost combining climbing mechanism and manipulator 
    # 4. compute the painting awards
    # 5. based on step 3 and step 4, the joint list is computed.
    # the painting cost is the motion cost 
    # the painting award is the net painting area 


