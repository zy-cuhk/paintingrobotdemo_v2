clc,clear all,close all;
tic;
path3='/data/ros/renov_robot_ws/src/paintingrobot_zy/paintingrobotdemo_v2/paintingrobotdemo_data/second_scan_2/data/second_scan_data1.mat';
path4='/data/ros/renov_robot_ws/src/paintingrobot_zy/paintingrobotdemo_v2/paintingrobotdemo_data/second_scan_2/data/second_scan_data2.mat';

%% obtaining the input
data=load(path3,'room_vertices','room_plane_norm_vector','room_plane_edge_cell','room_plane_edge_centroid','room_plane_triangle_edge_cell','room_plane_triangle_cell');
room_plane_norm_vector1=data.room_plane_norm_vector;
for i=1:1:size(room_plane_norm_vector1,2)
    n1=room_plane_norm_vector1{i}(1,1);
    n2=sign(room_plane_norm_vector1{i}(1,2))*sqrt(1-n1^2);
    n3=0;
    room_plane_norm_vector{i}(1,1)=n1;
    room_plane_norm_vector{i}(1,2)=n2;
    room_plane_norm_vector{i}(1,3)=n3;
end
room_plane_edge_cell=data.room_plane_edge_cell;
room_plane_edge_centroid=data.room_plane_edge_centroid;
room_plane_triangle_edge_cell=data.room_plane_triangle_edge_cell;
room_plane_triangle_cell=data.room_plane_triangle_cell;
room_vertices=data.room_vertices;

%% painting process parameters are listed as follows:
%% the adjustable parameters can be: painting_gun_to_wall_distance and painting_ellipse_long_axis_length
wall2_manipulator_paintinggun_distance=0.31;
painting_ellipse_long_axis_length=0.40;
painting_ellipse_short_axis_length=0.10;
painting_path_interval=painting_ellipse_long_axis_length*2/3;
waypoints_interval=painting_ellipse_short_axis_length/2;

%% generating renovation and mobile base planes 
[renovation_plane_edge_cell,renovation_plane_norm_vector,renovation_plane_triangle_edge_cell]=room_panning_planes_generation(room_plane_norm_vector,room_plane_edge_cell,room_plane_triangle_edge_cell,wall2_manipulator_paintinggun_distance);
manipulator_paintinggun2mobilebase_distance=1.23;
panning_distance2=wall2_manipulator_paintinggun_distance+manipulator_paintinggun2mobilebase_distance;
[manipulatorbase_plane_edge_cell,manipulatorbase_plane_norm_vector,manipulatorbase_plane_triangle_edge_cell]=room_panning_planes_generation(room_plane_norm_vector,room_plane_edge_cell,room_plane_triangle_edge_cell,panning_distance2);
% renovation_planes_visualization(room_plane_edge_cell, renovation_plane_edge_cell,manipulatorbase_plane_edge_cell);

%% generating renovation waypoints and waypaths
[renovation_effective_waypoints,renovation_effective_waypaths,room_plane_boundary,distance_waypoints2wallboundary_direction1,distance_waypoints2wallboundary_direction2]=renovation_planes_waypoint_generation(room_plane_edge_cell,room_plane_norm_vector,room_vertices,room_plane_triangle_cell,waypoints_interval,painting_path_interval,wall2_manipulator_paintinggun_distance);
distance_waypoints2wallboundary_direction1;
distance_waypoints2wallboundary_direction2;

%% visualizing renovation waypoints and waypaths 
% renovation_planes_waypoint_visualization(renovation_effective_waypoints,room_plane_edge_cell,renovation_plane_edge_cell,renovation_effective_waypaths);

%% save matlab data: renovation_effective_waypoints, renovation_effective_waypoints
save(path4,'renovation_effective_waypoints','renovation_effective_waypaths');


toc;


