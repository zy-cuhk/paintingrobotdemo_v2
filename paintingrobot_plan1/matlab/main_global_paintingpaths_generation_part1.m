clc,clear all,close all;
tic;
path1="/data/ros/renov_robot_ws/src/paintingrobot_zy/paintingrobotdemo_v2/paintingrobotdemo_data/second_scan_2/second_scan_2.stl";
path2='/data/ros/renov_robot_ws/src/paintingrobot_zy/paintingrobotdemo_v2/paintingrobotdemo_data/second_scan_2/data/second_scan_data1.mat';

%% generating room planes of interior environment
[house_facet,house_vertices,house_norm_vector]=house_stl_reading(path1);
house_stl_matplot(house_facet,house_vertices,house_norm_vector);


% [room_facet,room_vertices,room_norm_vector]=vertices_processing(house_facet,house_vertices,house_norm_vector);
% room_stl_matplot(room_facet,room_vertices,room_norm_vector)

%% generating room planes 
% [room_plane_norm_vector,room_plane_edge_cell,room_plane_edge_centroid,room_plane_triangle_cell,room_plane_triangle_edge_cell]=room_planes_generation(room_facet,room_vertices,room_norm_vector);
% save(path2,'room_vertices','room_plane_norm_vector','room_plane_edge_cell','room_plane_edge_centroid','room_plane_triangle_edge_cell','room_plane_triangle_cell');


toc;


