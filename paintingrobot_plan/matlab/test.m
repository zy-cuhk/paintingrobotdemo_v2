% clc,clear all,close all;
% tic;
% path2='/data/ros/renov_robot_ws/src/paintingrobotdemo_v2/paintingrobotdemo_data/scan_guangtian/data/second_scan_data1.mat';

%% generating room planes of interior environment
% path1_1="/data/ros/renov_robot_ws/src/paintingrobotdemo_v2/paintingrobotdemo_data/scan_guangtian/wall2.stl";
% [house_facet,house_vertices,house_norm_vector]=house_stl_reading(path1_1);
% house_stl_matplot(house_facet,house_vertices,house_norm_vector);
% [room_facet,room_vertices,room_norm_vector]=vertices_processing(house_facet,house_vertices,house_norm_vector);
% room_stl_matplot(room_facet,room_vertices,room_norm_vector);
% room_vertices1=room_vertices;


clc; clear all; close all;
x = rand(10, 3);
figure;
quiver3(x(1, 1), x(1, 2), x(1, 3), x(2, 1), x(2, 2), x(2, 3),'r');
% title('By lyqmath', 'FontWeight', 'Bold', 'Color', 'r');