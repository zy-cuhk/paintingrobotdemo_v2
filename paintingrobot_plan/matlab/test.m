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


% clc; clear all; close all;
% x = rand(10, 3);
% figure;
% quiver3(x(1, 1), x(1, 2), x(1, 3), x(2, 1), x(2, 2), x(2, 3),'r');
% title('By lyqmath', 'FontWeight', 'Bold', 'Color', 'r');

% the the first point is:
% [(-1.63167865233561-2.21760258294989)/2,(1.67985990789396+1.80906187201800)/2,0,0,0,1.35375981662403]
% % the second point is:
% [(-1.63167865233561-2.21760258294989)/2,(1.67985990789396+1.80906187201800)/2,0,0,0,2.921908618914288]
% % the third point is:
% [(-3.52722114340887-2.16550978489986)/2,(-0.685469730917361-0.980230597842753)/2,0,0,0,2.921908618914288]
% % the fourth point is:
% [(-3.52722114340887-2.16550978489986)/2,(-0.685469730917361-0.980230597842753)/2,0,0,0,-1.77676520257574]
% 
% %% point 1:
% [-1.9246    1.7445         0         0         0    1.3538]
% %% point 2:
% [-1.9246    1.7445         0         0         0    2.9219]
% %% point 3:
% [-2.8464   -0.8329         0         0         0    2.9219]
% %% point 4:
% [ -2.8464   -0.8329         0         0         0   -1.7768]


% p1=[-1.56733316609568,2.77277439002086,0];
% p2=[-1.68718986043678,2.23599289553580,0];
% p1+(p2-p1)/0.55*0.04
% 
% p3=[-2.60245916267794,-1.86306578962291,0];
% p4=[-2.63852214390535,-2.02457484062128,0];
% p4+(p3-p4)/0.55*0.04
% 
% p5=[-3.47827797872251,-0.695695513551658,0,0,0,-1.77676520257574];



offset_length=0.04
p1=[-1.56733316609568,2.77277439002086,0,0,0,2.92190861891429]
theta1=2.92190861891429
p2=[p1(1)+0.04*cos(theta1+pi/2),]






















