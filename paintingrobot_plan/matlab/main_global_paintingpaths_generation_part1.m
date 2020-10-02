clc,clear all,close all;
tic;
path2='/data/ros/renov_robot_ws/src/paintingrobotdemo_v2/paintingrobotdemo_data/scan_guangtian/data/second_scan_data1.mat';

%% generating room planes of interior environment
path1_1="/data/ros/renov_robot_ws/src/paintingrobotdemo_v2/paintingrobotdemo_data/scan_guangtian/wall1.stl";
[house_facet,house_vertices,house_norm_vector]=house_stl_reading(path1_1);
house_stl_matplot(house_facet,house_vertices,house_norm_vector);
[room_facet,room_vertices,room_norm_vector]=vertices_processing(house_facet,house_vertices,house_norm_vector);
room_stl_matplot(room_facet,room_vertices,room_norm_vector);
[room_plane_norm_vector1,room_plane_edge_cell1,room_plane_edge_centroid1,room_plane_triangle_cell1,room_plane_triangle_edge_cell1]=room_planes_generation(room_facet,room_vertices,room_norm_vector);
room_vertices1=room_vertices;

path1_2="/data/ros/renov_robot_ws/src/paintingrobotdemo_v2/paintingrobotdemo_data/scan_guangtian/wall2.stl";
[house_facet,house_vertices,house_norm_vector]=house_stl_reading(path1_2);
house_stl_matplot(house_facet,house_vertices,house_norm_vector);
[room_facet,room_vertices,room_norm_vector]=vertices_processing(house_facet,house_vertices,house_norm_vector);
room_stl_matplot(room_facet,room_vertices,room_norm_vector);
[room_plane_norm_vector2,room_plane_edge_cell2,room_plane_edge_centroid2,room_plane_triangle_cell2,room_plane_triangle_edge_cell2]=room_planes_generation(room_facet,room_vertices,room_norm_vector);
room_vertices2=room_vertices;

path1_3="/data/ros/renov_robot_ws/src/paintingrobotdemo_v2/paintingrobotdemo_data/scan_guangtian/wall3.stl";
[house_facet,house_vertices,house_norm_vector]=house_stl_reading(path1_3);
house_stl_matplot(house_facet,house_vertices,house_norm_vector);
[room_facet,room_vertices,room_norm_vector]=vertices_processing(house_facet,house_vertices,house_norm_vector);
room_stl_matplot(room_facet,room_vertices,room_norm_vector);
[room_plane_norm_vector3,room_plane_edge_cell3,room_plane_edge_centroid3,room_plane_triangle_cell3,room_plane_triangle_edge_cell3]=room_planes_generation(room_facet,room_vertices,room_norm_vector);
room_vertices3=room_vertices;

path1_4="/data/ros/renov_robot_ws/src/paintingrobotdemo_v2/paintingrobotdemo_data/scan_guangtian/wall4_2.stl";
[house_facet,house_vertices,house_norm_vector]=house_stl_reading(path1_4);
house_stl_matplot(house_facet,house_vertices,house_norm_vector);
[room_facet,room_vertices,room_norm_vector]=vertices_processing(house_facet,house_vertices,house_norm_vector);
room_stl_matplot(room_facet,room_vertices,room_norm_vector);
[room_plane_norm_vector4,room_plane_edge_cell4,room_plane_edge_centroid4,room_plane_triangle_cell4,room_plane_triangle_edge_cell4]=room_planes_generation(room_facet,room_vertices,room_norm_vector);
room_vertices4=room_vertices;
room_plane_norm_vector4{1}(1,2)=-room_plane_norm_vector4{1}(1,2);

room_plane_norm_vector{1}=room_plane_norm_vector1{1};
room_plane_norm_vector{2}=room_plane_norm_vector2{1};
room_plane_norm_vector{3}=room_plane_norm_vector3{1};
room_plane_norm_vector{4}=room_plane_norm_vector4{1};

room_plane_edge_cell{1}=room_plane_edge_cell1{1};
room_plane_edge_cell{2}=room_plane_edge_cell2{1};
room_plane_edge_cell{3}=room_plane_edge_cell3{1};
room_plane_edge_cell{4}=room_plane_edge_cell4{1};

room_plane_edge_centroid{1}=room_plane_edge_centroid1{1};
room_plane_edge_centroid{2}=room_plane_edge_centroid2{1};
room_plane_edge_centroid{3}=room_plane_edge_centroid3{1};
room_plane_edge_centroid{4}=room_plane_edge_centroid4{1};

room_plane_triangle_cell{1}=room_plane_triangle_cell1{1};
room_plane_triangle_cell{2}=room_plane_triangle_cell2{1};
room_plane_triangle_cell{3}=room_plane_triangle_cell3{1};
room_plane_triangle_cell{4}=room_plane_triangle_cell4{1};

room_plane_triangle_edge_cell{1}=room_plane_triangle_edge_cell1{1};
room_plane_triangle_edge_cell{2}=room_plane_triangle_edge_cell2{1};
room_plane_triangle_edge_cell{3}=room_plane_triangle_edge_cell3{1};
room_plane_triangle_edge_cell{4}=room_plane_triangle_edge_cell4{1};

room_vertices={};
room_vertices{1}=room_vertices1{1};
room_vertices{2}=room_vertices2{1};
room_vertices{3}=room_vertices3{1};
room_vertices{4}=room_vertices4{1};

save(path2,'room_vertices','room_plane_norm_vector','room_plane_edge_cell','room_plane_edge_centroid','room_plane_triangle_edge_cell','room_plane_triangle_cell');

% save(path2,'room_vertices2','room_plane_norm_vector2','room_plane_edge_cell2','room_plane_edge_centroid2','room_plane_triangle_edge_cell','room_plane_triangle_cell');

toc;


