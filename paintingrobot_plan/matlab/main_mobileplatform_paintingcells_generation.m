clc,clear all,close all;
tic;

path5='/data/ros/renov_robot_ws/src/paintingrobotdemo_v2/paintingrobotdemo_data/scan_guangtian/data/second_scan_data2.mat';
path6='/data/ros/renov_robot_ws/src/paintingrobotdemo_v2/paintingrobotdemo_data/scan_guangtian/data/second_scan_data3.mat';

% path5='/Users/zhouyang/Desktop/github_packages/paintingrobotdemo_v2/paintingrobotdemo_data/scan_guangtian/data/second_scan_data2.mat';
% path6='/Users/zhouyang/Desktop/github_packages/paintingrobotdemo_v2/paintingrobotdemo_data/scan_guangtian/data/second_scan_data3.mat';

data=load(path5,'renovation_effective_waypoints','renovation_effective_waypaths','manipulatorbase_plane_edge_cell','renovation_plane_norm_vector','renovation_plane_edge_cell','painting_path_interval','room_plane_edge_cell');
renovation_effective_waypoints=data.renovation_effective_waypoints; 
renovation_effective_waypaths=data.renovation_effective_waypaths;
manipulatorbase_plane_edge_cell=data.manipulatorbase_plane_edge_cell;
renovation_plane_norm_vector=data.renovation_plane_norm_vector;
renovation_plane_edge_cell=data.renovation_plane_edge_cell;
room_plane_edge_cell=data.room_plane_edge_cell;
path_interval=data.painting_path_interval;

%% cell parameters are shown as follows
cell_length=0.60;
length_interval=cell_length;
path_distance=path_interval;


for i=1:1:size(renovation_plane_norm_vector,2)
    a=renovation_plane_norm_vector{i}(1,1);
    b=renovation_plane_norm_vector{i}(1,2);
    c=renovation_plane_norm_vector{i}(1,3);
    x0=manipulatorbase_plane_edge_cell{i}(1,1);
    y0=manipulatorbase_plane_edge_cell{i}(1,2);
    z0=manipulatorbase_plane_edge_cell{i}(1,3);
    d=-a*x0-b*y0-c*z0;
    renovation_manipulatorbase_planes{i}(1,1)=a;
    renovation_manipulatorbase_planes{i}(1,2)=b;
    renovation_manipulatorbase_planes{i}(1,3)=d;
end

%% obtain two types of norm vector of intersection plane norm vector for the renovation planes edges
for i=1:1:size(renovation_plane_norm_vector,2)
    renovation_planes_point(i,1:3)=renovation_plane_edge_cell{i}(1,1:3);
    renovation_planes_parameters(i,1:3)=renovation_plane_norm_vector{i}(1,1:3);
    renovation_planes_parameters(i,4)=-renovation_plane_norm_vector{i}(1,1:3)*renovation_planes_point(i,1:3)';
end
intersect_plane_norm_vector=zeros(size(renovation_plane_norm_vector,2),3);
intersect_plane_norm_vector1=zeros(size(renovation_plane_norm_vector,2),3);
for i=1:1:size(intersect_plane_norm_vector,1)
    sin_theta=-renovation_planes_parameters(i,1);
    cos_theta=sqrt(1-sin_theta^2);
    if cos_theta~=0
        sin_beta=renovation_planes_parameters(i,2)/cos_theta;
        cos_beta=renovation_planes_parameters(i,3)/cos_theta;
    else
        sin_beta=0;
        cos_beta=1;
    end
    intersect_plane_norm_vector1(i,1:3)=[cos_theta, sin_theta*sin_beta, sin_theta*cos_beta];
end


%% obtain dmin and dmax of intersection plane for the renovation planes edges.
for i=1:1:size(renovation_plane_edge_cell,2)
    for j=1:1:size(renovation_plane_edge_cell{i},1)
        renovation_planes_edge_points{i}(2*j-1,1:3)=renovation_plane_edge_cell{i}(j,1:3);
        renovation_planes_edge_points{i}(2*j,1:3)=renovation_plane_edge_cell{i}(j,4:6);
    end
    renovation_planes_edge_points{i}=unique(renovation_planes_edge_points{i},'rows');
end

for i=1:1:size(renovation_planes_edge_points,2)
    a=renovation_plane_norm_vector{i}(1,1);
    b=renovation_plane_norm_vector{i}(1,2);
    a1=intersect_plane_norm_vector1(i,1);
    b1=intersect_plane_norm_vector1(i,2);
    intersect_plane_norm_vector(i,1:3)=intersect_plane_norm_vector1(i,1:3);
    
    for j=1:1:size(renovation_planes_edge_points{i},1)
        intersect_plane_d_candidate{i}(1,j)=-intersect_plane_norm_vector(i,1:3)*renovation_planes_edge_points{i}(j,1:3)';
    end
    intersect_plane_dmin_max{i}(1,1)=min(intersect_plane_d_candidate{i});
    intersect_plane_dmin_max{i}(1,2)=max(intersect_plane_d_candidate{i});
end

for i=1:1:size(intersect_plane_dmin_max,2)
    dmin=intersect_plane_dmin_max{i}(1,1);
    dmax=intersect_plane_dmin_max{i}(1,2);
    plane_num=floor(abs(dmax-dmin)/length_interval);
    if plane_num==0
        intersect_plane_d{i}(1,1)=dmin+length_interval;
    else
        for j=1:1:plane_num
            intersect_plane_d{i}(1,j)=dmin+j*length_interval;
        end
    end
end



%% gather renovation cells edges into renovation cells, the output is:renovation_cells_waypaths
for i=1:1:size(intersect_plane_d,2)
    cell_num=1;
    h_num=size(intersect_plane_d{i},2)+1;
    for j=1:1:h_num
        renovation_cells_edges_num=1;
        for n=1:1:size(renovation_effective_waypaths{i},1)
            p1=renovation_effective_waypaths{i}(n,1:3);
            p1_d1=-intersect_plane_norm_vector(i,1:3)*p1';
            p2=renovation_effective_waypaths{i}(n,4:6);
            p2_d1=-intersect_plane_norm_vector(i,1:3)*p2';
            flag1=0;
            flag_1=0;
            if j==1
                if p1_d1<=intersect_plane_d{i}(1,j)-0.01 && p2_d1<=intersect_plane_d{i}(1,j)-0.01 && p1_d1>=intersect_plane_dmin_max{i}(1,1)+0.05 && p2_d1>=intersect_plane_dmin_max{i}(1,1)+0.05
                    flag1=1;
                    flag_1=1;
                end
            end
            if j~=size(intersect_plane_d{i},2)+1
                if j~=1
                    if p1_d1>=intersect_plane_d{i}(1,j-1)-0.01 && p1_d1<=intersect_plane_d{i}(1,j)+0.01 && p2_d1>=intersect_plane_d{i}(1,j-1)-0.01 && p2_d1<=intersect_plane_d{i}(1,j)+0.01
                        flag1=1;
                        flag_1=2;
                    end
                end
            end
            if j==size(intersect_plane_d{i},2)+1
                if p1_d1>=intersect_plane_d{i}(1,j-1)+0.01 && p2_d1>=intersect_plane_d{i}(1,j-1)+0.01 && p1_d1<=intersect_plane_dmin_max{i}(1,2)-0.05 && p2_d1<=intersect_plane_dmin_max{i}(1,2)-0.05
                    flag1=1;
                    flag_1=2;
                end
            end
            if flag1==1
                renovation_cells_waypaths{i}{j}(renovation_cells_edges_num,1:6)=renovation_effective_waypaths{i}(n,1:6);
                renovation_cells_edges_num=renovation_cells_edges_num+1;
            end
            
        end
    end
end

%% 
for i=1:1:size(renovation_cells_waypaths,2)
    for j=1:1:size(renovation_cells_waypaths{i},2)
        intersection_line{i}(j,1)=renovation_plane_norm_vector{i}(1,1);
        intersection_line{i}(j,2)=renovation_plane_norm_vector{i}(1,2);
        dmin=intersect_plane_dmin_max{i}(1,1);
        intersection_line{i}(j,3)=dmin+(j-0.5)*length_interval;
    end
end

for i=1:1:size(intersection_line,2)
    for j=1:1:size(intersection_line{i},1)
        a1=renovation_manipulatorbase_planes{i}(1,1);
        b1=renovation_manipulatorbase_planes{i}(1,2);
        c1=renovation_manipulatorbase_planes{i}(1,3);
        
        a2=intersect_plane_norm_vector1(i,1);
        b2=intersect_plane_norm_vector1(i,2);
        
        dmin=intersect_plane_dmin_max{i}(1,1);
        dmax=intersect_plane_dmin_max{i}(1,2);
        c2=dmin+(j-0.5)*length_interval;
        
        %% the modification position for determining the mobile base positions
        if i==2 && j==1
            c2=dmin+0.35;
        end
        if i==2 && j==size(intersection_line{i},1)
            c2=dmax-0.40;
        end
        if i==3 && j==1
            c2=dmax-0.40;
        end
        
        renovation_horizontalcells_mobilebase_points{i}(j,1)=(c2*b1-c1*b2)/(a1*b2-a2*b1);
        renovation_horizontalcells_mobilebase_points{i}(j,2)=(c1*a2-c2*a1)/(a1*b2-a2*b1);
        renovation_horizontalcells_mobilebase_points{i}(j,3)=0;
    end
end

%% add orientations into renovation path waypoints
theta_x=-pi/2;
theta_z=pi/2;
for i=1:1:size(renovation_plane_norm_vector,2)
    nx=renovation_plane_norm_vector{i}(1,1);
    ny=renovation_plane_norm_vector{i}(1,2);
    nz=renovation_plane_norm_vector{i}(1,3);
    sin_theta_y=nx;
    cos_theta_y=ny;
    if sin_theta_y>=0
        if cos_theta_y>=0
            theta_y(i)=asin(sin_theta_y);
        else
            theta_y(i)=pi-asin(sin_theta_y);
        end
    else
        if cos_theta_y>=0
            theta_y(i)=2*pi-asin(abs(sin_theta_y));
        else
            theta_y(i)=pi+asin(abs(sin_theta_y));
        end
    end
    renovation_waypaths_orientation{i}(1,1:3)=[theta_x,theta_y(i),theta_z];
end



for i=1:1:size(renovation_plane_norm_vector,2)
    nx=renovation_plane_norm_vector{i}(1,1);
    ny=renovation_plane_norm_vector{i}(1,2);
    nz=renovation_plane_norm_vector{i}(1,3);
    base_theta_z(i)=atan2(ny,nx);
end 

for i=1:1:size(renovation_cells_waypaths,2)
    for j=1:1:size(renovation_cells_waypaths{i},2)
        renovation_cells_mobilebase_positions{i}(j,1)=renovation_horizontalcells_mobilebase_points{i}(j,1);
        renovation_cells_mobilebase_positions{i}(j,2)=renovation_horizontalcells_mobilebase_points{i}(j,2);
        renovation_cells_mobilebase_positions{i}(j,3)=0;
        renovation_cells_mobilebase_positions{i}(j,4)=0;
        renovation_cells_mobilebase_positions{i}(j,5)=0;
        renovation_cells_mobilebase_positions{i}(j,6)=base_theta_z(i);
    end
end

% renovation_cells_waypath_visualization(renovation_cells_waypaths,renovation_cells_mobilebase_positions,renovation_plane_edge_cell,room_plane_edge_cell);

toc;


%% the selected renovation cells are shown as follows:
% renovation_cells_mobilebase_positions{i}(:,1:6)
% renovation_cells_waypaths{i}{j}(:,1:6)
% renovation_waypaths_orientation{i}(1,1:3)

renovation_cells_mobilebase_positions1{1}=renovation_cells_mobilebase_positions{1};
% renovation_cells_mobilebase_positions1{2}(1,1:6)=[-1.9246,1.7445,0,0,0,1.3538];
renovation_cells_mobilebase_positions1{2}(1,1:6)=[-1.9246,1.7445,0,0,0,2.9219];
renovation_cells_mobilebase_positions1{3}=renovation_cells_mobilebase_positions{2};
% renovation_cells_mobilebase_positions1{4}(1,1:6)=[-2.8464,-0.8329,0,0,0,2.9219];
renovation_cells_mobilebase_positions1{4}(1,1:6)=[-2.8464,-0.8329,0,0,0,-1.7768];
renovation_cells_mobilebase_positions1{5}=renovation_cells_mobilebase_positions{3};

renovation_cells_waypaths1{1}=renovation_cells_waypaths{1};
renovation_cells_waypaths1{2}={};
renovation_cells_waypaths1{3}=renovation_cells_waypaths{2};
renovation_cells_waypaths1{4}={};
renovation_cells_waypaths1{5}=renovation_cells_waypaths{3};


%% the modification of the last plane's traversing sequence
for i=1:1:size(renovation_cells_mobilebase_positions{4},1)
    renovation_cells_mobilebase_positions1{6}(i,:)=renovation_cells_mobilebase_positions{4}(size(renovation_cells_mobilebase_positions{4},1)-i+1,:);
    renovation_cells_waypaths1{6}{i}=renovation_cells_waypaths{4}{size(renovation_cells_mobilebase_positions{4},1)-i+1};
end


renovation_waypaths_orientation1{1}=renovation_waypaths_orientation{1};
renovation_waypaths_orientation1{2}={};
renovation_waypaths_orientation1{3}=renovation_waypaths_orientation{2};
renovation_waypaths_orientation1{4}={};
renovation_waypaths_orientation1{5}=renovation_waypaths_orientation{3};
renovation_waypaths_orientation1{6}=renovation_waypaths_orientation{4};


save(path6,'renovation_cells_waypaths1','renovation_cells_mobilebase_positions1','renovation_waypaths_orientation1');
% renovation_cells_waypath_visualization(renovation_cells_waypaths,renovation_cells_mobilebase_positions1,renovation_plane_edge_cell,room_plane_edge_cell);











