function renovation_cells_waypath_visualization(renovation_cells_waypaths,renovation_cells_mobilebase_positions,renovation_plane_edge_cell,room_plane_edge_cell)

%% drawing all data
figure;

for i=1:1:size(renovation_cells_waypaths,2)
    for j=1:1:size(renovation_cells_waypaths{i},2)
% for i=2:1:2
%     for j=1:1:1
        for m=1:1:size(renovation_cells_waypaths{i}{j},1)-1
            xlist=[renovation_cells_waypaths{i}{j}(m,1),renovation_cells_waypaths{i}{j}(m,4)];
            ylist=[renovation_cells_waypaths{i}{j}(m,2),renovation_cells_waypaths{i}{j}(m,5)];
            zlist=[renovation_cells_waypaths{i}{j}(m,3),renovation_cells_waypaths{i}{j}(m,6)];
            plot3(xlist,ylist,zlist,'b','LineWidth',1);
            hold on;
%             x1=renovation_cells_waypaths{i}{j}(m,1);
%             y1=renovation_cells_waypaths{i}{j}(m,2);
%             z1=renovation_cells_waypaths{i}{j}(m,3);
%             hold on;
%             x2=renovation_cells_waypaths{i}{j}(m,4);
%             y2=renovation_cells_waypaths{i}{j}(m,5);
%             z2=renovation_cells_waypaths{i}{j}(m,6);
%             scatter3(x2,y2,z2);
%             hold on;
        end
        axis equal;
    end
end

% for i=1:1:size(renovation_cells_waypaths,2)
%     num=1;
%     for j=1:1:size(renovation_cells_waypaths{i},2)
%         for m=1:1:size(renovation_cells_waypaths{i}{j},1)
%             renovation_cells_waypoints{i}(num,1:3)=renovation_cells_waypaths{i}{j}(m,1:3);
%             num=num+1;
%             renovation_cells_waypoints{i}(num,1:3)=renovation_cells_waypaths{i}{j}(m,4:6);
%             num=num+1;
%         end
%     end
%     point_x=renovation_cells_waypoints{i}(:,1);
%     point_y=renovation_cells_waypoints{i}(:,2);
%     point_z=renovation_cells_waypoints{i}(:,3);
%     scatter3(point_x,point_y,point_z);
%     hold on;
% end

for i=1:1:size(renovation_cells_mobilebase_positions,2)
    point_x=renovation_cells_mobilebase_positions{i}(:,1);
    point_y=renovation_cells_mobilebase_positions{i}(:,2);
    point_z=renovation_cells_mobilebase_positions{i}(:,3);
    scatter3(point_x,point_y,point_z);
    hold on;
end
% for i=1:1:size(renovation_cells_manipulatorbase_positions,2)
%     point_x=renovation_cells_manipulatorbase_positions{i}(:,1);
%     point_y=renovation_cells_manipulatorbase_positions{i}(:,2);
%     point_z=renovation_cells_manipulatorbase_positions{i}(:,3);
%     scatter3(point_x,point_y,point_z);
%     hold on;
% end

for i=1:1:size(room_plane_edge_cell,2)
    for j=1:1:size(room_plane_edge_cell{i},1)
        xlabel("x axis");
        ylabel("y axis");
        zlabel("z axis");
        title('3D model of interior surfaces framework','FontSize',24);
        x1=[room_plane_edge_cell{i}(j,1),room_plane_edge_cell{i}(j,4)];
        y1=[room_plane_edge_cell{i}(j,2),room_plane_edge_cell{i}(j,5)];
        z1=[room_plane_edge_cell{i}(j,3),room_plane_edge_cell{i}(j,6)];
        plot3(x1 ,y1,z1,'b','LineWidth',1);
        axis equal;
        view(-114,24);
        hold on;
    end
end
hold on;
for i=1:1:size(renovation_plane_edge_cell,2)
    for j=1:1:size(renovation_plane_edge_cell{i},1)
        xlabel("x axis");
        ylabel("y axis");
        zlabel("z axis");
        title('3D model of interior surfaces framework','FontSize',24);
        x1=[renovation_plane_edge_cell{i}(j,1),renovation_plane_edge_cell{i}(j,4)];
        y1=[renovation_plane_edge_cell{i}(j,2),renovation_plane_edge_cell{i}(j,5)];
        z1=[renovation_plane_edge_cell{i}(j,3),renovation_plane_edge_cell{i}(j,6)];
        plot3(x1,y1,z1,'r','LineWidth',1);
        axis equal;
        view(-114,24);
        hold on;
    end
    hold on;
end
% hold on;

end