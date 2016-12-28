[ellip_x, ellip_y]=ellipse_from_Guassian(est_robot_pose(1,k),est_robot_pose(2,k),odometry_covar_mat_temp(1:2,1:2));

xline2_plot=[cos(robot_pose(3,k)) -sin(robot_pose(3,k));
    sin(robot_pose(3,k)) cos(robot_pose(3,k))]*xline2;

xline2_plot2=[cos(est_robot_pose(3,k)) -sin(est_robot_pose(3,k));
    sin(est_robot_pose(3,k)) cos(est_robot_pose(3,k))]*xline2;



%Plot Actual Robot Position
set(handle_circ, 'XData', robot_pose(1,k)+circ_x,'YData', robot_pose(2,k)+circ_y);
set(handle_line, 'XData', robot_pose(1,k)+[xline1(1) xline2_plot(1)], 'YData', robot_pose(2,k)+[xline1(2) xline2_plot(2)]);

%Plot Estimated Robot Position
set(handle_circ2, 'XData', est_robot_pose(1,k)+circ_x,'YData', est_robot_pose(2,k)+circ_y);
set(handle_line2, 'XData', est_robot_pose(1,k)+[xline1(1) xline2_plot2(1)], 'YData', est_robot_pose(2,k)+[xline1(2) xline2_plot2(2)]);

%Plot Uncertainty Ellipse
set(handle_ellip, 'XData', ellip_x, 'YData', ellip_y);


if plot_odo
    [ellip_x2, ellip_y2]=ellipse_from_Guassian(est_robot_pose2(1,k),est_robot_pose2(2,k),odometry_covar_mat_temp2(1:2,1:2));
    xline2_plot3=[cos(est_robot_pose2(3,k)) -sin(est_robot_pose2(3,k));
        sin(est_robot_pose2(3,k)) cos(est_robot_pose2(3,k))]*xline2;
    %Plot Estimated Robot Position -NO Kalman Filter
    set(handle_circ3, 'XData', est_robot_pose2(1,k)+circ_x,'YData', est_robot_pose2(2,k)+circ_y);
    set(handle_line3, 'XData', est_robot_pose2(1,k)+[xline1(1) xline2_plot3(1)], 'YData', est_robot_pose2(2,k)+[xline1(2) xline2_plot3(2)]);
    %  Plot Uncertainty Ellipse-NO Kalman Filter
    %  set(handle_ellip2, 'XData', ellip_x2, 'YData', ellip_y2);
end

%Plot LIDAR Data - Nothing is plot if zmeas==zmax
if (mean(z_meas)<zmax)
    set(handle_lidar,'Visible','on')
    set(handle_lidar, 'XData', est_robot_pose(1,k)+z_meas.*cos(sense_angles+est_robot_pose(3,k)), 'YData', est_robot_pose(2,k)+z_meas.*sin(sense_angles+est_robot_pose(3,k)));
    if length(z_meas)>1
        %                 set(handle_line_extract, 'XData',x_line_extract,'YData',y_line_extract);
        %                 set(handle_line_extract_pred, 'XData',x_line_extract_pred,'YData',y_line_extract_pred);
    else
        %                 set(handle_line_extract, 'Visible','off');
        %                 set(handle_line_extract_pred, 'Visible','off');
    end
else
    set(handle_lidar,'Visible','off')
    set(handle_line_extract, 'Visible','off');
    set(handle_line_extract_pred, 'Visible','off');
end

if  k>=TsPC/Ts*kP2
    
    kP2=kP2+1;
    title(['Simulation Time:  ', num2str(k*Ts),' (sec)'],'FontSize',16)
end
drawnow
