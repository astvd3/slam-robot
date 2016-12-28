close all
clear all
clc
commandwindow;

%Robot Initial Position
% x0=16.7;
% y0=-27.64;
% th0=.7638+pi/2;

x0=-10;
y0= -18;
th0=0;
count=0;
line_ext_alg=3;  % 1-Incremental, 2-Split and Merge, 3-Iterative Endpoint

%Robot parameters
robot_params.mT=10;  %Total mass
robot_params.mW=2;   %Mass of 1 wheel
robot_params.r=0.05; %Wheel radius
robot_params.b=0.4;  %half the robot width
robot_params.d=0.1;  %CG offset from rear axle
robot_params.Iyy=1;  %Wheel moment of Iertia
robot_params.IT=5;   %Platform total moment of inertia
robot_params.Fv=.5;  %Coefficient of Fiscous friction
robot_params.Fd=.8;  %Coefficient of Colomb friction
robot_params.tau_max=1000;  %Maximum motor torque

behavior=2;
goal_position=[15;15];

%Robot Initial Velocity
V0=[1;0];
prev_af=0;
prev_bf=0;

%End time and simulation sampling time
Tend=500;
Ts=.005;

%Lidar Update Sampling Time
TsL=.1;

%PLOT Update Sampling Time
TsP=1;

%update Time on Plot
TsPC=2;

%Lidar max sensing distance and sensor coverage
sense_angles= -pi/2: (2*pi)/20 : 3*pi/2;
zmax=20;
zmax_s=10;
prev_x_line=zeros(1,20);

prev_y_line=zeros(1,20);

%Load a map for testing
get_file_name=1;
filename='map1.txt';
EE301_load_map;
figure(1)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Set up plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%heading line
rad=0.25;
xline1=[0; 0];
xline2=[1.5*rad; 0];


% Plot robot in default position and store plot handles for updating
circ_numPts= 21;    % Estimate circle as circ_numPts-1 lines
circ_ang=linspace(0,2*pi,circ_numPts);
circ_rad=ones(1,circ_numPts)*rad;
[circ_x circ_y]= pol2cart(circ_ang,circ_rad);
[ellip_x, ellip_y]=ellipse_from_Guassian(x0,y0,eye(2,2));
[ellip_x2, ellip_y2]=ellipse_from_Guassian(x0,y0,eye(2,2));
handle_circ= plot(x0+circ_x,y0+circ_y,'b-','LineWidth',1.5);
handle_circ2= plot(x0+circ_x,y0+circ_y,'c-','LineWidth',1.5);
handle_circ3= plot(x0+circ_x,y0+circ_y,'k-','LineWidth',1.5);
handle_line= plot(x0+[xline1(1) xline2(1)],y0+[xline1(2) xline2(2)],'b-','LineWidth',1.5);
handle_line2= plot(x0+[xline1(1) xline2(1)],y0+[xline1(2) xline2(2)],'c-','LineWidth',1.5);
handle_line3= plot(x0+[xline1(1) xline2(1)],y0+[xline1(2) xline2(2)],'k-','LineWidth',1.5);
handle_lidar=plot(x0+zmax*cos(sense_angles+th0),y0+zmax*sin(sense_angles+th0),'r-','LineWidth',2);
handle_ellip= plot(ellip_x,ellip_y,'g-','LineWidth',1.5);
handle_ellip2= plot(ellip_x2,ellip_y2,'k-','LineWidth',1.5);
set(figure_mapmaker,'UserData',[handle_circ ; handle_line; handle_ellip; handle_ellip2;handle_lidar])
handle_line_extract=plot(x0+zmax*cos(sense_angles+th0),y0+zmax*sin(sense_angles+th0),'b','LineWidth',2);
handle_line_extract_pred=plot(x0+zmax*cos(sense_angles+th0),y0+zmax*sin(sense_angles+th0),'m','LineWidth',2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% SONAR models:
% model 1: "realistic"
sonar_model{1}.p_hit = 0.9;
sonar_model{1}.p_short = 0.025;
sonar_model{1}.p_max = 0.05;
sonar_model{1}.p_rand = 0.025;
sonar_model{1}.sigma_hit = 0.0025;
sonar_model{1}.lambda_short = 1;
sonar_model{1}.z_max = zmax_s;
sonar_model{1}.z_max_eps = 0.02;
sonar_model{1}.sonar_angles=sense_angles;
sonar_temp=sonar_model(1);
sonar_model{1}.sample=sonar_sample_model(sonar_temp,1000);

% laser models:
% model 1: "realistic"
laser_model{1}.p_hit = 0.98;
laser_model{1}.p_short = 0.005;
laser_model{1}.p_max = 0.01;
laser_model{1}.p_rand = 0.005;
laser_model{1}.sigma_hit = 0.0025;
laser_model{1}.lambda_short = 1;
laser_model{1}.z_max = zmax;
laser_model{1}.z_max_eps = 0.02;
laser_model{1}.laser_angles=sense_angles;
laser_temp=laser_model(1);
laser_model{1}.sample=laser_sample_model(laser_temp,1000);

% model 2: noise free
laser_model{2}.p_hit = 1.0;
laser_model{2}.p_short = 0;
laser_model{2}.p_max = 0;
laser_model{2}.p_rand = 0;
laser_model{2}.sigma_hit = 0;
laser_model{2}.lambda_short = 1;
laser_model{2}.z_max = zmax;
laser_model{2}.z_max_eps = 0.02;
laser_model{2}.laser_angles = sense_angles;
laser_temp=laser_model(2);
laser_model{2}.sample=laser_sample_model(laser_temp,10);

% model 3: "Always hit with Gaussian noise"
laser_model{3}.p_hit = 1;
laser_model{3}.p_short = 0;
laser_model{3}.p_max = 0;
laser_model{3}.p_rand = 0;
laser_model{3}.sigma_hit = 0.0025;
laser_model{3}.lambda_short = 1;
laser_model{3}.z_max = zmax;
laser_model{3}.z_max_eps = 0.02;
laser_model{3}.laser_angles=sense_angles;
laser_temp=laser_model(3);
laser_model{3}.sample=laser_sample_model(laser_temp,1000);

%select the laser model by changing the index in laser_model(-)
sonar_temp=sonar_model(1);

%Actual Robot Position - DO NOT MODIFY
robot_pose=zeros(3,floor(Tend/Ts)+1);
V=ones(2,floor(Tend/Ts));
V(1,:)=V(1,:)*V0(1);
V(2,:)=V(2,:)*V0(2);
robot_pose(:,1)=[x0;y0;th0];
desired_robot_pose=robot_pose;

Vd=V;
Vd_dot=Vd*0;

%Robot Control Torque
cntl_trq=Vd*0;
%Estimated Robot Position & Covariance Matrix
%Kalman Filter Localization
est_robot_pose=robot_pose;
estV=zeros(2,floor(Tend/Ts));
odometry_covar_mat=zeros(Tend/Ts+1,3,3);
odometry_covar_mat(1,:,:)=0*eye(3);

%These parameters will plot the result when only and encoder is utilized
est_robot_pose2=robot_pose;
estV2=zeros(2,floor(Tend/Ts));
odometry_covar_mat2=zeros(Tend/Ts+1,3,3);

writerObj = VideoWriter('video2.avi');
open(writerObj);


kL=0;
kP=0;
kP2=0;
lidar_new=0;
figure(1)
axis([min(min([walls(:,1) walls(:,3)]))-1 max(max([walls(:,1) walls(:,3)]))+1 min(min([walls(:,2) walls(:,4)]))-1 max(max([walls(:,2) walls(:,4)]))+1])
for k=1:Tend/Ts
    
    %Take Lidar Reading
    if k>=TsL/Ts*kL
        
        %generate the sensor measurement based on the real position -
        %results are in the robot reference frame
        z_true=ray_cast(walls, sonar_temp, robot_pose(:,k));
        z_meas=sonar_meas_model(z_true,sonar_temp);
        
        kL=kL+1;
        lidar_new=1;
    end
    
    %Basic (and ideal) Obstacle Avoidance
    [Vd(:,k),Vd_dot(:,k)]=robot_nav(robot_params,Vd(:,k),z_meas,sense_angles,behavior,goal_position,est_robot_pose(:,k));
    desired_robot_pose(:,k+1)=diff_drive(desired_robot_pose(:,k),Vd(:,k),Ts);
    %Robot Velocity tracking Control law (Take home test 1)
    cntl_trq(:,k)=robot_backstepping_control(robot_params,V(:,k),Vd(:,k),Vd_dot(:,k),[robot_pose(:,k)' V(:,k)' desired_robot_pose(:,k)' Vd(:,k)' Vd_dot(:,k)']);
    
    %Robot Dynamics (Take home test 1)
    V(:,k+1)=diff_drive_dyn(robot_params,V(:,k),cntl_trq(:,k),Ts);
    
    %Get next robot pose based on current velocity - (Homework 1)
    robot_pose(:,k+1)=diff_drive(robot_pose(:,k),V(:,k),Ts);
    
    %Update Encoder and odometry Estimates
    odometry_covar_mat_temp=odometry_covar_mat(k,:,:);
    odometry_covar_mat_temp=reshape(odometry_covar_mat_temp,3,3);
    [est_robot_pose(:,k+1),estV(:,k),odometry_covar_mat_temp]=diff_drive_encoder(Vd(:,k),est_robot_pose(:,k),odometry_covar_mat_temp,Ts);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %This one DOES NOT use Kalman Filter, Visualize effects of using
    %odometry only
%     odometry_covar_mat_temp2=odometry_covar_mat2(k,:,:);
%     odometry_covar_mat_temp2=reshape(odometry_covar_mat_temp2,3,3);
%     [est_robot_pose2(:,k+1),estV2(:,k),odometry_covar_mat_temp2]=diff_drive_encoder(Vd(:,k),est_robot_pose2(:,k),odometry_covar_mat_temp2,Ts);
%     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %store the odometry covariance matrix
    est_robot_pose_temp=est_robot_pose(:,k+1);
    if lidar_new
        lidar_new=0;
        
        %generate expected sensor reading based on estimated position
        %results are in the robot reference frame
        z_predict=ray_cast(walls,sonar_temp, est_robot_pose(:,k));
        
        %If a new Lidar scan is available and correct, extract lines (HW6 Q2) and
        %update the Kalman filter parameters:  est_robot_pose_temp, odometry_covar_mat_temp
        
        %Extract Measured Lines (robot_pose used for plotting only)
        %Extract Predicted Lines (robot_pose_est used for plotting only)
        if length(z_meas)>1
            switch line_ext_alg
                case 1  %Incremental Line Extraction
                    [r,a,covM,x_line_extract,y_line_extract]=inc_line_extract(z_meas, sense_angles,sonar_temp{1}.sigma_hit,0,robot_pose(:,k));
                    [rp,ap,covMp,x_line_extract_pred,y_line_extract_pred]=inc_line_extract(z_predict, sense_angles,0,0,est_robot_pose(:,k));
                case 2  %Split and Merge Line Extraction
                    [r,a,covM,x_line_extract,y_line_extract]=split_merge_line_extract(z_meas, sense_angles,sonar_temp{1}.sigma_hit,0,robot_pose(:,k),1);
                    [rp,ap,covMp,x_line_extract_pred,y_line_extract_pred]=split_merge_line_extract(z_predict, sense_angles,0,0,est_robot_pose(:,k),1);
                case 3  %Iterative End Point
                    [r,a,covM,x_line_extract,y_line_extract]=split_merge_line_extract(z_meas, sense_angles,sonar_temp{1}.sigma_hit,0,robot_pose(:,k),0);
                    [rp,ap,covMp,x_line_extract_pred,y_line_extract_pred]=split_merge_line_extract(z_predict, sense_angles,0,0,est_robot_pose(:,k),0);
                otherwise
            end
            
            %Jacobian for predicted lines
            for j43=1:length(ap)
                Hj{j43}=[0                           0                   -1;
                    -cos(ap(j43)+est_robot_pose(3,k)) -sin(ap(j43)+est_robot_pose(3,k)) 0];
                
            end
            
            dist_min=0;
            covM_temp_mat=[];
            Hj_mat=[];
            vij_vec=[];
            i45=0;
            for i43=1:length(a)
                vij=[];
                Sig_ij=[];
                mah_dist=0;
                
                if iscell(covM)
                    covM_temp=covM{i43};
                else
                    covM_temp=covM;
                end
                
                for j43=1:length(ap)
                    vij{j43}=[a(i43);r(i43)]-[ap(j43);rp(j43)];
                    
                    Sig_ij{j43}=Hj{j43}*odometry_covar_mat_temp*Hj{j43}'+covM_temp;
                    
                    mah_dist(j43)=vij{j43}'*(Sig_ij{j43}^(-1))*vij{j43};
                end
                
                [dist_min,j_update]=min(mah_dist);
                
                %If there was a match between the measured line and a
                %predicted line, update the Kalman Filter estimates using
                %the Cholesky Update method
                if abs(dist_min)<10;
                    [W1,W2]=KF_Gain_cholesky_param(odometry_covar_mat_temp,covM_temp,Hj{j_update(1)});
                    est_robot_pose_temp=est_robot_pose_temp+W1*vij{j_update(1)};
                    odometry_covar_mat_temp=odometry_covar_mat_temp-W2*W2';
%                     x= x + W*v; % update
%                     P= P - W1*W1';
                end
            end %end for i43=1:length(a)           
        end % end if length(z_meas)>1
    end  %end if lidar_new
    
    %Store results of Kalman filter parameter updates
    est_robot_pose(:,k+1)=est_robot_pose_temp;
    temp2=reshape(odometry_covar_mat_temp,1,3,3);
    odometry_covar_mat(k+1,:,:)=temp2;
    x_line_extract_approx=round(x_line_extract_pred);
    y_line_extract_approx=round(y_line_extract_pred);
    a=[x_line_extract_pred;y_line_extract_pred];
    
    figure(2);
    af=sum(x_line_extract_pred,2);
    bf=sum(y_line_extract_pred,2);
    if(prev_af==af)
    else    
        for i=1:length(x_line_extract_pred)
        diff=robot_pose(1:2,k)-a(:,i);
        diff=diff.^2;
        diff=sqrt(sum(diff,1));
        if(diff>7)
            a(:,i)=a(:,i)-a(:,i);
%             plot(x_line_extract_approx,y_line_extract_approx,'rx');
%             hold on
        end
        end
     %figure(2);
      xlim([-20 20]);
      ylim([-20 20]);
     plot(a(1,:),a(2,:),'rx');
   
         hold on
        
    end

     
    prev_af=af;
    prev_bf=bf;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %PLOT RESULTS
    if  k>=TsP/Ts*kP
        
        kP=kP+1;
        [ellip_x, ellip_y]=ellipse_from_Guassian(est_robot_pose(1,k),est_robot_pose(2,k),odometry_covar_mat_temp(1:2,1:2));
%        [ellip_x2, ellip_y2]=ellipse_from_Guassian(est_robot_pose2(1,k),est_robot_pose2(2,k),odometry_covar_mat_temp2(1:2,1:2));
        
        xline2_plot=[cos(robot_pose(3,k)) -sin(robot_pose(3,k));
            sin(robot_pose(3,k)) cos(robot_pose(3,k))]*xline2;
        
        xline2_plot2=[cos(est_robot_pose(3,k)) -sin(est_robot_pose(3,k));
            sin(est_robot_pose(3,k)) cos(est_robot_pose(3,k))]*xline2;
        
       % xline2_plot3=[cos(est_robot_pose2(3,k)) -sin(est_robot_pose2(3,k));
        %    sin(est_robot_pose2(3,k)) cos(est_robot_pose2(3,k))]*xline2;
        
        %Plot Actual Robot Position
        set(handle_circ, 'XData', robot_pose(1,k)+circ_x,'YData', robot_pose(2,k)+circ_y);
        set(handle_line, 'XData', robot_pose(1,k)+[xline1(1) xline2_plot(1)], 'YData', robot_pose(2,k)+[xline1(2) xline2_plot(2)]);
        
        %Plot Estimated Robot Position
        set(handle_circ2, 'XData', est_robot_pose(1,k)+circ_x,'YData', est_robot_pose(2,k)+circ_y);
        set(handle_line2, 'XData', est_robot_pose(1,k)+[xline1(1) xline2_plot2(1)], 'YData', est_robot_pose(2,k)+[xline1(2) xline2_plot2(2)]);
        
        %Plot Estimated Robot Position -NO Kalman Filter
%         set(handle_circ3, 'XData', est_robot_pose2(1,k)+circ_x,'YData', est_robot_pose2(2,k)+circ_y);
%         set(handle_line3, 'XData', est_robot_pose2(1,k)+[xline1(1) xline2_plot3(1)], 'YData', est_robot_pose2(2,k)+[xline1(2) xline2_plot3(2)]);
%         
        %Plot Uncertainty Ellipse
        set(handle_ellip, 'XData', ellip_x, 'YData', ellip_y);
        
        %Plot Uncertainty Ellipse-NO Kalman Filter
        set(handle_ellip2, 'XData', real(ellip_x2), 'YData', real(ellip_y2));
        
        %Plot LIDAR Data - Nothing is plot if zmeas==zmax
        if (mean(z_meas)<zmax)
            set(handle_lidar,'Visible','on')
            set(handle_lidar, 'XData', robot_pose(1,k)+z_meas.*cos(sense_angles+robot_pose(3,k)), 'YData', robot_pose(2,k)+z_meas.*sin(sense_angles+robot_pose(3,k)));
            if length(z_meas)>1
                set(handle_line_extract, 'XData',x_line_extract,'YData',y_line_extract);
                set(handle_line_extract_pred, 'XData',x_line_extract_pred,'YData',y_line_extract_pred);
            else
                set(handle_line_extract, 'Visible','off');
                set(handle_line_extract_pred, 'Visible','off');
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
        F2=getframe(1);
        writeVideo(writerObj,F2);
        pause(0.1)
        
    end
            
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
close(writerObj);
plot(robot_pose(1,1:k),robot_pose(2,1:k),'b.-')
%currFrame = getframe;
%writeVideo(vidObj,currFrame);
%close(vidObj);

figure(2)
subplot 211
plot((0:k-1)*Ts, V(1,(1:k)))
xlabel('Time (Sec)', 'FontSize', 16)
ylabel('Linear Velocity (m/s)', 'FontSize', 16)
title('Robot Velocities','FontSize',16)

subplot 212
plot((0:k-1)*Ts,V(2,(1:k)))
xlabel('Time (Sec)', 'FontSize', 16)
ylabel('Angular Velocity (rad/s)', 'FontSize', 16)