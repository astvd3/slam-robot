function [Vout,Vout_dot]=robot_nav(robot_params,V,z_true,sense_angles,behavior,goal,robot_pose)

persistent first_hit
persistent wall_follow
persistent sub_behavior
persistent first_pass
persistent intial_position
persistent free_space

if isempty(first_hit)
    first_hit=1;
    wall_follow=0;
    sub_behavior=1;
    first_pass=1;
    intial_position=0;
    free_space=0;
end

%Robot States
xr=robot_pose(1);
yr=robot_pose(2);
theta=robot_pose(3);

%Goal Location
xg=goal(1);
yg=goal(2);



%enforce max wheel speed
wRL_max=6;
Vmin=0;

%Robot Physical Parameters
r=0.25;      %wheel radius
b=0.8;       %wheel base
R=[r/2 r/2;
    r/b -r/b];

%Set max v and w based on max wheel speed
Vmax(1)=R(1,:)*[wRL_max;wRL_max];
Vmax(2)=R(2,:)*[wRL_max;-wRL_max];

v=V(1);
switch behavior
    case 1  %Potential Function Navigation
        %Put your code here to implement potential function navigation
        k_att=.1;
        k_rep=1;
        obs_thresh=2;
        
        d_shift=.1; %point shift from slide 61 of Lecture 7
        
        Ux_att=-k_att*(xr-xg);
        Uy_att=-k_att*(yr-yg);
        
        Ux_rep=.1;
        Uy_rep=.1;
        
        
        iobs=find(z_true<obs_thresh);
        
        if ~isempty(iobs)
            %Consider the total potential (all points within threshold)
            obs_angle=sense_angles(iobs);
            obs_meas=z_true(iobs);
            
            %Or just consider the single closest point to the robot
            %             [obs_meas,iobs]=min(z_true);
            %             obs_angle=sense_angles(iobs);
            for i=1:length(obs_meas);
                Ux_rep=Ux_rep-k_rep*(1/obs_meas(i)-1/obs_thresh)*1/(obs_meas(i)^2)*cos(theta+obs_angle(i));
                Uy_rep=Uy_rep-k_rep*(1/obs_meas(i)-1/obs_thresh)*1/(obs_meas(i)^2)*sin(theta+obs_angle(i));
            end
            
        end
        
        Ux=Ux_att+Ux_rep;
        Uy=Uy_att+Uy_rep;
        
        V=[1 0;0 1/d_shift]*[cos(theta) sin(theta);-sin(theta) cos(theta)]*[Ux;Uy];
        v=V(1);
        w=V(2);
        
        
        Vmin=0;
    case 2 %wall follow
        obs_thresh=1;
        iobs=find(z_true<obs_thresh);
        if isempty(iobs)& ~wall_follow
            w=0;
        else
            wall_follow=1;
            
            %Find the distance to the closest obstacle
            [obs_meas,iobs]=min(z_true);
            
            %Get corresponding angle to closest obstacle
            obs_angle=sense_angles(iobs(1));
            v=1;
            
            %Clockwise follow
            w=5*atan2(sin(abs(obs_angle)-pi/2),cos(abs(obs_angle)-pi/2))+5*(obs_meas-obs_thresh);
            
            %Counterclockwise Follow
            w=-w;
        end
        Vmin=.5;
    case 3 %Bug2
        %Put your code here to implement the Bug2 algorithm
        obs_thresh=2;
        
        if first_pass
            first_pass=0;
            wall_follow=-1;  %direction of wall fall 1 - clockwise -1 counter clockwise
            intial_position=robot_pose;
            free_space=[0;0];
        end
        
        switch sub_behavior
            case 1  %Goal Seeking
                k_att=.1;
                
                d_shift=.1; %point shift from slide 61 of Lecture 7
                
                Ux=-k_att*(xr-xg);
                Uy=-k_att*(yr-yg);
                V=[1 0;0 1/d_shift]*[cos(theta) sin(theta);-sin(theta) cos(theta)]*[Ux;Uy];
                v=V(1);
                w=V(2);
                
                [obs_meas,iobs]=sort(z_true);
                obs_angle=sense_angles(iobs);
                
                
                %Vote for which direction to follow the wall
                
                %Group the sensor readings for positive sensor angles
                [NP,CP]=hist(obs_meas(obs_angle>0),2);
                
                %Group the sensor readings for negative sensor angles
                [NN,CN]=hist(obs_meas(obs_angle<0),2);
                
                %Vote based on largest cluster of greater sensor readings
                if NP(end)>=NN(end)
                    free_space(1)=free_space(1)+1; %go in direction of max reading (free space)
                else
                    free_space(2)=free_space(2)+1;
                end
                
                %2nd vote based on largest average of cluster sensor
                %readings
                if CP(end)>=CN(end)
                    free_space(1)=free_space(1)+1; %go in direction of max reading (free space)
                else
                    free_space(2)=free_space(2)+1;
                end
                
                iobs=find(z_true<obs_thresh);
                if ~isempty(iobs)
                    [obs_meas,iobs]=min(z_true);
%                     [obs_meas,iobs]=sort(z_true);
                    obs_angle=sense_angles(iobs);
                    del_x=xg-xr;
                    del_y=yg-yr;
                    
                    alpha=atan2(del_y,del_x);
                    alpha=atan2(sin(alpha),cos(alpha));
                    
                    
                    if abs(obs_angle(1))<=pi/2 && abs(theta+obs_angle(1)-alpha)<pi/2 %obstacle is not behind the robot, goal is blocked by obstacle
                        sub_behavior=2;

                        if free_space(1)>free_space(2)
                            wall_follow=-1;
                        else
                            wall_follow=1;
                        end
                        free_space=[0;0];  %reset vote
                        
                    end
                end
                Vmin=0;
            case 2
                del_x=xg-xr;
                del_y=yg-yr;
                
                alpha=atan2(del_y,del_x);
                alpha=atan2(sin(alpha),cos(alpha));
                
                %Find the distance to the closest obstacle
                [obs_meas,iobs]=min(z_true);
                
                %Get corresponding angle to closest obstacle
                obs_angle=sense_angles(iobs(1));
                v=1;
                
                w=5*atan2(sin(abs(obs_angle)-pi/2),cos(abs(obs_angle)-pi/2))+5*(obs_meas-obs_thresh);
                w=wall_follow*w;
                
                d=abs((xg-intial_position(1))*(intial_position(2)-yr)-(intial_position(1)-xr)*(yg-intial_position(2)))/sqrt((xg-intial_position(1))^2+(yg-intial_position(2))^2);
                if d<1e-2  && abs(theta+obs_angle-alpha)>pi/2 %path from intial position to goal is reached and goal is not blocked by obstacle
                    sub_behavior=1;
                end
                Vmin=.5;
            otherwise
        end
        
        
    otherwise
end

%Enforce Maximum Wheel Speed
V(1)=v;
V(2)=w;

wRL=inv(R)*V;  % calculate wheel speeds

if abs(wRL(1))>wRL_max || abs(wRL(2))>wRL_max
    
    if abs(wRL(1))>abs(wRL(2))
        wRL(1)=sign(wRL(1))*.9*wRL_max;
        if abs(w)> .5
            wRL(2)=-(b/r)*(Vmax(2)*sign(w)-(r/b)*wRL(1));
        else
            wRL(2)=(2/r)*(Vmax(1)*sign(v)-(r/2)*wRL(1));
        end
    else
        wRL(2)=sign(wRL(2))*.9*wRL_max;
        if abs(w)>.5
            wRL(1)=(b/r)*(Vmax(2)*sign(w)-(r/b)*wRL(2));
        else
            wRL(1)=(2/r)*(Vmax(1)*sign(v)-(r/2)*wRL(2));
        end
    end
    
    V=R*wRL;
    V(1)=max(V(1),Vmin);
    v=V(1);
    w=V(2);
    
end

wtemp=min(abs(w),4);
w=sign(w)*wtemp;
Vout=[v;w];
Vout=[v;w];
Vout_dot=[0;0];


% function [Vout,Vout_dot]=robot_nav(robot_params,V,z_true,sense_angles,behavior,goal,robot_pose)
% 
% 
% persistent first_hit
% persistent wall_follow
% persistent sub_behavior
% persistent first_pass
% persistent intial_position
% persistent free_space
% 
% if isempty(first_hit)
%     first_hit=1;
%     wall_follow=0;
%     sub_behavior=1;
%     first_pass=1;
%     intial_position=0;
%     free_space=0;
% end
% 
% obs_thresh=2;
% iobs=find(z_true<obs_thresh);
% %Robot States
% xr=robot_pose(1);
% yr=robot_pose(2);
% theta=robot_pose(3);
% 
% 
% 
% %Goal Location
% xg=goal(1);
% yg=goal(2);
% 
% %enforce max wheel speed
% wRL_max=6;
% Vmin=0;
% 
% %Robot Physical Parameters
% r=0.25;      %wheel radius
% b=0.8;       %wheel base
% R=[r/2 r/2;
%     r/b -r/b];
% 
% 
% %Set max v and w based on max wheel speed
% Vmax(1)=R(1,:)*[wRL_max;wRL_max];
% Vmax(2)=R(2,:)*[wRL_max;-wRL_max];
% 
% 
% v=V(1);
% switch behavior
%     case 1  %Potential Function Navigation
%          k_att=1;
%         k_rep=2;
%         obs_thresh=1;
%         
%         d_shift=.1; %point shift from slide 61 of Lecture 7
%         
%         Ux_att=-k_att*(xr-xg);
%         Uy_att=-k_att*(yr-yg);
%         
%         Ux_rep=k_rep*(1-1/(xr-xg));
%         Uy_rep=k_rep*(1-1/(yr-yg));
%         
%         
%         iobs=find(z_true<obs_thresh);
%         
%         if ~isempty(iobs)
%             %Consider the total potential (all points within threshold)
%             obs_angle=sense_angles(iobs);
%             obs_meas=z_true(iobs);
%             
%             %Or just consider the single closest point to the robot
%             %             [obs_meas,iobs]=min(z_true);
%             %             obs_angle=sense_angles(iobs);
%             for i=1:length(obs_meas);
%                 Ux_rep=Ux_rep-k_rep*(1/obs_meas(i)-1/obs_thresh)*1/(obs_meas(i)^2)*cos(theta+obs_angle(i));
%                 Uy_rep=Uy_rep-k_rep*(1/obs_meas(i)-1/obs_thresh)*1/(obs_meas(i)^2)*sin(theta+obs_angle(i));
%             end
%             
%         end
%         
%         Ux=Ux_att+Ux_rep;
%         Uy=Uy_att+Uy_rep;
%         
%         V=[1 0;0 1/d_shift]*[cos(theta) sin(theta);-sin(theta) cos(theta)]*[Ux;Uy];
%         v=V(1);
%         w=V(2);
%         
%         
%         Vout_dot=[0;0]; %Calculate the derivative of the control velocity (for backstepping control)
%         Vmin=0;
%     case 2 %wall follow
%         if isempty(iobs)& ~wall_follow
%             w=0;
%             Vout_dot=[0;0]; %Calculate the derivative of the control velocity (for backstepping control)
%         else
%             wall_follow=1;
%             
%             %Find the distance to the closest obstacle
%             [obs_meas,iobs]=min(z_true);
%             
%             %Get corresponding angle to closest obstacle
%             obs_angle=sense_angles(iobs(1));
%             v=1;
%             
%             %Clockwise follow
%             w=5*atan2(sin(obs_angle-pi/2),cos(obs_angle-pi/2))+5*(obs_meas-obs_thresh);
%             Vout_dot=[0;0]; %Calculate the derivative of the control velocity (for backstepping control)
%             %Counterclockwise Follow
%             %                 w=-w;
%         end
%         Vmin=.5;
%     case 3 %Bug2
%         obs_thresh=1;
%         
%         if first_pass
%             first_pass=0;
%             wall_follow=1;  %direction of wall fall 1 - clockwise -1 counter clockwise
%             intial_position=robot_pose;
%             free_space=[0;0];
%         end
%         
%         switch sub_behavior
%             case 1  %Goal Seeking
%                 k_att=.3;
%                 
%                 d_shift=.1; %point shift from slide 61 of Lecture 7
%                 
%                 Ux=-k_att*(xr-xg);
%                 Uy=-k_att*(yr-yg);
%                 V=[1 0;0 1/d_shift]*[cos(theta) sin(theta);-sin(theta) cos(theta)]*[Ux;Uy];
%                 v=V(1);
%                 w=V(2);
%                 
%                 [obs_meas,iobs]=sort(z_true);
%                 obs_angle=sense_angles(iobs);
%                 
%                 
%                 %Vote for which direction to follow the wall
%                 
%                 %Group the sensor readings for positive sensor angles
%                 [NP,CP]=hist(obs_meas(obs_angle>0),2);
%                 
%                 %Group the sensor readings for negative sensor angles
%                 [NN,CN]=hist(obs_meas(obs_angle<0),2);
%                 
%                 %Vote based on largest cluster of greater sensor readings
%                 if NP(end)>=NN(end)
%                     free_space(1)=free_space(1)+1; %go in direction of max reading (free space)
%                 else
%                     free_space(2)=free_space(2)+1;
%                 end
%                 
%                 %2nd vote based on largest average of cluster sensor
%                 %readings
%                 if CP(end)>=CN(end)
%                     free_space(1)=free_space(1)+1; %go in direction of max reading (free space)
%                 else
%                     free_space(2)=free_space(2)+1;
%                 end
%                 
%                 iobs=find(z_true<obs_thresh);
%                 if ~isempty(iobs)
%                     [obs_meas,iobs]=min(z_true);
% %                     [obs_meas,iobs]=sort(z_true);
%                     obs_angle=sense_angles(iobs);
%                     del_x=xg-xr;
%                     del_y=yg-yr;
%                     
%                     alpha=atan2(del_y,del_x);
%                     alpha=atan2(sin(alpha),cos(alpha));
%                     
%                     
%                     if abs(obs_angle(1))<=pi/2 && abs(theta+obs_angle(1)-alpha)<pi/2 %obstacle is not behind the robot, goal is blocked by obstacle
%                         sub_behavior=2;
% 
%                         if free_space(1)>free_space(2)
%                             wall_follow=-1;
%                         else
%                             wall_follow=1;
%                         end
%                         free_space=[0;0];  %reset vote
%                         
%                     end
%                 end
%                 Vmin=0;
%             case 2
%                 del_x=xg-xr;
%                 del_y=yg-yr;
%                 
%                 alpha=atan2(del_y,del_x);
%                 alpha=atan2(sin(alpha),cos(alpha));
%                 
%                 %Find the distance to the closest obstacle
%                 [obs_meas,iobs]=min(z_true);
%                 
%                 %Get corresponding angle to closest obstacle
%                 obs_angle=sense_angles(iobs(1));
%                 v=1;
%                 
%                 w=5*atan2(sin(abs(obs_angle)-pi/2),cos(abs(obs_angle)-pi/2))+5*(obs_meas-obs_thresh);
%                 w=wall_follow*w;
%                 
%                 d=abs((xg-intial_position(1))*(intial_position(2)-yr)-(intial_position(1)-xr)*(yg-intial_position(2)))/sqrt((xg-intial_position(1))^2+(yg-intial_position(2))^2);
%                 if d<1e-2  && abs(theta+obs_angle-alpha)>pi/2 %path from intial position to goal is reached and goal is not blocked by obstacle
%                     sub_behavior=1;
%                 end
%                 Vmin=.5;
%                 otherwise
%         end
%         Vout_dot=[0;0]; %Calculate the derivative of the control velocity (for backstepping control)
%     case 4  %random wander with obstacle avoidance
%         Krep=15;
%         
%         if isempty(iobs)
%             w=0;
%             first_hit=1;
%             Vout_dot=[0;0]; %Calculate the derivative of the control velocity (for backstepping control)
%         else
%             obs_meas=z_true(iobs);
%             [y,iobs_min]=min(z_true);
%             obs_angle=sense_angles(iobs_min);
%             w=-sum(Krep*(1./obs_meas-1/obs_thresh).*(1./obs_meas.^2));
%             if abs(sense_angles(iobs_min))>1e-1;
%                 w=w*sin(obs_angle);
%             end
%             if first_hit
%                 first_hit=0;
%                 %         beep;
%             end
%             Vout_dot=[0;0]; %Calculate the derivative of the control velocity (for backstepping control)
%         end
%     otherwise
%         Vout_dot=[0;0]; %Calculate the derivative of the control velocity (for backstepping control)
% end
% 
% V(1)=v;
% V(2)=w;
% 
% wRL=inv(R)*V;  % calculate wheel speeds
% 
% if abs(wRL(1))>wRL_max || abs(wRL(2))>wRL_max
%     
%     if abs(wRL(1))>abs(wRL(2))
%         wRL(1)=sign(wRL(1))*.9*wRL_max;
%         if abs(w)> .5
%             wRL(2)=-(b/r)*(Vmax(2)*sign(w)-(r/b)*wRL(1));
%         else
%             wRL(2)=(2/r)*(Vmax(1)*sign(v)-(r/2)*wRL(1));
%         end
%     else
%         wRL(2)=sign(wRL(2))*.9*wRL_max;
%         if abs(w)>.5
%             wRL(1)=(b/r)*(Vmax(2)*sign(w)-(r/b)*wRL(2));
%         else
%             wRL(1)=(2/r)*(Vmax(1)*sign(v)-(r/2)*wRL(2));
%         end
%     end
%     
%     V=R*wRL;
%     V(1)=max(V(1),Vmin);
%     v=V(1);
%     w=V(2);
%     
% end
% 
% Vout=[v;w];