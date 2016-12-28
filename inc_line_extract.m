function [r,a,covM,x_line,y_line]=inc_line_extract(P, Q,sigp,sigt,robot_pos)

n = length(P);
x_line=[];
y_line=[];

x=robot_pos(1);
y=robot_pos(2);
th=robot_pos(3);
R=[cos(th) -sin(th); sin(th) cos(th)];

%thresholds for classifying lines as the same
r_thresh=.25;
a_thresh=.01;

%Extract First Line
[r_temp,a_temp,covM_temp]=compute_line_params(P(1:2),Q(1:2),sigp,sigt);
i_start=1;
line_count=0;
i=3;

if n<3
    r=r_temp;
    a=a_temp;
    covM=covM_temp;
    [xline_temp,yline_temp]=compute_ra_line(r,a,P(1),Q(1),P(2),Q(2));
    xy_temp=[x x;y y]+R*[xline_temp;yline_temp];
    
    x_line=[x_line,xy_temp(1,:)];
    y_line=[y_line,xy_temp(2,:)];
end

while i <= n
    
    [r_temp2,a_temp2,covM_temp2]=compute_line_params(P(i_start:i),Q(i_start:i),sigp,sigt);
    
    %time to split, point i didn't work
    if abs(1-r_temp2/r_temp)>r_thresh || abs(a_temp-a_temp2)>a_thresh
        line_count=line_count+1;
        r(line_count)=r_temp;
        a(line_count)=a_temp;
        covM{line_count}=covM_temp;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Plotting Purposes Only
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [xline_temp,yline_temp]=compute_ra_line(r(line_count),a(line_count),P(i_start),Q(i_start),P(i-1),Q(i-1));

        xy_temp=[x x;y y]+R*[xline_temp;yline_temp];
        
        x_line=[x_line,xy_temp(1,:)];
        y_line=[y_line,xy_temp(2,:)];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        i_start=i-1;
        
        [r_temp,a_temp,covM_temp]=compute_line_params(P(i_start:i),Q(i_start:i),sigp,sigt);
        
    else  %lines are the same, goto next point
        r_temp=r_temp2;
        a_temp=a_temp2;
        covM_temp=covM_temp2;
        
        %last point in scan, save line
        if i==n
            line_count=line_count+1;
            r(line_count)=r_temp;
            a(line_count)=a_temp;
            covM{line_count}=covM_temp;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Plotting Purposes Only
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            [xline_temp,yline_temp]=compute_ra_line(r(line_count),a(line_count),P(i_start),Q(i_start),P(end),Q(end));

            xy_temp=[x x;y y]+R*[xline_temp;yline_temp];
            
            x_line=[x_line,xy_temp(1,:)];
            y_line=[y_line,xy_temp(2,:)];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
    i=i+1;
end
