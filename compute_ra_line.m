function [xline_temp,yline_temp]=compute_ra_line(r,a,P_start,Q_start,P_end,Q_end);

slope=tan(a+pi/2);
xline_temp=[P_start*cos(Q_start),P_end*cos(Q_end)];
if abs(slope)<1e2
    
    x_mid=r*cos(a);
    y_mid=r*sin(a);
    
    b=y_mid-slope*x_mid;
    
    yline_temp=slope*xline_temp+b;
else
    yline_temp=[P_start*sin(Q_start),P_end*sin(Q_end)];
end