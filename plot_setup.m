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

handle_circ= plot(x0+circ_x,y0+circ_y,'b-','LineWidth',1.5);
handle_circ2= plot(x0+circ_x,y0+circ_y,'c-','LineWidth',1.5);

handle_line= plot(x0+[xline1(1) xline2(1)],y0+[xline1(2) xline2(2)],'b-','LineWidth',1.5);
handle_line2= plot(x0+[xline1(1) xline2(1)],y0+[xline1(2) xline2(2)],'c-','LineWidth',1.5);

handle_lidar=plot(x0+zmax*cos(sense_angles+th0),y0+zmax*sin(sense_angles+th0),'r-','LineWidth',2);
handle_ellip= plot(ellip_x,ellip_y,'g-','LineWidth',1.5);

set(figure_mapmaker,'UserData',[handle_circ ; handle_line; handle_ellip; handle_lidar])

if plot_odo
%     [ellip_x2, ellip_y2]=ellipse_from_Guassian(x0,y0,eye(2,2));
%     handle_ellip2= plot(ellip_x2,ellip_y2,'k-','LineWidth',1.5);
    handle_line3= plot(x0+[xline1(1) xline2(1)],y0+[xline1(2) xline2(2)],'k-','LineWidth',1.5);
    handle_circ3= plot(x0+circ_x,y0+circ_y,'k-','LineWidth',1.5);
end

% handle_line_extract=plot(x0+zmax*cos(sense_angles+th0),y0+zmax*sin(sense_angles+th0),'b','LineWidth',2);
% handle_line_extract_pred=plot(x0+zmax*cos(sense_angles+th0),y0+zmax*sin(sense_angles+th0),'m','LineWidth',2);