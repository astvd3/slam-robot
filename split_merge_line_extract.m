function [r,a,covM,x_line,y_line]=split_merge_line_extract(P, Q,sigp,sigt,robot_pos,split_merge)

n = length(P);
x_line=[];
y_line=[];
i_save=[];

x=robot_pos(1);
y=robot_pos(2);
th=robot_pos(3);
R=[cos(th) -sin(th); sin(th) cos(th)];



%thresholds for classifying lines as the same
d_thresh=.5;
r_thresh=.25;
a_thresh=.1;

pt_list=[1:n];
ptr_list=[1 n];
next_ptr_list=[];
line_count=0;
split_line=1;
while ~isempty(ptr_list)
    
    %run through entire list of ptrs, create next ptr_list for next run
    while ~isempty(ptr_list)
        %Extract Next Line in Pointer List
        p_start=ptr_list(1);
        p_end=ptr_list(2);
        
        % 1 - Conventional Split and Merge, 0 - Iterative-End-Point-Fit, cf. page 250
        if split_merge
            %Conventional Split and Merge
            [r_temp,a_temp,covM_temp]=compute_line_params(P(p_start:p_end),Q(p_start:p_end),sigp,sigt);
        else
            %Iterative-End-Point-Fit, cf. page 250
            [r_temp,a_temp,covM_temp]=compute_line_params(P([p_start,p_end]),Q([p_start,p_end]),sigp,sigt);
        end
        [xline_temp,yline_temp]=compute_ra_line(r_temp,a_temp,P(p_start),Q(p_start),P(p_end),Q(p_end));
        d=point_to_line(P(p_start:p_end),Q(p_start:p_end),xline_temp,yline_temp);
        
        [d,i_split]=max(d);
        i_split=i_split+p_start-1;
        
        if d<d_thresh  %all points were close enough (i.e.,only 1 line)
            i_save=[i_save;p_start,p_end];
            line_count=line_count+1;
            r_save(line_count)=r_temp;
            a_save(line_count)=a_temp;
            pt_list(ptr_list(1):ptr_list(2))=0;
            ptr_list(1:2)=[];
        else
            ptr_list_temp=ptr_list(1:2);
            ptr_list(1:2)=[];
            if i_split==ptr_list_temp(1)
                i_split=i_split+1;
            elseif i_split==ptr_list_temp(2)
                i_split=i_split-1;
            end
            
            if (find(pt_list==i_split))
                next_ptr_list=[next_ptr_list ptr_list_temp(1) i_split i_split ptr_list_temp(2)];
            end
            
            
            clean_list=1;
            while clean_list
                if next_ptr_list(2)<=next_ptr_list(1)
                    next_ptr_list(1:2)=[];
                else
                    clean_list=0;
                end
            end
        end
    end
    ptr_list=next_ptr_list;
    next_ptr_list=[];
end

% merge lines
line_list=[1:length(r_save)];
line_count=0;
while ~isempty(line_list)
    r_temp=r_save(line_list(1));
    a_temp=a_save(line_list(1));
    remove_list=[1];
    j=2;
    while j<=length(line_list)
        r_temp2=r_save(line_list(j));
        a_temp2=a_save(line_list(j));
        if (abs(1-r_temp2/r_temp)<r_thresh) && (abs(a_temp-a_temp2)<a_thresh)
            remove_list=[remove_list j];
        end
        j=j+1;
    end
    line_count=line_count+1;
    i_save_temp=i_save(remove_list,:);
    i_save2(line_count,:)=[min(i_save_temp(:,1)),max(i_save_temp(:,2))];
    
    line_list(remove_list)=[];
    i_save(remove_list,:)=[];
    remove_list=[];
end



[~,i]=sort(i_save2(:,1));
for j=1:length(i)
    k=i(j);
    p_start=i_save2(k,1);
    p_end=i_save2(k,2);
    
    if split_merge
        %Conventional Split and Merge
        [r(k),a(k),covM{k}]=compute_line_params(P(p_start:p_end),Q(p_start:p_end),sigp,sigt);
    else
        %Iterative-End-Point-Fit, cf. page 250
        [r(k),a(k),covM{k}]=compute_line_params(P([p_start,p_end]),Q([p_start,p_end]),sigp,sigt);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Plotting Purposes Only
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %         [xline_temp,yline_temp]=compute_ra_line(r(k),a(k),P(p_start),Q(p_start),P(p_end),Q(p_end));
    xline_temp=[P(p_start)*cos(Q(p_start)) P(p_end)*cos(Q(p_end))];
    yline_temp=[P(p_start)*sin(Q(p_start)) P(p_end)*sin(Q(p_end))];
    xy_temp=[x x;y y]+R*[xline_temp;yline_temp];
    
    x_line=[x_line,xy_temp(1,:)];
    y_line=[y_line,xy_temp(2,:)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
