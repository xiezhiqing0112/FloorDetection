 
function angle=angle_x(A,B)
%向量与X轴的夹角（0--360度）
if size(A)~=[1 2] | size(B)~=[1 2]
    disp('error:the size of A and B should be equal.')
else
    distance_x=B(1)-A(1);
    distance_y=B(2)-A(2);
    if distance_x>0 && distance_y>=0
        angle=atan(distance_y/distance_x);
    elseif distance_x<=0 && distance_y>0
        angle=pi/2+atan(abs(distance_x)/distance_y);
    elseif distance_x<0 && distance_y<=0
        angle=pi+atan(distance_y/distance_x);
    elseif distance_x>=0 && distance_y<0
        angle=3*pi/2+atan(distance_x/abs(distance_y));
    end   
end
angle=angle/pi*180;
