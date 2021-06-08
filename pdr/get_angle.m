function angle=get_angle(a, b) 
        lng_b=b(2);
        lat_b=b(1);
        lng_a=a(2);
        lat_a=a(1);
        y = sin(lng_b-lng_a) * cos(lat_b);  
        x = cos(lat_a)*sin(lat_b) - sin(lat_a)*cos(lat_b)*cos(lng_b-lng_a);  
        angle = atan2(x, y);  
  
        angle = angle*180/pi;  
        if(angle < 0)
            angle = angle +360;
        end
end