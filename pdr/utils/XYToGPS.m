function lonlat = XYToGPS(Origin, X, Y)
    % Origin ：最初POSI经纬度  X: N*1  Y: N*1  
    % return : N*2 经纬度坐标
%     Origin = [39.9940738588622, -0.069456420276627]; 
%     X = [0;-1.44555827447421;-1.93452032252926;8.99550968985537;9.06959216983493;11.1593089606198;11.2672877191806;-1.11408023923933;-1.11408023923933;11.2672877191806;11.1593089606198;9.06959216983493;8.99550968985537;-1.93452032252926;-1.44555827447421;0];
%     Y = [0;-2.60614429585360;-3.47702467053068;-9.31134263851163;-13.1210892846667;-13.0822239317752;-19.8388298183971;-9.73617715274600;-9.73617715274600;-19.8388298183971;-13.0822239317752;-13.1210892846667;-9.31134263851163;-3.47702467053068;-2.60614429585360;0]
%     X = X;
%     Y = Y;
    XY = [X Y];
    M_PI = 3.14159265358979323846;
    ori = GPSToXY(Origin);
    lonlat_coordinate = [];
    L = 6381372 * M_PI * 2;
    W = L;
    H = L /2;
    mill = 2.3;
    lats = [];
    lons = [];

    for i=1:length(XY)
        x = XY(i, 1)+ ori(1);
        y = XY(i, 2)+ ori(2);
        lat = ((H / 2 - y )*2*mill)/(1.25 * H);
        lat = ((atan(exp(lat)) - 0.25*M_PI))/0.4*180/M_PI;
        lon = (x - W/2)*2/W*180;
        lats = [lats lat];
        lons = [lons lon];
    end
%     plot(lats, lons, 'r');
    lonlat_coordinate = [lats' lons'];
    lonlat = lonlat_coordinate;
end