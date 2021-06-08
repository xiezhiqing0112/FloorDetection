function llh=pixel2llh(enu_p)
    px=1489;
    py=1104;
    lx=[367 787 1151 1137 703 350 179 931];
    ly=[158 143 155 1071 1056 1059 876 646];
    llh_mk=dlmread('lib_CalibrationPoints.csv',',',1,1);llh_mk(:,3)=0;
    orgxyz=llh_mk(1,:);
    mk_enu=llh2enu(orgxyz,llh_mk);
    mk_enu=mk_enu(:,1:2);
    
    tx=367;
    ty=158;
%     T = rotation_angle(ly(1:2)',lx(1:2)',[ones(2,1) mk_enu(1:2,:)]);
    sx=(mk_enu(2,1)-mk_enu(1,1))/double(lx(2)-lx(1));
    sy=(mk_enu(2,2)-mk_enu(1,2))/double(ly(2)-ly(1));
%     temp=[enu ones(length(enu(:,1)),1)];
%     temp=round(temp*T);
    enu(:,1)=(enu_p(:,1)-tx)*sx;
    enu(:,2)=(enu_p(:,2)-ty)*sy;
    
    orgxyz=llh_mk(1,:);
    enu(:,3)=0;
    llh=enu2llh(enu,orgxyz);  
    
%     figure
end

