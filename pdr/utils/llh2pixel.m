function enu=llh2pixel(posi_p)
    px=1489;
    py=1104;
    lx=[367 787 1151 1137 703 350 179 931];
    ly=[158 143 155 1071 1056 1059 876 646];
    llh_mk=dlmread('lib_CalibrationPoints.csv',',',1,1);
    position=posi_p;
    position(:,3)=0;
    llh_mk(:,3)=0;
    orgxyz=llh_mk(1,:);
    llh=position;
    enu=llh2enu(orgxyz,llh);
    enu=enu(:,1:2);
    mk_enu=llh2enu(orgxyz,llh_mk);
    mk_enu=mk_enu(:,1:2);
    tx=367;
    ty=158;
%     T = rotation_angle(ly(1:2)',lx(1:2)',[ones(2,1) mk_enu(1:2,:)]);
    sx=double(lx(2)-lx(1))/(mk_enu(2,1)-mk_enu(1,1));
    sy=double(ly(2)-ly(1))/(mk_enu(2,2)-mk_enu(1,2));
%     temp=[enu ones(length(enu(:,1)),1)];
%     temp=round(temp*T);
    enu(:,1)=round(enu(:,1)*sx+tx);
    enu(:,2)=round(enu(:,2)*sy+ty);
%     figure
end

