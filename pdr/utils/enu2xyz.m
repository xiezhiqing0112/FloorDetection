function xyz=enu2xyz(enu,orgllh)
    phi = orgllh(1);
    lam = orgllh(2);
    sinphi = sind(phi);
    cosphi = cosd(phi);
    sinlam = sind(lam);
    coslam = cosd(lam);
    R = [ -sinlam          coslam         0     ; ...
      -sinphi*coslam  -sinphi*sinlam  cosphi; ...
       cosphi*coslam   cosphi*sinlam  sinphi];
    [L,U]=lu(R');
    difxyz=enu/(R');
    temporg=llh2xyz(orgllh);
    tmpxyz(:,1) = difxyz(:,1) + temporg(1);
    tmpxyz(:,2) = difxyz(:,2) + temporg(2);
    tmpxyz(:,3) = difxyz(:,3) + temporg(3);
    
    xyz=tmpxyz;
end