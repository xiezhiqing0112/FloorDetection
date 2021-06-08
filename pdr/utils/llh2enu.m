function enu=llh2enu(orgllh,llh)
    for i=1:size(llh,1)
        xyz(i,:)=llh2xyz(llh(i,:));
    end
    orgxyz = llh2xyz(orgllh);
    enu=xyz2enu(xyz,orgxyz);
end
%[4893126.93717698,-5931.66286211470,4077481.48800809]
%[4893126.93717698,-5931.66286211470,4077481.48800809]