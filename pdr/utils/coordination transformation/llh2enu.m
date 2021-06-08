function enu=llh2enu(orgllh,llh)
for i=1:size(llh,1)
    xyz(i,:)=llh2xyz(llh(i,:));
end
orgxyz = llh2xyz(orgllh);
enu=xyz2enu(xyz,orgxyz);