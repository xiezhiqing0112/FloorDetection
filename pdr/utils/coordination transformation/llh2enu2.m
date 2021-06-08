function enu=llh2enu2(orgxyz,llh)
for i=1:size(llh,1)
    xyz(i,:)=llh2xyz(llh(i,:));
end
enu=xyz2enu(xyz,orgxyz);