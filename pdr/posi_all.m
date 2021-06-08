path_='/home/mxc/wonderland/IPIN/markpoint/';
files = dir(fullfile(path_,'*.txt'));
lengthFiles = length(files);
posi=[];
for i=1:lengthFiles
    f=strcat(path_,files(i).name);
    temp=dlmread(f,',');
    posi=[posi;temp];
end
dlmwrite('/home/mxc/wonderland/pixel_/markpoint_all.txt',posi,'delimiter',',','precision','%.15f');