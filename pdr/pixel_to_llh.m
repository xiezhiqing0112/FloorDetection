file_path='res_floor_1_1129.txt';
pixel=dlmread(file_path,' ',0,0);
addpath('utils');
llh=pixel2llh(pixel(:,1:2));