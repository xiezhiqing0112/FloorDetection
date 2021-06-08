OutputDir = '/home/mxc/wonderland/mess/IPIN/code-ori/';%输出文件夹路径
%读入所有png格式的图像
files = dir(fullfile('//home/mxc/wonderland/IPIN/code-ori/getsensordata_matlab-master/','*.png'));
lengthFiles = length(files);
posi_w=dlmread('/home/mxc/wonderland/pixel_/floor1_w.csv');
posi=dlmread('/home/mxc/wonderland/pixel_/floor1.csv');

% posi=dlmread('/home/mxc/wonderland/pixel_/floor5_w.csv');
%进行所有图像的遍历
addpath('utils')
wifi_=[];
for n = 1:lengthFiles
%     temp=dlmread(strcat('/home/mxc/wonderland/pixel_/',files(n).name),' ',3,6);
%     lat=max(temp(:,1:3),[],2);
%     lon=min(temp(:,4:8),[],2);
%     wifi_=[wifi_;lat lon];
    Img = imread(strcat('/home/mxc/wonderland/IPIN/code-ori/getsensordata_matlab-master/','lib_floor01.png'));
    %对读入图像进行像素的遍历

    for i=1:length(posi(:,1))        
        x=int64(posi(i,2));
        y=int64(posi(i,3));
        Img(y,x,:)=[0,0,255];    
        for j=1:3
            for k=1:3
                Img(y+j,x+k,:)=[0,0,255]; 
            end
        end
    end
    
    for i=1:length(posi_w(:,1))
        x=int64(posi_w(i,2));
        y=int64(posi_w(i,3));
        Img(y,x,:)=[255 0 0];  
        for j=1:3
            for k=1:3
                Img(y+j,x+k,:)=[255 0 0]; 
            end
        end
        
    end
    imshow(Img);
    %单独保存图像
%     imwrite(Img,[OutputDir,int2str(n),'.png']);
break;
end
% wifi_p=llh2pixel(wifi_);
% wifi_=[wifi_p wifi_]; 
% 
% wifi_path='/home/mxc/wonderland/datasets/IPIN/pseudo_gps_csv/wifi_posi_new.csv';
% dlmwrite(wifi_path,wifi_,'delimiter',','); 