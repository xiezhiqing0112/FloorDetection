train=0;
align_path='/home/mxc/wonderland/ipin_data/01-Training/03c-FloorTransitions/';
% save_path='/home/mxc/wonderland/datasets/IPIN/pseudo_gps_csv/02-Validation-givenyaw/';
align_folder = dir(align_path);
% addpath('utils');
for i=3:length(align_folder)
    data_p=strcat(align_folder(i).folder,'/',align_folder(i).name);
    align_subfolder = dir(data_p);
    floor_all=[];
    floor_=[];
    if train == 1
        for j=3:length(align_subfolder)
            cur_file=align_subfolder(j).name;
            data_s_p = strcat(align_subfolder(j).folder,'/',cur_file);
            file_p=strcat(data_s_p,'/PRES.csv');
            posi_p=strcat(data_s_p,'/POSI.csv');
            pres=dlmread(file_p,',',2,2);
            floor=dlmread(posi_p,',',1,2);
            floor=floor(:,[1,5,6]);
            n_floor=ones(length(pres(:,1)),2);
            n_floor(:,1)=n_floor(:,1)*floor(1,2);
            n_floor(:,2)=n_floor(:,2)*floor(1,3);
            floor_all=[floor_all;n_floor pres(:,3)];
        end
    else 
        cur_file=align_folder(i).name;
        data_s_p = strcat(align_folder(i).folder,'/',cur_file);
        file_p=strcat(data_s_p,'/PRES.csv');
        posi_p=strcat(data_s_p,'/POSI.csv');
        pres=dlmread(file_p,',',2,2);
        floor=dlmread(posi_p,',',1,2);  
        floor_=[floor_;floor];
        floor=floor(:,[1,5,6]);
        idx=[];
        for i=1:length(floor(:,1))
            startTime=zhidao_nearest(pres(:,1),floor(i,1));
            startIndex=find(pres(:,1)==startTime);
            idx=[idx;startIndex];
        end
        figure 
        scatter(idx,pres(idx,3));
        
        n_floor=ones(length(pres(:,1)),2);
        n_floor(:,1)=n_floor(:,1)*floor(1,2);
        n_floor(:,2)=n_floor(:,2)*floor(1,3);
        floor_all=[floor_all;n_floor pres(:,3)];      
    end
end