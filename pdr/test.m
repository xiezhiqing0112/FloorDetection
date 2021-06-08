Kp=0.0002;Ki=0;  
step_sz=[0.6933 -0.01902 0.005733 ]; 
x0= [step_sz 1 0 zeros(1,9)]; 

% pos_path = 'F:\Homework\baseline\Logfiles\processed\Training\03c-FloorTransitions\';
% path='F:\ipin\Logfiles\01-Training\03c-FloorTransitions\';
% pos_files=dir(pos_path);
% i=3;
% while i <= 4
%         file_pre = pos_files(i).name;
%        get_pseudo_pos(pos_path,path,strcat(file_pre(2:7),'.txt'),file_pre(2:7),x0);
%     i=i+1;
% end
% 
% pos_path = 'F:\Homework\baseline\Logfiles\processed\Training\03c-FloorTransitions\';
% path = 'F:\pseudo_posi\01-Training\03c-FloorTransitions\';
% files=dir(path);
% i=3;
% while i <= 4
%     file_pre = files(i).name;
%     load(strcat(path,files(i).name));
%     markpoint=load(strcat(pos_path,'P',file_pre(1:6),'.txt'));
%     GPS_pos=markpoint(:,2:3);
%     latlon = reXYToGPS(GPS_pos(1,:), res(:,2),res(:,3));
%     i=i+1;
%     res=[res(:,1) latlon];
%     save(['F:\pseudo_gps\', strcat(file_pre(1:6),'.mat')], 'res');
% end

% function tocsv(path,save_path)
%     path = '/home/mxc/wonderland/datasets/IPIN/pseudo_gps/02-Validation/';
%     files=dir(path);
%     save_path = '/home/mxc/wonderland/datasets/IPIN/pseudo_gps_csv/02-Validation/';
%     i=3;
%     while i <= length(files)
%         file_pre = files(i).name;
%         GPS_pos = load(strcat(path,file_pre));
%         dlmwrite(strcat(save_path,file_pre(1:3),'.csv'),GPS_pos.res,'precision','%.15f');
%         i=i+1;
%     end
% end

target=2;
align_path='/home/mxc/wonderland/ipin_data/01-Training/';
save_path='/home/mxc/wonderland/datasets/IPIN/pseudo_gps_csv/02-Validation-givenyaw/';
align_folder = dir(align_path);
for i=3:length(align_folder)
    data_p=strcat(align_folder(i).folder,'/',align_folder(i).name);
    align_subfolder = dir(data_p);
    if target == 1
        for j=3:length(align_subfolder)
            cur_file=align_subfolder(j).name;
            data_s_p = strcat(align_subfolder(j).folder,'/',cur_file);
            file_p=strcat(data_s_p,'/ACCEGYROMAGNAHRS.csv');
            posi_p=strcat(data_s_p,'/POSI.csv');
            get_pseudo_posi_with_givenyaw(posi_p,file_p,save_path,cur_file,[]);
    %         get_pseudo_pos(posi_p,file_p,save_path,cur_file(1:6),[]);
        end
    elseif target==0
        cur_file=align_folder(i).name;
        data_s_p = strcat(align_folder(i).folder,'/',cur_file);
        file_p=strcat(data_s_p,'/ACCEGYROMAGNAHRS.csv');
        posi_p=strcat(data_s_p,'/POSI.csv');
        get_pseudo_posi_with_givenyaw(posi_p,file_p,save_path,cur_file(1:3),[]);
%             get_pseudo_pos(posi_p,file_p,save_path,cur_file(1:3),[]);
    
    else
        align_path='/home/mxc/wonderland/ipin_data/';
        save_path='/home/mxc/wonderland/datasets/IPIN/pseudo_gps_csv/'; 
        file_p=strcat(align_path,'ACCEGYROMAGNAHRS.csv');
        posi_p=strcat(save_path,'wifi_posi_and_floor_new.csv');
        cur_file='evaluation_llh';
        pseudo_posi_without_mk(posi_p,file_p,save_path,cur_file,[]);
        break;
    end
end

% step_sz=[ 0.6133    0.0187   -0.0530];
% 
% x_min=fminsearch(@ optim_use_origin_yaw,step_sz);%,options);
% 
% x_min