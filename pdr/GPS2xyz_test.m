train=0;
align_path='/home/mxc/wonderland/ipin_data/01-Training/';
save_path='/home/mxc/wonderland/datasets/IPIN/pseudo_gps_csv/';
align_folder = dir(align_path);
for i=3:length(align_folder)
    data_p=strcat(align_folder(i).folder,'/',align_folder(i).name);
    align_subfolder = dir(data_p);
    for j=3:length(align_subfolder)
        cur_file=align_subfolder(j).name;
        data_s_p = strcat(align_subfolder(j).folder,'/',cur_file);
        file_p=strcat(data_s_p,'/ACCEGYROMAGNAHRS.csv');
        posi_p=strcat(data_s_p,'/POSI.csv');
              
        addpath('utils')
        re_position=dlmread(posi_p,',',1,2);
        re_position=re_position(:,[3 4]);
        re_position(:,3)=0;
        orgxyz=re_position(1,:);
        llh=re_position;
        enu=llh2enu(orgxyz,llh);
        figure
        plot(re_position(:,1),re_position(:,2),'b');
        hold on
        plot(enu(:,1),enu(:,2),'r');
%         enu(:,3)=0;
%         xyz = enu2xyz(enu,re_position(1,:));
        llh=enu2llh(enu,re_position(1,:));
        
        plot(gps(:,1),gps(:,2),'r');
    end
end