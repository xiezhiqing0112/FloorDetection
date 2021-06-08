function tra=generate_tra()
    llh=dlmread('/home/mxc/wonderland/datasets/IPIN/pseudo_gps_csv/wifi_posi.csv',',',1,0);
%     floor=dlmread('evaluation_floor',',',1,1);
%     floor=llh(:,4);
    floor = ones(length(llh(:,1)),1)*6;
    building_id=ones(length(floor(:,1)),1)*70;
    point_id=[1:length(floor(:,1))]';
    tra=[point_id, llh(:,2:3), floor , building_id];
%     colName={'Point_Index','Latitude','Longitude', 'Floor_ID', 'Building_ID'};
%     data_t=array2table(array,'VariableNames',colName);
%     tra={};
%     tra{1,1}=point_id;
%     tra{1,2}=llh(:,2:3);
%     tra{1,3}=floor;
%     tra{1,4}=building_id;
    l=length(tra(:,1));
    for i = 0:ceil(l/40)
        save_n=strcat('/home/mxc/wonderland/mess/code-ori/getsensordata_matlab-master/temp_pos/temp_eval_',int2str(i),int2str(i),int2str(i),'.tra');
        dlmwrite(save_n,tra(i*40+1:min(l,(i+1)*40),:),'delimiter',' ','precision','%.15f'); 
    end
