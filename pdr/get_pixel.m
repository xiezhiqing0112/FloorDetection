path_='/home/mxc/wonderland/datasets/IPIN/pseudo_gps_csv/evaluation_llh.csv';
wifi_path='/home/mxc/wonderland/datasets/IPIN/pseudo_gps_csv/wifi_posi_new.csv';
floor_path='/home/mxc/wonderland/datasets/IPIN/pseudo_gps_csv/wifi_posi_and_floor_new.csv';

posi_llh=dlmread(path_,',',0,0);
posi_llh(:,5:6)=-posi_llh(:,5:6);
wifi_llh=dlmread(wifi_path,',',0,0);
f_t=dlmread(floor_path,',',1,2);
f_t=f_t(:,1);
wifi_llh=[f_t wifi_llh];
% wifi_llh=wifi_llh(:,2:3);
pixel_=llh2pixel(posi_llh(:,2:3));
pixel_ac=enu2pixel(posi_llh(:,5:6));

floor1=[];
floor2=[];
floor3=[];
floor4=[];
floor5=[];
floor1_w=[];
floor2_w=[];
floor3_w=[];
floor4_w=[];
floor5_w=[];
f=3;

l=length(pixel_ac(:,1));
delta_t=[];
for i=2:l
    delta_t=[delta_t;posi_llh(i,1)-posi_llh(i-1,1)];
end
for i=1:l-1
    pixel_ac(i,1)=pixel_ac(i,1)/delta_t(i);
    pixel_ac(i,2)=pixel_ac(i,2)/delta_t(i);
    
end


for i=1:length(posi_llh(:,1))
    fTime=zhidao_nearest(f_t(:,1),posi_llh(i,1));
    idx=find(f_t(:,1)==fTime);
    fIndex=floor(idx);
    if fIndex==1
        floor1=[floor1;i pixel_(i,:) pixel_ac(i,:) fIndex posi_llh(i,:)]; 
    elseif fIndex==2
        floor2=[floor2;i pixel_(i,:) pixel_ac(i,:) fIndex posi_llh(i,:)]; 
    elseif fIndex==3
        floor3=[floor3;i pixel_(i,:) pixel_ac(i,:) fIndex posi_llh(i,:)]; 
    elseif fIndex==4
        floor4=[floor4;i pixel_(i,:) pixel_ac(i,:) fIndex posi_llh(i,:)]; 
    elseif fIndex==5
        floor5=[floor5;i pixel_(i,:) pixel_ac(i,:) fIndex posi_llh(i,:)]; 
    end
end
for i=1:length(wifi_llh(:,1))
    fTime=zhidao_nearest(f_t(:,1),wifi_llh(i,1));
    idx=find(f_t(:,1)==fTime);
    fIndex=floor(idx);
    if fIndex==1
        floor1_w=[floor1_w;i fIndex wifi_llh(i,:)]; 
    elseif fIndex==2
        floor2_w=[floor2_w;i fIndex wifi_llh(i,:)]; 
    elseif fIndex==3
        floor3_w=[floor3_w;i fIndex wifi_llh(i,:)]; 
    elseif fIndex==4
        floor4_w=[floor4_w;i fIndex wifi_llh(i,:)]; 
    elseif fIndex==5
        floor5_w=[floor5_w;i fIndex wifi_llh(i,:)]; 
    end
end

save_n='/home/mxc/wonderland/pixel_/floor1.csv';
dlmwrite(save_n,floor1,'delimiter',' ','precision','%.15f'); 


save_n='/home/mxc/wonderland/pixel_/floor2.csv';
dlmwrite(save_n,floor2,'delimiter',' ','precision','%.15f'); 

save_n='/home/mxc/wonderland/pixel_/floor3.csv';
dlmwrite(save_n,floor3,'delimiter',' ','precision','%.15f'); 

save_n='/home/mxc/wonderland/pixel_/floor4.csv';
dlmwrite(save_n,floor4,'delimiter',' ','precision','%.15f'); 

save_n='/home/mxc/wonderland/pixel_/floor5.csv';
dlmwrite(save_n,floor5,'delimiter',' ','precision','%.15f'); 

save_n='/home/mxc/wonderland/pixel_/floor1_w.csv';
dlmwrite(save_n,floor1_w(:,[1 4 5 2 3 6 7]),'delimiter',' ','precision','%.15f'); 
save_n='/home/mxc/wonderland/pixel_/floor2_w.csv';
dlmwrite(save_n,floor2_w(:,[1 4 5 2 3 6 7]),'delimiter',' ','precision','%.15f'); 

save_n='/home/mxc/wonderland/pixel_/floor3_w.csv';
dlmwrite(save_n,floor3_w(:,[1 4 5 2 3 6 7]),'delimiter',' ','precision','%.15f'); 

save_n='/home/mxc/wonderland/pixel_/floor4_w.csv';
dlmwrite(save_n,floor4_w(:,[1 4 5 2 3 6 7]),'delimiter',' ','precision','%.15f'); 

save_n='/home/mxc/wonderland/pixel_/floor5_w.csv';
dlmwrite(save_n,floor5_w(:,[1 4 5 2 3 6 7]),'delimiter',' ','precision','%.15f'); 

