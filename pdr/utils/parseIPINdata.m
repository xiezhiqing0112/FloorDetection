function trial = parseIPINdata(file)

data=importdata(file);
trial=zeros(length(data),23);
trial(:,1)=data(:,2);
trial(:,2)=data(:,1);
trial(:,7:9)=data(:,6:8);
trial(:,4:6)=data(:,3:5);
trial(:,10:12)=data(:,9:11);
% trial(:,21:23)=data(:,11:13);
end