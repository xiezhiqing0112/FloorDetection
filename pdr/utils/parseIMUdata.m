function trial = parseIMUdata(file)

A=importdata(file);
data=A.data;
trial=zeros(length(data),23);
trial(:,1)=data(:,10);
trial(:,7:9)=data(:,4:6);
trial(:,4:6)=data(:,1:3);
trial(:,10:12)=data(:,7:9);
trial(:,21:23)=data(:,11:13);

end