function parserCardboard(file)
    data = dlmread(file);
    data = data(58:end,:);
    flag=0;
    acc=[];
    gyro=[];
    idx=1;
    cnta=0;
    cntb=0;
    acc_gyro=zeros(52,6);
    quaterinion=[];
    for i=1:length(data(:,1))
        if data(i,1) == 1
            acc =[acc; data(i,3:5)];
            cnta=cnta+1;
        elseif data(i,1) == 2 
            gyro =[gyro; data(i,3:5)];
            cntb=cntb+1;
        elseif data(i,1) == 3 
            quaterinion = [quaterinion ; data(i,3:6)];
            flag=1;
        end
        if flag==1 && cntb>=1 && cnta>=1
            if cntb>1
                gyro = mean(gyro,1);
            end
            if cnta>1
                acc = mean(acc,1);
            end
            temp= [acc gyro];
            acc_gyro(idx,1:6)=temp;
            acc=[];
            gyro=[];
            flag=0;
            idx=idx+1;
            cnta=0;
            cntb=0;
        end
    end
    dlmwrite('/home/mxc/wonderland/datasets/IPIN/cardboard_accgyro.csv',acc_gyro,'precision','%.15f');
    dlmwrite('/home/mxc/wonderland/datasets/IPIN/cardboard_quanter.csv',quaterinion,'precision','%.15f');
end