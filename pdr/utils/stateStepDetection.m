% file = 'D:\2020\IPIN2020\Logfiles1\01-Training\01a-Regular\T01_01.txt';
function [PkValue, PeakLocation]=stateStepDetection(accx,accy,accz)
    
%     data = load(file);
    zhenjia = false;
    FILTERING_VALUE = 0.84;
    value_A = 0.0;
    value_B = 0.0;
    lowX = 0;
    lowY = 0;
    lowZ = 0;
    s = 0;
    sums = 0;
    x = 0;
    y = 0;
    z = 0;
    w = 0;
    b = 0;
    c = 0;
    d = 0;
    last_step = 0;
    newsum = 0;
    time_s2 = 0;
    biaoji = 0;
    newsum2 = 0;
    o = 0;
    num = 0;
    flag = 0;
%     accx =  data(5000:end,3);
%     accy = data(5000:end,4);
%     accz = data(5000:end,5);
    acc_mean = sqrt(accx.*accx + accy.*accy + accz.*accz);
    acc_value = acc_mean;
    station = [];
    l = length(accx);
    for i=1:length(accx)
        lowX = lowX * FILTERING_VALUE + accx(i)*(1-FILTERING_VALUE);
        lowY = lowY * FILTERING_VALUE + accy(i)*(1-FILTERING_VALUE);
        lowZ = lowZ * FILTERING_VALUE + accz(i)*(1-FILTERING_VALUE);
        acc_value(i) = sqrt(lowX * lowX +lowY * lowY + lowZ*lowZ);
        flag = 0;
        value_A = acc_value(i);
        value_B = value_A - 9.8;
        num = num + 1;
        if value_B > 6.0 || value_B < -5.0
            s = 0;
            num = 0;
        else
            if s == 0 && flag == 0
                flag = 1;
                biaoji = 0;
                num =0;
                if value_B < 0.5
                    s = 0;
                else
                    s = 1;
                end
            else
                if s==1 && flag==0
                    time_s2 = 0;
                    flag = 1;
                    if value_B<0.9 && value_B>=0.5
                        s=1;
                    end
                    if value_B >= 0.9
                        s=2;
                    end
                    if value_B<0.5
                        s=4;
                    end
                end
                if s==4 && flag==0
                    flag = 1;
                    if biaoji >=10
                        s=0;
                    else
                        if value_B >= 0.5
                            s=1;
                        else
                            biaoji = biaoji +1;
                        end
                    end

                end

            end

            if s==2 && flag==0
                flag=1;
                time_s2 = time_s2 + 1;
                if time_s2 > 100
                    s = 0;
                else
                    if value_B >=0.9
                        s=2;
                    end
                    if value_B < -0.5
                        s=3;
                    end
                end
            end

            if s==3 && flag==0
                flag = 1;
                s=6;
            end

            if s==6 && flag==0
                flag = 1;
                s=0;
                sums = sums + 1;

                if sums<4
                    newsum = newsum + 1;
                    zhenjia = false;
                    if b==0
                        b = b+1;
                        y = x;
                    else
                        if c==0
                            c = c+1;
                            z=y;
                        end
                        if d==0
                            d = d+1;
                            w=z;
                        end
                    end
                else
                    if (w-x)<300 && (w-x)>20
                        newsum = newsum +1;
                        zhenjia = true;
                        b=0;
                        x=w;
                    else
                        if (x-y)<300 && (x-y)>20
                            newsum = newsum +1;
                            zhenjia = true;
                            c=0;
                            b=1;
                            y=x;
                        end
                        if (y-z)<300 && (y-z)>20
                            newsum = newsum +1;
                            zhenjia = true;
                            d=0;
                            b=1;
                            y=x;
                        end
                        if (z-w)<300 && (z-w)>20
                            newsum = newsum +1;
                            zhenjia = true;
                            w=z;
                            d=1;
                        else
                            newsum = newsum + 1;
                            zhenjia = false;
                            sums = 1;
                            b=1;
                            c=0;
                            d=0;
                            x=40;
                            y=40;
                            z=0;
                            w=0;
                        end
                    end
                end
                if zhenjia == false
                    o=0;
                else
                    if o== 0
                        newsum2 = newsum2 +3;
                    end
                    newsum2 = newsum2 +1;
                    o=1;
                end
            end

        end

        if last_step ~= newsum
            station= [station i];
            last_step = newsum;
        end


    end
    last_step_diff = diff(station);
    PkValue = last_step_diff;
    PeakLocation = station;
    PkValue = PkValue';
    PeakLocation = PeakLocation';
%     save peak_num.txt station -ascii
%     save peak_diff.txt last_step_diff -ascii
%     plot(acc_value)
    


