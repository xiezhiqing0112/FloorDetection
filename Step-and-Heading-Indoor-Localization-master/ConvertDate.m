function [date] = ConvertDate(x)
date = datestr((x+28800000)/86400000 + datenum(1970,1,1),31);
end