function [RN,RM]=update_rnrm(pos,Re,e)
RN = (Re + pos(3))*(1 + e*sin(pos(2))^2);
RM = (Re + pos(3))*(1 - 2*e+3*e*sin(pos(2))^2);
end