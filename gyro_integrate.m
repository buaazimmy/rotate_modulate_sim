function [mat3x3,q] = gyro_integrate(q0,w,T)
delt_thita = w * T;
% delt_thitad = [0 -delt_thita(1) -delt_thita(2) -delt_thita(3);
%     delt_thita(1) 0 delt_thita(3) -delt_thita(2);
%     delt_thita(2) -delt_thita(3) 0 delt_thita(1);
%     delt_thita(3) delt_thita(2) -delt_thita(1) 0];
delt_thita0 = delt_thita(1)^2+delt_thita(2)^2+delt_thita(3)^2;
norm = sqrt(delt_thita0);
if norm>1e-20
    f = sin(norm/2)/(norm/2);
else
    f = 1;
end
q=[cos(norm/2);f/2*delt_thita];
q=quatmulti(q0,q);
% q = (cos(sqrt(delt_thita0)/2)*eye(4) + (sin(sqrt(delt_thita0)/2) /sqrt(delt_thita0)) * delt_thitad)*q0;
q = q/sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2);
mat3x3 = q2mat3x3(q);    
end