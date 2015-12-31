%% navigation calculate 
function [pos,vel,q,cnb_ins,diff_vep] = nav_process(T,pos0,vel0,cnb,q0,wibb,aibb) 
%% 

format long;

d2r = pi/180;  
Re=6378245;   
e=1/298.3;  
wie=7.2921*10e-5; 
%% movebase
% g=update_g(pos0,Re); 
% [RN,RM]=update_rnrm(pos0,Re,e);
%% static base
RN = 6.387042779316930e+06;
RM =6.361874375496884e+06;
g = 9.801827777279708;
%% Varibles init
lat=pos0(2);  lon =pos0(1);   h = pos0(3);
pos = pos0;
velocity = vel0;  
% cnb = q2mat3x3(q0);
cbn =cnb';

wien = [0,wie*cos(lat),wie*sin(lat)]';
wenn = [-vel0(2)/RM;vel0(1)/RN;vel0(1)/RN*tan(lat)];

aibn = cbn*aibb;
wen_x = [0 2*wien(3)+wenn(3) -2*wien(2)-wenn(2);
    -2*wien(3)-wenn(3) 0 2*wien(1)+wenn(1);
    2*wien(2)+wenn(2) -2*wien(1)-wenn(1) 0];
diff_vep = aibn + wen_x*velocity + [0;0;-g];
velocity = velocity + diff_vep * T;
lon = lon + velocity(1)/RN/cos(lat)*T;
lat = lat + velocity(2)/RM*T;
h =  h + velocity(3) * T;
pos = [lon;lat;h];
%------------------------------------------------------
wien = [0;wie*cos(lat);wie*sin(lat)];
wenn = [-velocity(2)/RM;velocity(1)/RN;velocity(1)/RN*tan(lat)];
winn = wenn + wien;
winb = cnb * winn;
wnbb = wibb - winb;
delt_thita = wnbb * T;
delt_thitad = [0 -delt_thita(1) -delt_thita(2) -delt_thita(3);
    delt_thita(1) 0 delt_thita(3) -delt_thita(2);
    delt_thita(2) -delt_thita(3) 0 delt_thita(1);
    delt_thita(3) delt_thita(2) -delt_thita(1) 0];
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
cnb = q2mat3x3(q);    
cbn = cnb';

att_ins = cnb2att(cnb);
vel = velocity;
cnb_ins = cnb;
cnb_ins= reshape(cnb_ins,9,1);
end