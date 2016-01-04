%% This is a navigation process function test script
clear;close all;format long;
Nav_wait = waitbar(0,'Navigation Preparing ...');
% mode define 
no_rotate_mode = 1; rotate_mode = 2;
simmode = rotate_mode;
% constant define
time = 2*60*60; T = 0.1; len = time/T;
d2r = pi / 180; wie=7.2921*10e-5;   e=1/298.3;  Re=6378245; 
% position init
lat = 39.9 * d2r;   lon = 116.3 * d2r;  h = 0;  pos0 = [lon;lat;h];Cbc=[1 0 0;0 1 0;0 0 1];
%velocity init
velocity0 = [0;0;0];
%rotate speed
w=pi/30;        wz = w;        wy = w/3;

[RN,RM]=update_rnrm(pos0,Re,e);
g=update_g(pos0,Re);
wien = [0,wie*cos(lat),wie*sin(lat)]';
wenn = [-velocity0(2)/RM;velocity0(1)/RN;velocity0(1)/RN*tan(lat)];
winn = wenn + wien;
cnb=[1 0 0;0 1 0;0 0 1];
%calc ccs: matrix between IMU sensor frame and rotate frame
ccs = att2mat_321( pi/4, atan(sqrt(2)),0);
ccs=[1 0 0;0 1 0;0 0 1];
q0 = [1;0;0;0];q_tmp0 = [1;0;0;0];q_tmp = [1;0;0;0];
winb = cnb * winn;
%varible preallocate
gyro = zeros(3,len);
wnbb=[0;0;0];wbss=[0;0;0];
aibb = zeros(3,len);
pos = zeros(3,len);
pos_dcm = zeros(3,len);
vel = zeros(3,len);
att= zeros(3,len);
q = zeros(4,len);
cnb_ins = zeros(9,len);
diff_vep = zeros(3,len);
%% data generate
gyro_noise_v = 1/1e3/3600*d2r;
acc_noise_v = 1/1e6*g;%ug
gyro_bias = 3/1e3/3600*d2r;
acc_bias =  5/1e6*g;%ug
gyro_scale_factor = 10/1e6 +1;% tens of ppm /1e6
acc_scale_factor = 10/1e6  +1; 
% gyro_noise_v = 0;
% acc_noise_v = 0;%mg
% gyro_bias = 0;
% acc_bias =  0;%mg
% gyro_scale_factor = 1;
% acc_scale_factor = 1;
if simmode==no_rotate_mode
    %% no rotate mode 
    for i=1:len
        wbss(:,i) = [0;0;0];
        %calculate cbs
        Czc = [1 0 0;0 1 0;0 0 1];
        Cbz = [1 0 0;0 1 0;0 0 1];
        Cbs=ccs*Czc*Cbz;
        wibb = winb + wnbb;
        wibs = Cbs * wibb;
        wiss = wibs + wbss(:,i);
        %calculate gyro angle speed
        gyro(:,i) = (wiss + gyro_noise_v*randn(3,1)+gyro_bias)*gyro_scale_factor;
        %calculate acceleration
        fibb = [0;0;g];
        fibs = Cbs * fibb;
        fbss = [0;0;0];
        fiss = fibs + fbss;
        aibb(:,i) = (fiss + acc_noise_v*randn(3,1)+acc_bias)*acc_scale_factor;
    end
end
if simmode == rotate_mode
     %% rotate mode 
    for i=1:len
        t= i*T;
%         if mod(t,abs(2*pi/w))==0
%             w = -w;
%         end  
%         wbss(:,i) = [0;-w/3;w];
        if mod(t,abs(2*pi/wy))==0
            wz = -wz;
        end
        if mod(t,abs(2*pi/wy))== 0
            wy = -wy;
        end
        wbss(:,i) = [0;wy;wz];
        Cbs=ccs*Cbc;
        wibb = winb + wnbb;
        wibs = Cbs * wibb;
        wiss = wibs + wbss(:,i);
        %calculate gyro angle speed
        gyro(:,i) = (wiss + gyro_noise_v*randn(3,1)+gyro_bias)*gyro_scale_factor;
        %calculate acceleration
        fibb = [0;0;g];
        fibs = Cbs * fibb;
        fbss = [0;0;0];
        fiss = fibs + fbss;
        aibb(:,i) = (fiss + acc_noise_v*randn(3,1)+acc_bias)*acc_scale_factor;
        [Cbc,q_tmp] = gyro_integrate(q_tmp0,wbss(:,i),T);q_tmp0 = q_tmp;
    end
end

%% process
for i=1:len
    [position,velocity,quat,matcnb,accb,attitude] = nav_process(T,pos0,velocity0,cnb,q0,gyro(:,i),aibb(:,i));
    pos0 = position;pos(:,i) = position;
    velocity0 = velocity; vel(:,i) = velocity;
    cnb_ins(:,i) = matcnb; cnb = reshape(matcnb,3,3);%cnb=[1 0 0;0 1 0;0 0 1];
    q0= quat; q(:,i)=quat;
    diff_vep(:,i) = accb;
	att(:,i) = attitude;
    pos_dcm(1,i)= (Re+pos0(3))*cos(pos0(2))*cos(pos0(1));
    pos_dcm(2,i)= (Re+pos0(3))*cos(pos0(2))*sin(pos0(1));
    pos_dcm(3,i)= (Re+pos0(3))*sin(pos0(2));
    
    Bar_remainder = mod(i,1000);
    if Bar_remainder == 0
        waitbar(i/len,Nav_wait,['Navigating...',num2str(fix(i/len*100)),'%']);
    end
end
close(Nav_wait);

%% plot
t = (1:len)*T/60;
scrsz = get(0,'ScreenSize');
figure('Position',[1*scrsz(3)/6 1 3*scrsz(3)/6 4*scrsz(4)/6]);
max(vel(1,:))
subplot(4,3,4),plot(t,vel(1,:),'r');
grid on,axis tight,xlabel('Time(min)'),ylabel('East velocity Error/m/s');
subplot(4,3,5),plot(t,vel(2,:),'g');
grid on,axis tight,xlabel('Time(min)'),ylabel('North velocity Error/m/s');
subplot(4,3,6),plot(t,vel(3,:),'b');
grid on,axis tight,xlabel('Time(min)'),ylabel('Up velocity Error/m/s');

subplot(4,3,7),plot(t,Re*pos(1,:)-Re*lon,'r');
grid on,axis tight,xlabel('Time(min)'),ylabel('East Postion Error/m');
subplot(4,3,8),plot(t,Re*pos(2,:)-Re*lat,'g');
grid on,axis tight,xlabel('Time(min)'),ylabel('North Postion Error/m');
subplot(4,3,9),plot(t,pos(3,:),'b');
grid on,axis tight,xlabel('Time(min)'),ylabel('Up Postion Error/m');

subplot(4,3,1),plot(t,diff_vep(1,:),'r');
grid on,axis tight,xlabel('Time(min)'),ylabel('East acc Error/m/s^2');
subplot(4,3,2),plot(t,diff_vep(2,:),'g');
grid on,axis tight,xlabel('Time(min)'),ylabel('North acc Error/m/s^2');
subplot(4,3,3),plot(t,diff_vep(3,:),'b');
grid on,axis tight,xlabel('Time(min)'),ylabel('Up acc Error/m/s^2');

subplot(4,3,10),plot(t,att(1,:)/d2r,'r');
grid on,axis tight,xlabel('Time(min)'),ylabel('thita/deg');
subplot(4,3,11),plot(t,att(2,:)/d2r,'g');
grid on,axis tight,xlabel('Time(min)'),ylabel('gama/deg');
subplot(4,3,12),plot(t,att(3,:)/d2r,'b');
grid on,axis tight,xlabel('Time(min)'),ylabel('kfai/deg');

% figure('Position',[4*scrsz(3)/6 1 2*scrsz(3)/6 scrsz(4)/2]);
% [x,y,z] = sphere;mesh(x.*Re,y.*Re,z.*Re);hold on;
% plot3(pos_dcm(1,:)',pos_dcm(2,:)',pos_dcm(3,:)','r');
figure 
plot(wbss(2,:),'r');hold on
plot(wbss(3,:),'g');