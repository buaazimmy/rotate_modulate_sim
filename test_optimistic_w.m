close all;
clear;
scrsz = get(0,'ScreenSize');

for i=1:12
    [vel_err(i),vel] = nav_func(pi/(i*10),2,1);

figure('Position',[scrsz(3)/6 (12-i)*scrsz(4)/16 3*scrsz(3)/6 1*scrsz(4)/6]);
subplot(1,3,1),plot(vel(1,:),'r');
grid on,axis tight,xlabel('Time(min)'),ylabel('East velocity Error/m/s');
subplot(1,3,2),plot(vel(2,:),'g');
grid on,axis tight,xlabel('Time(min)'),ylabel('North velocity Error/m/s');
subplot(1,3,3),plot(vel(3,:),'b');
grid on,axis tight,xlabel('Time(min)'),ylabel('Up velocity Error/m/s');

end
%%
figure;
plot(vel_err);