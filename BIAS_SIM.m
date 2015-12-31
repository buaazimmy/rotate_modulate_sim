clear
clc
DEG2RAD=pi/180;
% syms w t e1 e2 e3;
w=pi/90;
len=50000;
wibb=zeros(3,len);
e1=0.1 * DEG2RAD / 3600;
e2=0.1 * DEG2RAD / 3600;
e3=0.1 * DEG2RAD / 3600;
phi = pi/4; theta = atan(sqrt(2)); psi = 0;

  cpsi = cos(psi); spsi = sin(psi);
  cthe = cos(theta); sthe = sin(theta);
  cphi = cos(phi); sphi = sin(phi);

  C3 = [cpsi  spsi 0; ...
        -spsi cpsi 0; ...
         0     0   1];
  C2 = [cthe  0  -sthe; ...
          0   1     0 ; ...
        sthe  0   cthe];
  C1 = [1   0    0;   ...
        0  cphi sphi; ...
        0 -sphi cphi];  

   
  Ccs = C1 * C2 * C3;
  for i=1:len
      t=i/100;
      Czc = [cos(w*3*t),0,-sin(w*3*t);
                0,  1,  0;
            sin(w*3*t), 0,  cos(w*t*3)];
      Cbz=[cos(w.*t),sin(w.*t),0;
          -sin(w.*t),cos(w.*t),0;
          0,    0,  1];
      Cbs=Ccs*Czc*Cbz;
       reshape(Cbs,9,1);
      wibb(:,i)=Cbs*[e1;e2;e3];

  end
%   mean(wibb(1,:))
%   mean(wibb(2,:))
%   mean(wibb(3,:))
scrsz = get(0,'ScreenSize');
figure('Position',[1 1 scrsz(3)/6 scrsz(4)/2]);
plot(wibb(1,:),'DisplayName','wx');hold all;plot(wibb(2,:),'DisplayName','wy');plot(wibb(3,:),'DisplayName','wz');hold off;

