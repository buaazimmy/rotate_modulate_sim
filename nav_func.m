function [biggest_verr,vel] = nav_func(w,simmode,hours)

    % constant define
    no_rotate_mode = 1; rotate_mode = 2;
    time = hours*60*60; T = 0.1; len = time/T;
    d2r = pi / 180; wie=7.2921*10e-5;   e=1/298.3;  Re=6378245; 
    % position init
    lat = 39.9 * d2r;   lon = 116.3 * d2r;  h = 0;  pos0 = [lon;lat;h];Cbc=[1 0 0;0 1 0;0 0 1];
    %velocity init
    velocity0 = [0;0;0];
    %rotate speed
    % w=pi/30;        
    wz = w;        wy = w/3;

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
    wnbb=[0;0;0];
    aibb = zeros(3,len);
    pos = zeros(3,len);
    pos_dcm = zeros(3,len);
    vel = zeros(3,len);
    att= zeros(3,len);
    q = zeros(4,len);
    cnb_ins = zeros(9,len);
    diff_vep = zeros(3,len);
    %% data generate
    gyro_noise_v = 0.001/3600*d2r;
    acc_noise_v = 0.001/1000*g;%mg
    gyro_bias = 0.01/3600*d2r;
    acc_bias =  0.01/1000*g;%mg
    gyro_scale_factor = 1.00001;% tens of ppm /1e6
    acc_scale_factor = 1.00001; 
    % gyro_noise_v = 0;
    % acc_noise_v = 0;%mg
    % gyro_bias = 0;
    % acc_bias =  0;%mg
    % gyro_scale_factor = 1;
    % acc_scale_factor = 1;
    if simmode==no_rotate_mode
        %% no rotate mode 
        for i=1:len
            wbss = [0;0;0];
            %calculate cbs
            Czc = [1 0 0;0 1 0;0 0 1];
            Cbz = [1 0 0;0 1 0;0 0 1];
            Cbs=ccs*Czc*Cbz;
            wibb = winb + wnbb;
            wibs = Cbs * wibb;
            wiss = wibs + wbss;
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
            if mod(t,abs(2*pi/w))==0
                w = -w;
            end  
            wbss = [0;-w/3;w];
    %         if mod(t,abs(2*pi/wy))==0
    %             wz = -wz;
    %         end
    %         if mod(t,abs(2*pi/wy))== abs(pi/wy)
    %             wy = -wy;
    %         end
    %         wbss = [0;wy;wz];
            Cbs=ccs*Cbc;
            wibb = winb + wnbb;
            wibs = Cbs * wibb;
            wiss = wibs + wbss;
            %calculate gyro angle speed
            gyro(:,i) = (wiss + gyro_noise_v*randn(3,1)+gyro_bias)*gyro_scale_factor;
            %calculate acceleration
            fibb = [0;0;g];
            fibs = Cbs * fibb;
            fbss = [0;0;0];
            fiss = fibs + fbss;
            aibb(:,i) = (fiss + acc_noise_v*randn(3,1)+acc_bias)*acc_scale_factor;
            [Cbc,q_tmp] = gyro_integrate(q_tmp0,wbss,T);q_tmp0 = q_tmp;
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

    end

biggest_verr = max(vel(1,:));
end