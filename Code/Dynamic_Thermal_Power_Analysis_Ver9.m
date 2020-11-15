clc; clear; close all; feature accel on;
package_setup();

%~~~~~~~~~~~~~~~Inputs~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
tf = 10;                        %s
dt = 0.005;                     %s
tsim = [0 : dt : tf]';          %s
tol = 1e-6; 

initialAll = [0;0;0;  0;0;0;  0;0;0;   0;0;0;];
%Interpolation of Drag Force values, from 5x5 values, it becomes 181x181
values = DF_start();                            
%Interpolation of Drag Moment values
valuesM = DM_start();                          

%~~~~~~~~~~General Nature~~~~~~~~~~~~~~~~~~~~~~~~
g = 9.81;                       %m/s2, Gravity acceleration 
d = 1028.450;                   %kg/m3, Density of SaltWater

%~~~~~~~~~Water Speed~~~~~~~~~~~~~~~~~~~~~~~~~~~~
vx_w = 0;                       %speed value of i term on the xx' axis
vy_w = 0;                       %speed value of j term on the yy' axis
vz_w = 0;                       %speed value of k term on the zz' axis
%calling the vector info func to get all the values of a vector
v_water = Vector_Info(vx_w, vy_w, vz_w);  %Inertial Frame      


%~~~~~~~~~~~~~System~~~~~~~~~~~~~~~~~~~~~~~~~~~    
mass = 52.2615/9.81;            %kg Total mass 5.4326kg
Volume = 0.00518;               %m3 Total Volume of the vehicle

%Allinged with the axis system that has z positive down
I = [0.04 0 0; 0 -0.06 0; 0 0 -0.09];
% I = [0.99591 0.02663 -0.08637;  0.02649 -0.99965...
%-0.00282; -0.08642 0.00052 -0.99626];

%~~~~~~~~~~~~Thrusters~~~~~~~~~~~~~~~~~~~~~~~~~~
T1 = [2; 0; 0];                          %1i, 0j, 0k posible values
T2 = [4; 0; 0];                          %1i, 0j, 0k
T3 = [0; 0; 0];                          %0i, 0j, 1k
T4 = [0; 0; 0];                          %0i, 0j, 1k

%Distance of Thrusters from Center of Gravity
rT1 = [-0.03; -0.12; -0.04];         
rT2 = [-0.03;  0.12; -0.04];         
rT3 = [0.04;  -0.12; -0.02];        
rT4 = [0.04;   0.12; -0.02];         


%~~~~~~~~~~~~~~~~~Basic Equations~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Boyancy = Volume * d * g;               %N, Total boyancy force
WM =  [0;       0; -mass * g];          %N, Weight Matrix
BM =  [0;       0;   Boyancy];          %N, Boyancy matrix
rBM = [0.01;    0;         0];          %m, Dist of Boyancy point from CG


%~~~~~~~~~~~~~Calling the ODE45 Function~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
option = odeset('MaxStep',0.1,'abstol', tol, 'reltol', tol); 
[t,All] = ode45(@(t,All) myODE(t,All,rT1,rT2,rT3,rT4,T1,T2,T3,T4, BM,...
    rBM,WM,v_water(5:7), I,mass, v_water, values, valuesM),...
    tsim, initialAll, option);

uvw =   All(:,1:3);        %Linear Speed m/s body fixed frame
pqr = All(:,4:6);          %Angular Speed rad/s Body fixed frame
pos = All(:,7:9);          %m, Position Inetrial frame
ang = All(:,10:12);        %rad, Orientation roll, pitch, yaw Inertial fram

data = [pos,ang];

% create an object
new_object('path.mat',data,...
'model','f-16.mat','scale',0.05,... %f-16.mat or submarine.mat
'edge',[0 0 0],'face',[0 0 0],'alpha',1,...
'path','on','pathcolor',[.89 .0 .27],'pathwidth',1);
 
% aircraft trajectory visualization
flypath('path.mat',...
'animate','off','step',100,...
'axis','on','axiscolor',[0 0 0],'color',[1 1 1],...
'font','what','fontsize',10,...
'view',[21 31],...     
'output','aircraft_example.png','dpi',600);

Nu_h_Dens_vs_TEMP();
Power_Heat_Analysis();

%~~~~~~~~~~~~~My Ordinary Diferential Equation~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function All = myODE(t,All,rT1,rT2,rT3,rT4,T1M,T2M,T3M,T4M,BM,rBM,WM,aDF...
    ,I,mass,v_water,values,valuesM)  
uvw = All(1:3);               %m/s these are vertical Body fixed
pqr = All(4:6);             %rad/s Body fixed
xyz = All(7:9);             %m Inertial
ang = All(10:12);           %rad Inertial
Sqew=[0 -pqr(3) pqr(2); pqr(2) 0 -pqr(1); -pqr(2) pqr(1) 0];
TM = T1M + T2M + T3M + T4M; %N
% DFM=-uvw*0.4;
% 
 v_rel = v_water(2:4)' + uvw; %Relative Speed, v_water gives a line vector

 v_info = Vector_Info(v_rel(1),v_rel(2),v_rel(3));  %I basicaly only...
          %need the first value that this returns, the vector magnitude
 a_r = aDF' + ang;  %Relative angle, the aDF is a line vector
 u_v = [v_info(8), v_info(9), v_info(10)];

uvw_dot = (WM + BM + DF(values,v_info(1),a_r,u_v) + J1(TM,ang,true))/mass; 

pqr_dot = inv(I)*(-Sqew*I*pqr +...
  cross(rT1,T1M) + cross(rT2,T2M) + cross(rT3,T3M) + cross(rT4,T4M)...
 + J1(cross(rBM,BM),ang,false));%+J1(DM(valuesM,v_info(1),a_r),ang,false));

xyz_dot =J1(uvw,ang,true);   %roll, pitch, yaw

ang_dot =J2(pqr,ang(2),ang(1),true); %pitch, roll

All =  [uvw_dot; pqr_dot; xyz_dot; ang_dot];
end

%~~~~~~~~~~~~~~~Drag Forces at an Angle and Speed~~~~~~~~~~~~~~~~~~~~~~~~~~
function Back = DF_start()
th = 0:45:180;              %5 positions
sp = 0:0.1:0.4;           %5 positions
th_n = 0:1:180;            %181 positions
sp_n = 0:0.00221:0.4;     %181 positions


%Values for th_yx at the _yx plane  for speed=[0,0.1,0.2,0.3,0.4]
%0 moires done same with zx-90deg
    fx_yx(1,:) = [0,-0.0932,-0.3651,-0.8156,-1.4425]; 
    fy_yx(1,:) = [0,-0.0011,-0.0023,-0.0057,-0.0067];         
    fz_yx(1,:) = [0,-0.0086,-0.0315,-0.0702,-0.1198];
%45 moires done xy-45deg
    fx_yx(2,:) = [0,-0.1255,-0.4956,-1.1065,-1.9538]; 
    fy_yx(2,:) = [0, 0.0481, 0.1978, 0.4408, 0.7717];         
    fz_yx(2,:) = [0,-0.0167,-0.0673,-0.1587,-0.2910];
%90 moires done same with zy-90deg 
    fx_yx(3,:) = [0, 0.0205, 0.0829, 0.1898, 0.3414]; 
    fy_yx(3,:) = [0, 0.1329, 0.5208, 1.6229, 2.0499];         
    fz_yx(3,:) = [0, 0.0221, 0.0843, 0.1734, 0.3234];
%135 moires done xy-135deg
    fx_yx(4,:) = [0,-0.1345,-0.5206,-1.1931,-2.1186]; 
    fy_yx(4,:) = [0, 0.0050, 0.0143, 0.0415, 0.0686];         
    fz_yx(4,:) = [0,-0.0085,-0.0614,-0.0793,-0.1449];
%180 moires done 
    fx_yx(5,:) = [0,-0.0111,-0.0454,-0.0925,-0.1736]; 
    fy_yx(5,:) = [0,-0.1160,-0.4650,-1.0590,-1.8620];         
    fz_yx(5,:) = [0, 0.0490, 0.1960, 0.4155, 0.7210];

%Values for th_xz at the _xz   for speed=[0,0.1,0.2,0.3,0.4]
%0 moires done same with zy-0deg
    fx_xz(1,:) = [0, 0.0031, 0.0117, 0.0303, 0.0412];  
    fy_xz(1,:) = [0, 0.0026, 0.0056, 0.0142, 0.0189];         
    fz_xz(1,:) = [0,-0.3122,-1.2391,-2.7825,-4.9498];
 %45 moires done zx-45deg
    fx_xz(2,:) = [0,-0.2604,-1.0322,-2.3083,-4.0961]; 
    fy_xz(2,:) = [0,-0.0032,-0.0122,-0.0292,-0.0586];        
    fz_xz(2,:) = [0,-0.1274,-0.5080,-1.1472,-2.0387];
%90 moires done zx-90deg 
    fx_xz(3,:) = [0,-0.0932,-0.3651,-0.8156,-1.4425]; 
    fy_xz(3,:) = [0,-0.0011,-0.0023,-0.0057,-0.0067];         
    fz_xz(3,:) = [0,-0.0086,-0.0315,-0.0702,-0.1198];
%135 moires done zx-135deg
    fx_xz(4,:) = [0,-0.2357,-0.9362,-2.0969,-3.7270]; 
    fy_xz(4,:) = [0,-0.0020,-0.0080,-0.0151,-0.0223];         
    fz_xz(4,:) = [0, 0.1243, 0.4992, 1.1204, 1.9981];
%180 moires don same with zy-180deg
    fx_xz(5,:) = [0, 0.0109, 0.0431, 0.1028, 0.1820];
    fy_xz(5,:) = [0, 0.0025, 0.0133, 0.0221, 0.0399];         
    fz_xz(5,:) = [0, 0.3216, 1.2883, 2.9026, 5.1461];

%Values for th_zy at the _zy plane  for speed=[0,0.1,0.2,0.3,0.4]
%0 moires done zy-0deg
    fx_zy(1,:) = [0, 0.0031, 0.0117, 0.0303, 0.0412]; 
    fy_zy(1,:) = [0, 0.0026, 0.0056, 0.0142, 0.0189];         
    fz_zy(1,:) = [0,-0.3122,-1.2391,-2.7825,-4.9498];
%45 moires done zy-45deg
    fx_zy(2,:) = [0, 0.0129, 0.0578, 0.1314, 0.2451]; 
    fy_zy(2,:) = [0, 0.2382, 0.9418, 2.0627, 3.7535];         
    fz_zy(2,:) = [0, 0.4543,-0.1841,-0.3792,-0.7394];
%90 moires done zy-90deg
    fx_zy(3,:) = [0, 0.0205, 0.0829, 0.1898, 0.3414]; 
    fy_zy(3,:) = [0, 0.1329, 0.5208, 1.6229, 2.0499];         
    fz_zy(3,:) = [0, 0.0221, 0.0843, 0.1734, 0.3234];
%135 moires done zy-135deg
    fx_zy(4,:) = [0, 0.0218, 0.0895, 0.1986, 0.3564]; 
    fy_zy(4,:) = [0, 0.2442, 0.9674, 2.1693, 3.8520];         
    fz_zy(4,:) = [0,-0.0021,-0.0068,-0.0119,-0.0165];
%180 moires done zy-180deg
    fx_zy(5,:) = [0, 0.0109, 0.0431, 0.1028, 0.1820]; 
    fy_zy(5,:) = [0, 0.0025, 0.0133, 0.0221, 0.0399];         
    fz_zy(5,:) = [0, 0.3216, 1.2883, 2.9026, 5.1461];

% %Interpolating the 2D force matrix with the coordinates of (angle,speed)
for i=1:5
    DFx_yx(i,:) = interp1(sp,fx_yx(i,:),sp_n,'PCHIP'); 
    DFy_yx(i,:) = interp1(sp,fy_yx(i,:),sp_n,'PCHIP'); %_yx plane
    DFz_yx(i,:) = interp1(sp,fz_yx(i,:),sp_n,'PCHIP'); 
    
    DFx_xz(i,:) = interp1(sp,fx_xz(i,:),sp_n,'PCHIP'); 
    DFy_xz(i,:) = interp1(sp,fy_xz(i,:),sp_n,'PCHIP'); %_xz plane
    DFz_xz(i,:) = interp1(sp,fz_xz(i,:),sp_n,'PCHIP'); 
    
    DFx_zy(i,:) = interp1(sp,fx_zy(i,:),sp_n,'PCHIP'); 
    DFy_zy(i,:) = interp1(sp,fy_zy(i,:),sp_n,'PCHIP'); %_zy plane
    DFz_zy(i,:) = interp1(sp,fz_zy(i,:),sp_n,'PCHIP'); 
end
for j=1:181 
    DFx_yxe(:,j) = interp1(th,DFx_yx(:,j),th_n,'PCHIP'); 
    DFy_yxe(:,j) = interp1(th,DFy_yx(:,j),th_n,'PCHIP'); %_yx plane
    DFz_yxe(:,j) = interp1(th,DFz_yx(:,j),th_n,'PCHIP');
    
    DFx_xze(:,j) = interp1(th,DFx_xz(:,j),th_n,'PCHIP'); 
    DFy_xze(:,j) = interp1(th,DFy_xz(:,j),th_n,'PCHIP'); %_xz plane
    DFz_xze(:,j) = interp1(th,DFz_xz(:,j),th_n,'PCHIP'); 
    
    DFx_zye(:,j) = interp1(th,DFx_zy(:,j),th_n,'PCHIP'); 
    DFy_zye(:,j) = interp1(th,DFy_zy(:,j),th_n,'PCHIP'); %_zy plane
    DFz_zye(:,j) = interp1(th,DFz_zy(:,j),th_n,'PCHIP');
end

Back = vertcat(DFx_yxe,DFy_yxe,DFz_yxe, DFx_xze,DFy_xze,DFz_xze,...
    DFx_zye,DFy_zye,DFz_zye);
end
function Back = DF(values,speed,ang,unitVector)
%the angles are in rad so the function rad2deg is used
i = abs(unitVector(1));
j = abs(unitVector(2));
k = abs(unitVector(3));
DFx_yxe = values(1:181,1:181);
DFy_yxe = values(182:362,1:181);
DFz_yxe = values(363:543,1:181);

DFx_xze = values(544:724,1:181);
DFy_xze = values(725:905,1:181);
DFz_xze = values(906:1086,1:181);

DFx_zye = values(1087:1267,1:181);
DFy_zye = values(1268:1448,1:181);
DFz_zye = values(1449:1629,1:181);

speed_new = round(speed * 450 +1);
if speed_new >= 181; speed_new = 181; end
if speed_new <= 1;   speed_new = 1;   end

fx=0; fy=0; fz=0;
n=0;
th_yx = round(rad2deg(ang(3)));
n = fix(th_yx/360);
if n >= 1
    th_yx = th_yx - n * 360;
    n=0;
elseif n <= -1
    th_yx = th_yx - n * 360;
    n=0;
end
%Positive DFx_yxe(th_yx,speed_new) DFy_yxe(th_yx,speed_new)
if th_yx >= 0 && th_yx <90
    th_yx = th_yx + 1;
    fx = fx - abs(DFx_yxe(th_yx,speed_new));
    fy = fy - abs(DFy_yxe(th_yx,speed_new));
    fz = fz + DFz_yxe(th_yx,speed_new);
elseif th_yx >= 90 && th_yx <180
    th_yx = th_yx + 1;
    fx = fx + abs(DFx_yxe(th_yx,speed_new));
    fy = fy - abs(DFy_yxe(th_yx,speed_new));
    fz = fz + DFz_yxe(th_yx,speed_new);
elseif th_yx >= 180 && th_yx <270
    th_yx = 360 - th_yx + 1;
    fx = fx + abs(DFx_yxe(th_yx,speed_new));
    fy = fy + abs(DFy_yxe(th_yx,speed_new));
    fz = fz + DFz_yxe(th_yx,speed_new);   
elseif th_yx >= 270 && th_yx <360
    th_yx = 360 - th_yx + 1;
    fx = fx - abs(DFx_yxe(th_yx,speed_new));
    fy = fy + abs(DFy_yxe(th_yx,speed_new));
    fz = fz + DFz_yxe(th_yx,speed_new);   
elseif th_yx >= 360
    th_yx = th_yx - 360 + 1;
    fx = fx - abs(DFx_yxe(th_yx,speed_new));
    fy = fy - abs(DFy_yxe(th_yx,speed_new));
    fz = fz + DFz_yxe(th_yx,speed_new);    
end
%Negative
if th_yx < 0 && th_yx > -90
    th_yx = abs(th_yx) + 1;
    fx = fx - abs(DFx_yxe(th_yx,speed_new));
    fy = fy + abs(DFy_yxe(th_yx,speed_new));
    fz = fz + DFz_yxe(th_yx,speed_new);
elseif th_yx <= -90 && th_yx > -180
    th_yx = abs(th_yx) + 1;
    fx = fx + abs(DFx_yxe(th_yx,speed_new));
    fy = fy + abs(DFy_yxe(th_yx,speed_new));
    fz = fz + DFz_yxe(th_yx,speed_new);    
elseif th_yx <= -180 && th_yx > -270
    th_yx = 360 - abs(th_yx) + 1;
    fx = fx + abs(DFx_yxe(th_yx,speed_new));
    fy = fy - abs(DFy_yxe(th_yx,speed_new));
    fz = fz + DFz_yxe(th_yx,speed_new);   
elseif th_yx <= -270 && th_yx > -360
    th_yx = 360 - abs(th_yx) + 1;
    fx = fx - abs(DFx_yxe(th_yx,speed_new));
    fy = fy - abs(DFy_yxe(th_yx,speed_new));
    fz = fz + DFz_yxe(th_yx,speed_new);    
elseif th_yx <= -360
    th_yx = abs(th_yx) - 360 + 1;
    fx = fx - abs(DFx_yxe(th_yx,speed_new));
    fy = fy + abs(DFy_yxe(th_yx,speed_new));
    fz = fz + DFz_yxe(th_yx,speed_new);   
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
th_zy = round(rad2deg(ang(1)));
n = fix(th_zy/360);
if n >= 1
    th_zy = th_zy - n * 360;
    n=0;
elseif n <= -1
    th_zy = th_zy - n * 360;
    n=0;
end
%Positive
if th_zy >= 0 && th_zy <90
    th_zy = th_zy + 1;
    fx = fx + DFx_zye(th_zy,speed_new);
    fy = fy - abs(DFy_zye(th_zy,speed_new));
    fz = fz - abs(DFz_zye(th_zy,speed_new));
elseif th_zy >= 90 && th_zy <180
    th_zy = th_zy + 1;
    fx = fx + DFx_zye(th_zy,speed_new);
    fy = fy - abs(DFy_zye(th_zy,speed_new));
    fz = fz + abs(DFz_zye(th_zy,speed_new));
elseif th_zy >= 180 && th_zy <270
    th_zy = 360 - th_zy + 1;
    fx = fx + DFx_zye(th_zy,speed_new);
    fy = fy + abs(DFy_zye(th_zy,speed_new));
    fz = fz + abs(DFz_zye(th_zy,speed_new));
elseif th_zy >= 270 && th_zy <360
    th_zy = 360 - th_zy + 1;
    fx = fx + DFx_zye(th_zy,speed_new);
    fy = fy + abs(DFy_zye(th_zy,speed_new));
    fz = fz - abs(DFz_zye(th_zy,speed_new)); 
elseif th_zy >= 360
    th_zy = th_zy - 360 + 1;
    fx = fx + DFx_zye(th_zy,speed_new);
    fy = fy - abs(DFy_zye(th_zy,speed_new));
    fz = fz - abs(DFz_zye(th_zy,speed_new));    
end
%Negative
if th_zy < 0 && th_zy > -90
    th_zy = abs(th_zy) + 1;
    fx = fx + DFx_zye(th_zy,speed_new);
    fy = fy + abs(DFy_zye(th_zy,speed_new));
    fz = fz - abs(DFz_zye(th_zy,speed_new));   
elseif th_zy <= -90 && th_zy > -180
    th_zy = abs(th_zy) + 1;
    fx = fx + DFx_zye(th_zy,speed_new);
    fy = fy + abs(DFy_zye(th_zy,speed_new));
    fz = fz + abs(DFz_zye(th_zy,speed_new));    
elseif th_zy <= -180 && th_zy > -270
    th_zy = 360 - abs(th_zy) + 1;
    fx = fx + DFx_zye(th_zy,speed_new);
    fy = fy - abs(DFy_zye(th_zy,speed_new));
    fz = fz + abs(DFz_zye(th_zy,speed_new));   
elseif th_zy <= -270 && th_zy > -360
    th_zy = 360 - abs(th_zy) + 1;
    fx = fx + DFx_zye(th_zy,speed_new);
    fy = fy - abs(DFy_zye(th_zy,speed_new));
    fz = fz - abs(DFz_zye(th_zy,speed_new));   
elseif th_zy <= -360
    th_zy = abs(th_zy) - 360 + 1;
    fx = fx + DFx_zye(th_zy,speed_new);
    fy = fy + abs(DFy_zye(th_zy,speed_new));
    fz = fz - abs(DFz_zye(th_zy,speed_new));    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
th_xz = round(rad2deg(ang(2)));
n = fix(th_xz/360);
if n >= 1
    th_xz = th_xz - n * 360;
    n=0;
elseif n <= -1
    th_xz = th_xz - n * 360;
    n=0;
end
%Positive
if th_xz >= 0 && th_xz <90
    th_xz = th_xz + 90 + 1;
    fx = fx - abs(DFx_xze(th_xz,speed_new));
    fy = fy + DFy_xze(th_xz,speed_new);
    fz = fz + abs(DFz_xze(th_xz,speed_new));
elseif th_xz >= 90 && th_xz <180
    th_xz = 270 - th_xz + 1;
    fx = fx + abs(DFx_xze(th_xz,speed_new));
    fy = fy + DFy_xze(th_xz,speed_new);
    fz = fz + abs(DFz_xze(th_xz,speed_new));
elseif th_xz >= 180 && th_xz <270
    th_xz = 270 - th_xz + 1;
    fx = fx + abs(DFx_xze(th_xz,speed_new));
    fy = fy + DFy_xze(th_xz,speed_new);
    fz = fz - abs(DFz_xze(th_xz,speed_new));
elseif th_xz >= 270 && th_xz <360
    th_xz = th_xz - 270 + 1;
    fx = fx - abs(DFx_xze(th_xz,speed_new));
    fy = fy + DFy_xze(th_xz,speed_new);
    fz = fz - abs(DFz_xze(th_xz,speed_new));
elseif th_xz >= 360
    th_xz = th_xz - 270 + 1;
    fx = fx + abs(DFx_xze(th_xz,speed_new));
    fy = fy + DFy_xze(th_xz,speed_new);
    fz = fz + abs(DFz_xze(th_xz,speed_new));
end
%Negative
if th_xz < 0 && th_xz > -90
    th_xz = 90 + th_xz + 1;
    fx = fx - abs(DFx_xze(th_xz,speed_new));
    fy = fy + DFy_xze(th_xz,speed_new);
    fz = fz - abs(DFz_xze(th_xz,speed_new));
elseif th_xz <= -90 && th_xz > -180
    th_xz = abs(th_xz + 90) + 1;
    fx = fx + abs(DFx_xze(th_xz,speed_new));
    fy = fy + DFy_xze(th_xz,speed_new);
    fz = fz - abs(DFz_xze(th_xz,speed_new));
elseif th_xz <= -180 && th_xz > -270
    th_xz = abs(th_xz + 90) + 1;
    fx = fx + abs(DFx_xze(th_xz,speed_new));
    fy = fy + DFy_xze(th_xz,speed_new);
    fz = fz + abs(DFz_xze(th_xz,speed_new));
elseif th_xz <= -270 && th_xz > -360
    th_xz = th_xz + 450 + 1;
    fx = fx - abs(DFx_xze(th_xz,speed_new));
    fy = fy + DFy_xze(th_xz,speed_new);
    fz = fz + abs(DFz_xze(th_xz,speed_new));
elseif th_xz <= -360
    th_xz = th_xz + 450 + 1;
    fx = fx - abs(DFx_xze(th_xz,speed_new));
    fy = fy + DFy_xze(th_xz,speed_new);
    fz = fz - abs(DFz_xze(th_xz,speed_new));
end


Back = [i*fx; j*fy; k*fz];
end

%~~~~~~~~~~~~~~~Drag Moment at an Angle~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function Back = DM_start()
th = 0:45:180;            %5 positions
sp = 0:0.1:0.4;           %5 positions
th_n = 0:1:180;           %181 positions
sp_n = 0:0.00221:0.4;     %181 positions


%Values for th_yx at the _yx plane  for speed=[0,0.1,0.2,0.3,0.4]
%0 moires done same with zx-90deg
    mx_yx(1,:) = [0,-0.0001,-0.0006,-0.0015,-0.0026];
    my_yx(1,:) = [0, 0.0007, 0.0026, 0.0059, 0.0104];         
    mz_yx(1,:) = [0,-0.0003,-0.0008,-0.0020,-0.0024];
%45 moires done
    mx_yx(2,:) = [0,-0.0038,-0.0154,-0.0357,-0.0644];
    my_yx(2,:) = [0,-0.0001,-0.0005,-0.0001, 0.0016];         
    mz_yx(2,:) = [0, 0.0205, 0.0819, 0.1828, 0.3223];
%90 moires done same with zy-90deg
    mx_yx(3,:) = [0,-0.0026,-0.0106,-0.0242,-0.0428]; 
    my_yx(3,:) = [0,-0.0022,-0.0086,-0.0163,-0.0318];         
    mz_yx(3,:) = [0, 0.0217, 0.0854, 0.1904, 0.3373];
%135 moires done
    mx_yx(4,:) = [0,-0.0024,-0.0122,-0.0221,-0.0386]; 
    my_yx(4,:) = [0,-0.0062,-0.0284,-0.0574,-0.1017];         
    mz_yx(4,:) = [0, 0.0101, 0.0393, 0.0893, 0.1589];
%180 moires done
    mx_yx(5,:) = [0,-0.0111,-0.0454,-0.0925,-0.1736]; 
    my_yx(5,:) = [0,-0.1160,-0.4650,-1.0598,-1.8620];         
    mz_yx(5,:) = [0, 0.0490, 0.1960, 0.4155, 0.7210];

%Values for th_xz at the _xz   for speed=[0,0.1,0.2,0.3,0.4]
%0 moires done
    mx_xz(1,:) = [0, 0.0023, 0.0095, 0.0209, 0.0372]; 
    my_xz(1,:) = [0, 0.0064, 0.0264, 0.0536, 0.1060];      
    mz_xz(1,:) = [0, 0.0353, 0.1397, 0.3052, 0.5567];
%45 moires done
    mx_xz(2,:) = [0,-0.0003,-0.0014,-0.0039,-0.0070]; 
    my_xz(2,:) = [0, 0.0476, 0.1886, 0.4239, 0.7527];      
    mz_xz(2,:) = [0,-0.0011,-0.0044,-0.0106,-0.0195];
%90 moires done
    mx_xz(3,:) = [0,-0.0001,-0.0006,-0.0015,-0.0026]; 
    my_xz(3,:) = [0, 0.0007, 0.0026, 0.0059, 0.0104];      
    mz_xz(3,:) = [0,-0.0003,-0.0008,-0.0020,-0.0024];
%135 moires done
    mx_xz(4,:) = [0, 0.0001, 0.0004, 0.0009, 0.0005]; 
    my_xz(4,:) = [0,-0.0479,-0.1910,-0.4282,-0.7617];      
    mz_xz(4,:) = [0,-0.0006,-0.0025,-0.0051,-0.0090];
%180 moires done
    mx_xz(5,:) = [0,-0.0001,-0.0005,-0.0012,-0.0022]; 
    my_xz(5,:) = [0,-0.0466,-0.1867,-0.4203,-0.7447];      
    mz_xz(5,:) = [0, 0.0014, 0.0068, 0.0144, 0.0263];

%Values for th_zy at the _zy plane  for speed=[0,0.1,0.2,0.3,0.4]
%0 moires done yz-0deg
    mx_zy(1,:) = [0, 0.0002, 0.0011, 0.0027, 0.0045]; 
    my_zy(1,:) = [0, 0.0445, 0.1761, 0.3948, 0.7018];      
    mz_zy(1,:) = [0, 0.0005, 0.0010, 0.0027, 0.0028];
%45 moires done yz-45deg
    mx_zy(2,:) = [0, 0.0023, 0.0095, 0.0209, 0.0372]; 
    my_zy(2,:) = [0, 0.0064, 0.0264, 0.0536, 0.1060];      
    mz_zy(2,:) = [0, 0.0353, 0.1397, 0.3052, 0.5567];
%90 moires done zy-90deg
    mx_zy(3,:) = [0,-0.0026,-0.0106,-0.0242,-0.0428]; 
    my_zy(3,:) = [0,-0.0022,-0.0086,-0.0163,-0.0318];       
    mz_zy(3,:) = [0, 0.0217, 0.0854, 0.1904, 0.3373];
%135 moires done zy-135deg
    mx_zy(4,:) = [0,-0.0081,-0.0327,-0.0732,-0.1304]; 
    my_zy(4,:) = [0, 0.0006, 0.0030, 0.0071, 0.0129];      
    mz_zy(4,:) = [0, 0.0369, 0.1464, 0.3294, 0.5841];
%180 moires done zy-180deg
    mx_zy(5,:) = [0,-0.0001,-0.0005,-0.0012,-0.0022]; 
    my_zy(5,:) = [0,-0.0466,-0.1867,-0.4203,-0.7447];       
    mz_zy(5,:) = [0, 0.0014, 0.0068, 0.0144, 0.0263];

% %Interpolating the 2D force matrix with the coordinates of (angle,speed)
for i=1:5
    DMx_yx(i,:) = interp1(sp,mx_yx(i,:),sp_n,'PCHIP'); 
    DMy_yx(i,:) = interp1(sp,my_yx(i,:),sp_n,'PCHIP'); %_yx plane
    DMz_yx(i,:) = interp1(sp,mz_yx(i,:),sp_n,'PCHIP'); 
    
    DMx_xz(i,:) = interp1(sp,mx_xz(i,:),sp_n,'PCHIP'); 
    DMy_xz(i,:) = interp1(sp,my_xz(i,:),sp_n,'PCHIP'); %_xz plane
    DMz_xz(i,:) = interp1(sp,mz_xz(i,:),sp_n,'PCHIP'); 
    
    DMx_zy(i,:) = interp1(sp,mx_zy(i,:),sp_n,'PCHIP'); 
    DMy_zy(i,:) = interp1(sp,my_zy(i,:),sp_n,'PCHIP'); %_zy plane
    DMz_zy(i,:) = interp1(sp,mz_zy(i,:),sp_n,'PCHIP'); 
end
for j=1:181 
    DMx_yxe(:,j) = interp1(th,DMx_yx(:,j),th_n,'PCHIP'); 
    DMy_yxe(:,j) = interp1(th,DMy_yx(:,j),th_n,'PCHIP'); %_yx plane
    DMz_yxe(:,j) = interp1(th,DMz_yx(:,j),th_n,'PCHIP');
    
    DMx_xze(:,j) = interp1(th,DMx_xz(:,j),th_n,'PCHIP'); 
    DMy_xze(:,j) = interp1(th,DMy_xz(:,j),th_n,'PCHIP'); %_xz plane
    DMz_xze(:,j) = interp1(th,DMz_xz(:,j),th_n,'PCHIP'); 
    
    DMx_zye(:,j) = interp1(th,DMx_zy(:,j),th_n,'PCHIP'); 
    DMy_zye(:,j) = interp1(th,DMy_zy(:,j),th_n,'PCHIP'); %_zy plane
    DMz_zye(:,j) = interp1(th,DMz_zy(:,j),th_n,'PCHIP');
end

Back = vertcat(DMx_yxe,DMy_yxe,DMz_yxe, DMx_xze,DMy_xze,DMz_xze, DMx_zye,DMy_zye,DMz_zye);
end
function Back = DM(valuesM,speed,ang)
%the angles are in rad so the function rad2deg is used

DMx_yxe = valuesM(1:181,1:181);
DMy_yxe = valuesM(182:362,1:181);
DMz_yxe = valuesM(363:543,1:181);

DMx_xze = valuesM(544:724,1:181);
DMy_xze = valuesM(725:905,1:181);
DMz_xze = valuesM(906:1086,1:181);

DMx_zye = valuesM(1087:1267,1:181);
DMy_zye = valuesM(1268:1448,1:181);
DMz_zye = valuesM(1449:1629,1:181);

speed = round(speed * 450 +1);
if speed >= 181; speed = 181; end
if speed <= 1;   speed = 1;   end

n=0; mx=0; my=0; mz=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
th_xz = round(rad2deg(ang(2)));
n = fix(th_xz/360);
if n >= 1
    th_xz = th_xz - n * 360;
    n=0;
elseif n <= -1
    th_xz = th_xz - n * 360;
    n=0;
end
%Positive
if th_xz >= 0 && th_xz <90
    th_xz = th_xz + 90 + 1;
    mx = mx + DMx_xze(th_xz,speed);
    my = my + DMy_xze(th_xz,speed);
    mz = mz + DMz_xze(th_xz,speed);
elseif th_xz >= 90 && th_xz <180
    th_xz = 270 - th_xz + 1;
    mx = mx + DMx_xze(th_xz,speed);
    my = my + DMy_xze(th_xz,speed);
    mz = mz + DMz_xze(th_xz,speed);
elseif th_xz >= 180 && th_xz <270
    th_xz = 270 - th_xz + 1;
    mx = mx + DMx_xze(th_xz,speed);
    my = my + DMy_xze(th_xz,speed);
    mz = mz + DMz_xze(th_xz,speed);
elseif th_xz >= 270 && th_xz <360
    th_xz = th_xz - 270 + 1;
    mx = mx + DMx_xze(th_xz,speed);
    my = my + DMy_xze(th_xz,speed);
    mz = mz + DMz_xze(th_xz,speed);
elseif th_xz >= 360
    th_xz = th_xz - 270 + 1;
    mx = mx + DMx_xze(th_xz,speed);
    my = my + DMy_xze(th_xz,speed);
    mz = mz + DMz_xze(th_xz,speed);
end
%Negative
if th_xz < 0 && th_xz > -90
    th_xz = 90 + th_xz + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;
elseif th_xz <= -90 && th_xz > -180
    th_xz = abs(th_xz + 90) + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;
elseif th_xz <= -180 && th_xz > -270
    th_xz = abs(th_xz + 90) + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;
elseif th_xz <= -270 && th_xz > -360
    th_xz = th_xz + 450 + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;  
elseif th_xz <= -360
    th_xz = th_xz + 450 + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n=0;
th_yx = round(rad2deg(ang(3)));
n = fix(th_yx/360);
if n >= 1
    th_yx = th_yx - n * 360;
    n=0;
elseif n <= -1
    th_yx = th_yx - n * 360;
    n=0;
end
%Positive
if th_yx >= 0 && th_yx <90
    th_yx = th_yx + 1;
    mx = mx + DMx_yxe(th_yx,speed);
    my = my + DMy_yxe(th_yx,speed);
    mz = mz + DMz_yxe(th_yx,speed);
elseif th_yx >= 90 && th_yx <180
    th_yx = th_yx + 1;
    mx = mx + DMx_yxe(th_yx,speed);
    my = my + DMy_yxe(th_yx,speed);
    mz = mz + DMz_yxe(th_yx,speed);
elseif th_yx >= 180 && th_yx <270
    th_yx = 360 - th_yx + 1;
    mx = mx + DMx_yxe(th_yx,speed);
    my = my + DMy_yxe(th_yx,speed);
    mz = mz + DMz_yxe(th_yx,speed);
elseif th_yx >= 270 && th_yx <360
    th_yx = 360 - th_yx + 1;
    mx = mx + DMx_yxe(th_yx,speed);
    my = my + DMy_yxe(th_yx,speed);
    mz = mz + DMz_yxe(th_yx,speed);
elseif th_yx >= 360
    th_yx = th_yx - 360 + 1;
    mx = mx + DMx_yxe(th_yx,speed);
    my = my + DMy_yxe(th_yx,speed);
    mz = mz + DMz_yxe(th_yx,speed);
end
%Negative
if th_yx < 0 && th_yx > -90
    th_yx = abs(th_yx) + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;
elseif th_yx <= -90 && th_yx > -180
    th_yx = abs(th_yx) + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;
elseif th_yx <= -180 && th_yx > -270
    th_yx = 360 - abs(th_yx) + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;
elseif th_yx <= -270 && th_yx > -360
    th_yx = 360 - abs(th_yx) + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;
elseif th_yx <= -360
    th_yx = abs(th_yx) - 360 + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
th_zy = round(rad2deg(ang(1)));
n = fix(th_zy/360);
if n >= 1
    th_zy = th_zy - n * 360;
    n=0;
elseif n <= -1
    th_zy = th_zy - n * 360;
    n=0;
end
%Positive
if th_zy >= 0 && th_zy <90
    th_zy = th_zy + 1;
    mx = mx + DMx_zye(th_zy,speed);
    my = my + DMy_zye(th_zy,speed);
    mz = mz + DMz_zye(th_zy,speed);
elseif th_zy >= 90 && th_zy <180
    th_zy = th_zy + 1;
    mx = mx + DMx_zye(th_zy,speed);
    my = my + DMy_zye(th_zy,speed);
    mz = mz + DMz_zye(th_zy,speed);
elseif th_zy >= 180 && th_zy <270
    th_zy = 360 - th_zy + 1;
    mx = mx + DMx_zye(th_zy,speed);
    my = my + DMy_zye(th_zy,speed);
    mz = mz + DMz_zye(th_zy,speed);
elseif th_zy >= 270 && th_zy <360
    th_zy = 360 - th_zy + 1;
     mx = mx + DMx_zye(th_zy,speed);
    my = my + DMy_zye(th_zy,speed);
    mz = mz + DMz_zye(th_zy,speed);
elseif th_zy >= 360
    th_zy = th_zy - 360 + 1;
    mx = mx + DMx_zye(th_zy,speed);
    my = my + DMy_zye(th_zy,speed);
    mz = mz + DMz_zye(th_zy,speed);
end
%Negative
if th_zy < 0 && th_zy > -90
    th_zy = abs(th_zy) + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;
elseif th_zy <= -90 && th_zy > -180
    th_zy = abs(th_zy) + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;
elseif th_zy <= -180 && th_zy > -270
    th_zy = 360 - abs(th_zy) + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;
elseif th_zy <= -270 && th_zy > -360
    th_zy = 360 - abs(th_zy) + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;   
elseif th_zy <= -360
    th_zy = abs(th_zy) - 360 + 1;
    mx = mx + 0;
    my = my + 0;
    mz = mz + 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Back = [mx; my; mz];
end


% %~~~~~~~~~~~~~~~Eulers Angles - Rotation Matrix~~~~~~~~~~~~~~~~~~~~~~~~~~
function back = J1(Local, ang, bodytoearth)
%The angles are in rad so the trigonometric functions are for rads 
r=ang(1); p=ang(2); y=ang(3);%r=roll, p=pitch, y=yaw 

S = @ (x) sin(x);
C = @ (x) cos(x);

Rx = [1 0 0; 0 C(r) -S(r); 0 S(r) C(r)]; %Roll
Ry = [C(p) 0 S(p);  0 1 0;  -S(p) 0 C(p)]; %Pitch
Rz = [C(y) -S(y) 0;  S(y) C(y) 0;  0 0 1]; %Yaw 
Rzyx = Rz * Ry * Rx;
                                      
if bodytoearth
    %Inertial = Transform Matrix * Body Fixed
    back = Rzyx * Local;
else
    %Body Fixed = Transform Matrix * Inertial
    back = inv(Rzyx) * Local;
end
end

function newMatrix = J2(Local, p, r, bodytoearth)
%The angles are in rad so the trigonometric functions are for rads 

R = [cos(p) 0 -cos(r)*sin(p); 0 1 sin(r); sin(p) 0 cos(r)*cos(p)];
if bodytoearth
    %Inertial = Transform Matrix * Body Fixed
    newMatrix = inv(R) * Local;
else
    %Body Fixed = Transform Matrix * Inertial
    newMatrix = R * Local;
end
end    

%~~~~~~~~~~~~~~~Analysing The Vectors to Angles & Parts~~~~~~~~~~~~~~~~~~~~
function Back = Vector_Info(Vx, Vy, Vz)
%Everything is in rad
%Metro
V = sqrt(Vx^2 + Vy^2 + Vz^2);
if isnan(V) == 1; V = 0;disp("gtfytfytft"); end;
    
UnitVetrors = [Vx Vy Vz] / V;
i = UnitVetrors(1);
j = UnitVetrors(2);
k = UnitVetrors(3);
if isnan(i) == 1; i=0; end
if isnan(j) == 1; j=0; end
if isnan(k) == 1; k=0; end

if Vx == 0 && Vy == 0 && Vz == 0
    A = 0;    B = 0;    C = 0;
end

%Angles in rads
A = (acos(abs(Vx)/V));
B = (acos(abs(Vy)/V));
C = (acos(abs(Vz)/V));

if Vx == 0 && Vy == 0 && Vz == 0
    A = 0;    B = 0;    C = 0;
end

Back = [V, Vx, Vy, Vz, A, B, C, i, j, k];
end

%~~~~~~~~~~~~~~~Power vs Thrust plot~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%the graphs allow for perfecting the degree of power of each polyonim
function Back = PowerFromMotorThrust(thrust)
thr = -4:1:6;           %thrust steps in lbf
thr = thr * 4.448;      %Convert the thrust to newtons

power(1) = 160;         %power values in watt
power(2) = 108;
power(3) = 58;
power(4) = 23;
power(5) = 0;
power(6) = 12;
power(7) = 30;
power(8) = 58;
power(9) = 85;
power(10) = 125;
power(11) = 160;

p1_fx = polyfit(thr,power,5);
p2_fx = polyval(p1_fx,thr);

figure("Name",'Submarine Analysis: Positon, Heat, Operatio Time')
title('Power vs Thrust')
xlabel('Thrust(each motor) [N]')
ylabel('Power [W]')
hold on
plot(thr,p2_fx,'b')

Back = polyval(p1_fx,thrust);
end

%~~~~~~~~~~~~~~~Nusselt, h, dynamic viscosity vs Temp~~~~~~~~~~~~~~~~~~~~~~
function Nu_h_Dens_vs_TEMP()
    Cp_saltw=4000;  %J/(kg*K)
    k_wat=0.6;      %W/(m*K)
    Ts = 19;        %C

    m_inf = -3.4000e-05 * 19 + 0.00189; %Pa*s
    D = 0.2166;     %m
    V_water = 0.1;  %m/s
    density = 1025; %kg/m3

    Pr = (Cp_saltw*m_inf) / (k_wat);
    Re = (V_water * density * D)/m_inf;
    if Pr <= 10; n=0.36; else; n=0.37; end
    if Re >= 1 && Re <= 40; c=0.75; m=0.4;
    elseif Re > 40 && Re <= 1e3; c=0.51; m=0.5;
    elseif Re > 1e3 && Re <= 2e5; c=0.26; m=0.6;
    elseif Re > 2e5 && Re <= 1e6; c=0.076; m=0.7;
    end

    for Ts=0:100
        Dyn_Visc =  0.0000259 * 10^ (247.8/(Ts+273.16 - 140));
            if Dyn_Visc <= 0; Dyn_Visc=0.00002; end
        Pr_s = (Cp_saltw*Dyn_Visc) / (k_wat);
        Nu(Ts+1) = c * Re^m * Pr^n * (Pr/Pr_s)^0.25;
        h(Ts+1) = (Nu(Ts+1) * k_wat) / D;
        p(Ts+1) = Dyn_Visc;
    end
    
    figure('Name','Nusselt and h value')
    subplot(3,1,1)
    plot(0:100,Nu)
    ylabel('Nusselt')
    xlabel('Surface Temperature [C]')

    subplot(3,1,2)
    plot(0:100,h)
    ylabel('h value [W/(m^2K)]')
    xlabel('Surface Temperature [C]')

    subplot(3,1,3)
    plot(0:100,p)
    ylabel('Dynamic Viscosity [Pa*s]')
    xlabel('Surface Temperature [C]')
    
    for T=0:1:100
        r(T+1) = (1002 - 1028)/(100-20) * T + 1031.6;
    end
    
     figure("Name","Density vs Temperature")
     plot(0:100,r)
     title("Density vs Temperature")
     xlabel('Temp [C]')
     ylabel('Density [kg/m3]')
end

%~~~~~~~~~~~~~~~~~~~Power Analysis~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function Power_Heat_Analysis()
%It does not take the consumbtion of each part live, but the worst 
%case senario values    
BE = 117;           %Wh, Energy stored in lipo Battery 

%N, the newtons of thrust by each motor, a basic average value
Thrust = 1.5;          
aveSpeed = 0.1;      %m/s, the average speed of the vehicle
aveWaterSpeed = 0;   %m/s, the average speed of the water current

%W, power of the two Lights that are outside the enclosure
LightsPower = 30 * 2;  

%W, power of motor + power of all the rest + power of the 2 lights
%(this effects only the steady state TEMP)
PC = 2 * PowerFromMotorThrust(Thrust) + 16.4352 + LightsPower;  

%s, The etimated total operation time, how long the battery will last  
%(this effects only the settling time)
OperTime = (BE / PC) * 60 * 60;                                 

%~~~~~~~~~~~~~~~~~~~Distanse Calculation~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TotDist = OperTime * (aveSpeed);    %m, 0.1 is the average speed 
%~~~~~~~~~~~~~~~~~~Work Produced by the Thrusters~~~~~~~~~~~~~~~~~~~~~~~~~~
%J, Work done by the thrusters, including all the drag forces and moments 
WorkThr = TotDist * Thrust * 2;          

%%%~~~~~~~~~~~~~~~~~~~~Thermal Analysis~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%W, energy that is converted from chemical to thermal, (TotalEnergy = Heat
%+ WorkMotors + Lights)
Qout = BE / (OperTime/3600) - (WorkThr / OperTime + LightsPower);

%~~~~~~~~~~~~~~~Boundarie Conditions Temps~~~~~~~~~~~~~~~~~~
Tin(1) = 20;            %Celcius, Inside Temperature
Tout(1) = 20;           %Celcius, Outside at infinite distanse, Temperature
Q_buildup(1) = 0;       %W, heat build up inside the submarine

%~~~~~~~~~~~~~~Properties of Problem~~~~~~~~~~~~~~~~~~~~~~~~
%kg, Mass of the submarine that in the core and can keep heat 
%(this effects only the settling time)
mass_TH = 1.5176 ;      
d = 1028.450;           %kg/m3, Density of SaltWater
k_plex = 0.185;         %W/mK, Thermal conduction coefficient of plexiglas
k_al = 237;             %W/mK, Thermal conduction coefficient of aluminum
k_saltwater = 0.6;      %W/mK, Thermal conduction coefficien of salt water
Cp_saltwater = 4000;    %J/kgK, Heat Capasity of salt water
Rin = 0.05;             %m, Inside Diameter
Rout = 0.057;           %m, Outside Diameter
L = 0.326;              %m, Length of tube
t_cpa = 0.012;          %m, Thickness of back plate aluminum
t_cpp = 0.012;          %m, Thickness of front plate plexiglas

%m^2, Area of back plate from the outside 
%(I estimated the area to be the mean diameter in & out)
A_bp = 2 * pi * ((Rout + Rin) / 2)^2;       

%~~~~~~Total Spesific Heat Capasity Calculation~~~~~~~~~~~~~
CpAir = 1007;           %J/kgK, Heat Capasity of air in 20C and 1bar

%J/kgK, Heat Capasity of the plastics inside(pcbs, wires, etc)
CpPlastic = 1100;       
CpCopper = 385;         %J/kgK, Heat Capasity of copper
CpAluminum = 897;       %J/kgK, Heat Capasity of aluminum

%J/kgK, Heat Capasity in general of the whole submarine 
%(this effects only the settling time)
CpTotal = 0.5 * CpAir + 0.3 * CpPlastic + 0.195 * CpAluminum+0.05*CpCopper;       
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%Reynolds, Prundlt and Nussels for external forced convection heat exchange
%m/s, this is the combined speed 
%(speed of vehicle=0.2 + speed of water in the opposite direction=0.2)
Vmax = aveSpeed + aveWaterSpeed;   

%m, Characteristic diameter, it is set to be 10% larger 
%than the enclosures crossection             
D_characteristic = Rout + Rout * (1 - 0.1);                      

%Pa*s, The dynamic viscosity is changing with the temperature(Ts=19C)
Dyn_Visc =  0.0000259 * 10^ (247.8/(19+273.16 - 140));         
Re = (Vmax * D_characteristic * d) / Dyn_Visc;           %Reynolds number
Pr = (Dyn_Visc * Cp_saltwater) / k_saltwater;            %Prandtl number

%~~~~Conditions for the constants for calculating Nusselt~~~~
if Pr <= 10; n=0.36; else; n=0.37; end
if Re >= 1 && Re <= 40; c=0.75; m=0.4;
elseif Re > 40 && Re <= 1e3; c=0.51; m=0.5;
elseif Re > 1e3 && Re <= 2e5; c=0.26; m=0.6;
elseif Re > 2e5 && Re <= 1e6; c=0.076; m=0.7;
end

%~~~~~~~~Heat Flow Resistanse~~~~~~~~~~~~~~~~~~~~~~~~~~~     
%K/W, Resistanse of the Pipe the ln in matlab is log                                     
R_ctp = log(Rout/Rin) / (2 * pi * L * k_plex);                  
R_cpa = t_cpa / (k_al * A_bp);       %K/W, Resistanse of the aluminum plate
R_cpp = t_cpp / (k_plex * A_bp);     %K/W, Resistanse of the plastic plate

%This is from the energy balance equation, there is heat generation, 
%heat loss and heat stored. The change of the inside temp and the heat 
%build up every second for time seconds, every second. Since the heat used 
%here is per second if we multiply by 1 sec then it's J
for i = 1:1:round(OperTime-1)
    %Celsius a vector is created for ploting later 
    Tout(i+1) = Tout(i);   
    
    %Pa*s, The dynamic viscosity is changing with the temperature(Ts, C)                                             
    Dyn_Visc =  0.0000259 * 10^ (247.8/(Tout(i+1)+273.16 - 140));                               
    Pr_s = (Cp_saltwater*Dyn_Visc) / (k_saltwater);
    Nu(i+1) = c * Re^m * Pr^n * (Pr/Pr_s)^0.25;
    h_saltwater(i+1) = (Nu(i+1) * k_saltwater) / D_characteristic;
    
    %K/w, there are 3 surfaces that are parallel     
    R_conv = 1 / (1 / (h_saltwater(i+1) * (2 * pi * Rout * L)) + 2 /...
        (h_saltwater(i+1) * A_bp)); 
    
    %K/w, Total Heat Resistance
    R_tot = 1/(1/R_ctp + 1/R_cpa + 1/R_cpp) + R_conv;       
    
    %W, heat been lost through all the walls of the submarine                             
    Q_lose(i) = (Tin(i) - Tout(i)) / R_tot;        
    
    %W, heat been consentraited inside the sabmarine mass                                  
    Q_buildup(i+1) = Qout - Q_lose(i) + Q_buildup(i);                                          
    if Q_lose(i) > Qout; disp("heat loss is greater than the heat added");
    end
    
    %Celsius, The q_buildup is in J/s so by mul/ing it with 1 second makes
    %it Joules
    Tin(i+1) = Tout(i) + (Q_buildup(i+1) * 1) / (mass_TH * CpTotal);                            
end

disp("~~~(The Power Consumption changes the steady state of Tin)");
disp("~~~(The Operation Time changes the settling time of the final Tin)");
disp("The total power consumed by all the parts, worst case: " +PC+" [W]");
disp("The operation time is: " +OperTime/60+" [min], worst case scenario");
disp("The total estimated operation distance:" + TotDist + " [m]");
disp("The total work done by 2 thrusters is: "+ WorkThr/1000 + " [KJ]");
disp("The energy per sec that is converted from chemical to thermal"+...
"inside the vehicle is: " + Qout + " [W]");
disp("~~~(The Mass changes the settling time of the final Tin)");    
disp("the h of the salt water is:  "+ h_saltwater(end) + " [W/(m^2K)]");
disp("Heat Capacity of the inside mass is: " + CpTotal + " [J/kgK]");
disp("The total heat flow resistance is: " + R_tot + " [K/w]");
disp("The Heat Generated in the core is: " + Qout + " [W]");
disp("The max Tin is: " + Tin(end) + " [C]");

%~~~~~~~~Ploting~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
figure("Name", "Temp change wrt Time, Tin and Tout")
title("Temp change wrt Time, Tin and Tout") %wrt: with respect to
xlabel("t [min]")
ylabel("T [Celsius ]")
hold on
axis([0,OperTime/60+10,0,Tin(end)+10]);
plot((1:round(OperTime))/60, Tin, 'r')%The red is the Temp inside
%The blue is the Temp of the water outside
plot((1:round(OperTime))/60, Tout, 'b')
end