%% Standard DH
clear;
clc;
% 连杆0基座会以机械臂总坐标偏移的形式加入,单位为mm
L(1) = Link('revolute','d',45.75,'a',0,'alpha',pi/2);
% revolute形式的转动关节的theta会以offset形式进行赋值
L(2) = Link('revolute','d',0,'a',300,'alpha',0,'offset',pi/2);
L(3) = Link('revolute','d',0,'a',sqrt(255^2+68^2),'alpha',0,'offset',-atan(225/68));
L(4) = Link('revolute','d',0,'a',0,'alpha',pi/2,'offset',atan(225/68));
L(5) = Link('revolute','d',45,'a',0,'alpha',0);
Five_dof = SerialLink(L,'name','5-dof');
%使用坐标偏移加入底座
Five_dof.base = transl(0,0,69.25);
Five_dof.teach

%% Modified DH
clear;
clc;
L(1) = Link('revolute','d',0,'a',115,'modified');
L(2) = Link('revolute','d',0,'a',0,'offset',pi/2,'alpha',pi/2,'modified');
L(3) = Link('revolute','d',0,'a',300,'offset',-atan(225/68),'alpha','0','modified');
L(4) = Link('revolute','d',0,'a',sqrt(225^2+68^2),'offset',atan(225/68),'alpha','0','modified');
L(5) = Link('revolute','d',45,'a',0,'offset',0,'alpha',pi/2,'modified');