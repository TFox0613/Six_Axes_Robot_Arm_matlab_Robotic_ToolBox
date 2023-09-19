%% Robotic toolbox 
%% 建立ur5 的D-H 表
alpha   = [pi/2  ,      0,        0,    pi/2,    -pi/2,       0];
a       = [0     , -0.425, -0.39225,       0,        0,       0];
d       = [.08916,      0,        0, 0.10915,  0.09456,  0.0823];
m       = [   3.7,  8.393,     2.33,   1.219,    1.219,  0.1879];
offset = 0;

%% 建立link 做建立表與圖像連結
L1 = Link(  'd', d(1),  'a', a(1),    'alpha',   alpha(1)  ,'standard' );
L2 = Link(  'd', d(2),  'a', a(2),    'alpha',   alpha(2)  ,'standard' );
L3 = Link(  'd', d(3),  'a', a(3),    'alpha',   alpha(3)  ,'standard' );
L4 = Link(  'd', d(4),  'a', a(4),    'alpha',   alpha(4)  ,'standard' );
L5 = Link(  'd', d(5),  'a', a(5),    'alpha',   alpha(5)  ,'standard');
L6 = Link(  'd', d(6),  'a', a(6),    'alpha',   alpha(6)  ,'standard');

robot_UR5=SerialLink([L1,L2,L3,L4,L5,L6],'name','UR5');   %SerialLink 將連桿做連接
robot_UR5.display();  % 顯示D-H函數


%% 順向運動學
%   robot_UR5.fkine([   theta1,     theda2,     theda3,     theda4,     theda5,     theda6])
T06 = robot_UR5.fkine([pi/2,pi/2,0,0,0,0]);
%% 逆向運動學
%   robot_UR5.ikine(T06)
theda = robot_UR5.ikine(T06);
%% 軌跡規劃
%%%%%%%%%%%%%%%%  T  = transl(" position") * rpy2tr( "eurly angle")
% T0 初始位置座標  T1 末端點位置座標 計算初始、末端Transfer Function 可以計算每一個關節angle
T0 = transl(-0.39,  0.3 , 0.3) * rpy2tr( 0 , 0 , 0); 
T1 = transl(-0.39,  -0.3 , 0.3) * rpy2tr( 0 , 0 , 0); 
theda0 = robot_UR5.ikine(T0);
theda1 = robot_UR5.ikine(T1);
t  = 0 : 0.05 :2;
[q,qd,qdd] = jtraj(theda0,theda1,t );
robot_UR5.plot( q, 'trail','b-','movie','traj1.gif')

%% test
Preal=zeros(41,3);
for j=1:41
    T=robot_UR5.fkine(q(j,:));
    Preal(j,:) = T.t;
end


