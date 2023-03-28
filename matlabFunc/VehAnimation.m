function [result] = VehAnimation(xCenter,yCenter,x_toVehCoor,y_toVehCoor,x,y,theta)
veh.width = 1.7;
veh.lf = 3.2;
veh.lb = 0.5;
veh.wb = 2.7;

result = x(1) + 0;

sz=get(0,'screensize');
figure('outerposition',sz);

videoFWriter = VideoWriter('Parking.mp4','MPEG-4');
open(videoFWriter);

% 画道路中心线
plot(xCenter,yCenter,'-- r')
hold on
% 画目标轨迹
plot(x_toVehCoor,y_toVehCoor,'-- k')
title("traj generate and track with MPC");
axis equal
xlim([-10 50]);
ylim([-5 25]);
xlabel('x [m]')
ylabel('y [m]')   
% 规划出来的轨迹，蓝色曲线
plot(x,y,'b')
px = x(1);
py = y(1);
pth = theta(1);
% 根据后轴中心的位姿计算车辆边框的位姿
[vehx,vehy] = getVehTran(px,py,pth,veh);
% 车辆边框
h1 = plot(vehx,vehy,'k');
% 车辆后轴中心
h2 = plot(px,px,'rx','MarkerSize',10);
img = getframe(gcf);
writeVideo(videoFWriter,img);
for i = 2:length(theta)
    px = x(i);
    py = y(i);
    pth = theta(i);
    [vehx,vehy] = getVehTran(px,py,pth,veh);
    % 更新h1图像句柄,把车辆边框四个角点的x坐标添加进去
    h1.XData = vehx;
    h1.YData = vehy;
    % 更新h2图像句柄,把车辆边框四个角点的y坐标添加进去
    h2.XData = px;
    h2.YData = py;
    img = getframe(gcf);
    writeVideo(videoFWriter,img);
    %         pause(0.005)
end
close(videoFWriter);
end

function [x,y] = getVehTran(x,y,theta,veh)
W = veh.width;
LF = veh.lf;
LB = veh.lb;

% 车辆的边框由四个角点确定
Cornerfl = [LF, W/2];
Cornerfr = [LF, -W/2];
Cornerrl = [-LB, W/2];
Cornerrr = [-LB, -W/2];
% 后轴中心坐标
Pos = [x,y];
% 计算四个角点的旋转矩阵,由于是刚体的一部分，旋转矩阵相同，
% 将角度转换为方向余弦矩阵，旋转顺序是ZYX
dcm = angle2dcm(-theta, 0, 0);
% 旋转变换，Cornerfl旋转后形成的列向量，位置向量3*1，最后一个是z坐标
tvec = dcm*[Cornerfl';0];
tvec = tvec';
% 平移变换
Cornerfl = tvec(1:2)+Pos;

tvec = dcm*[Cornerfr';0];
tvec = tvec';
Cornerfr = tvec(1:2)+Pos;

tvec = dcm*[Cornerrl';0];
tvec = tvec';
Cornerrl = tvec(1:2)+Pos;

tvec = dcm*[Cornerrr';0];
tvec = tvec';
Cornerrr = tvec(1:2)+Pos;

% 返回车辆边框四个角点的x,y坐标
x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];

end