% R=2.0;
% %Q=8;
% %M=500;
% alpha=0:pi/50:2*pi;%角度[0,2*pi] 
% a = gray(100);

AgentNumber = 7;
% framestart = 2000;
FrameNumber = 100 ;

% LEFT = -100.0;
% RIGHT = 1200.0;
% TOP = 250.0;
% BOTTOM = -250.0;
 LEFT = 50;
  RIGHT = 65;
  TOP = 50.0;
  BOTTOM = 10.0;
  
CENTER_RIGHT = 0;
CENTER_TOP = 0;

set(0,'DefaultFigureVisible', 'off')

% for frame = framestart:1:FrameNumber
    for frame = 1:1:FrameNumber
szBuffer = sprintf('../output/3d/s6/cmx%05d.txt',frame); 
f1=fopen(szBuffer);
cla;
i=1;
while 1
nextline = fgetl(f1); %读第一行 
if ~isstr(nextline), break, end %读到最后跳出 
%disp(nextline);%这行可以不要 
a = sscanf(nextline, '%f %f %f');%读取数据，根据你自己的需要改 
data(i) = a(1);
data(i+1) = a(2);
data(i+2) = a(3);
% data(i+3) = a(4);
i=i+3;
end 
fclose(f1);

szBuffer = sprintf('../input/3d/3dbuilding (2).txt');
f1=fopen(szBuffer);
i=1;
nextline = fgetl(f1);%读第一行 
if ~isstr(nextline), break, end %读到最后跳出 
%disp(nextline);%这行可以不要 
a = sscanf(nextline, '%d');%读取数据，根据你自己的需要改 
for num=1:1:a
    nextline = fgetl(f1); %读第一行 
    if ~isstr(nextline), break, end %读到最后跳出 
   % disp(nextline);%这行可以不要 
    b = sscanf(nextline, '%f %f');
    data_obj(i)   = b(1);
    data_obj(i+1) = b(2);
    i = i+2;
end
nextline = fgetl(f1); %读第一行 
a = sscanf(nextline, '%d');%读取数据，根据你自己的需要改。障碍物个数

for num=1:1:a %
    nextline = fgetl(f1); %读第一行 
    if ~isstr(nextline), break, end %读到最后跳出 
   % disp(nextline);%这行可以不要   
    vertnum = sscanf(nextline, '%d ');%顶点个数
    
    for i=2:1:vertnum(1)+1
        b(1) = vertnum(i)+1;
            t = i+1;
        if t> vertnum(1)+1
            t = 2;
        end
        b(2) = vertnum(t)+1;
        plot([data_obj(2*b(1)-1) data_obj(2*b(2)-1)],[data_obj(2*b(1)) data_obj(2*b(2))]);
    end
     
end
fclose(f1);

szBuffer = sprintf('../input/3d/1.txt');
f1=fopen(szBuffer);
i=1;
nextline = fgetl(f1); %读第一行 
% if ~isstr(nextline), break, end %读到最后跳出 
% disp(nextline);%这行可以不要 
a = sscanf(nextline, '%d');%读取数据，根据你自己的需要改 
for num=1:1:a
    nextline = fgetl(f1); %读第一行 
    if ~isstr(nextline), break, end %读到最后跳出 
   % disp(nextline);%这行可以不要 
    b = sscanf(nextline, '%f %f');
    data_roadmap(i)   = b(1);
    data_roadmap(i+1) = b(2);
    %plot(data_roadmap(i), data_roadmap(i+1),'m*');
    i = i+2;
   
end
nextline = fgetl(f1); %读第一行 
a = sscanf(nextline, '%d');%读取数据，根据你自己的需要改 
for num=1:1:a
    nextline = fgetl(f1); %读第一行 
    if ~isstr(nextline), break, end %读到最后跳出 
   % disp(nextline);%这行可以不要 
    b = sscanf(nextline, '%d %d');
    b(1) = b(1)+1;
    b(2) = b(2)+1;
    %plot([data_roadmap(2*b(1)-1) data_roadmap(2*b(2)-1)],[data_roadmap(2*b(1)) data_roadmap(2*b(2))],'b');
end
fclose(f1);


plot([LEFT,RIGHT],[BOTTOM,BOTTOM]);
plot([LEFT,RIGHT],[TOP,TOP]);
plot([LEFT,LEFT],[BOTTOM,TOP]);
plot([RIGHT,RIGHT],[BOTTOM,TOP]);


% plot([CENTER_RIGHT-2,CENTER_RIGHT+2],[CENTER_TOP-2,CENTER_TOP-2],'r');
% plot([CENTER_RIGHT-2,CENTER_RIGHT+2],[CENTER_TOP+2,CENTER_TOP+2],'r');
% plot([CENTER_RIGHT-2,CENTER_RIGHT-2],[CENTER_TOP-2,CENTER_TOP+2],'r');
% plot([CENTER_RIGHT+2,CENTER_RIGHT+2],[CENTER_TOP-2,CENTER_TOP+2],'r');

% %画出roadmap中的点
%   vec_road_x = [-10.0,-10.0,10.0,10.0,-10.0];
%   vec_road_y = [-23.0,23.0,23.0,-23.0,-23.0];
% 
% % vec_road_x = [-15.0,-15.0,15.0,15.0,-15.0];
% % vec_road_y = [-30.0,30.0,30.0,-30.0,-30.0];
% 
% plot(vec_road_x(1), vec_road_y(1),'*',vec_road_x(2), vec_road_y(2),'*',vec_road_x(3), vec_road_y(3),'*',vec_road_x(4), vec_road_y(4),'*');
% line(vec_road_x, vec_road_y);
 
%画出当前帧的所有agent位置,circle
% for agent = 0:1:AgentNumber-1
% t=deg2rad(0:360);
% x=cos(t)*R;
% y=sin(t)*R;    
% plot(data(agent*3+1)+x,data(agent*3+2)+y);
% axis([LEFT RIGHT BOTTOM TOP]); 
% hold on;
% fill(data(agent*3+1)+x,data(agent*3+2)+y,'r')
% end

for agent = 0:1:AgentNumber-1
plot(data(agent*4+1),data(agent*4+2),'r.','MarkerSize',12);
h = quiver(data(agent*4+1),data(agent*4+2),data(agent*4+3),data(agent*4+4),'LineWidth',1);
set(h,'maxheadsize',50);
%quiver(...,LineSpec,'filled');
set(h,'autoscalefactor',10);
axis([LEFT RIGHT BOTTOM TOP]); 
hold on;
%fill(data(agent*3+1)+x,data(agent*3+2)+y,'r')
end
szBuffer = sprintf('../output/3d/jepg_output_s6/cmx%05d.jpeg',frame);
print(gcf,'-djpeg',szBuffer);
 
end

%axis([-60 60 -60 60]); 

  