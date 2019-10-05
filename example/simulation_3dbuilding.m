AgentNumber =150;
FrameNumber =300;
for frame = 0:1:FrameNumber
    szBuffer = sprintf('../output/3d/s21/cmx%05d.txt',frame); 
f1=fopen(szBuffer);
cla;
i=1;
while 1
nextline = fgetl(f1); %����һ�� 
if ~ischar(nextline), break, end %����������� 
%disp(nextline);%���п��Բ�Ҫ 
a = sscanf(nextline, '%f %f');%��ȡ���ݣ��������Լ�����Ҫ�� 
data(i) = a(1);
data(i+1) = a(2);
i=i+2;
end 
fclose(f1);

  %for wall---------------
 LEFT = -250;
  RIGHT = 300;
  TOP = 600;
  BOTTOM = -300;
%------------------------- 

%�ϰ����ļ�
inBuffer = sprintf('../input/3d/building.txt');

%���ϰ����ļ�
f3=fopen(inBuffer);
i=1;
nextline = fgetl(f3);%����һ�� 
if ~isstr(nextline), break, end %����������� 
%disp(nextline);%���п��Բ�Ҫ 
a = sscanf(nextline, '%d');%��ȡ���ݣ��������Լ�����Ҫ�� 
for num=1:1:a
    nextline = fgetl(f3); %����һ�� 
    if ~isstr(nextline), break, end %����������� 
    b = sscanf(nextline, '%f %f');
    data_obj(i)   = b(1);
    data_obj(i+1) = b(2);
    i = i+2;
end
nextline = fgetl(f3); %����һ�� 
a = sscanf(nextline, '%d');%��ȡ���ݣ��������Լ�����Ҫ�ġ��ϰ������

for num=1:1:a %
    nextline = fgetl(f3); %����һ�� 
    if ~isstr(nextline), break, end %����������� 
    vertnum = sscanf(nextline, '%d ');%�ϰ��ﶥ�����
    
    %��ȡ�������꣬�������ϰ���
    for i=2:1:vertnum(1)+1
        b(1) = vertnum(i)+1;
        t = i+1;
        if t> vertnum(1)+1
            t = 2;
        end
        b(2) = vertnum(t)+1;
        plot([data_obj(2*b(1)-1) data_obj(2*b(2)-1)],[data_obj(2*b(1)) data_obj(2*b(2))],'LineWidth',1.5);
         %axis([LEFT RIGHT BOTTOM TOP]); 
        %hold on;
    end
     
end
fclose(f3);

% % ��ȡroadmap����Ϣ��������
%  szBuffer = sprintf('../input/3d/roadmap2.txt');
%  f1=fopen(szBuffer);
%  i=1;
%  nextline = fgetl(f1); %����һ�� 
%  Points_Number = sscanf(nextline, '%d');%��ȡ���ݣ��������Լ�����Ҫ�� 
%  for num=1:1:Points_Number
%      nextline = fgetl(f1); %����һ�� 
%      if ~isstr(nextline), break, end %����������� 
%      b = sscanf(nextline, '%f %f');
%      data_roadmap(i)   = b(1);
%      data_roadmap(i+1) = b(2);
%      i = i+2;
%  end
%  nextline = fgetl(f1); %����һ�� 
%  a = sscanf(nextline, '%d');%��ȡ���ݣ��������Լ�����Ҫ�� %  
%  for num=1:1:a
%      nextline = fgetl(f1); %����һ�� 
%      if ~isstr(nextline), break, end %����������� 
%      b = sscanf(nextline, '%d %d');
%      b(1) = b(1)+1;
%      b(2) = b(2)+1;
%      plot([data_roadmap(2*b(1)-1) data_roadmap(2*b(2)-1)],[data_roadmap(2*b(1)) data_roadmap(2*b(2))],'g','LineWidth',1);
%  end
%  
%  i = 1;
%  for num = 1:1:Points_Number
%       plot(data_roadmap(i), data_roadmap(i+1),'m.','MarkerSize',5);
%      i = i+2;
%  end
%  plot([LEFT,RIGHT],[BOTTOM,BOTTOM]);
%  plot([LEFT,RIGHT],[TOP,TOP]);
%  plot([LEFT,LEFT],[BOTTOM,TOP]);
%  plot([RIGHT,RIGHT],[BOTTOM,TOP]);
% axis on;
% fclose(f1);

plot([LEFT,RIGHT],[BOTTOM,BOTTOM]);
plot([LEFT,RIGHT],[TOP,TOP]);
plot([LEFT,LEFT],[BOTTOM,TOP]);
plot([RIGHT,RIGHT],[BOTTOM,TOP]);

for agent = 0:1:AgentNumber-1  
plot(data(agent*2+1),data(agent*2+2),'Marker','o','MarkerSize',4.8, 'MarkerEdgeColor',[0.56,0.55,0.56], 'MarkerFaceColor',[1,0,0]);
axis([LEFT RIGHT BOTTOM TOP]); 
hold on;
end
szBuffer = sprintf('../output/3d/jepg_output_s21/cmx%05d.jpeg',frame);
print(gcf,'-djpeg',szBuffer);

end

