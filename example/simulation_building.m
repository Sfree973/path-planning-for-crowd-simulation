AgentNumber = 6;
FrameNumber = 72;
for frame =72:1:FrameNumber
    szBuffer = sprintf('../output/3d/s6/cmx%05d.txt',frame); 
f1=fopen(szBuffer);
cla;
i=1;
while 1
nextline = fgetl(f1); %����һ�� 
if ~isstr(nextline), break, end %����������� 
%disp(nextline);%���п��Բ�Ҫ 
a = sscanf(nextline, '%f %f');%��ȡ���ݣ��������Լ�����Ҫ�� 
data(i) = a(1);
data(i+1) = a(2);
i=i+2;
end 
fclose(f1);

  %for wall---------------
  LEFT = 50;
  RIGHT = 70.0;
  TOP = 10;
  BOTTOM = 50;
%------------------------- 

%�ϰ����ļ�
inBuffer = sprintf('../input/3d/3dbuilding (2).txt');

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
        plot([data_obj(2*b(1)-1) data_obj(2*b(2)-1)],[data_obj(2*b(1)) data_obj(2*b(2))]);
         %axis([LEFT RIGHT BOTTOM TOP]); 
        %hold on;
    end
     
end
fclose(f3);

plot([LEFT,RIGHT],[BOTTOM,BOTTOM]);
plot([LEFT,RIGHT],[TOP,TOP]);
plot([LEFT,LEFT],[BOTTOM,TOP]);
plot([RIGHT,RIGHT],[BOTTOM,TOP]);

for agent = 0:1:AgentNumber-1  
plot(data(agent*2+1),data(agent*2+2),'ro','MarkerSize',2);
axis([LEFT RIGHT BOTTOM TOP]); 
hold on;
end

szBuffer = sprintf('../output/3d/jepg_output6/cmx%05d.jpeg',frame);
print(gcf,'-djpeg',szBuffer);

end

