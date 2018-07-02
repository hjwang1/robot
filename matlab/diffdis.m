clc;
clear;
%jsonlab-1.5 toolbox is necessary
%addpath('D:\Program Files\MATLAB\R2013b\toolbox\jsonlab-1.5')
jsondata=loadjson('E:\inforency\hjwhead-json\Adhead-10001-l36-1530073415655200.json');
curv_a=jsondata.curv3_ds;%ges_ds;%
otherjson=loadjson('E:\inforency\hjwhead-json\jbhead-10005-l36-1530073432373152.json');
curv_b=otherjson.curv3_ds;%ges_ds;%
%curv_a=[1,2,3,4,3,2,6,7,8,2];
%curv_b=[0,2,3,4,9,4,9,9,6,5];
%curv_a=curv_a+0.1;
%curv_b=curv_b+0.1;
res=-2;
ya=roundn(curv_a, res);
yb=roundn(curv_b, res);
ra=tabulate(ya);
rb=tabulate(yb);

z_on=1;
if z_on == 1
    plot(ra(:,1),ra(:,2),'r');
    hold on;
    plot(rb(:,1),rb(:,2),'b');
    anotherjson=loadjson('E:\inforency\hjwhead-json\basketball-10006-l36-1530068188373585.json');
    curv_c=anotherjson.curv3_ds;%ges_ds;%
    yc=roundn(curv_c, res);
    rc=tabulate(yc);
    plot(rc(:,1),rc(:,2),'g');
    legend('Mom\primes 3D face','Baby\primes 3D face','Toy');
    %legend('boxoff');
end
[c, ia, ib] = intersect(ra(:,1), rb(:,1));
xa=ra(ia,1);
ha=ra(ia,2);
hb=rb(ib,2);
pa=ha/sum(ha);
pb=hb/sum(hb);
ca=length(c)/length(ra);
cb=length(c)/length(rb);
cl=length(c);
cerror=ha-hb;
cmean=mean(cerror);
%cvar=var(cerror);
cstd=std(cerror);
if z_on == 0
    plot(xa,pa,'r');
    hold on;
    plot(xa,pb,'b');
end
%legend('某一视角','另一视角');
%legend('boxoff');
%title('不同视角&多分辨率感知人脸的形状空间分布');
