clc;
close all;
clear;

StartPoint = [1,5];
LegLength1 = 12;
LegLength2 = 10;
Height = 18;
step = 100;

xMax = StartPoint(1) + LegLength1 + LegLength2;
xMin = StartPoint(1) - LegLength1 - LegLength2;
yMax = StartPoint(2) + 2;
yMin = StartPoint(2) - LegLength1 -LegLength2;

AngleMin = -acos((Height-LegLength2)/(LegLength1));
AngleMax = acos((Height)/(LegLength1+LegLength2));

RangeAngleLeg1 = linspace(AngleMin,AngleMax,step);

x = zeros(step,3);
y = zeros(step,3);

for i=1:step
    x(i,2) = LegLength1 * sin(RangeAngleLeg1(i));
    y(i,2) = LegLength1 * cos(RangeAngleLeg1(i));
    Angleleg2 = acos((Height-y(i,2))/(LegLength2));
    x(i,3) = LegLength2 * sin(Angleleg2) + x(i,2);
    y(i,3) = Height;
end

y = y *-1;
x = x + StartPoint(1);
y = y + StartPoint(2);

try
   while 1
       for i=1:step
           plot(x(i,1:3),y(i,1:3));
           title("algo");
           axis([xMin,xMax,yMin,yMax]);
           grid;
           pause(0.01);
       end
   end
catch exception
end