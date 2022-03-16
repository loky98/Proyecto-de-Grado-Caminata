 clc;
close all;
clear;

StartPoint = [-3,-3];
LegLength1 = 12;
LegLength2 = 10;
Height = 18;
HeightRadius = 8;
step = 200;
HalfStep = step/2;

xMax = StartPoint(1) + LegLength1 + LegLength2;
xMin = StartPoint(1) - LegLength1 - LegLength2;
yMax = StartPoint(2) + 2;
yMin = StartPoint(2) - LegLength1 -LegLength2;

AngleMin = -acos((Height-LegLength2)/(LegLength1));
AngleMax = acos((Height)/(LegLength1+LegLength2));

RangeAngleLeg1 = linspace(AngleMin,AngleMax,HalfStep);

x = zeros(step,3);
y = zeros(step,3);

for i=1:HalfStep
    x(i,2) = LegLength1 * sin(RangeAngleLeg1(i));
    y(i,2) = LegLength1 * cos(RangeAngleLeg1(i));
    Angleleg2 = acos((Height-y(i,2))/(LegLength2));
    x(i,3) = LegLength2 * sin(Angleleg2) + x(i,2);
    y(i,3) = Height;
end

y = y *-1;
x = x + StartPoint(1);
y = y + StartPoint(2);

xStartCircle = abs( x(1,3) - x(HalfStep,3) ) / 2;
StartPointCircle = [x(1,3)+xStartCircle,y(1,3)-HeightRadius];

Radius = sqrt((x(1,3) - StartPointCircle(1,1))^2 + (y(1,3) - StartPointCircle(1,2))^2);

Xcircle = linspace( x(HalfStep,3), x(1,3), HalfStep );
%Xcircle = Xcircle(2:end-1);

Ycircle = Xcircle;

for i=1:length(Ycircle)
    x(HalfStep + i, 3) = Xcircle(1,i);
    Ycircle(1,i) = StartPointCircle(1,2) + sqrt( Radius^2 -(Xcircle(1,i) - StartPointCircle(1,1))^2 );
    y(HalfStep + i, 3) = Ycircle(1,i);
end

try
   while 1
       for i=1:step
           plot(x(i,1:3),y(i,1:3),Xcircle,Ycircle);
           title("algo");
           axis([xMin,xMax,yMin,yMax]);
           grid;
           pause(0.01);
       end
   end
catch exception
end