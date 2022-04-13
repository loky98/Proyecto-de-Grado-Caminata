clc;
close all;
clear;

StartPoint = [1,5];
LegLength1 = 12;
LegLength2 = 10;
Height = 18;
step = 100;
HalfStep = step / 2;

xMax = StartPoint(1) + LegLength1 + LegLength2;
xMin = StartPoint(1) - LegLength1 - LegLength2;
yMax = StartPoint(2) + 2;
yMin = StartPoint(2) - LegLength1 - LegLength2;

AngleMin = -acos((Height)/(LegLength1 + LegLength2));

xmin = (LegLength1 + LegLength2) * sin(AngleMin);

RangeX = linspace(xmin, xmin * -1, HalfStep);

x = zeros(step,3);
y = zeros(step,3);

AngleLeg = zeros(HalfStep,1);
AngleTrian = zeros(HalfStep,1);

for i=1:HalfStep
    x(i, 3) = RangeX(1, i);
    y(i,3) = Height;
    l3 = sqrt((x(i, 3)-x(1,1))^2 + (Height)^2);
    AngleTrian(i,1) = acosd((LegLength1^2 + l3^2 - LegLength2^2) / (2 * LegLength1 *l3));
    AngleLeg(i,1) = atand(Height / x(i, 3));
    if AngleLeg(i,1) < 0
        AngleLeg(i,1) = AngleLeg(i,1) + 180;
    end
    AngleFinal = AngleLeg(i,1) + AngleTrian(i,1);
    x(i, 2) = LegLength1 * cosd(AngleFinal);
    y(i, 2) = LegLength1 * sind(AngleFinal);
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