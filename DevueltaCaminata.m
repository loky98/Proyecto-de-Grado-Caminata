 clc;
close all;
clear;

StartPoint = [0,0];
LegLength1 = 12;
LegLength2 = 10;
Height = 18;
HeightRadius = 8;
step = 200;
HalfStep = step/2;

xMax = StartPoint(1) + LegLength1 + LegLength2;
xMin = StartPoint(1) - LegLength1 - LegLength2;
yMax = StartPoint(2) + LegLength1 + LegLength2;
yMin = StartPoint(2) - LegLength1 - LegLength2;

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

xStartCircle = ( x(HalfStep,3) - x(1,3)) / 2;
StartPointCircle = [x(1,3)+xStartCircle,y(1,3)+HeightRadius];

Radius = sqrt((x(1,3) - StartPointCircle(1,1))^2 + (y(1,3) - StartPointCircle(1,2))^2);

Xcircle = linspace( x(HalfStep,3), x(1,3), HalfStep );

Ycircle = Xcircle;

for i=1:length(Ycircle)
    x(HalfStep + i, 3) = Xcircle(1,i);
    Ycircle(1,i) = StartPointCircle(1,2) - sqrt( Radius^2 -(Xcircle(1,i) - StartPointCircle(1,1))^2 );
    y(HalfStep + i, 3) = Ycircle(1,i);
end

AngleLeg = zeros(HalfStep,1);
AngleTrian = zeros(HalfStep,1);

for i=1:length(Ycircle)
    AngleLeg(i,1) = atand(y(HalfStep + i, 3) / x(HalfStep + i, 3));
    if AngleLeg(i,1) < 0
        AngleLeg(i,1) = AngleLeg(i,1) + 180;
    end
    l3 = sqrt((x(HalfStep + i, 3)-x(1,1))^2+(y(HalfStep +i, 3))^2);
    AngleTrian(i,1) = acosd((LegLength1^2 + l3^2 - LegLength2^2) / (2 * LegLength1 *l3));
    AngleFinal = AngleLeg(i,1) + AngleTrian(i,1);
    x(HalfStep + i, 2) = LegLength1 * cosd(AngleFinal);
    y(HalfStep + i, 2) = LegLength1 * sind(AngleFinal);
end

y = y * -1;
Ycircle = Ycircle * -1;
Xcircle = Xcircle + StartPoint(1);
Ycircle = Ycircle + StartPoint(2);
x = x + StartPoint(1);
y = y + StartPoint(2);

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