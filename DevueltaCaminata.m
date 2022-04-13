 clc;
close all;
clear;

StartPoint = [0,0];
LegLength1 = 20;
LegLength2 = 15;
Height = 25;
HeightRadius = 50;
step = 100;
HalfStep = step/2;

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

for i=2:length(Ycircle)-1
    AngleLeg(i,1) = atand(y(HalfStep + i, 3) / x(HalfStep + i, 3));
    if AngleLeg(i,1) < 0
        AngleLeg(i,1) = AngleLeg(i,1) + 180;
    end
    l3 = sqrt((x(HalfStep + i, 3)-x(1,1))^2+(y(HalfStep +i, 3))^2);
    AngleTrian(i,1) = acosd((-LegLength1^2 - l3^2 + LegLength2^2) / (-2 * LegLength1 *l3));
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

XCentroLeg1 = Centroides(x, 1, 2);
YCentroLeg1 = Centroides(y, 1, 2);
XCentroLeg2 = Centroides(x, 2, 3);
YCentroLeg2 = Centroides(y, 2, 3);

XcentroLegs = SumCentroidesLegs(XCentroLeg1, XCentroLeg2,LegLength1, LegLength2);
YcentroLegs = SumCentroidesLegs(YCentroLeg1, YCentroLeg2,LegLength1, LegLength2);

x(step, :) = [];
x(HalfStep + 1, :) = [];
y(step, :) = [];
y(HalfStep + 1, :) = [];


try
    while 1
        for i=1:step -2
            title("algo");
            %plot(x(i,:),y(i,:));
            %plot(x(i,:),y(i,:),Xcircle,Ycircle, XCentroLeg1(i,1), YCentroLeg1(i,1));
            plot(x(i,1:3),y(i,1:3),Xcircle,Ycircle, XCentroLeg1(i,1), YCentroLeg1(i,1) , 'k*', XCentroLeg2(i,1), YCentroLeg2(i, 1), 'k*', XcentroLegs(i,1), YcentroLegs(i,1),'R*');
            %plot(XCentroLeg1(i,1), YCentroLeg1(i,1), XCentroLeg2(i,1), YCentroLeg2(i, 1), 'k*');
            grid;
            axis([xMin,xMax,yMin,yMax]);
            pause(0.1);
        end
    end
catch exception
end

function [centroideLeg] = Centroides(vector, p1, p2)
    centroideLeg = zeros(length(vector), 1);
    for i=1:length(vector)
        centroideLeg(i,1) = (vector(i,p1) + vector(i, p2)) / 2;
    end
end

function [CentroideLegs] = SumCentroidesLegs(vector1, vector2, lengthLeg1, lengthLeg2)
    CentroideLegs = zeros(length(vector1), 1);
    lengthTotal = lengthLeg1 + lengthLeg2;
    for i=1:length(vector1)
        CentroideLegs(i,1) = (lengthLeg1 * vector1(i,1) + lengthLeg2 * vector2(i,1)) / lengthTotal;
    end
end

function [CentroiMassLegs] = SumCentroidesMassLegs(vector1, vector2, MassLeg1, MassLeg2)
    CentroideLegs = zeros(length(vector1), 1);
    MassTotal = MassLeg1 + MassLeg2;
    for i=1:length(vector1)
        CentroideLegs(i,1) = (MassLeg1 * vector1(i,1) + MassLeg2 * vector2(i,1)) / MassTotal;
    end
end
