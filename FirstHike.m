clc;
close all;
clear;

StartPoint = [0,0];
LegLength1 = 20;
LegLength2 = 15;
Height = 25;
HeightStep = 50;
Width = 30;
Length = 40;
step = 100;
HalfStep = step/2;

bodyX = Length / 2;
bodyY = Width / 2;
bodyZ = Height;

BaseX = [-bodyX, bodyX, bodyX, -bodyX, -bodyX];
BaseY = [bodyY, bodyY, -bodyY, -bodyY, bodyY];
BaseZ = [bodyZ, bodyZ, bodyZ, bodyZ, bodyZ];

xMax = bodyX + LegLength1;
xMin = -bodyX - LegLength1;
yMax = bodyX + LegLength1;
yMin = -bodyX - LegLength1;
zMax = Height;
zMin = -5;

RangeX = RangeStep (Height,LegLength1, LegLength2, HalfStep);

[x,z,Xcircle,Zcircle] =  PositionLegXZ (HalfStep,RangeX,Height,LegLength1,LegLength2, HeightStep,step);

x(step, :) = [];
x(HalfStep + 1, :) = [];
z(step, :) = [];
z(HalfStep + 1, :) = [];

z = z * -1;
Zcircle = Zcircle * -1;
Xcircle = Xcircle + StartPoint(1);
Zcircle = Zcircle + StartPoint(2);

y = ones(1,3);

x1 = x * -1;
x4 = x * -1;

x1 = x1 - bodyX;
x2 = x + bodyX;
x3 = x + bodyX;
x4 = x4 - bodyX;

y1 = y * BaseY(1);
y2 = y * BaseY(2);
y3 = y * BaseY(3);
y4 = y * BaseY(4);

z1 = z + bodyZ;
z2 = z + bodyZ;
z3 = z + bodyZ;
z4 = z + bodyZ;

%x = x + StartPoint(1);
%z = z + StartPoint(2);

X1CentroLeg1 = Centroides(x1, 1, 2);
X2CentroLeg1 = Centroides(x2, 1, 2);

Z1CentroLeg1 = Centroides(z1, 1, 2);
Z2CentroLeg1 = Centroides(z2, 1, 2);

X1CentroLeg2 = Centroides(x1, 2, 3);
X2CentroLeg2 = Centroides(x2, 2, 3);

Z1CentroLeg2 = Centroides(z1, 2, 3);
Z2CentroLeg2 = Centroides(z2, 2, 3);

X1centroLegs = SumCentroidesLegs(X1CentroLeg1, X1CentroLeg2,LegLength1, LegLength2);
X2centroLegs = SumCentroidesLegs(X2CentroLeg1, X2CentroLeg2,LegLength1, LegLength2);

Z1centroLegs = SumCentroidesLegs(Z1CentroLeg1, Z1CentroLeg2,LegLength1, LegLength2);
Z2centroLegs = SumCentroidesLegs(Z2CentroLeg1, Z2CentroLeg2,LegLength1, LegLength2);

try
    while 1
        for i=1:step -2
            title("algo");
            %plot(x(i,:),z(i,:),Xcircle,Zcircle, XCentroLeg1(i,1), ZCentroLeg1(i,1));
            %plot(x(i,1:3),z(i,1:3),Xcircle,Zcircle, XCentroLeg1(i,1), ZCentroLeg1(i,1) , 'k*', XCentroLeg2(i,1), ZCentroLeg2(i, 1), 'k*', ...
                %XcentroLegs(i,1), ZcentroLegs(i,1),'R*');
            %plot(XCentroLeg1(i,1), ZCentroLeg1(i,1), XCentroLeg2(i,1), ZCentroLeg2(i, 1), 'k*');
            plot3(BaseX, BaseY, BaseZ, x1(i,:), y1(1,:), z1(i,:), ...
                x2(i,:), y2(1,:), z2(i,:), x3(i,:), y3(1,:), z3(i,:), x4(i,:), y4(1,:), z4(i,:));
            grid;
            %axis([xMin,xMax,yMax,yMin,zMin,zMax]);
            zlim([zMin yMax])
            xlim([xMin xMax])
            ylim([yMin yMax])
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

function [RangeStep] = RangeStep (Height,LegLength1, LegLength2, Range)
    AngleMin = -acos((Height)/(LegLength1 + LegLength2));
    xmin = (LegLength1 + LegLength2) * sin(AngleMin);
    RangeStep = linspace(xmin, xmin * -1, Range);
end

function [x, z,Xcircle,Zcircle] = PositionLegXZ (HalfStep,RangeX,Height,LegLength1,LegLength2, HeightStep,step,StepFront,StepBack)
    x = zeros(step,3);
    z = zeros(step,3);

    AngleLeg = zeros(HalfStep,1);
    AngleTrian = zeros(HalfStep,1);

    for i=1:HalfStep
        x(i, 3) = RangeX(1, i);
        z(i,3) = Height;
        l3 = sqrt((x(i, 3)-x(1,1))^2 + (Height)^2);
        AngleTrian(i,1) = acosd((LegLength1^2 + l3^2 - LegLength2^2) / (2 * LegLength1 *l3));
        AngleLeg(i,1) = atand(Height / x(i, 3));
        if AngleLeg(i,1) < 0
            AngleLeg(i,1) = AngleLeg(i,1) + 180;
        end
        AngleFinal = AngleLeg(i,1) + AngleTrian(i,1);
        x(i, 2) = LegLength1 * cosd(AngleFinal);
        z(i, 2) = LegLength1 * sind(AngleFinal);
    end

    xStartCircle = ( x(HalfStep,3) - x(1,3)) / 2;
    StartPointCircle = [x(1,3)+xStartCircle,z(1,3)+HeightStep];

    Radius = sqrt((x(1,3) - StartPointCircle(1,1))^2 + (z(1,3) - StartPointCircle(1,2))^2);

    Xcircle = linspace( x(HalfStep,3), x(1,3), HalfStep );

    Zcircle = Xcircle;

    for i=1:length(Zcircle)
        x(HalfStep + i, 3) = Xcircle(1,i);
        Zcircle(1,i) = StartPointCircle(1,2) - sqrt( Radius^2 -(Xcircle(1,i) - StartPointCircle(1,1))^2 );
        z(HalfStep + i, 3) = Zcircle(1,i);
    end

    AngleLeg = zeros(HalfStep,1);
    AngleTrian = zeros(HalfStep,1);

    for i=2:length(Zcircle)-1
        AngleLeg(i,1) = atand(z(HalfStep + i, 3) / x(HalfStep + i, 3));
        if AngleLeg(i,1) < 0
            AngleLeg(i,1) = AngleLeg(i,1) + 180;
        end
        l3 = sqrt((x(HalfStep + i, 3)-x(1,1))^2+(z(HalfStep +i, 3))^2);
        AngleTrian(i,1) = acosd((LegLength1^2 + l3^2 - LegLength2^2) / (2 * LegLength1 *l3));
        AngleFinal = AngleLeg(i,1) + AngleTrian(i,1);
        x(HalfStep + i, 2) = LegLength1 * cosd(AngleFinal);
        z(HalfStep + i, 2) = LegLength1 * sind(AngleFinal);
    end
end