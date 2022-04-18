clc;
close all;
clear;

StartPoint = [0,0];
LegLength1 = 20;
LegLength2 = 15;
Height = 25;
HeightStep = 50;
Width = 30;
Length = 50;
StepFront = 41;
StepBack =41;
step = StepFront + StepBack;

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

RangeX = RangeStep (Height,LegLength1, LegLength2, StepFront);

[x1,z1,x2,z2,Xcircle,Zcircle] =  PositionLegXZ (RangeX,Height,LegLength1,LegLength2, HeightStep,step,StepFront,StepBack);

x1(step, :) = [];
x1(StepFront + 1, :) = [];
z1(step, :) = [];
z1(StepFront + 1, :) = [];
x2(step, :) = [];
x2(StepFront + 1, :) = [];
z2(step, :) = [];
z2(StepFront + 1, :) = [];

z1 = z1 * -1;
z2 = z2 * -1;
Zcircle = Zcircle * -1;
Xcircle = Xcircle + StartPoint(1);
Zcircle = Zcircle + StartPoint(2);

y = ones(1,3);

x4 = x1 - bodyX;
x1 = x1 - bodyX;
x3 = x2 + bodyX;
x2 = x2 + bodyX;

y1 = y * BaseY(1);
y2 = y * BaseY(2);
y3 = y * BaseY(3);
y4 = y * BaseY(4);

z4 = z1 + bodyZ;
z1 = z1 + bodyZ;
z3 = z2 + bodyZ;
z2 = z2 + bodyZ;

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

o=41;

try
    while 1
        for i=1:step -2
            title("algo");
            %plot(x(i,:),z(i,:),Xcircle,Zcircle, XCentroLeg1(i,1), ZCentroLeg1(i,1));
            %plot(x(i,1:3),z(i,1:3),Xcircle,Zcircle, XCentroLeg1(i,1), ZCentroLeg1(i,1) , 'k*', XCentroLeg2(i,1), ZCentroLeg2(i, 1), 'k*', ...
                %XcentroLegs(i,1), ZcentroLegs(i,1),'R*');
            %plot(XCentroLeg1(i,1), ZCentroLeg1(i,1), XCentroLeg2(i,1), ZCentroLeg2(i, 1), 'k*');
            plot3(BaseX, BaseY, BaseZ, x1(i,:), y1(1,:), z1(i,:), ...
                x2(o,:), y2(1,:), z2(o,:), x3(i,:), y3(1,:), z3(i,:), x4(o,:), y4(1,:), z4(o,:));
            grid;
            %axis([xMin,xMax,yMax,yMin,zMin,zMax]);
            zlim([zMin yMax])
            xlim([xMin xMax])
            ylim([yMin yMax])
            o=o+1;
            if o==81
                o=1;
            end
            pause(0.01);
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

function [RangeStep] = RangeStep (Height,LegLength1, LegLength2, StepFront)
    AngleMin = -acos((Height)/(LegLength1 + LegLength2));
    xmin = (LegLength1 + LegLength2) * sin(AngleMin);
    RangeStep = linspace(xmin, xmin * -1, StepFront);
end

function [x1,z1,x2,z2,Xcircle,Zcircle] = PositionLegXZ (RangeX,Height,LegLength1,LegLength2, HeightStep,step,StepFront,StepBack)

x1 = zeros(step,3);
z1 = zeros(step,3);
x2 = zeros(step,3);
z2 = zeros(step,3);

AngleLeg = zeros(StepFront,1);
AngleTrian = zeros(StepFront,1);

for i=1:StepFront
    x1(i, 3) = RangeX(1, i);
    z1(i,3) = Height;
    l3 = sqrt((x1(i, 3)-x1(1,1))^2 + (Height)^2);
    AngleTrian(i,1) = acosd((LegLength1^2 + l3^2 - LegLength2^2) / (2 * LegLength1 *l3));
    AngleLeg(i,1) = atand(Height / x1(i, 3));
    if AngleLeg(i,1) < 0
        AngleLeg(i,1) = AngleLeg(i,1) + 180;
    end
    AngleFinal = AngleLeg(i,1) - AngleTrian(i,1);
    x1(i, 2) = LegLength1 * cosd(AngleFinal);
    z1(i, 2) = LegLength1 * sind(AngleFinal);
end

StartPointCircle = [0, Height + HeightStep];
Radius = sqrt((x1(1,3) - StartPointCircle(1,1))^2 + (z1(1,3) - StartPointCircle(1,2))^2);
Xcircle = linspace( x1(StepFront,3), x1(1,3), StepBack );
Zcircle = Xcircle;

for i=1:length(Zcircle)
    x1(StepFront + i, 3) = Xcircle(1,i);
    Zcircle(1,i) = StartPointCircle(1,2) - sqrt( Radius^2 -(Xcircle(1,i) - StartPointCircle(1,1))^2 );
    z1(StepFront + i, 3) = Zcircle(1,i);
end

AngleLeg = zeros(StepBack,1);
AngleTrian = zeros(StepBack,1);

for i=2:length(Zcircle)-1
    AngleLeg(i,1) = atand(z1(StepFront + i, 3) / x1(StepFront + i, 3));
    if AngleLeg(i,1) < 0
        AngleLeg(i,1) = AngleLeg(i,1) + 180;
    end
    l3 = sqrt((x1(StepFront + i, 3)-x1(1,1))^2+(z1(StepFront +i, 3))^2);
    AngleTrian(i,1) = acosd((LegLength1^2 + l3^2 - LegLength2^2) / (2 * LegLength1 *l3));
    AngleFinal = AngleLeg(i,1) - AngleTrian(i,1);
    x1(StepFront + i, 2) = LegLength1 * cosd(AngleFinal);
    z1(StepFront + i, 2) = LegLength1 * sind(AngleFinal);
end

    AngleLeg = zeros(StepFront,1);
    AngleTrian = zeros(StepFront,1);

    for i=1:StepFront
        x2(i, 3) = RangeX(1, i);
        z2(i,3) = Height;
        l3 = sqrt((x2(i, 3)-x2(1,1))^2 + (Height)^2);
        AngleTrian(i,1) = acosd((LegLength1^2 + l3^2 - LegLength2^2) / (2 * LegLength1 *l3));
        AngleLeg(i,1) = atand(Height / x2(i, 3));
        if AngleLeg(i,1) < 0
            AngleLeg(i,1) = AngleLeg(i,1) + 180;
        end
        AngleFinal = AngleLeg(i,1) + AngleTrian(i,1);
        x2(i, 2) = LegLength1 * cosd(AngleFinal);
        z2(i, 2) = LegLength1 * sind(AngleFinal);
    end

    StartPointCircle = [0, Height + HeightStep];
    Radius = sqrt((x2(1,3) - StartPointCircle(1,1))^2 + (z2(1,3) - StartPointCircle(1,2))^2);
    Xcircle = linspace( x2(StepFront,3), x2(1,3), StepBack );
    Zcircle = Xcircle;

    for i=1:length(Zcircle)
        x2(StepFront + i, 3) = Xcircle(1,i);
        Zcircle(1,i) = StartPointCircle(1,2) - sqrt( Radius^2 -(Xcircle(1,i) - StartPointCircle(1,1))^2 );
        z2(StepFront + i, 3) = Zcircle(1,i);
    end

    AngleLeg = zeros(StepBack,1);
    AngleTrian = zeros(StepBack,1);

    for i=2:length(Zcircle)-1
        AngleLeg(i,1) = atand(z2(StepFront + i, 3) / x2(StepFront + i, 3));
        if AngleLeg(i,1) < 0
            AngleLeg(i,1) = AngleLeg(i,1) + 180;
        end
        l3 = sqrt((x2(StepFront + i, 3)-x2(1,1))^2+(z2(StepFront +i, 3))^2);
        AngleTrian(i,1) = acosd((LegLength1^2 + l3^2 - LegLength2^2) / (2 * LegLength1 *l3));
        AngleFinal = AngleLeg(i,1) + AngleTrian(i,1);
        x2(StepFront + i, 2) = LegLength1 * cosd(AngleFinal);
        z2(StepFront + i, 2) = LegLength1 * sind(AngleFinal);
    end
end