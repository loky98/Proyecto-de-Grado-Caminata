clc;
close all;
clear;

StartPoint = [0,0];
LegLength1 = 20;
WeightLeg1 = 0.4;
LegLength2 = 15;
WeightLeg2 = 0.2;
Height = 25;
HeightStep = 50;
Width = 30;
Length = 60;
WeightBase = 0.5;
StepFront = 41;
StepBack =41;
step = StepFront + StepBack;
WeightLegs = WeightLeg1 + WeightLeg2;

bodyX = Length / 2;
bodyY = Width / 2;
bodyZ = Height;

BaseX = [-bodyX, bodyX, bodyX, -bodyX, -bodyX];
BaseY = [bodyY, bodyY, -bodyY, -bodyY, bodyY];
BaseZ = [bodyZ, bodyZ, bodyZ, bodyZ, bodyZ];

xMax = bodyX + LegLength1 + LegLength2;
xMin = -bodyX - LegLength1 - LegLength2;
yMax = bodyX + LegLength1 + LegLength2;
yMin = -bodyX - LegLength1 - LegLength2;
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
%___________________________________________________________________________________

o=41;

centroWeightX = zeros(length(X1centroLegs), 1);
centroWeighty = zeros(length(X1centroLegs), 1);
centroWeightz = zeros(length(X1centroLegs), 1);

StabilityX = zeros(length(X1centroLegs),5);
StabilityY = zeros(length(X1centroLegs),5);

for i=1:step -2
    centroWeightX(i,1) = SumCentroidesWeight(WeightBase,0,WeightLegs,X1centroLegs(i),X2centroLegs(o),X2centroLegs(i),X1centroLegs(o));
    centroWeighty(i,1) = SumCentroidesWeight(WeightBase,0,WeightLegs,y1(1,1),y2(1,1),y3(1,1),y4(1,1));
    centroWeightz(i,1) = SumCentroidesWeight(WeightBase,Height,WeightLegs,Z1centroLegs(i),Z2centroLegs(o),Z2centroLegs(i),Z1centroLegs(o));

    vec = [z1(i,3),z2(o,3),z3(i,3),z4(o,3)];
    count = 1;
    position = 1;
    for ii = vec
        if ii == 0
            switch count
            case 1
                StabilityX(i,position) = x1(i,3);
                StabilityY(i,position) = y1(1,1);
            case 2
                StabilityX(i,position) = x2(o,3);
                StabilityY(i,position) = y2(1,1);
            case 3
                StabilityX(i,position) = x3(i,3);
                StabilityY(i,position) = y3(1,1);
            case 4
                StabilityX(i,position) = x4(o,3);
                StabilityY(i,position) = y4(1,1);
            end
            position = position + 1;
        end
        count = count + 1;
        if count >= 5
            position = 1;
            count = 1;
        end
    end

    o=o+1;
    if o==81
        o=1;
    end
end

tamano = size(StabilityX);
for f=1:tamano(1,1)
    for c=1:tamano(1,2)
        if StabilityX(f,c) == 0
            if c == 5
                StabilityX(f,c) = StabilityX(f,1);
                StabilityY(f,c) = StabilityY(f,1);
            else
                StabilityX(f,c) = StabilityX(f,c -1);
                StabilityY(f,c) = StabilityY(f,c -1);
            end
        end
    end
end

o=41;

try
    while 1
        for i=1:step -2
            title("algo");
            %plot(x(i,:),z(i,:),Xcircle,Zcircle, XCentroLeg1(i,1), ZCentroLeg1(i,1));
            %plot(x(i,1:3),z(i,1:3),Xcircle,Zcircle, XCentroLeg1(i,1), ZCentroLeg1(i,1) , 'k*', XCentroLeg2(i,1), ZCentroLeg2(i, 1), 'k*', ...
                %XcentroLegs(i,1), ZcentroLegs(i,1),'R*');
            %plot(XCentroLeg1(i,1), ZCentroLeg1(i,1), XCentroLeg2(i,1), ZCentroLeg2(i, 1), 'k*');
            subplot(2,2,1);
            plot3(BaseX, BaseY, BaseZ, x1(i,:), y1(1,:), z1(i,:), ...
                x2(o,:), y2(1,:), z2(o,:), x3(i,:), y3(1,:), z3(i,:), x4(o,:), y4(1,:), z4(o,:), ...
                X1CentroLeg1(i),y1(1,1),Z1CentroLeg1(i),'ko',X1CentroLeg2(i),y1(1,1),Z1CentroLeg2(i),'ko', ...
                X2CentroLeg1(o),y2(1,1),Z2CentroLeg1(o),'ko',X2CentroLeg2(o),y2(1,1),Z2CentroLeg2(o),'ko', ...
                X2CentroLeg1(i),y3(1,1),Z2CentroLeg1(i),'ko',X2CentroLeg2(i),y3(1,1),Z2CentroLeg2(i),'ko', ...
                X1CentroLeg1(o),y4(1,1),Z1CentroLeg1(o),'ko',X1CentroLeg2(o),y4(1,1),Z1CentroLeg2(o),'ko', ...
                X1centroLegs(i),y1(1,1),Z1centroLegs(i),'Ro', ...
                X2centroLegs(o),y2(1,1),Z2centroLegs(o),'Ro', ...
                X2centroLegs(i),y3(1,1),Z2centroLegs(i),'Ro', ...
                X1centroLegs(o),y4(1,1),Z1centroLegs(o),'Ro', ...
                centroWeightX(i,1),centroWeighty(i,1),centroWeightz(i,1),'R.');
            grid;
            %axis([xMin,xMax,yMax,yMin,zMin,zMax]);
            zlim([zMin zMax])
            xlim([xMin xMax])
            ylim([yMin yMax])

            subplot(2,2,2);
            plot3(centroWeightX,centroWeighty,centroWeightz, ...
                centroWeightX(i,1),centroWeighty(i,1),centroWeightz(i,1),'R*');
            grid;
            title('Centro de masa');

            subplot(2,2,3);
            plot(StabilityX(i,:),StabilityY(i,:),centroWeightX(i,1),centroWeighty(i,1),'R.');
            axis([xMin,xMax,yMin,yMax]);
            grid;
            title('Estabilidad');

            o=o+1;
            if o==81
                o=1;
            end
            pause(0.1);
        end
    end
catch exception
end