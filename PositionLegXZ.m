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