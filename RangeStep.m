function [RangeStep] = RangeStep (Height,LegLength1, LegLength2, StepFront)
    AngleMin = -acos((Height)/(LegLength1 + LegLength2));
    xmin = (LegLength1 + LegLength2) * sin(AngleMin);
    RangeStep = linspace(xmin, xmin * -1, StepFront);
end