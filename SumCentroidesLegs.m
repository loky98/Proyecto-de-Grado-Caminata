function [CentroideLegs] = SumCentroidesLegs(vector1, vector2, WeightLeg1, WeightLeg2)
    CentroideLegs = zeros(length(vector1), 1);
    WeightTotal = WeightLeg1 + WeightLeg2;
    for i=1:length(vector1)
        CentroideLegs(i,1) = (WeightLeg1 * vector1(i,1) + WeightLeg2 * vector2(i,1)) / WeightTotal;
    end
end