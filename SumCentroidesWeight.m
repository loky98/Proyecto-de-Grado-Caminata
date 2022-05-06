function [CentroiMassLegs] = SumCentroidesWeight(WeightB,pB,WeightLegs,p1,p2,p3,p4)
    WeightTotal = WeightB + (4 * WeightLegs);
    CentroiMassLegs = ((WeightB * pB)+(WeightLegs * p1)+(WeightLegs * p2)+(WeightLegs * p3)+(WeightLegs * p4))/(WeightTotal);
end