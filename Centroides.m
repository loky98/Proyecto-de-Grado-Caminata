function [centroideLeg] = Centroides(vector, p1, p2)
    centroideLeg = zeros(length(vector), 1);
    for i=1:length(vector)
        centroideLeg(i,1) = (vector(i,p1) + vector(i, p2)) / 2;
    end
end