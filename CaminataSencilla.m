clc;
close all;
clear;

leg_higher = 40;
Leg_lower = 30;
height = 60;

linea = [1,5; 3,2];
plot (linea);
%[a, b] = centroidLine(linea);

%plot (a, b,'o');
grid ;

function [x,y] = centroidLine(linea)
    % Esta función sirve para Hallar el centroide de una linea 
    x = (linea(1)+linea(3))/2;
    y = (3+3)/2;
    end