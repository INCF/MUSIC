% makeNeuronGrid.m by Johannes Hjorth
%
% This function generates a grid of neurons for use with
% viewevents.cpp
%
% makeNeuronGrid('neuronGrid.data', ...
%                (-1:1)*150,(-1:1)*150, (-1:1)*150, 30, ...
%                [0.25 0.53 0.1], [1 0.9 0])
%
%makeNeuronGrid('neuronGridPlane.data',(-11:11)*50,(-11:11)*50,0, ...
%               10, [0.53 0.25 0.1],[0.9 1 0])
%
%
% This version only supports one colour, see VisualiseNeurons for help.

function makeNeuronGrid(filename, x,y,z,r, colMin, colMax)

[X,Y,Z] = meshgrid(x,y,z);

fid = fopen(filename,'w');

fprintf(fid,'1\n');
fprintf(fid,'%d %d %d\n', colMin);
fprintf(fid,'%d %d %d\n', colMax);

for i=1:length(X(:))
  fprintf(fid,'%d %d %d %d 0\n', X(i),Y(i),Z(i),r);
end

fclose(fid);
