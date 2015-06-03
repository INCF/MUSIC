% makeTestSpikes('viewevents-spikes0.dat',5000,10,1) 

function makeTestSpikes(filename, width, freq, maxTime)

spikeTimes = sort(maxTime*rand(ceil(width*freq*maxTime),1));
spikeTimes(spikeTimes < 1e-5) = [];

id = floor(width*rand(size(spikeTimes)));

fid = fopen(filename,'w');
for i=1:length(spikeTimes)
    fprintf(fid,'%d %d\n', spikeTimes(i), id(i));
end

fclose(fid);

