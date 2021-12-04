clc;
clear all;
close all;

%Settings
resolution = 0.050;
maporiginx = 8.385313;
maporiginy = 15.076283;

xorigin = (1 / resolution) * maporiginx;
yorigin = (1 / resolution) * maporiginy;

xmin = 100;
xmax = 500;
ymin = 250;
ymax = 500;

map = imread('modified_pa_track.pgm');
OutputFilePath = 'H:\School\Senior Year\ECE 350-R\pa_track';
OutputFileName = 'modified_pa_track_waypoints.csv';


%show map at the right scale
figure;
hold on;
imshow(map);
xlim([xmin xmax])
ylim([ymin ymax])
hold off;

%get user input
[x,y] = ginput;

hold on;
plot(x,y);
xlim([xmin xmax])
ylim([ymin ymax])

%store values in frame relative coords
localx = (x - xorigin) .* resolution;
localy = (y - yorigin) .* -resolution;

%output to csv
filename = fullfile(OutputFilePath, OutputFileName);

[fid, msg] = fopen(filename, 'wt');
if fid < 0
    error('Could not open file "%s" because "%s"', fid, msg);
end


for i = 1:size(localx)
    fprintf(fid, '%f,%f,%f\n', localx(i), localy(i), 1.0);
end
fclose(fid);