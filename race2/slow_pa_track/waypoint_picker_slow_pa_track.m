clc;
clear all;
close all;

%Settings
resolution = 0.050;
maporiginx = 4.675838;
maporiginy = 9.588414;

xorigin = (1 / resolution) * maporiginx;
yorigin = (1 / resolution) * maporiginy;

xmin = 0;
xmax = 400;
ymin = 0;
ymax = 400;

map = imread('slow_pa_track-20210419T192853Z-001\slow_pa_track\converted_slow_pa_track.pgm');
OutputFilePath = '\\client\c$\Users\Nick\OneDrive - Lehigh University\Lehigh\Spring 2021\ECE 450-011 (Intro to Robotics)\Lab 6';
OutputFileName = 'target_waypoints.csv';


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