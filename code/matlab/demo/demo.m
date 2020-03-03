% Note: make sure you've previously ran ../do_addpath.m

% define the data file location:
%data_dir = 'c:\work\misc\fx\read_nmea\demo\';
data_dir = fileparts(mfilename('fullpath'));
filename = '190110.LOG';
filepath = fullfile(data_dir, filename);
if ~exist(filepath, 'file')
    unzip([filepath '.ZIP'])
end

% load the data:
nmea = read_nmea_gsv_gps(filepath);
nmea = fix_nmea_gsv_angle (nmea);

% disply the data for each satellite:
% (zoom in to inspect just parts of the data)
snr_plot4_all(nmea, [0 45])
