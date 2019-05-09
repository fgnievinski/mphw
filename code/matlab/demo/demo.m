lib_dir = 'c:\work\misc\fx\read_nmea\lib\';
addpath(genpath(lib_dir))

data_dir = 'c:\work\misc\fx\read_nmea\demo\';
filename = '190110.LOG';
filepath = fullfile(data_dir, filename);
if ~exist(filepath, 'file')
    unzip([filepath '.ZIP'])
end

nmea = read_nmea_gsv_gps(filepath);
nmea = fix_nmea_gsv_angle (nmea);

snr_plot4_all(nmea, [0 45])
