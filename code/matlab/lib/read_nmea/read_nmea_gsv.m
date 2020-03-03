function answer = read_nmea_gsv (filepath, input_sys, downsample_interv, max_num_lines_in_mem)
%READ_NMEA_GSV: Read NMEA Satellites in View (GSV) data.

  input_sys_default = 'G';
  %if (nargin < 2) || isempty(input_sys),  input_sys = input_sys_default;  end
  if (nargin < 2),  input_sys = input_sys_default;  end
  assert(ischar(input_sys) || isempty(input_sys))
  if (nargin < 3),  downsample_interv = [];  end
  if (nargin < 4),  max_num_lines_in_mem = [];  end
  
  if (input_sys ~= 'G')
    error('MATLAB:read_nmea_gsv:unkSys', 'Only GPS supported.')
  end

  answer = read_nmea_gsv_gps (filepath, downsample_interv, max_num_lines_in_mem);  
  %TODO: read_nmea_gsv_glo (GLONASS)
  %TODO: read_nmea_gsv (multi-GNSS)
end
