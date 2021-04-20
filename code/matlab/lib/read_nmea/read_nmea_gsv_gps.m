function obs = read_nmea_gsv_gps (filepath, downsample_interv, max_num_lines_in_mem, discard_nans)
%READ_NMEA_GSV: Read NMEA Satellites in View (GSV) data -- GPS only.
% 
% SEE ALSO: read_rinex_obs3_single
  
  if (nargin < 2),  downsample_interv = [];  end
  if (nargin < 3),  max_num_lines_in_mem = [];  end
  if (nargin < 4) || isempty(discard_nans),  discard_nans = false;  end
  
  if iscell(filepath)
    C = filepath;  % shortcut for testing
  else
    C = textscanline(filepath);
  end

  if ~any(strstart('$GPRMC', C)) ...
  || ~any(strstart('$GPGGA', C))
    error('MATLAB:read_nmea_gsv_gps:badFile', ...
      ['No NMEA data detected; '...
       'check file path, name, and extension:\n%s'], filepath);
  end
  
  C = read_nmea_gsv_decim (C, downsample_interv);
  
  [val, epoch, epoch_unique, num_sat_per_block, status, status_unique] = ...
    read_nmea_gsv_mem (C, max_num_lines_in_mem);

  info = struct();
  info.prn  = val(:,1);
  info.elev = val(:,2);
  info.azim = val(:,3);
  info.epoch = zeros(0,1);
  info.status = zeros(0,1);

  info.prn_unique = unique(info.prn(~isnan(info.prn)));
  info.num_sats = numel(info.prn_unique);
  info.num_sat_per_epoch = num_sat_per_block;
  
  info.epoch = epoch;
  info.epoch_unique = epoch_unique;
  info.num_epochs = numel(info.epoch_unique);

  info.num_sys = 1;
  info.sys_unique = 'G';
  info.num_obs_types = 1;
  info.obs_type = {'S1C'};
  
  info.status = status;
  info.status_unique = status_unique;

  snr = val(:,end);
  obs = struct('data',snr, 'info',info);
  
  if ~discard_nans,  return;  end
  idx = isnan(obs.info.prn) ...
      | isnan(obs.info.elev) ...
      | isnan(obs.info.azim) ...
      | isnan(obs.info.epoch) ...
      | isnan(obs.info.status);
  if ~any(idx),  return;  end
  obs.data(idx) = [];
  obs.info.prn(idx) = [];
  obs.info.elev(idx) = [];
  obs.info.azim(idx) = [];
  obs.info.epoch(idx) = [];
  obs.info.status(idx) = [];

  obs.info.prn_unique = unique(obs.info.prn);
  obs.info.num_sats = numel(obs.info.prn_unique);
  %obs.info.epoch_unique = unique(info.epoch);
  [obs.info.epoch_unique, obs.info.num_sat_per_epoch] = unique2(info.epoch);
  obs.info.num_epochs = numel(info.epoch_unique);
  obs.info.status_unique = unique(info.epoch);
end

%%
function C = read_nmea_gsv_decim (C, downsample_interv)
%READ_NMEA_GSV_DECIM: Decimate data based on sampling interval (in seconds).

  if isempty(downsample_interv) || (downsample_interv == 0),  return;  end
  isint = @(x) (x==round(x));
  assert(isint(downsample_interv) && 0 <= downsample_interv && downsample_interv <= 60);

  idx_rmc = strstart('$GPRMC', C);
  C_rmc = C(idx_rmc);

  f = @(x) x(12:15);
  sec_str = cell2mat(cellfun2(f, C_rmc));
  sec = char2doublesum(sec_str);
    if isempty(sec),  sec = zeros(0,1);  end
  
  idx = is_multiple(sec, downsample_interv);
  %C_rmc(~idx) = {'INVALIDATED'};
  %C(idx_rmc) = C_rmc;
  
  ind_rmc = find(idx_rmc);
  ind_rmc2 = ind_rmc(idx);
  C_rmc2 = C_rmc(idx);
    assert(isequaln(C_rmc2, C(ind_rmc2)));
  
  idx_gga = strstart('$GPGGA', C);
  C_gga = C(idx_gga);
  %f = @(x) x(7:(1:8));
  f = @(x) x((7+1):min(end,7+8));  % empty output if necessary
  %gga_time = cell2mat(cellfun2(f, C_gga));
  %rmc_time = cell2mat(cellfun2(f, C_rmc2));
  %[idx, ind_rmc23] = ismember(gga_time, rmc_time, 'rows');
  gga_time = cellfun2(f, C_gga);
  rmc_time = cellfun2(f, C_rmc2);
  [idx, ind_rmc23] = ismember(gga_time, rmc_time);
  
  ind_gga = find(idx_gga);
  ind_gga3 = ind_gga(idx);  % [1 301 601 ...]
  ind_rmc3 = ind_rmc2(ind_rmc23(ind_rmc23>0));  % [5 305 605 ...]

  % ind_good = [1 2 3 4 5 301 302 303 304 305 601 602 ...]';
  ind_good = cell2mat(arrayfun2(@(a,b) (a:b)', ind_gga3, ind_rmc3));
  C = C(ind_good);
end

%% parse file in chunks that fit in memory:
function varargout = read_nmea_gsv_mem (C, max_num_lines_in_mem)
  if (nargin < 2) || isempty(max_num_lines_in_mem),  max_num_lines_in_mem = 10e3;  end
  num_lines = numel(C);
  if isinf(max_num_lines_in_mem),  max_num_lines_in_mem = num_lines;  end
  num_runs = ceil(num_lines / max_num_lines_in_mem);
  varargouts = cell(num_runs,nargout);
  %ws = warning('off', 'MATLAB:read_nmea_gsv_gps:gsvIsOut');
  I_rmc = strstart('$GPRMC', C);
  k_min = 1;
  for i=1:num_runs
    %k_min = max_num_lines_in_mem * (i-1) + 1;
    k_max = k_min + max_num_lines_in_mem - 1;
    k_max = min(num_lines, k_max);
    if (i==num_runs),  k_max = num_lines;  end
    k_max = find(I_rmc(1:k_max), 1, 'last');
      %[k_min k_max]  % DEBUG
    Ci = C(k_min:k_max);
    [varargouts{i,1:nargout}] = read_nmea_gsv_aux (Ci);
    k_min = k_max + 1;
  end
  %warning(ws);
  if (nargout == 1)
    varargout = varargouts;
    return;
  end
  varargout = cell(nargout,1);
  for j=1:nargout
    %[val, epoch, epoch_unique, num_sat_per_block, status, status_unique] = read_nmea_gsv_aux (C)  %% epoch; status; 
    varargout{j} = vertcat(varargouts{:,j});
  end
end

%%
function [val, epoch, epoch_unique, num_sat_per_block, status, status_unique] = read_nmea_gsv_aux (C)
%READ_NMEA_GSV_AUX: Auxiliary function to read NMEA GSV data.

  % Terminology:
  % - a "block" is a data portion collected at the same epoch
  % - a block may contain a number of "sentences" (not necessarily one per line).
  % - a complete block starts with a GGA sentence.
  % - a complete block ends with an RMC sentence.
  % - a complete block may contain multiple GSV sentences.
  % - a "GSV sentence" may be made of multiple GSV messages (normally 2 or 3)
  % - a "GSV message" (one per line) contains multiple GSV tuples (up to four)
  % - a "GSV tuple" describes one satellite with four GSV field values.
  % - the "GSV fields" are given in the following order: PRN, elevation, azimuth, SNR;
  % - the GSV SNR values may be non-integer.
  % - missing values (often SNR) are delimited by ",," (two subsequent commas with no intervening space)
  
  %% find beginning and end of each block:
  I_gga = strstart('$GPGGA', C);
  I_rmc = strstart('$GPRMC', C);
  I_beg = I_gga;
  I_end = I_rmc;
  clear I_gga
  clear I_rmc
  
  %% discard truncated blocks (beginnings with no ends or vice versa):
  ind_beg = find(I_beg);
  ind_end = find(I_end);
  clear I_beg I_end
  [ind_beg, ind_end, num_block] = read_nmea_discard_truncated_blocks (ind_beg, ind_end);

  %% discard blocks not having exactly one initial GSV message (zero or more than one):
  I_ini = ~cellfun(@isempty, regexp(C, '^\$GPGSV,.,1'));
  ind_ini = find(I_ini);
  %is_ini_in_blk = bsxfun(@lt, ind_beg', ind_ini) ...
  %              & bsxfun(@gt, ind_end', ind_ini);
  %is_blk_bad = ~any(is_ini_in_blk);
  [~, is_ini_in_blk] = is_in_period(ind_ini, [ind_beg ind_end], 2, false);
  %is_blk_bad = full(~any(is_ini_in_blk)');  % WRONG! doesn't check for more than one.
  is_blk_bad = full(sum(is_ini_in_blk)'~=1);
  if any(is_blk_bad)
    warning('MATLAB:read_nmea_gsv_gps:blkIsBad', ...
      'Discarding %d bad block(s) with missing initial GSV message.', sum(is_blk_bad));
  end
  ind_beg(is_blk_bad) = [];
  ind_end(is_blk_bad) = [];
  num_block = num_block - sum(is_blk_bad);
  
  %% discard GSV messages outside any blocks:
  I_msg = strstart('$GPGSV', C);
  ind_msg = find(I_msg);
  %is_msg_in_blk = bsxfun(@lt, ind_beg', ind_msg) ...
  %              & bsxfun(@gt, ind_end', ind_msg);
  %is_msg_out = ~is_msg_out;
  % avoid out-of-memory errors using sparse matrix:
  [is_msg_in, is_msg_in_blk] = is_in_period (ind_msg, ...
    [ind_beg ind_end], 2, false);
  is_msg_out = ~is_msg_in;  clear is_msg_in
  if any(is_msg_out)
    warning('MATLAB:read_nmea_gsv_gps:gsvIsOut', ...
      'Discarding %d GSV message(s) not in any block.', full(sum(is_msg_out)));
  end
  I_msg(ind_msg(is_msg_out)) = false;
  I_ini = I_ini & I_msg;
  ind_msg_old = ind_msg;
  clear ind_ini ind_msg
  
  %% discard incomplete blocks, with missing GSV messages, i.e., fewer than announced:
  I_ini_old = I_ini;
  % number of satellites per block -- announced version:
  num_sat_per_block = char2doublesum(cell2mat(cellfun2(@(x) x(12:13), C(I_ini))));
  % number of messages per block -- actual count:
  num_msg_per_block = char2double(cellfun(@(x) x(08), C(I_ini)));
  num_msg_per_block2 = sum(is_msg_in_blk)';
  is_blk_bad = (num_msg_per_block ~= num_msg_per_block2) | (num_sat_per_block == 0);
  if any(is_blk_bad)
    warning('MATLAB:read_nmea_gsv_gps:blkIsBad', ...
      'Discarding %d bad block(s) with missing or empty GSV messages.', sum(is_blk_bad));
  end
  ind_beg(is_blk_bad) = [];
  ind_end(is_blk_bad) = [];
  %clear ind_beg ind_end
  num_block = num_block - sum(is_blk_bad);

  %% discard existing GSV messages in incomplete blocks (with missing GSV messages):
  is_msg_in_badblk = any(is_msg_in_blk(:,is_blk_bad), 2);
  if any(is_msg_in_badblk)
    warning('MATLAB:read_nmea_gsv_gps:msgIsBad', ...
      'Discarding %d GSV messages in bad blocks.', full(sum(is_msg_in_badblk)));
  end
  I_msg(ind_msg_old(is_msg_in_badblk)) = false;
  I_ini = I_ini & I_msg;
  assert(sum(I_ini) == num_block)

  %% extract GSV message counts and counters:
  msg_ind = char2double(cellfun(@(x) x(10), C(I_msg)));  % message index within a block (counter)
  if ~isequal(I_ini, I_ini_old)
    num_msg_per_block = char2double(cellfun(@(x) x(08), C(I_ini)));  % number of messages per block (count)
    num_sat_per_block = char2doublesum(cell2mat(cellfun2(@(x) x(12:13), C(I_ini))));  % number of satellites per block
  end
  
  %% define block counter and repeate for all GSV messages in a block:
  ind_block = (1:num_block)';
  msg_ind_block = NaN(sum(I_msg),1);
  for i=1:max(num_msg_per_block)
    msg_ind_block(msg_ind==i) = ind_block(num_msg_per_block>=i);
  end

  %% parse GSV tuples:
  % (note: blank values are signaled by ",," and may occur anywhere in the sentence.)
  f = @(x) cell2mat(textscan(x(15:end-3), '%f', 'Delimiter',','))';
  tmp = C(I_msg);
  tmp = strrep(tmp, ',*', ',,*');  % fix signaling for blank SNR at the end of a sentence.
  val1 = cellfun2(f, tmp);
    %celldisp(val(1:2))

  %% deal with incomplete GSV tuples (missing some of the four values):
  % (this is not to be confused with blank values or NaN which are fine)
  num_val_per_tuple = 4;  % PRN, elev, azim, SNR.
  max_num_msg_per_block = 4;
  num_val_per_msg = cellfun(@numel, val1);
  idx = ~is_multiple(num_val_per_msg, num_val_per_tuple);
  discard_incomplete_messages = true;
  if any(idx)
    if discard_incomplete_messages
      % invalidate the whole message because truncated tuples do not necessarily occur at the end of the message:
      num_tuples_per_msg = num_val_per_msg ./ num_val_per_tuple;
      num_tuples_per_msg = min(num_tuples_per_msg, max_num_msg_per_block);
      tmp = ceil(num_tuples_per_msg);
      tmp2 = arrayfun2(@(x) NaN(1,x), tmp(idx)*num_val_per_tuple);
      val1(idx) = tmp2;
      %val1(idx) = {[]};  % WRONG! will invalidate not just the message but also the whole block.
      num_val_per_msg = cellfun(@numel, val1);
    else
      num_val_missing = ceil(ceil(num_val_per_msg./num_val_per_tuple)*num_val_per_tuple-num_val_per_msg);
      tmp = arrayfun2(@(x) NaN(1,x), num_val_missing);
      val1 = cellfun2(@(x,y) [x y], val1, tmp);
      num_val_per_msg = cellfun(@numel, val1);
      assert(none(rem(num_val_per_msg, 4)))
    end
  end
  num_tup_per_msg = num_val_per_msg / num_val_per_tuple;

  %% join GSV tuples of a same block:
  val2 = arrayfun2(@(x) cat(2, val1{msg_ind_block==x}), ind_block);
  num_val_per_block = cellfun(@numel, val2);
  num_tup_per_block = num_val_per_block / 4;
   clear val1
  
  %% deal with corrupted blocks:
  is_blk_bad2 = (num_sat_per_block ~= num_tup_per_block);
  if any(is_blk_bad2)
    warning('MATLAB:read_nmea_gsv_gps:blkIsBad', ...
      'Discarding %d corrupted block(s).', full(sum(is_blk_bad2)));
    ind_beg(is_blk_bad2) = []; %#ok<NASGU>
    ind_end(is_blk_bad2) = [];
    num_sat_per_block(is_blk_bad2) = [];
    num_tup_per_block(is_blk_bad2) = [];
    num_msg_per_block(is_blk_bad2) = [];
    val2(is_blk_bad2) = [];
    
    %ind_block(is_blk_bad2) = [];  % WRONG!
    ind_block_old = ind_block;
    num_block_old = num_block;
    num_block = num_block_old - sum(is_blk_bad2);
    ind_block = (1:num_block)';
    [~, ind_block_old2new] = ismember(ind_block_old, ind_block_old(~is_blk_bad2));
    
    idx_tmp = ~ismember(msg_ind_block, ind_block_old(is_blk_bad2));
    msg_ind_block = msg_ind_block(idx_tmp);
    msg_ind_block = ind_block_old2new(msg_ind_block);
    num_tup_per_msg = num_tup_per_msg(idx_tmp);
  end
   myassert(num_sat_per_block, num_tup_per_block)
  clear I_msg I_ini

  %% get block index for each tuple:
  tup_ind_block  = cell2mat(arrayfun2(@(x,y) repmat(x, y, 1), ind_block,     num_tup_per_block));
  tup_ind_block2 = cell2mat(arrayfun2(@(x,y) repmat(x, y, 1), msg_ind_block, num_tup_per_msg));
   myassert(tup_ind_block, tup_ind_block2)

  %% organize tuples as a table:
  val3 = cellfun2(@(x) reshape(x, 4, [])', val2);
  val = cell2mat(val3);
  if isempty(val),  val = reshape(val, [0 3]);  end
  
  %% extract epoch (date/time) for each block:
  findk1 = @(idx, k) getel(find(idx), k);
  ind_rmc = ind_end;
  ind_gga = ind_beg;
  C_rmc = C(ind_rmc);
  C_gga = C(ind_gga);
  f = @(x) x(findk1(x==',', 9)+(1:6));
  date_str = cell2mat(cellfun2(f, C_rmc));
  f = @(x) x(7+(1:6));
  time_str = cell2mat(cellfun2(f, C_rmc));
  time_str2= cell2mat(cellfun2(f, C_gga));
  f = @(x) x(findk1(x=='.', 1):(findk1(x==',', 2)-1));
  frac_str = cellfun2(f, C_rmc);  % fractional part of time
  epoch_dont_match = ~strcmp(time_str, time_str2);
  
  %% extract fix status flag:
  % (A stands for active or valid and V for void or warning, although:
  % "A status of V means the GPS has a valid fix that is below an internal
  % quality threshold, e.g. because the dilution of precision is too high
  % or an elevation mask test failed."
  % <http://www.catb.org/gpsd/NMEA.html#_rmc_recommended_minimum_navigation_information>
  %f = @(x) x(19);  % WRONG!  time width is variable
  f = @(x) x(findk1(x==',', 2)+1);
  status = cell2mat(cellfun2(f, C_rmc));
    if isempty(status),  status = zeros(0,1);  end
  status(~(status=='A' | status=='V')) = ' ';  % blank unknown status

  %% convert epoch from string to numeric:
  epoch_unique = mydatestri([date_str time_str], 'ddmmyyHHMMSS');
  %epoch_unique = mydatestri([date_str time_str], 'ddmmyyHHMMSS.FFF');
  %epoch_unique1 = mydatestri([date_str time_str(:,1:6)], 'ddmmyyHHMMSS');
  %epoch_unique2 = mydateseci(char2doublesum(time_str(:,7:10)))
  %epoch_unique = epoch_unique1 + epoch_unique2;

  %% check invalid date:
  % GPS Time is a uniformly counting time scale beginning at midnight Jan 05-06, 1980.
  if ~isempty(date_str)
    tmp = cellstr(date_str);
  else
    tmp = {};
  end
  idx = strcmp(tmp, '050180');
  %idx = strcmp(tmp, '080180') | idx;  % another weird case.
  if any(idx)
    warning('MATLAB:read_nmea_gsv_gps:epochIsBad', ...
      'Discarding %d bad epoch(s) with initial GPS date (Jan 05-06, 1980).', sum(idx));
    epoch_unique(idx) = NaN;
  end
  if any(epoch_dont_match)
    warning('MATLAB:read_nmea_gsv_gps:epochDontMatch', ...
      'Discarding %d bad epoch(s) with different initial and ending time.', sum(epoch_dont_match));
    epoch_unique(epoch_dont_match) = NaN;
  end
  
  %% convert fractional seconds:
%   ns = cellfun(@numel, frac_str);
%   if all(ns==0)
%     frac = 0;
%   elseif is_uniform(ns)
%     frac_str = cell2mat(frac_str);
%     assert(frac_str(1,1)=='.')
%     frac = char2doublesum(frac_str, 1);
%   else
%     frac = str2double(frac_str);
%   end
%   %epoch_unique = epoch_unique + mydateseci(frac);
%   % round up to input numerical resolution:
%   temp = mydatesec(epoch_unique, false) + mydateseci(frac);
%   temp = roundn(temp, -(max(ns)-1));
%   epoch_unique = mydateseci(temp, false);    
  n = cellfun(@numel, frac_str);
  if all(n==0)
    frac = 0;
  elseif is_uniform(n)
    frac_str = cell2mat(frac_str);
    assert(frac_str(1,1)=='.')
    frac = char2doublesum(frac_str, 1);
  else
    frac = str2double(frac_str);
  end
  epoch_unique = epoch_unique + mydateseci(frac);
  
  %% round epoch up to input precision:
  nm = max(n);
  %nm = nm + 1;  % WRONG (n already counts the decimal separator)
  % find equivalent precision in epoch format:
  nm2 = -round(log10(mydateseci(10.^-nm)));
  epoch_unique = roundn(epoch_unique, -nm2);
  clear n nm
  
  %% replicate epoch for each tuple:
  epoch = epoch_unique(tup_ind_block);
  
  %% replicate status for each tuple:
  %status_unique = status;  % WRONG!
  status_unique = unique(status);
  status = status(tup_ind_block);  
end

%!test
%! %% input data:
%! C = {...
%!   '$GPGGA,131200.086,,,,,0,0,,,M,,M,,*47'
%!   '$GPGLL,,,,,131200.086,V,N*75'
%!   '$GPGSA,A,1,,,,,,,,,,,,,,,*1E'
%!   '$GPGSV,2,1,08,31,,,32.3,27,,,34.3,23,,,35.6,11,,,33.2*7A'
%!   '$GPGSV,2,2,08,10,,,25.9,14,,,31.1,22,,,38.3,18,,,31.3*72'
%!   '$GPRMC,131200.086,V,,,,,0.00,0.00,220618,,,N*4D'
%!   '$GPVTG,0.00,T,,M,0.00,N,0.00,K,N*32'
%!   ''
%!   '$GPGGA,191958.000,3000.2964,S,05007.7461,W,1,8,1.00,-18.6,M,4.0,M,,*7D'
%!   '$GPGLL,3000.2964,S,05007.7461,W,191958.000,A,A*57'
%!   '$GPGSA,A,3,05,09,28,30,27,07,23,08,,,,,1.29,1.00,0.82*0D'
%!   '$GPGSV,3,1,11,07,62,183,39.0,09,60,105,30.8,30,55,250,42.9,28,33,346,43.7*78'
%!   '$GPGSV,3,2,11,23,28,074,37.5,05,19,224,48.7,08,16,086,36.9,27,08,120*72'
%!   '$GPGSV,3,3,11,06,06,309,23.5,02,04,271,40.5,03,02,023,37.0*5E'
%!   '$GPRMC,191958.000,A,3000.2964,S,05007.7461,W,4.82,302.09,220618,,,A*69'
%!   '$GPVTG,302.09,T,,M,4.82,N,8.94,K,A*3E'
%! };
%! 
%! %% manual answer, part 1:
%! tmp = cell(2,1);
%! tmp{1} = [...
%!   31,NaN,NaN,32.3
%!   27,NaN,NaN,34.3
%!   23,NaN,NaN,35.6
%!   11,NaN,NaN,33.2
%!   10,NaN,NaN,25.9
%!   14,NaN,NaN,31.1
%!   22,NaN,NaN,38.3
%!   18,NaN,NaN,31.3
%! ];
%! tmp{2} = [...
%!   07,62,183,39.0
%!   09,60,105,30.8
%!   30,55,250,42.9
%!   28,33,346,43.7
%!   23,28,074,37.5
%!   05,19,224,48.7
%!   08,16,086,36.9
%!   27,08,120,NaN
%!   06,06,309,23.5
%!   02,04,271,40.5
%!   03,02,023,37.0
%! ];
%! % redefine answer to disable whole line when data is missing:
%! tmp{2} = [...
%!   07,62,183,39.0
%!   09,60,105,30.8
%!   30,55,250,42.9
%!   28,33,346,43.7
%!   NaN,NaN,NaN,NaN
%!   NaN,NaN,NaN,NaN
%!   NaN,NaN,NaN,NaN
%!   NaN,NaN,NaN,NaN
%!   06,06,309,23.5
%!   02,04,271,40.5
%!   03,02,023,37.0
%! ];
%! tmpc = cell2mat(tmp);
%! epoch_unique = mydatenum([...
%!   2018 06 22  13 12 00.086
%!   2018 06 22  19 19 58.000
%! ]);
%! 
%! %% truncate first or last blocks:
%! for k=1:3  
%! switch k
%! case 1
%!    % use the original unmodified data:
%!    C_orig = C;
%!    tmp_orig = tmp;
%!    epoch_unique_orig = epoch_unique;
%! case 2
%!    % truncate first block:
%!    C = C_orig(5:end);
%!    tmp = tmp_orig(2);
%!    tmpc = tmp_orig{2};
%!    epoch_unique = epoch_unique_orig(2);
%! case 3
%!    % truncate last block:
%!    C = C_orig(1:end-4);
%!    tmp = tmp_orig(1);
%!    tmpc = tmp_orig{1};
%!    epoch_unique = epoch_unique_orig(1);
%! end
%! %end  % WRONG! end only after everything.
%! 
%! %% manual answer, part 2:
%! info = struct();
%! info.prn  = tmpc(:,1);
%! info.elev = tmpc(:,2);
%! info.azim = tmpc(:,3);
%! 
%! %info.prn_unique = unique(info.prn);
%! info.prn_unique = unique(info.prn(~isnan(info.prn)));
%! info.num_sats = numel(info.prn_unique);
%! info.num_sat_per_epoch = cellfun(@(x) size(x,1), tmp);
%! info.num_epochs = numel(epoch_unique);
%! 
%! info.epoch_unique = epoch_unique;
%! info.epoch = cell2mat(arrayfun2(@(x,y) repmat(x, y, 1), ...
%!   info.epoch_unique, info.num_sat_per_epoch));
%! 
%! info.num_sys = 1;
%! info.sys_unique = 'G';
%! info.num_obs_types = 1;
%! info.obs_type = {'S1C'};
%! 
%! data = tmpc(:,end);
%! answer = struct('data',data, 'info',info);
%! 
%! %%% write the data to file:
%! %filepath = tempname();
%! %fid = fopen_error(filepath, 'wt');
%! %for i=1:length(C),  fprintf (fid, '%s\n', C{i});  end
%! %fclose(fid);
%! 
%! %% read back the data file:
%! answer2 = read_nmea_gsv_gps (C);
%! %answer2 = read_nmea_gsv_gps (filepath);
%! %delete(filepath)
%! 
%! %% compare answers:
%! %answer.data, answer2.data  % DEBUG
%! %[answer.data, answer2.data]  % DEBUG
%! %answer.data - answer2.data  % DEBUG
%! myassert(answer.data, answer2.data)
%! 
%! answer.info, answer2.info  % DEBUG
%!  %keyboard()  % DEBUG
%! all_ok = true;
%! epoch_tol = mydateseci(0.001/2);
%! fn = fieldnames(answer.info);
%! for i=1:numel(fn)
%!   if ~isequaln(answer2.info.(fn{i}), answer.info.(fn{i}))
%!     if ~strstart('epoch', fn{i})
%!       %disp(fn{i})  % DEBUG
%!       all_ok = false;
%!       continue;
%!     else
%!       %disp(answer2.info.(fn{i}) - answer.info.(fn{i}))  % DEBUG
%!       if (max(abs(answer2.info.(fn{i}) - answer.info.(fn{i}))) > epoch_tol)
%!         %disp(fn{i})  % DEBUG
%!         all_ok = false;
%!       end
%!     end
%!   end
%! end
%!  %if ~all_ok,  keyboard();  end  % DEBUG
%! assert(all_ok)
%! %myassert(answer.info, answer2.info)
%! end  % for k...

%!test
%! %% input data:
%! C= {...
%!     '$GPGSV,3,1,09,25,80,042,47.8,29,56,183,50.3,20,48,036,44.7,21,39,299,47.7*77'
%!     '$GPGGA,124322.000,3001.2759,S,05113.2840,W,1,8,1.03,-8.8,M,4.5,M,,*42'
%!     '$GPGLL,3001.2759,S,05113.2840,W,124322.000,A,A*51'
%!     '$GPGSA,A,3,25,29,20,21,12,31,26,05,,,,,1.35,1.03,0.87*05'
%!     '$GPGSV,3,1,09,25,80,042,47.8,29,56,183,50.3,20,48,036,44.7,21,39,299,47.7*77'
%!     '$GPGSV,3,2,09,12,37,045,37.1,05,32,115,46.6,31,28,255,52.0,26,10,221,39.6*72'
%!     '$GPGSV,3,3,09,02,06,133,28.4*55'
%!     '$GPRMC,124322.000,A,3001.2759,S,05113.2840,W,0.01,287.23,090318,,,A*68'
%!     '$GPVTG,287.23,T,,M,0.01,N,0.01,K,A*31'
%!     ''
%!     '$GPGGA,124323.000,3001.2760,S,05113.2840,W,1,8,1.03,-8.8,M,4.5,'
%!     '$GPGGA,124325.000,3001.2760,S,05113.2840,W,1,8,1.03,-8.8,M,4.5,'
%!     '$GPGGA,124327.000,3001.2760,S,05113.2839,W,1,8,1.03,-8.8,M,4.5,'
%!     '$GPGGA,124330.000,3001.2761,S,05113.2839,W,1,8,1.03,-8.9,M,4.5,'
%!     '$GPGGA,124332.000,3001.2761,S,05113.2839,W,1,8,1.03,-8.9,M,4.5,'
%!     '$PMTK011,MTKGPS*08'
%!     '$PMTK010,001*2E'
%!     ''
%!     '$GPGGA,124348.575,3001.2747,S,05113.2841,W,1,8,1.03,-12.8,M,4.5,M,,*7C'
%!     '$GPGLL,3001.2747,S,05113.2841,W,124348.575,A,A*54'
%!     '$GPGSA,A,3,20,25,29,21,12,05,31,26,,,,,1.35,1.03,0.87*05'
%!     '$GPGSV,3,1,09,25,80,041,46.3,29,56,182,49.9,20,48,036,41.9,21,39,298,46.3*72'
%!     '$GPGSV,3,2,09,12,37,045,28.4,05,32,116,45.7,31,28,255,52.7,26,10,221,42.2*77'
%!     '$GPGSV,3,3,09,02,06,132,*44'
%!     '$GPRMC,124348.575,A,3001.2747,S,05113.2841,W,2.37,346.51,090318,,,A*63'
%!     '$GPVTG,346.51,T,,M,2.37,N,4.39,K,A*30'
%! };
%! 
%! %% manual answer, part 1:
%! tmp = cell(2,1);
%! tmp{1} = [...
%!   25,80,042,47.8
%!   29,56,183,50.3
%!   20,48,036,44.7
%!   21,39,299,47.7
%!   12,37,045,37.1
%!   05,32,115,46.6
%!   31,28,255,52.0
%!   26,10,221,39.6
%!   02,06,133,28.4
%! ];
%! tmp{2} = [...
%!   25,80,041,46.3
%!   29,56,182,49.9
%!   20,48,036,41.9
%!   21,39,298,46.3
%!   12,37,045,28.4
%!   05,32,116,45.7
%!   31,28,255,52.7
%!   26,10,221,42.2
%!   02,06,132,NaN
%! ];
%! tmpc = cell2mat(tmp);
%! epoch_unique = mydatenum([...
%!   2018 03 09 12 43 22.000
%!   2018 03 09 12 43 48.575
%! ]);
%! 
%! %% manual answer, part 2:
%! info = struct();
%! info.prn  = tmpc(:,1);
%! info.elev = tmpc(:,2);
%! info.azim = tmpc(:,3);
%! 
%! info.prn_unique = unique(info.prn);
%! info.num_sats = numel(info.prn_unique);
%! info.num_sat_per_epoch = cellfun(@(x) size(x,1), tmp);
%! info.num_epochs = numel(epoch_unique);
%! 
%! info.epoch_unique = epoch_unique;
%! info.epoch = cell2mat(arrayfun2(@(x,y) repmat(x, y, 1), ...
%!   info.epoch_unique, info.num_sat_per_epoch));
%! 
%! info.num_sys = 1;
%! info.sys_unique = 'G';
%! info.num_obs_types = 1;
%! info.obs_type = {'S1C'};
%! 
%! data = tmpc(:,end);
%! answer = struct('data',data, 'info',info);
%! 
%! %%% write the data to file:
%! %filepath = tempname();
%! %fid = fopen_error(filepath, 'wt');
%! %for i=1:length(C),  fprintf (fid, '%s\n', C{i});  end
%! %fclose(fid);
%! 
%! %% read back the data file:
%! answer2 = read_nmea_gsv_gps (C);
%! %answer2 = read_nmea_gsv_gps (filepath);
%! %delete(filepath)
%! 
%! %% compare answers:
%! %answer.data, answer2.data  % DEBUG
%! %answer.data - answer2.data  % DEBUG
%! myassert(answer.data, answer2.data)
%! 
%! %%
%!  %answer.info, answer2.info  % DEBUG
%! all_ok = true;
%! epoch_tol = mydateseci(0.001/2);
%! fn = fieldnames(answer.info);
%! for i=1:numel(fn)
%!   if ~isequaln(answer2.info.(fn{i}), answer.info.(fn{i}))
%!     if ~strstart('epoch', fn{i})
%!       %disp(fn{i})  % DEBUG
%!       all_ok = false;
%!       continue;
%!     else
%!       %disp(answer2.info.(fn{i}) - answer.info.(fn{i}))  % DEBUG
%!       if (max(abs(answer2.info.(fn{i}) - answer.info.(fn{i}))) > epoch_tol)
%!         %disp(fn{i})  % DEBUG
%!         all_ok = false;
%!       end
%!     end
%!   end
%! end
%! assert(all_ok)
%! %myassert(answer.info, answer2.info)

%!test
%! % problematic input to test (1)
%! C= {...
%!   '$GPGGA,235949.099,,,,,0,0,,,M,,M,,*48                                                          '
%!   '$GPGSA,A,1,,,,,,,,,,,,,,23,,,38.1,11,,,29.5,01,,,33.3*63                                       '
%!   '$GPRMC,235949.099,V,,,,,0.00,0.00,050180,,,N*41                                                '
%!   '$GPVTG,0.00,T,,M,0.00,N,0.00,K,N*32                                                            '
%!   '$GPGGA,235949.200,,,,,0,0,,,M,,M,,*4A                                                          '
%! };
%! answer2 = read_nmea_gsv_gps (C);
%!  %disp('hw!')

%!test
%! % problematic input to test (2)
%! C= {...
%!   '$GPGGA,194724.600,2813.9274,S,04839.0624,W,1,11,0.77,14.7,M,2.70624,W,0.88,130.35,080518,,,A*6C'
%!   '$GPVTG,130.35,T,,M,0.88,N,1.63,K,A*3D                                                          '
%!   '$GPGSV,3,3,12,01,19,354,45.3,30,09,316,44.4,11,05,3                                            '
%!   '$GPGGA,195200.000,2813.9233,S,04839.0615,W,1,11,0.80,14.8,M,2.7,M,,*6B                         '
%!   '$GPGLL,2813.9233,S,04839.0615,W,195200.000,A,A*5E                                              '
%!   '$GPGSA,A,3,03,23,22,09,07,16,06,01,26,30,11,,1.58,0.80,1.36*09                                 '
%!   '$GPGSV,3,1,12,03,67,066,36.8,23,55,166,37.3,09,42,231,36.9,22,41,049,47.3*7B                   '
%!   '$GPGSV,3,2,12,07,35,313,49.8,16,31,096,46.4,06,23,238,47.1,26,19,125,31.9*7F                   '
%!   '$GPGSV,3,3,12,01,19,354,46.3,30,09,316,44.7,11,05,356,41.5,18,03,010,32.2*74                   '
%!   '$GPRMC,195200.000,A,2813.9233,S,04839.0615,W,0.43,179.83,080518,,,A*6E                         '
%!   '$GPVTG,179.83,T,,M,0.43,N,0.80,K,A*36                                                          '
%! };
%! answer2 = read_nmea_gsv_gps (C);
%!  %disp('hw!')

%!test
%! % problematic input to test (3)
%! C= {...
%!   '$GPGGA,235956.000,,,,,0,0,,,M,,M,,*46                                                          '
%!   '$GPGSA,A,1,,,,,,,,,,,,,,,,,36.9,17,,,25.7,09,,,33.9,11,,,35.1*73                               '
%!   '$GPGSV,2,2,05,18,,,34.0*6C                                                                     '
%!   '$GPRMC,235956.000,V,,,,,0.00,0.00,050180,,,N*4F                                                '
%!   '$GPVTG,0.00,T,,M,0.00,N,0.00,K,N*32                                                            '
%! };
%! answer2 = read_nmea_gsv_gps (C);
%!  %disp('hw!')

%!test
%! % problematic input to test (4)
%! C= {...
%!   '$GPGGA,235942.900,,,,,0,0,,,M,,M,,*4A                                                          '
%!   '$GPGSA,A,1,,,,,,,,,,,,,,,*1E                                                                   '
%!   '$GPGSV,1,1,00*79                                                                               '
%!   '$GPRMC,235942.900,V,,,,,0.00,0.00,050180,,,N*43                                                '
%! };
%! answer2 = read_nmea_gsv_gps (C);
%!  %disp('hw!')

%!test
%! % problematic input to test (5)
%! C= {...
%!   '$GPGGA,164857.000,3000.2940,S,05007.7443,W,1,11,0.87,-7.6,M,4.0,M,,*77                                                           '
%!   '$GPGLL,3000.2940,S,05007.7443,W,164857.000,A,A*55                                                                                '
%!   '$GPGSA,A,3,09,17,03,31,11,22,23,01,18,14,26,,1.37,0.87,1.06*09                                                                   '
%!   '$GPGSV,4,1,14,01,72,309,50.7,22,67,154,52.2,18,55,358,51.0,03,53,204,53.2*75                                                     '
%!   '$GPGSV,4,2,14,23,48,268,49.2,11,45,321,49.5,31,37,127,50.4,14,20,133,43.9*76                                                     '
%!   '$GPGSV,4,3,14,09,14,288,37.1,26,10,070,40.2,17,09,242,36.8,16,03,045,25.6*76                                                     '
%!   '$GPGSV,4,4,14,19,01,221,23.3,08,01,347,31.4*79                                                                                   '
%!   '$GPRMC,164857.000,A,3000.2940,S,05007.7443,W,0.01,195.46,230518,,,A*61                                                           '
%!   '$GPVTG,195.46,T,,M,0.01,N,0.02,K,A*31                                                                                            '
%!   '$GPGGA,164858.000,3000.2940,S,05007.7443,W,1,11,0.87,-7.6,M,4.0,M,,*78                                                           '
%!   '$GPGLL,3000.2940,S,05007.7443,W,164858.000,A,A*5A                                                                                '
%!   '$GPGSA,A,3,09,17,03,31,11,22,23,01,18,14,26,,1.37,0.87,1.06*09                                                                   '
%!   '$GPGSV,4,1,14,01,72,309,50.7,22,67,154,51.1,18,55,358,49.6,03,53,204,53.2*7A                                                     '
%!   '$GPGSV,4,2,14,23,48,268,46.9,11,45,321,49.9,31,37,127,50.7,14,20,133,39.6*7F                                                     '
%!   '$GPGSV,4,3,14,09,15,288,37.7,26,10,070,42.4,17,09,242,33.8,1$GPGGA,204844.000,2953.6743,S,05017.2093,W,1,11,0.77,9.2,M,4.0,M,,*52'
%!   '$GPGLL,2953.6743,S,05017.2093,W,204844.000,A,A*58                                                                                '
%!   '$GPGSA,A,3,28,07,09,23,30,03,16,02,06,08,27,,1.31,0.77,1.07*0C                                                                   '
%!   '$GPGSV,3,1,12,07,67,220,27.5,09,63,142,26.6ìIº?bþ33.6,23,35,089,24.8*71                                                          '
%!   '$GPGSV,3,2,12,28,18,348,18.1,06,15,296,19.0,03,15,025,30.3,16,10,143,34.3*7C                                                     '
%!   '$GPGSV,3,3,12,08,10,071,24.7,02,09,258,19.2,05,08,219,32.6,27,06,106,29.8*7D                                                     '
%!   '$GPRMC,204844.000,A,2953.6743,S,05017.2093,W,45.99,205.30,230518,,,A*57                                                          '
%!   '$GPVTG,205.30,T,,M,45.99,N,85.22,K,A*35                                                                                          '
%!   '3,14,09,15,288,34.1,26,10,070,40.5,17,09,242,44.9,16,03,045,29.6*7B                                                              '
%!   '$GPGSV,4,4,14,19,01,221,34.1,08,01,347,32.1*7B                                                                                   '
%!   '$GPRMC,164859.000,A,3000.2940,S,05007.7443,W,0.03,195.46,230518,,,A*6D                                                           '
%!   '$GPVTG,195.46,T,,M,0.03,N,0.05,K,A*34                                                                                            '
%!   '$GPGGA,164900.000,3000.2940,S,05007.7443,W,1,11,0.87,-7.5,M,4.0,M,,*77                                                           '
%!   '$GPGLL,3000.2940,S,05007.7443,W,164900.000,A,A*56                                                                                '
%!   '$GPGSA,A,3,09,17,03,31,11,22,23,01,18,14,26,,1.37,0.87,1.06*09                                                                   '
%!   '$GPGSV,4,1,14,01,72,309,50.1,22,67,154,45.7,18,55,358,47.9,03,53,204,50.9*76                                                     '
%!   '$GPGSV,4,2,14,23,48,268,43.2,11,45,321,49.7,31,37,127,46.2,14,20,133,45.2*72                                                     '
%!   '$GPGSV,4,3,14,09,15,288,35.6,26,10,070,39.3,17,09,242,39.5,16,03,045,32.4*7B                                                     '
%!   '$GPGSV,4,4,14,19,01,221,32.1,08,01,347,31.6*79                                                                                   '
%!   '$GPRMC,164900.000,A,3000.2940,S,05007.7443,W,0.00,195.46,230518,,,A*63                                                           '
%!   '$GPVTG,195.46,T,,M,0.00,N,0.00,K,A*32                                                                                            '
%! };
%! answer2 = read_nmea_gsv_gps (C);
%!  %disp('hw!')

%!test
%! % problematic input to test (6)
%! C= {...
%!   '$GPGGA,204834.000,2953.6037,S,05017.1157,W,1,11,0.77,16.5,M,4.0,M,,*62 '
%!   '$GPGLL,2953.6037,S,05017.1157,W,204834.000,A,A*51'
%!   '$GPGSA,A,3,28,07,09,23,30,03,16,02,06,08,27,,1.31,0.77,1.07*0C'
%!   '$GPGSV,3,1,12,07,67,221,25.2,09,63,142,31.6,30,49,273,30.8,23,35,089,37.6*72'
%!   '$GPGSV,3,2,12,28,18,348,27.1,06,15,296,34.7,03,15,025,21.8,16,10,143,22.3*74'
%!   '$GPGSV,3,3,12,08,10,071,29.3,02,09,258,32.6,05,07,219,34.1,27,06,106,23.4*71'
%!   '$GPRMC,204834.000,A,2953.6037,S,05017.1157,W,37.26,280.69,230518,,,A*5E'
%!   '$GPVTG,280.69,T,,M,37.26,N,69.05,K,A*32'
%!   '$GPGGA,204835.000,2953.6040,S,05017.1281,W,1,11,0.77,13.9,M,4.0,M,,*62'
%!   '$GPGLL,2953.6040,S,05017.1281,W,204835.000,A,A*58'
%!   '$GPGSA,A,3,28,07,09,23,30,03,16,02,06,08,27,,1.31,0.77,1.07*0C'
%!   '$GPGSV,3,1,12,07,67,221,29.4,09,63,142,32.2,30,49,273,31.9,23,35,089,32.4*78'
%!   '$GPGSV,3,2,12,28,18,348,25.6,06,15,296,27.1,03,15,025,29.9,16,10,143,26.0*7B'
%!   '$GPGSV,3,3,12,08,10,071,25.3,02,09,258,44.0,05,07,219,30.1,27,06,106,22.3*78'
%!   '$GPRMC,204835.000,A,2953.6040,S,05017.1281,W,38.27,269.47,230518,,,A*52'
%!   '$GPVTG,269.47,T,,M,38.27,N,70.91,K,A*32'
%!   'þ&þ501÷.1404,W,1,11,0.77,13.0,M,4.0,M,,*60'
%!   '$GPGLL,2953.6052,S,05017.1404,W,204836.000,A,A*53'
%!   '$GPGSA,A,3,28,07,09,23,30,03,16,02,06,08,27,,1.32,0.77,1.07*0F'
%!   '$GPGSV,3,1,12,07,67,221,33.&	Êb²?b?¢þ2,27.7,3þüæ&iªb?ÂÊbþ25.6*7F'
%!   '$GPGSV,3,2,12,28,18,348,30.0,06,15,296,27.1,03,15,025,24.1,16,10,143,22.2*7A'
%!   '$GPGSV,3,3,12,08,10,071,35.9,02,09,258,37.3,05,07,219,37.9,27,06,106,19.8*78'
%!   '$GPRMC,204836.000,A,2953.6052,S,05017.1404,W,38.56,262.47,230518,,,A*54'
%!   '$GPVTG,262.47,T,,M,38.56,N,71.45,K,A*37'
%!   '$GPGGA,204840.000,2953.6288,S,05017.1826,W,1,11,0.76,11.0,M,4.0,M,,*6B'
%!   '$GPGLL,2953.6288,S,05017.1826,W,204840.000,A,A*5B'
%!   '$GPGSA,A,3,28,07,09,23,30,03,16,02,06,08,27,,1.05,0.76,0.73*08'
%!   '$GPGSV,3,1,12,07,67,220,25.3,09,63,142,25.7,30,49,273,29.9,23,35,089,37.6*7F'
%!   '$GPGSV,3,2,12,28,18,348,24.6,06,15,296,28.6,03,15,025,32.0,16,10,143,30.9*7F'
%!   '$GPGSV,3,3,12,08,10,071,31.6,02,09,258,26.2,05,08,219,40.8,27,06,106,28.9*7F'
%!   '$GPRMC,204840.000,A,2953.6288,S,05017.1826,W,39.68,216.87,230518,,,A*5F'
%!   '$GPVTG,216.87,T,,M,39.68,N,73.54,K,A*36'
%! };
%! answer2 = read_nmea_gsv_gps (C);
%!  %disp('hw!')

%!test
%! % problematic input to test (7)
%! C= {...
%!  '$GPGGA,000000.000,3001.2742,S,05113.2830,W,2,07,1.08,6.6,M,4.5,M,0000,0000*51'
%!  '$GPGSV,3,1,11,12,65,144,45,02,59,122,36,25,44,215,38,24,44,351,35*7C'
%!  '$GPGSV,3,2,11,29,39,266,48,51,20,288,45,05,18,055,43,06,16,141,32*7D'
%!  '$GPGSV,3,3,11,32,04,260,44,14,02,240,48,31,01,215,*40'
%!  '$GPRMC,000000.000,A,3001.2742,S,05113.2830,W,0.02,86.75,300719,,,D*58'
%!  ''
%!  '$GPGGA,000001.000,3001.2742,S,05113.2830,W,2,07,1.08,6.6,M,4.5,M,0000,0000*50'
%!  '$GPGSV,3,1,11,12,65,144,45,02,59,122,36,25,44,215,30,24,44,351,35*74'
%!  '$GPGSV,3,2,11,29,39,266,49,51,20,288,45,05,18,055,43,06,16,141,32*7C'
%!  '$GPGSV,3,3,11,32,04,260,43,14,02,240,48,31,01,215,*47'
%!  '$GPRMC,000001.000,A,3001.2742,S,05113.2830,W,0.01,129.37,300719,,,D*68'
%!  ''
%!  '$GPGGA,000002.000,3001.2742,S,05113.2830,W,2,07,1.08,6.6,M,4.5,M,0000,0000*53'
%!  '$GPGSV,3,1,11,12,65,144,45,02,59,122,36,25,44,215,39,24,44,351,35*7D'
%!  '$GPGSV,3,2,11,29,39,266,48,51,20,288,45,05,18,055,43,06,16,141,33*7C'
%!  '$GPGSV,3,3,11,32,04,260,43,14,02,240,48,31,01,215,*47'
%!  '$GPRMC,000002.000,A,3001.2742,S,05113.2830,W,0.01,119.43,300719,,,D*6B'
%! };
%! answer2 = read_nmea_gsv_gps (C);
%!  %disp('hw!')

%!test
%! % problematic input to test (8)
%! C= {...
%!   '$GPGGA,200621.700,3004.4150,S,05107.2679,W,1,12,0.77,68.5,M,4.4,M,,*65'
%!   '$GPGSV,3,1,12,11,68,292,44.1,14,55,141,46.0,01,53,216,42.7,22,45,227,44.2*7A'
%!   '$GPGSV,3,2,12,31,40,065,44.1,08,30,327,39.5,32,28,138,48.3,03,27,245,31.1*75'
%!   '$GPGSV,3,3,12,04,21,313,44.3,23,15,310,41.4,27,12,355,26.0,10,08,099,35.8*7F'
%!   '$GPRMC,200621.700,A,3004.4150,S,05107.2679,W,0.10,58.97,130220,,,A*5F'
%!   ''
%!   '$GPGGA,200621.800,3004.4150,S,05107.2679,W,1,12,0.77,68.5,M,4.4,M,,*6A'
%!   '$GPGSV,3,1,12,11,68,292,44.7,14,55,141,46.7,01,53,216,44.4,22,45,227,45.2*7F'
%!   '$GPGSV,3,2,12,31,40,065,44.1,08,30,327,39.5,32,28,138,47.6,03,27,245,31.1*7F'
%!   '$GPGSV,3,3,12,04,21,313,43.,41.9,27,12,355,28.1,10,08,099,38.1*7A'
%!   '$GPRMC,200622.100,A,3004.4150,S,05107.2679,W,0.06,58.97,130220,,,A*5D'
%!   ''
%!   '$GPGGA,200622.200,3004.4150,S,05107.2679,W,1,12,0.77,68.4,M,4.4,M,,*62'
%!   '$GPGSV,3,1,12,11,68,292,46.8,14,55,141,45.6,01,53,216,47.4,22,45,227,43.1*76'
%!   '$GPGSV,3,2,12,31,40,065,45.7,08,30,327,40.5,32,28,138,48.2,03,27,245,34.6*7F'
%!   '$GPGSV,3,3,12,04,21,313,44.0,23,15,310,39.4,27,12,355,28.1,10,08,099,38.1*78'
%!   '$GPRMC,200622.200,A,3004.4150,S,05107.2679,W,0.18,62.68,130220,,,A*58'
%! };
%! answer2 = read_nmea_gsv_gps (C);
%!  %disp('hw!')
