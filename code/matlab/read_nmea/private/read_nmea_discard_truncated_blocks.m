function [ind_beg, ind_end, num_block] = read_nmea_discard_truncated_blocks (ind_beg, ind_end, j_max)
  if (nargin < 3),  j_max = [];  end
  %keyboard()  % DEBUG
  
  % case 0: only truncated blocks exist.
  if isempty(ind_beg) ...
  || isempty(ind_end)
    num_block = 0;
    ind_beg = [];
    ind_end = [];
    return;
  end

  % case 1: first block is truncated: end; beg, end; beg, end;
  while (ind_beg(1) > ind_end(1))
    ind_end(1) = [];
  end
  num_block_end = numel(ind_end);
  
  % case 2: last block is truncated: beg, end; beg, end; beg,
  while (ind_beg(end) > ind_end(end))
    ind_beg(end) = [];
  end
  num_block_beg = numel(ind_beg);

  % case 3: no truncated block exists: beg, end; beg, end; beg, end;
  if (num_block_beg == num_block_end) ...
  && all(ind_beg < ind_end)
    num_block = num_block_beg;
    return;
  end
  
  % case 4: a middle block is truncated
  %   beg, end; beg, end; beg, beg, end;
  %   beg, end; beg, end; end; beg, end;  
  %   [1 3; 4 6; 7 9]: [1 4 7]-[3 6 9]
  %   [1 3; 4 NaN; 7 9]: [1 4 7]-[3 9]: 1-3; 7-9
  %   [1 3; NaN 6; 7 9]: [1 7]-[3 6 9]: 1-3; 7-9
  num_block = max(num_block_beg, num_block_end);
  ind_beg(end+1:num_block) = NaN;
  ind_end(end+1:num_block) = NaN;
  i = 1;
  while true
    %disp(ind_beg)  % DEBUG
    diff_beg = ind_beg(i+1) - ind_beg(i);
    diff_end = ind_end(i+1) - ind_end(i);
    diff_lat = ind_end(i)   - ind_beg(i);  % lateral difference
    if (diff_beg == 1)
      ind_beg(i) = [];
      rm_blk_counter();
    elseif (diff_end == 1)
      ind_end(i+1) = [];
      %ind_end(i) = [];  % wrong!  keep closest one.
      rm_blk_counter();
    %elseif (diff_lat < 0)
    elseif (diff_lat <= 1)  % empty block
      %ind_beg(i) = [];  % WRONG!
      ind_end(i) = [];
      rm_blk_counter();
    elseif (diff_beg <= diff_lat)
      ind_beg(i) = [];
      rm_blk_counter();
    elseif (diff_end <= diff_lat)
      ind_end(i+1) = [];
      %ind_end(i) = [];  % wrong!  keep closest one.
      rm_blk_counter();
    end
    if (i>=(num_block-1)),  break;  end
    i = i + 1;
  end
  function rm_blk_counter ()
    i = i - 1;
    %i = i - 2;  if (i < 1),  i = 1;  end  % EXPERIMENTAL!
    num_block = num_block - 1;    
  end
  ind_beg(isnan(ind_beg)) = [];
  ind_end(isnan(ind_end)) = [];
  if isempty(j_max),  j_max = 10;  end
  for j=1:j_max
    num_block_beg = numel(ind_beg);
    num_block_end = numel(ind_end);
    check = (num_block_beg == num_block_end) && all(ind_beg < ind_end);
    if check,  break;  end
    %keyboard();  % DEBUG
    if (j==j_max)
      error('MATLAB:read_nmea_gsv_gps:badData', ...
        'Unable to recover block structure.');
    end
    [ind_beg, ind_end, num_block] = read_nmea_discard_truncated_blocks (ind_beg, ind_end, j_max-1);
  end
  myassert(check)
  num_block = num_block_beg;
end

%%
% test('read_nmea_discard_truncated_blocks', [], 'C:\work\m\gnss\file\private');
%!shared
%! ind_beg = [1 4 7];
%! ind_end = [3 6 9];

%!test
%! % case 0: only truncated blocks exist.
%! ind_beg_in = ind_beg(1);
%! ind_end_in = [];
%! ind_beg_out = [];
%! ind_end_out = [];
%! [ind_beg_out2, ind_end_out2] = read_nmea_discard_truncated_blocks (ind_beg_in, ind_end_in);
%! myassert(ind_beg_out2, ind_beg_out)
%! myassert(ind_end_out2, ind_end_out)

%!test
%! % case 1: first block is truncated:
%! ind_beg_in = ind_beg(2:end);
%! ind_end_in = ind_end;
%! ind_beg_out = ind_beg(2:end);
%! ind_end_out = ind_end(2:end);
%! [ind_beg_out2, ind_end_out2] = read_nmea_discard_truncated_blocks (ind_beg_in, ind_end_in);
%! myassert(ind_beg_out2, ind_beg_out)
%! myassert(ind_end_out2, ind_end_out)

%!test
%! % case 2: last block is truncated:
%! ind_beg_in = ind_beg;
%! ind_end_in = ind_end(1:end-1);
%! ind_beg_out = ind_beg(1:end-1);
%! ind_end_out = ind_end(1:end-1);
%! [ind_beg_out2, ind_end_out2] = read_nmea_discard_truncated_blocks (ind_beg_in, ind_end_in);
%! myassert(ind_beg_out2, ind_beg_out)
%! myassert(ind_end_out2, ind_end_out)

%!test
%! % case 3: no truncated block exists:;
%! ind_beg_in = ind_beg;
%! ind_end_in = ind_end;
%! ind_beg_out = ind_beg;
%! ind_end_out = ind_end;
%! [ind_beg_out2, ind_end_out2] = read_nmea_discard_truncated_blocks (ind_beg_in, ind_end_in);
%! myassert(ind_beg_out2, ind_beg_out)
%! myassert(ind_end_out2, ind_end_out)

%!test
%! % case 4: a middle block is truncated
%! ind_beg_in = ind_beg;
%! ind_end_in = ind_end([1 end]);
%! ind_beg_out = ind_beg([1 end]);
%! ind_end_out = ind_end([1 end]);
%! [ind_beg_out2, ind_end_out2] = read_nmea_discard_truncated_blocks (ind_beg_in, ind_end_in);
%! myassert(ind_beg_out2, ind_beg_out)
%! myassert(ind_end_out2, ind_end_out)

%!test
%! % case 4: a middle block is truncated -- second variant:
%! ind_beg_in = ind_beg([1 end]);
%! ind_end_in = ind_end;
%! ind_beg_out = ind_beg([1 end]);
%! ind_end_out = ind_end([1 end]);
%! [ind_beg_out2, ind_end_out2] = read_nmea_discard_truncated_blocks (ind_beg_in, ind_end_in);
%!   %ind_beg_out, ind_beg_out2  % DEBUG
%!   %ind_end_out, ind_end_out2  % DEBUG
%!   %disp('hw!')  % DEBUG
%! myassert(ind_beg_out2, ind_beg_out)
%! myassert(ind_end_out2, ind_end_out)

%!test
%! % case 5: mulpliple beginnings without ends
%! ind_beg_in = [2 11 12 13 14 15 19];
%! ind_end_in = [8 25];
%! ind_beg_out = [2 19];
%! ind_end_out = [8 25];
%! [ind_beg_out2, ind_end_out2] = read_nmea_discard_truncated_blocks (ind_beg_in, ind_end_in);
%!   %ind_beg_out, ind_beg_out2  % DEBUG
%!   %ind_end_out, ind_end_out2  % DEBUG
%!   %disp('hw!')  % DEBUG
%! myassert(ind_beg_out2, ind_beg_out)
%! myassert(ind_end_out2, ind_end_out)

%!test
%! % case 6: problematic
%! ind_beg_in = [1 10 27];
%! ind_end_in = [8 21 25 34];
%! ind_beg_out = [1 10 27];
%! ind_end_out = [8 21 34];
%! [ind_beg_out2, ind_end_out2] = read_nmea_discard_truncated_blocks (ind_beg_in, ind_end_in);
%!   %ind_beg_out, ind_beg_out2  % DEBUG
%!   %ind_end_out, ind_end_out2  % DEBUG
%!   %disp('hw!')  % DEBUG
%! myassert(ind_beg_out2, ind_beg_out)
%! myassert(ind_end_out2, ind_end_out)
