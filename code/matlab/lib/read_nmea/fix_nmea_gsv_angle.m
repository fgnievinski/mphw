function obs = fix_nmea_gsv_angle (obs)
%FIX_NMEA_GSV_ANGLE: Correct NMEA GSV quantized elevation angle and azimuth.

  assert(isscalar(obs.info.sys_unique))

  prn   = obs.info.prn;
  epoch = obs.info.epoch;
  
  if ~isfield(obs.info, 'elev_orig'),  obs.info.elev_orig = obs.info.elev;  end
  if ~isfield(obs.info, 'azim_orig'),  obs.info.azim_orig = obs.info.azim;  end
  elev = obs.info.elev_orig;
  azim = obs.info.azim_orig;
  
  status = obs.info.status;
  active = (status == 'A');
  
  for i=1:obs.info.num_sats
    idx = (prn == obs.info.prn_unique(i));
    idx = idx & active; 
    elevi2 = fix_nmea_gsv_angle_aux (epoch(idx), elev(idx));
    azimi2 = fix_nmea_gsv_angle_aux (epoch(idx), azim(idx), true);
    elev(idx) = elevi2;
    azim(idx) = azimi2;
  end
  
  obs.info.elev = elev;
  obs.info.azim = azim;
end

%%
function angle2 = fix_nmea_gsv_angle_aux (time, angle, is_circ)
  if (nargin < 3),  is_circ = false;  end
  %method = '';
  %method = 'spline';
  method = 'pchip';
  %method = 'linear';
  
  if (numel(unique(angle)) < 3)
    angle2 = angle;
    return;
  end
  
  if ~is_circ
    dangle = diff(angle);
  else
    dangle = azimuth_diff(angle);
  end
  
  ind1 = find(dangle);
  ind2 = ind1 + 1;
  time0 = (time(ind1) + time(ind2)) ./ 2;
  angles = [angle(ind1) angle(ind2)];
   %figure, hold on, plot(time0, angle0, 'or'), plot(time, angle, '.k')  % DEBUG

  if ~is_circ
    angle0 = mean(angles, 2);
    angle2 = interp1_fastest (time0, angle0, time, method, 'extrap');
    angle2(angle2 < 0 | angle2 > 90) = NaN;
  else
    angle0 = azimuth_mean(angles, 2);
    angle2 = azimuth_interp (time0, angle0, time, method, 'extrap');  
    angle2 = azimuth_range_positive (angle2);
    angle2(angle2 < 0 | angle2 > 360) = NaN;
  end
  
  %figure, hold on, plot(time0, angle0, 'or'), plot(time, angle, '.k'), plot(time, angle2, '-b')  % DEBUG
end
