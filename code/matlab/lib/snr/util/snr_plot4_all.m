function snr_plot4_all (nmea, elev_lim)
  if (nargin < 2),  elev_lim = [];  end
  if isfieldempty(nmea, 'info', 'doy')
    nmea.info.doy = mydatedoy(nmea.info.epoch);
  end
  for i=1:numel(nmea.info.prn_unique)
    prn = nmea.info.prn_unique(i);
    
    ah=figure();
    maximize(ah)
    [el, status] = snr_plot4 (nmea, prn, elev_lim);
    if status,  disp('Paused; hit Enter to continue or Ctrl+C to stop.');  pause();  end
    if ishghandle(ah)
      delete(el)
      close(ah)
    end
  end
end
