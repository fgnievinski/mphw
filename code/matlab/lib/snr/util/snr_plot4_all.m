function snr_plot4_all (nmea, elev_lim)
  if (nargin < 2),  elev_lim = [];  end
  if isfieldempty(nmea, 'info', 'doy')
    nmea.info.doy = mydatedoy(nmea.info.epoch);
  end
  for i=1:numel(nmea.info.prn_unique)
    prn = nmea.info.prn_unique(i);
    
    figure
    maximize()
    el = snr_plot4 (nmea, prn, elev_lim);
    pause()
    delete(el)
    close(gcf())
  end
end
