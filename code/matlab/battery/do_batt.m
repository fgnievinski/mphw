%% load data
filename = '190909.BAT';
fid = fopen(filename, 'rt');
data = textscan(fid, '%2f %2f %2f %2f %2f %2f\t%f');
fclose(fid);
data = cell2mat(data);
time = data(:,4) + data(:,5)./60 + data(:,6)./3600;
volt = data(:,end);

%%
figure
  plot(time, volt, '.-k')
  grid on
  xlabel('Time (h)')
  ylabel('Voltage (V)')
  title(sprintf('Min.: %g V', min(volt)))
