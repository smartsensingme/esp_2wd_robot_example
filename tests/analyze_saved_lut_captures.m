addpath('components/esp_angle_lut/tools/octave');
for side={'left','right'}
 files=dir(['tests/cal_' side{1} '_*.mat']); [~,latest]=max([files.datenum]); d=load(fullfile('tests',files(latest).name)); c=d.capture; t=c.time_s(:); a=c.raw_angle_deg(:); duty=c.duty_percent(:); lut=d.calibration.correction_counts; corrected=esp_angle_lut_apply(a,lut,16384); edges=[1;find(diff(duty)!=0)+1]; ends=[edges(2:end)-1;numel(t)]; profiles=[];
 fprintf('\n%s CRC %08X\n',side{1},d.calibration.payload_crc32);
 for k=1:numel(edges)
 idx=find((1:numel(t))'>=edges(k) & (1:numel(t))'<=ends(k) & t>=t(edges(k))+1.5 & t<=t(ends(k))-.5);
 r=diff(unwrap(a(idx)*pi/180))./diff(t(idx))*60/(2*pi); q=diff(unwrap(corrected(idx)*pi/180))./diff(t(idx))*60/(2*pi);
 local=esp_angle_lut_calculate(t,a,{idx},64,16384,1024); profiles(:,k)=local.correction_deg;
 fprintf('stage %d duty %.0f mean %.3f rpm speed RMS %.4f -> %.4f reduction %.1f%% turns %d local max %.3f deg\n',k,duty(edges(k)),mean(r),sqrt(mean(detrend(r).^2)),sqrt(mean(detrend(q).^2)),100*(1-sqrt(mean(detrend(q).^2))/sqrt(mean(detrend(r).^2))),local.revolution_count,max(abs(local.correction_deg)));
 end
 disp('Per-stage correction profile correlations (64 bins, diagnostic only):'); disp(corrcoef(profiles));
 fprintf('stage1/stage3 profile difference RMS %.4f deg\n',sqrt(mean((profiles(:,1)-profiles(:,3)).^2)));
end
