% Search for correspondances in message time between rgb and depth
% Correspond if time difference is smaller than tolerance
% Order the messages indexes

tolerance = 1e-3;

if size(depth_msg_time_vec,1) < size(rgb_msg_time_vec,1)
    length = size(depth_msg_time_vec,1);
    short_vec = depth_msg_time_vec;
    long_vec = rgb_msg_time_vec;
elseif size(depth_msg_time_vec,1) > size(rgb_msg_time_vec,1)
    length = size(rgb_msg_time_vec,1);
    short_vec = rgb_msg_time_vec;
    long_vec = depth_msg_time_vec;
else
    length = size(rgb_msg_time_vec,1);
    short_vec = rgb_msg_time_vec;
    long_vec = depth_msg_time_vec;
end

idx_long = zeros(length,1);
idx_short = [1:length]';

for k_short=1:1:length
    dist = abs(short_vec(k_short)-long_vec);
    idx = find(dist < tolerance);
    
    if ~isempty(idx)
    idx_long(k_short) = idx;
    end

end

%%
figure(1)
scatter(1:623,idx_long, 2,'filled')
xlabel('depth frames')
ylabel('rgb frames')
title('Correspandance between rgb and depth camera message time stamps')



