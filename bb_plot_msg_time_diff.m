% Plot the message_time_vec generated in bb_script
% CARE: This are unordered and do not correspond due to different
%       exposures!


figure(1)
plot(diff(rgb_msg_time_vec))
hold on
plot(diff(depth_msg_time_vec))
xlabel('Frames')
ylabel('Difference between timestamps of sequential frames [s]')
legend('/medium_resolution/rgb_undistorted/image','/medium_resolution/depth_registered/image', 'Interpreter', 'none')
title('Time synchronisation of measurements times')

%%
offset = rgb_msg_time_vec(1);
figure(2)
plot(1:623, rgb_msg_time_vec(1:623)-offset)
hold on
plot(1:623, depth_msg_time_vec-offset)
xlabel('Frames')
ylabel('Timestamps of sequential frames minus offset [s]')
legend('/medium_resolution/rgb_undistorted/image','/medium_resolution/depth_registered/image', 'Interpreter', 'none')
title('Time synchronisation of measurements times')


%%
