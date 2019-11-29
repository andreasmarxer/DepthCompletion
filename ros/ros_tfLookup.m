%% testing tf messages
tf_tree = rostf; % is an ongoing variable
% without sim time this is the actual time
% with sim time this has the simulated time

time_part_s = tf_tree.LastUpdateTime.Sec; 
time_part_ns = tf_tree.LastUpdateTime.Nsec;

time_s = time_part_s + time_part_ns/10^9; 
time_ns = time_s*10^9;

disp('Actual tf time')
datetime(time_ns,'ConvertFrom','epochtime','TicksPerSecond',1e9,'Format','dd-MMM-yyyy HH:mm:ss.SSSSSSSSS')


tf = getTransform(tf_tree, 'imu', 'mission', time_s+0.25, "Timeout", 5);
% takes into accound header time stamp ?!?

%% convert bag time to real time with date for control purpose

time_part_s = tf.Header.Stamp.Sec;
time_part_ns = tf.Header.Stamp.Nsec;

time_s = time_part_s + time_part_ns/10^9;
time_ns = time_s*10^9;

disp('Found tf - header time')
datetime(time_ns,'ConvertFrom','epochtime','TicksPerSecond',1e9,'Format','dd-MMM-yyyy HH:mm:ss.SSSSSSSSS')