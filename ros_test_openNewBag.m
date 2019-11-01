clear; clc; close all;
%%=========================================================================
filename = '/home/andreas/2019-10-31-17-33-18.bag';
bag = rosbag(filename);
% bagInfo = rosbag('info',filename)