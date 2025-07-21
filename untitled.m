data = readtable('joint_position_log.txt');
T = data{:,1}; 
T2 = data{:,2};
plot (T,T2,'x');