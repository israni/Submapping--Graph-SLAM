close all;
clear all;
time= [2.0421,1.8755,1.8542,1.5676,1.3101,1.0576,1.2198];
submaps = [2,4,6,10,20,100,500];
plot(submaps,time,'-o')
xlim([0 165]);
xlabel('Number of submaps');
ylabel('Time taken for optimization (s)');
title('Optimization time vs Number of Submaps');
gtext('Manhattan World 3500 Dataset')


