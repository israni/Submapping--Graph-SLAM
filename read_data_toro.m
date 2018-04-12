clear all;
close all;

import gtsam.*

data_file = fopen('INTEL_P_toro.graph');
input_line = fgetl(data_file);

%% Create graph container and add factors to it
graph = NonlinearFactorGraph;
initialEstimate = Values;

noise = zeros(3);

numPoints = 1200;
x_all = [];
y_all=[];

while ischar(input_line)
    %disp(input_line)
    split_line = strsplit(input_line);
    
    %% ADD VERTICES TO GRAPH - INITIAL ESTIMATE%%
    if (strcmp(split_line{1},'VERTEX2'))
        vertex_id = str2num(split_line{2});
        if (vertex_id <= numPoints)
            vertex_location = ([str2num(split_line{3}),str2num(split_line{4}),str2num(split_line{5})]);
            x = vertex_location(1);
            y = vertex_location(2);
            x_all= [x_all,x];
            y_all= [y_all,y];
            theta = vertex_location(3);
            initialEstimate.insert(vertex_id, Pose2(x,y,theta));
            
        end
    end
    
    %% ADD EDGES TO GRAPH - ODOMETRY AND POSE CONTRAINTS %%
    if (strcmp(split_line{1},'EDGE2'))
        vertex_id_1 = str2num(split_line{2});
        vertex_id_2 = str2num(split_line{3});
        if (vertex_id_1 <= numPoints && vertex_id_2 <= numPoints)
            odometry = ([str2num(split_line{4}),str2num(split_line{5}),str2num(split_line{6})]);
            dx = odometry(1);
            dy = odometry(2);
            dtheta = odometry(3);

            information_matrix(1,1)= str2double(split_line{7});
            information_matrix(1,2)= str2double(split_line{8});
            information_matrix(2,2)= str2double(split_line{9});
            information_matrix(3,3)= str2double(split_line{10});
            information_matrix(1,3)= str2double(split_line{11});
            information_matrix(2,3)= str2double(split_line{12});
            information_matrix(2,1)= str2double(split_line{8});
            information_matrix(3,1)= str2double(split_line{11});
            information_matrix(3,2)= str2double(split_line{12});

            chol_information_matrix = sqrtm(information_matrix);
            if (noise(1,1) == 0)
                noise = chol_information_matrix;
            end

            model = noiseModel.Gaussian.SqrtInformation(chol_information_matrix);
            
%            model = noiseModel.Gaussian.SqrtInformation(eye(3));
            graph.add(BetweenFactorPose2(vertex_id_1, vertex_id_2, Pose2(dx, dy, dtheta), model));
        end
    end
    
    input_line = fgetl(data_file);
    
end

fclose(data_file);
%% Add prior
% priorNoise = noiseModel.Gaussian.SqrtInformation(noise);
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(PriorFactorPose2(0, Pose2(0, 0, 0), priorNoise)); % add directly to graph

graph.print(sprintf('\nFactor graph:\n'));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
%optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
%result = optimizer.optimizeSafely();
%result.print(sprintf('\nFinal result:\n'));

parameters = DoglegParams();

% Stop iterating once the change in error between steps is less than this value
parameters.setRelativeErrorTol(1e-5);
% Do not perform more than N iteration steps
parameters.setMaxIterations(1000);
% Create the optimizer ...
tic
optimizer = gtsam.DoglegOptimizer(graph, initialEstimate, parameters);
% and optimize
result = optimizer.optimizeSafely();
time = toc
result.print(sprintf('\nFinal result:\n'));

%% Plot Covariance Ellipses
plot (x_all,y_all,'o')
figure,
hold on  
% plot([result.at(5).x;result.at(2).x],[result.at(5).y;result.at(2).y],'r-');
% marginals = Marginals(graph, result);

% plot2DTrajectory(result, [], marginals);
plot2DTrajectory(result);
% for i=1:5,marginals.marginalCovariance(i),end
axis equal
axis tight
view(2)

