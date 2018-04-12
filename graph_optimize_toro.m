clear all;
close all;

import gtsam.*

%% Set these variables
num_submaps = 10;
%num_points_total = 1200; % total # of points in dataset        INTEL
num_points_total = 3500; % total # of points in dataset          M3500
%num_points_total = 1044; % total # of points in dataset          CSAIL MIT
%num_points_total = 10000; % total # of points in dataset         M10000

num_points_submap = ceil(num_points_total / num_submaps);
separator_nodes = zeros(1,num_points_total);

%% Create graph containers
graph(num_submaps) = NonlinearFactorGraph;
initial_estimate(num_submaps) = Values;
for i = 1:num_submaps
    graph(i) = NonlinearFactorGraph;
    initial_estimate(i) = Values;
    separator_nodes(num_points_submap*(i-1)+1) = 1;
end
separator_graph = NonlinearFactorGraph;
separator_estimate = Values;
base_pose = -1 * ones(3,num_submaps);
base_vertex = -1 * ones(1,num_submaps);
% noise = zeros(3);

%% Parse data and add factors
%data_file = fopen('INTEL_P_toro.graph');                      % INTEL
data_file = fopen ('M3500_P_toro.graph');                     % M3500
%data_file = fopen ('CSAIL_P_toro.graph');                     % CSAIL MIT
%data_file= fopen('M10000_P_toro.graph');                      % M10000

input_line = fgetl(data_file);

while ischar(input_line)
    split_line = strsplit(input_line);
    
    %% ADD EDGES TO GRAPH - ODOMETRY AND POSE CONTRAINTS %%
    if (strcmp(split_line{1},'EDGE2'))
        vertex_id_1 = str2double(split_line{2});
        vertex_id_2 = str2double(split_line{3});
        if (vertex_id_1 < num_points_total && vertex_id_2 < num_points_total)
            odometry = ([str2double(split_line{4}),str2double(split_line{5}),str2double(split_line{6})]);
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
            model = noiseModel.Gaussian.SqrtInformation(chol_information_matrix);
            
            submap_idx_1 = floor(vertex_id_1 / num_points_submap) + 1;
            submap_idx_2 = floor(vertex_id_2 / num_points_submap) + 1;
            if (submap_idx_1 == submap_idx_2) % non-boundary node
                graph(submap_idx_1).add(BetweenFactorPose2(vertex_id_1, vertex_id_2, Pose2(dx, dy, dtheta), model));
            else % boundary node
                % keep track of which nodes are used for separator optimization
                separator_nodes(vertex_id_1+1) = 1;
                separator_nodes(vertex_id_2+1) = 1;
                separator_graph.add(BetweenFactorPose2(vertex_id_1, vertex_id_2, Pose2(dx, dy, dtheta), model));
            end          
        end
    end
    
    %% ADD VERTICES TO GRAPH - INITIAL ESTIMATE %%
    if (strcmp(split_line{1},'VERTEX2'))
        vertex_id = str2double(split_line{2});
        if (vertex_id < num_points_total)
            vertex_location = ([str2double(split_line{3}),str2double(split_line{4}),str2double(split_line{5})]);
            x = vertex_location(1);
            y = vertex_location(2);
            theta = vertex_location(3);
            
            submap_idx = floor(vertex_id / num_points_submap) + 1;
            initial_estimate(submap_idx).insert(vertex_id, Pose2(x,y,theta));
            
            % add base nodes to use as priors
            if (base_pose(1,submap_idx) == -1)
                base_pose(1,submap_idx) = x;
                base_pose(2,submap_idx) = y;
                base_pose(3,submap_idx) = theta;
                base_vertex(submap_idx) = vertex_id;
%                 separator_estimate.insert(vertex_id, Pose2(x,y,theta));
            end
            
            % add inter-measurement nodes but don't double count base nodes
%             if (separator_nodes(vertex_id+1) == 1 && base_vertex(submap_idx) ~= vertex_id)
            if (separator_nodes(vertex_id+1) == 1)
                separator_estimate.insert(vertex_id, Pose2(x,y,theta));
            end
        end
    end
    
    input_line = fgetl(data_file);
    
end

fclose(data_file);

%% Add priors
prior_noise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
for i = 1:num_submaps
    graph(i).add(PriorFactorPose2(base_vertex(i), Pose2(base_pose(1,i), base_pose(2,i), base_pose(3,i)), prior_noise));
end
separator_graph.add(PriorFactorPose2(base_vertex(1), Pose2(base_pose(1,1), base_pose(2,1), base_pose(3,1)), prior_noise));

%% Optimize and plot results

parameters = DoglegParams();
parameters.setRelativeErrorTol(1e-5);
parameters.setMaxIterations(1000);

all_submap_results = zeros(4,num_points_total);
results_idx = 1;
time=0;
% optimize and plot submaps
figure(1)
for i = 1:num_submaps
    tic
    optimizer = gtsam.DoglegOptimizer(graph(i), initial_estimate(i), parameters);
    result = optimizer.optimizeSafely();
    variable= toc;
    time = time + variable;
    keys = KeyVector(result.keys());
    
    % store submap results in matrix
    for j = 0:keys.size()-1
        key = keys.at(j);
        all_submap_results(1,results_idx) = key;
        all_submap_results(2,results_idx) = result.at(key).x();
        all_submap_results(3,results_idx) = result.at(key).y();
        all_submap_results(4,results_idx) = result.at(key).theta();
        results_idx = results_idx + 1;
    end
    if (num_submaps <= 100)
        if (num_submaps == 2)
            subplot(ceil(num_submaps/2),2,i);
        else
            subplot(ceil(sqrt(num_submaps)),ceil(sqrt(num_submaps)),i);
        end
    plot2DTrajectory(result);
    title_str = sprintf("Submap %i",i);
    title(title_str);
    axis equal
    axis tight
    view(2)
    end

end

% % optimize and plot separator
% % separator_graph.print(sprintf('\nFactor graph:\n'));
% parameters = LevenbergMarquardtParams();
% optimizer = gtsam.LevenbergMarquardtOptimizer(separator_graph, separator_estimate, parameters);
% result = optimizer.optimizeSafely();
% 
% keys = KeyVector(result.keys());
% all_separator_results = zeros(4,keys.size());
% results_idx = 1;
% for j = 0:keys.size()-1
%     key = keys.at(j);
%     all_separator_results(1,results_idx) = key;
%     all_separator_results(2,results_idx) = result.at(key).x();
%     all_separator_results(3,results_idx) = result.at(key).y();
%     all_separator_results(4,results_idx) = result.at(key).theta();
%     results_idx = results_idx + 1;
% end
% result.print(sprintf('\nFinal result:\n'));
% % optimizer.error()
% 
% figure(2);
% plot2DTrajectory(result);
% axis equal
% axis tight
% view(2)




%% Parse data and add factors
%data_file = fopen('INTEL_P_toro.graph');                       % INTEL
data_file = fopen ('M3500_P_toro.graph');                      % M3500
%data_file = fopen ('CSAIL_P_toro.graph');                       % CSAIL MIT
%data_file= fopen('M10000_P_toro.graph');                      % M10000
input_line = fgetl(data_file);

%% Create NEW graph container and add factors to it
graph_new = NonlinearFactorGraph;
initialEstimate_new = Values;

noise = zeros(3);

%% Add prior
% priorNoise = noise.Gaussian.SqrtInformation(noise);
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph_new.add(PriorFactorPose2(0, Pose2(0, 0, 0), priorNoise)); % add directly to graph


for i=1:(size(all_submap_results,2))
    %% ADD VERTICES TO GRAPH - INITIAL ESTIMATE%%
            x = all_submap_results(2,i);
            y = all_submap_results(3,i);
            theta = all_submap_results(4,i);
            initialEstimate_new.insert(i-1, Pose2(x,y,theta));
end

while ischar(input_line)
    split_line = strsplit(input_line);
    
    %% ADD EDGES TO GRAPH - ODOMETRY AND POSE CONTRAINTS %%
    if (strcmp(split_line{1},'EDGE2'))
        vertex_id_1 = str2double(split_line{2});
        vertex_id_2 = str2double(split_line{3});
        if (vertex_id_1 < num_points_total && vertex_id_2 < num_points_total)
            odometry = ([str2double(split_line{4}),str2double(split_line{5}),str2double(split_line{6})]);
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
            model = noiseModel.Gaussian.SqrtInformation(chol_information_matrix);
            
            submap_idx_1 = floor(vertex_id_1 / num_points_submap) + 1;
            submap_idx_2 = floor(vertex_id_2 / num_points_submap) + 1;
            if (submap_idx_1 == submap_idx_2) % non-boundary node
                model = noiseModel.Gaussian.SqrtInformation([1 0 0; 0 1 0; 0 0 1] );
                %dx = all_submap_results(2,vertex_id_2+1)-all_submap_results(2,vertex_id_1+1);
                %dy = all_submap_results(3,vertex_id_2+1)-all_submap_results(2,vertex_id_1+1);
                %dtheta = all_submap_results(4,vertex_id_2+1)-all_submap_results(2,vertex_id_1+1);
                graph_new.add(BetweenFactorPose2(vertex_id_1, vertex_id_2, Pose2(dx, dy, minimizedAngle(dtheta)), model));
            else % boundary node
                graph_new.add(BetweenFactorPose2(vertex_id_1, vertex_id_2, Pose2(dx, dy, minimizedAngle(dtheta)), model));
            end                  
                
        end
    end
    
    input_line = fgetl(data_file);
    
end

fclose(data_file);


%graph_new.print(sprintf('\nFactor graph:\n'));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
tic
optimizer = LevenbergMarquardtOptimizer(graph_new, initialEstimate_new);
result = optimizer.optimizeSafely();
time = time+toc
%result.print(sprintf('\nFinal result:\n'));

%parameters = DoglegParams();
%tic
% Stop iterating once the change in error between steps is less than this value
%parameters.setRelativeErrorTol(1e-10);
% Do not perform more than N iteration steps
%parameters.setMaxIterations(1000000);
% Create the optimizer ...
%optimizer = gtsam.DoglegOptimizer(graph_new, initialEstimate_new, parameters);
% and optimize
%result = optimizer.optimizeSafely();
%result.print(sprintf('\nFinal result:\n'));
%time=time+toc
%% Plot Covariance Ellipses
%cla;
%hold on  
% plot([result.at(5).x;result.at(2).x],[result.at(5).y;result.at(2).y],'r-');
% marginals = Marginals(graph, result);
figure,
% plot2DTrajectory(result, [], marginals);
plot2DTrajectory(result);
% for i=1:5,marginals.marginalCovariance(i),end
axis equal
axis tight
view(2)


