clear all;
close all;

import gtsam.*

%% Set these variables
num_submaps = 4;
num_points_total = 900; % total # of points in dataset
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
data_file = fopen('input_INTEL_g2o.g2o');
input_line = fgetl(data_file);

while ischar(input_line)
    split_line = strsplit(input_line);
    
    %% ADD EDGES TO GRAPH - ODOMETRY AND POSE CONTRAINTS %%
    if (strcmp(split_line{1},'EDGE_SE2'))
        vertex_id_1 = str2double(split_line{2});
        vertex_id_2 = str2double(split_line{3});
        if (vertex_id_1 < num_points_total && vertex_id_2 < num_points_total)
            odometry = ([str2double(split_line{4}),str2double(split_line{5}),str2double(split_line{6})]);
            dx = odometry(1);
            dy = odometry(2);
            dtheta = odometry(3);

            information_matrix(1,1)= str2double(split_line{7});
            information_matrix(1,2)= str2double(split_line{8});
            information_matrix(1,3)= str2double(split_line{9});
            information_matrix(2,1)= str2double(split_line{8});
            information_matrix(2,2)= str2double(split_line{10});
            information_matrix(2,3)= str2double(split_line{11});
            information_matrix(3,1)= str2double(split_line{9});
            information_matrix(3,2)= str2double(split_line{11});
            information_matrix(3,3)= str2double(split_line{12});

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
    if (strcmp(split_line{1},'VERTEX_SE2'))
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

% optimize and plot submaps
figure(1)
for i = 1:num_submaps
    optimizer = gtsam.DoglegOptimizer(graph(i), initial_estimate(i), parameters);
    result = optimizer.optimizeSafely();
    
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
    
    subplot(ceil(num_submaps/2),ceil(num_submaps/2),i);
    plot2DTrajectory(result);
    title_str = sprintf("Submap %i",i);
    title(title_str);
    axis equal
    axis tight
    view(2)
end

% optimize and plot separator
% separator_graph.print(sprintf('\nFactor graph:\n'));
parameters = LevenbergMarquardtParams();
optimizer = gtsam.LevenbergMarquardtOptimizer(separator_graph, separator_estimate, parameters);
result = optimizer.optimizeSafely();

keys = KeyVector(result.keys());
all_separator_results = zeros(4,keys.size());
results_idx = 1;
for j = 0:keys.size()-1
    key = keys.at(j);
    all_separator_results(1,results_idx) = key;
    all_separator_results(2,results_idx) = result.at(key).x();
    all_separator_results(3,results_idx) = result.at(key).y();
    all_separator_results(4,results_idx) = result.at(key).theta();
    results_idx = results_idx + 1;
end
result.print(sprintf('\nFinal result:\n'));
% optimizer.error()

figure(2);
plot2DTrajectory(result);
axis equal
axis tight
view(2)
