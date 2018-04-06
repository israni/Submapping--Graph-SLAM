clear all;
close all;

import gtsam.*

data_file = fopen('input_INTEL_g2o.g2o');
input_line = fgetl(data_file);

num_submaps = 4;
num_points_total = 810; % total # of points in dataset
num_points_submap = ceil(num_points_total / num_submaps);

%% Create graph containers and add factors to it
graph(num_submaps) = NonlinearFactorGraph;
initial_estimate(num_submaps) = Values;
prior_pose = -1 * ones(1,4);

while ischar(input_line)
    %disp(input_line)
    split_line = strsplit(input_line);
    
    %% ADD VERTICES TO GRAPH - INITIAL ESTIMATE %%
    if (strcmp(split_line{1},'VERTEX_SE2'))
        vertex_id = str2num(split_line{2});
        if (vertex_id <= num_points_total)
            vertex_location = ([str2num(split_line{3}),str2num(split_line{4}),str2num(split_line{5})]);
            x = vertex_location(1);
            y = vertex_location(2);
            theta = vertex_location(3);
            
            submap_idx = floor(vertex_id / num_points_submap) + 1;
            initial_estimate(submap_idx).insert(vertex_id, Pose2(x,y,theta));
            
            if (prior_pose(submap_idx) == -1)
                prior_pose(submap_idx) = vertex_id;
            end
        end
    end
    
    %% ADD EDGES TO GRAPH - ODOMETRY AND POSE CONTRAINTS %%
    if (strcmp(split_line{1},'EDGE_SE2'))
        vertex_id_1 = str2num(split_line{2});
        vertex_id_2 = str2num(split_line{3});
        if (vertex_id_1 <= num_points_total && vertex_id_2 <= num_points_total)
            odometry = ([str2num(split_line{4}),str2num(split_line{5}),str2num(split_line{6})]);
            dx = odometry(1);
            dy = odometry(2);
            dtheta = odometry(3);

            information_matrix(1,1)= str2num(split_line{7});
            information_matrix(1,2)= str2num(split_line{8});
            information_matrix(1,3)= str2num(split_line{9});
            information_matrix(2,1)= str2num(split_line{8});
            information_matrix(2,2)= str2num(split_line{10});
            information_matrix(2,3)= str2num(split_line{11});
            information_matrix(3,1)= str2num(split_line{9});
            information_matrix(3,2)= str2num(split_line{11});
            information_matrix(3,3)= str2num(split_line{12});

            chol_information_matrix = sqrtm(information_matrix);
            model = noiseModel.Gaussian.SqrtInformation(chol_information_matrix);
%             if (noise(1,1) == 0)
%                 noise = chol_information_matrix;
%             end
            
            submap_idx_1 = floor(vertex_id_1 / num_points_submap) + 1;
            submap_idx_2 = floor(vertex_id_2 / num_points_submap) + 1;
            if (submap_idx_1 == submap_idx_2) % non-boundary node
                graph(submap_idx_1).add(BetweenFactorPose2(...
                    vertex_id_1, vertex_id_2, Pose2(dx, dy, dtheta), model));
            else % boundary node
                % do stuff
            end          
        end
    end
    
    input_line = fgetl(data_file);
    
end

fclose(data_file);

