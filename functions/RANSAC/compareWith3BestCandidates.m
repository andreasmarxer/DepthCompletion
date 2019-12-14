function best = compareWith3BestCandidates(random2, inliers, inlier_array, param, dist, best)
% compare the current plane with the 3 best and return ordered struct 
% containing 3 best plane candidates
%
% input:    inliers:            number of lines inside the threshold
%           inlier_array:       boolean array of lines which are inliers 
%           dist:               sum of distance of all points to plane
%           best:               struct with plane candidates to compare
%           param:              parameter of the actual plane
%
% output:   best:           struct with first, second, third plane candidates


    if inliers > nnz(best.first.inlier_array)
        best.third = best.second;
        best.second = best.first;
        best.first.inlier_array = inlier_array;
        best.first.dist = dist;
        best.first.param = param;
        best.first.random2 = random2;

    elseif inlier_array == best.first.inlier_array
        if (dist <= best.first.dist)
            best.first.dist = dist;
            best.first.param = param;
            best.first.random2 = random2; 
        end

    elseif (inliers == nnz(best.first.inlier_array)) && (dist < best.first.dist)
        % can't be the same inlier array as first due to previous elseif
        if inlier_array ~= best.second.inlier_array
            best.third = best.second;
            best.second = best.first;
            best.first.inlier_array = inlier_array;
            best.first.dist = dist;
            best.first.param = param;
            best.first.random2 = random2;
        elseif inlier_array == best.second.inlier_array
            % delete best.second, best.third stays
            best.second = best.first;
            best.first.inlier_array = inlier_array;
            best.first.dist = dist;
            best.first.param = param;
            best.first.random2 = random2;
        end
    % second  
    elseif inliers > nnz(best.second.inlier_array)
        best.third = best.second;
        best.second.inlier_array = inlier_array;
        best.second.dist = dist;
        best.second.param = param;
        best.second.random2 = random2;

    elseif inlier_array == best.second.inlier_array
        if (dist <= best.second.dist)
            best.second.dist = dist;
            best.second.param = param;
            best.second.random2 = random2; 
        end

    elseif (inliers == nnz(best.second.inlier_array)) && (dist < best.second.dist)
        % can't be the same inlier array as first or second due to previous elseif
        % BUT it could be the same as the third, this can be neglected 
        % due to the fact that the third is anyway deleted!!!!
        best.third = best.second;
        best.second.inlier_array = inlier_array;
        best.second.dist = dist;
        best.second.param = param;
        best.second.random2 = random2;

    % third    
    elseif inliers > nnz(best.third.inlier_array)
        best.third.inlier_array = inlier_array;
        best.third.dist = dist;
        best.third.param = param;
        best.third.random2 = random2;

    elseif inlier_array == best.third.inlier_array
        if (dist <= best.third.dist)
            best.third.dist = dist;
            best.third.param = param;
            best.third.random2 = random2; 
        end

    elseif (inliers == nnz(best.third.inlier_array)) && (dist < best.third.dist)
            best.third.inlier_array = inlier_array;
            best.third.dist = dist;
            best.third.param = param;
            best.third.random2 = random2; 

    end



end