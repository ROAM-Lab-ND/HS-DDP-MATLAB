function [p2cloud_dist,I] = pointToCloud(p, cloud)
dist = zeros(1, size(cloud,2));
for i = 1:size(cloud,2)
    dist(i) = norm(p - cloud(:,i));
end
[p2cloud_dist,I] = min(dist);
end