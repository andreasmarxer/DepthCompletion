function distance = dist2plane(P, param)
% calculte shortest distance from point to plane
% input:    P = [x1, y1, z1; x2, y2, z2; x3, y3, z3; ... ; xN, yN, zN]
% output:   distance

a = param(1);
b = param(2);
c = -1;
d = param(3);

N = size(P,1);
distance = zeros(N,1);

    for i = 1:1:N
        distance(i) = abs(a*P(i,1)+b*P(i,2)+c*P(i,3)+d)/sqrt(a^2+b^2+c^2);
        %hold on
        %scatter3(P(i,1), P(i,2), P(i,3), 'g*')
    end

end
