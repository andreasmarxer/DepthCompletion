clear all, clc, close all;

P1 = [0,0,1];
P2 = [5,0,1];
P3 = [2,2,1];

P = [P1; P2; P3];

param = pts2plane(P);


%%
x = 1.5;
y = 1.5;

figure()
scatter3(P(:,1), P(:,2), P(:,3), 'r*')
hold on
patch([P1(1) P2(1) P3(1)], [P1(2) P2(2) P3(2)], [P1(3) P2(3) P3(3)], 'r', 'FaceAlpha',0.5)
grid on
xlabel('X')
ylabel('Y')

%%
PP = [10,0,0.7];
dist2plane(PP, param)

%% functions

% Assuming that the 3d data P is N-by-3 numeric array,  P = [x, y, z]
% Output a, b and d for fitting plane,                  z = ax + by + d

function param = pts2plane(P)
% input:    P = [x1, y1, z1; x2, y2, z2; x3, y3, z3]
% output:   param = [a; b; d]

% make linear regression for getting plane
param = [P(:,1), P(:,2), ones(size(P,1),1)] \ P(:,3);

end

function distance = dist2plane(P, param)
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
    hold on
    scatter3(P(i,1), P(i,2), P(i,3), 'g*')
end

end