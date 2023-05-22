clear;

load('../Results/test.mat');

avg_timestep = t(end)/length(t);
% Control real time speed of animation
time_scale = 0.1;

tic;
for i = 1:max(1,round(20*time_scale)):length(y)
    ax = show(rotor, y(i,1:4)', "Frames", "on", 'Preserveplot', false, 'FastUpdate', true);
    
    if t(i) - time_scale*toc > 0
        pause(max(0, t(i) - time_scale*toc))
    end
    disp(t(i));
end