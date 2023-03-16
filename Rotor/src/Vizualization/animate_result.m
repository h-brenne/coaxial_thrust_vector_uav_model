clear;

load('../../Results/test.mat');

avg_timestep = t(end)/length(t);
time_scale = 0.01;

tic;
for i = 1:max(1,round(50*time_scale)):length(y)
    ax = show(rotor, y(i,1:4)', "Frames", "off", 'Preserveplot', false, 'FastUpdate', true);
    
    if t(i) - time_scale*toc > 0
        pause(max(0, t(i) - time_scale*toc))
    end
    disp(t(i));
end