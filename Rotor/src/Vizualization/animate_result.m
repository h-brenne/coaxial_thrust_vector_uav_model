clear;
load('../Results/step026A_80hz_leishman.mat');

% Control real time speed of animation
time_scale = 0.01;
fps = 60;  % frames per second
start_time = 0.2;  

% Find the index corresponding to start_time
[~, start_idx] = min(abs(t - start_time));

% Define the duration between each frame based on the desired fps
frame_duration = 1 / fps;

% Interpolate y to have uniform time steps
t_uniform = t(start_idx):frame_duration*time_scale: t(end);
y_uniform = interp1(t, y, t_uniform, 'linear', 'extrap');


% Start the animation
tic;
for i = 1:length(y_uniform)
    ax = show(rotor, y_uniform(i,1:4)', "Frames", "on", 'Preserveplot', false, 'FastUpdate', true);
    view(90,0)
    grid(ax, "off")
    if i == 1
        pause(3)
        tic;
    end
    
    elapsed_time = toc;
    if (i / fps) > elapsed_time
        pause((i / fps) - elapsed_time);
    end
    
    disp(t_uniform(i));
end

