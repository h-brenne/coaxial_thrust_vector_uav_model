clear;
experiment = 'test';
load(strcat('../Results/', experiment, '.mat'));
motor_speed = y(:,5);

% csv header format Time,ID,Bus,Mode,Velocity,Torque,ControlVelocity,
% VelocityCommand,AmplitudeCommand,PhaseCommand,Temperature,Voltage
data_size = size(t);

t = t/86400; % convert seconds to days
startDate = datetime('2023-05-12 11:40:50', 'InputFormat', 'yyyy-MM-dd HH:mm:ss'); % start date as datetime
timeVector = startDate + t; % add elapsed time to start date

% Create a table
T = table();

% Add your time series vectors as columns to this table
T.Time = datestr(timeVector, 'yyyy-mm-dd HH:MM:SS.FFF');
T.ID = ones(data_size);
T.Bus = ones(data_size);
T.Mode = ones(data_size);
T.Velocity = motor_speed;
T.Torque = motor_torque';
T.ControlVelocity = omega' + omega_tilde';
T.VelocityCommand = omega';
T.AmplitudeCommand = amplitude';
T.PhaseCommand = phase';
T.Temperature = 20*ones(data_size);
T.Voltage = 20*ones(data_size);


% ... add other columns in the same manner ...

% Write table to CSV file
writetable(T, strcat('../Results/', experiment, '.csv'));