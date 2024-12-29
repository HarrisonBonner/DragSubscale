clc, clear, close all
% Load data from Excel file, skipping the first row
data = readtable('DragDatalog.xlsx', 'ReadVariableNames', true); % Replace with your actual file name

%Define sample number of moving means
movMeanSample = 10;

% Column 1: TIME
time = data{:, 1};
if iscell(time)
    time = cellfun(@str2double, time);
end
time = time(~isnan(time)); % Remove NaN values

% Column 2: ALTITUDE
altitude = data{:, 2};
if iscell(altitude)
    altitude = cellfun(@str2double, altitude);
end
altitude = altitude(~isnan(altitude)); % Remove NaN values

% Column 3: ORIENTATION_X
orientation_x = data{:, 3};
if iscell(orientation_x)
    orientation_x = cellfun(@str2double, orientation_x);
end
orientation_x = orientation_x(~isnan(orientation_x)); % Remove NaN values

% Column 4: ORIENTATION_Y
orientation_y = data{:, 4};
if iscell(orientation_y)
    orientation_y = cellfun(@str2double, orientation_y);
end
orientation_y = orientation_y(~isnan(orientation_y)); % Remove NaN values

% Column 5: ORIENTATION_Z
orientation_z = data{:, 5};
if iscell(orientation_z)
    orientation_z = cellfun(@str2double, orientation_z);
end
orientation_z = orientation_z(~isnan(orientation_z)); % Remove NaN values

% Column 6: ANGVELOCITY_X
angvelocity_x = data{:, 6};
if iscell(angvelocity_x)
    angvelocity_x = cellfun(@str2double, angvelocity_x);
end
angvelocity_x = angvelocity_x(~isnan(angvelocity_x)); % Remove NaN values

% Column 7: ANGVELOCITY_Y
angvelocity_y = data{:, 7};
if iscell(angvelocity_y)
    angvelocity_y = cellfun(@str2double, angvelocity_y);
end
angvelocity_y = angvelocity_y(~isnan(angvelocity_y)); % Remove NaN values

% Column 8: ANGVELOCITY_Z
angvelocity_z = data{:, 8};
if iscell(angvelocity_z)
    angvelocity_z = cellfun(@str2double, angvelocity_z);
end
angvelocity_z = angvelocity_z(~isnan(angvelocity_z)); % Remove NaN values

% Column 9: LINEARACCEL_X
linearaccel_x = data{:, 9};
if iscell(linearaccel_x)
    linearaccel_x = cellfun(@str2double, linearaccel_x);
end
linearaccel_x = linearaccel_x(~isnan(linearaccel_x)); % Remove NaN values
linearaccel_xMovMean = movmean(linearaccel_x, movMeanSample);

% Column 10: LINEARACCEL_Y
linearaccel_y = data{:, 10};
if iscell(linearaccel_y)
    linearaccel_y = cellfun(@str2double, linearaccel_y);
end
linearaccel_y = linearaccel_y(~isnan(linearaccel_y)); % Remove NaN values
linearaccel_yMovMean = movmean(linearaccel_y, movMeanSample);

% Column 11: LINEARACCEL_Z
linearaccel_z = data{:, 11};
if iscell(linearaccel_z)
    linearaccel_z = cellfun(@str2double, linearaccel_z);
end
linearaccel_z = linearaccel_z(~isnan(linearaccel_z)); % Remove NaN values
linearaccel_zMovMean = movmean(linearaccel_z, movMeanSample);

% Column 12: MAGNETOMETER_X
magnetometer_x = data{:, 12};
if iscell(magnetometer_x)
    magnetometer_x = cellfun(@str2double, magnetometer_x);
end
magnetometer_x = magnetometer_x(~isnan(magnetometer_x)); % Remove NaN values

% Column 13: MAGNETOMETER_Y
magnetometer_y = data{:, 13};
if iscell(magnetometer_y)
    magnetometer_y = cellfun(@str2double, magnetometer_y);
end
magnetometer_y = magnetometer_y(~isnan(magnetometer_y)); % Remove NaN values

% Column 14: MAGNETOMETER_Z
magnetometer_z = data{:, 14};
if iscell(magnetometer_z)
    magnetometer_z = cellfun(@str2double, magnetometer_z);
end
magnetometer_z = magnetometer_z(~isnan(magnetometer_z)); % Remove NaN values

% Column 15: ACCELEROMETER_X
accelerometer_x = data{:, 15};
if iscell(accelerometer_x)
    accelerometer_x = cellfun(@str2double, accelerometer_x);
end
accelerometer_x = accelerometer_x(~isnan(accelerometer_x)); % Remove NaN values
accelerometer_xMovMean = movmean(accelerometer_x, movMeanSample);


% Column 16: ACCELEROMETER_Y
accelerometer_y = data{:, 16};
if iscell(accelerometer_y)
    accelerometer_y = cellfun(@str2double, accelerometer_y);
end
accelerometer_y = accelerometer_y(~isnan(accelerometer_y)); % Remove NaN values
accelerometer_yMovMean = movmean(accelerometer_y, movMeanSample);

% Column 17: ACCELEROMETER_Z
accelerometer_z = data{:, 17};
if iscell(accelerometer_z)
    accelerometer_z = cellfun(@str2double, accelerometer_z);
end
accelerometer_z = accelerometer_z(~isnan(accelerometer_z)); % Remove NaN values
accelerometer_zMovMean = movmean(accelerometer_z, movMeanSample);

% Column 18: GRAVITY_X
gravity_x = data{:, 18};
if iscell(gravity_x)
    gravity_x = cellfun(@str2double, gravity_x);
end
gravity_x = gravity_x(~isnan(gravity_x)); % Remove NaN values

% Column 19: GRAVITY_Y
gravity_y = data{:, 19};
if iscell(gravity_y)
    gravity_y = cellfun(@str2double, gravity_y);
end
gravity_y = gravity_y(~isnan(gravity_y)); % Remove NaN values

% Column 20: GRAVITY_Z
gravity_z = data{:, 20};
if iscell(gravity_z)
    gravity_z = cellfun(@str2double, gravity_z);
end
gravity_z = gravity_z(~isnan(gravity_z)); % Remove NaN values

% Check for and remove NaN values if any conversions fail
time = time(~isnan(time));
altitude = altitude(~isnan(altitude));

% Adjust time column (subtract the first value and divide by 1000)
time_shifted = (time - time(1)) / 1000;

% Plot the data
% Plot: Altitude vs Time (single plot)
figure;
plot(time_shifted, altitude);
xlabel('Time (seconds)');
ylabel('Altitude (m)');
title('Altitude vs Time');
grid on;

% Plot: Orientation (X, Y, Z) vs Time
figure;
subplot(3, 1, 1);
plot(time_shifted, orientation_x);
xlabel('Time (seconds)');
ylabel('Orientation X (degrees)');
title('Orientation X vs Time');
grid on;

subplot(3, 1, 2);
plot(time_shifted, orientation_y);
xlabel('Time (seconds)');
ylabel('Orientation Y (degrees)');
title('Orientation Y vs Time');
grid on;

subplot(3, 1, 3);
plot(time_shifted, orientation_z);
xlabel('Time (seconds)');
ylabel('Orientation Z (degrees)');
title('Orientation Z vs Time');
grid on;

% Plot: Angular Velocity (X, Y, Z) vs Time
figure;
subplot(3, 1, 1);
plot(time_shifted, angvelocity_x);
xlabel('Time (seconds)');
ylabel('Angular Velocity X (rad/s)');
title('Angular Velocity X vs Time');
grid on;

subplot(3, 1, 2);
plot(time_shifted, angvelocity_y);
xlabel('Time (seconds)');
ylabel('Angular Velocity Y (rad/s)');
title('Angular Velocity Y vs Time');
grid on;

subplot(3, 1, 3);
plot(time_shifted, angvelocity_z);
xlabel('Time (seconds)');
ylabel('Angular Velocity Z (rad/s)');
title('Angular Velocity Z vs Time');
grid on;

% Plot: Linear Acceleration (X, Y, Z) vs Time
figure;
subplot(3, 1, 1);
plot(time_shifted, linearaccel_x, time_shifted, linearaccel_xMovMean);
xlabel('Time (seconds)');
ylabel('Linear Accel X (m/s^2)');
title('Linear Accel X vs Time');
grid on;

subplot(3, 1, 2);
plot(time_shifted, linearaccel_y, time_shifted, linearaccel_yMovMean);
xlabel('Time (seconds)');
ylabel('Linear Accel Y (m/s^2)');
title('Linear Accel Y vs Time');
grid on;

subplot(3, 1, 3);
plot(time_shifted, linearaccel_z, time_shifted, linearaccel_zMovMean);
xlabel('Time (seconds)');
ylabel('Linear Accel Z (m/s^2)');
title('Linear Accel Z vs Time');
grid on;

% Plot: Magnetometer (X, Y, Z) vs Time
figure;
subplot(3, 1, 1);
plot(time_shifted, magnetometer_x);
xlabel('Time (seconds)');
ylabel('Magnetometer X (µT)');
title('Magnetometer X vs Time');
grid on;

subplot(3, 1, 2);
plot(time_shifted, magnetometer_y);
xlabel('Time (seconds)');
ylabel('Magnetometer Y (µT)');
title('Magnetometer Y vs Time');
grid on;

subplot(3, 1, 3);
plot(time_shifted, magnetometer_z);
xlabel('Time (seconds)');
ylabel('Magnetometer Z (µT)');
title('Magnetometer Z vs Time');
grid on;

% Plot: Accelerometer (X, Y, Z) vs Time
figure;
subplot(3, 1, 1);
plot(time_shifted, accelerometer_x, time_shifted, accelerometer_xMovMean);
xlabel('Time (seconds)');
ylabel('Accelerometer X (m/s^2)');
title('Accelerometer X vs Time');
grid on;

subplot(3, 1, 2);
plot(time_shifted, accelerometer_y, time_shifted, accelerometer_yMovMean);
xlabel('Time (seconds)');
ylabel('Accelerometer Y (m/s^2)');
title('Accelerometer Y vs Time');
grid on;

subplot(3, 1, 3);
plot(time_shifted, accelerometer_z, time_shifted, accelerometer_zMovMean);
xlabel('Time (seconds)');
ylabel('Accelerometer Z (m/s^2)');
title('Accelerometer Z vs Time');
grid on;

% Plot: Gravity (X, Y, Z) vs Time
figure;
subplot(3, 1, 1);
plot(time_shifted, gravity_x);
xlabel('Time (seconds)');
ylabel('Gravity X (m/s^2)');
title('Gravity X vs Time');
grid on;

subplot(3, 1, 2);
plot(time_shifted, gravity_y);
xlabel('Time (seconds)');
ylabel('Gravity Y (m/s^2)');
title('Gravity Y vs Time');
grid on;

subplot(3, 1, 3);
plot(time_shifted, gravity_z);
xlabel('Time (seconds)');
ylabel('Gravity Z (m/s^2)');
title('Gravity Z vs Time');
grid on;
