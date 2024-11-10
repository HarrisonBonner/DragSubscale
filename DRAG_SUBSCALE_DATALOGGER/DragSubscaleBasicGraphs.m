clc, clear, close all
% Load data from Excel file, skipping the first row
data = readtable('DragDatalog.xlsx', 'ReadVariableNames', true); % Replace with your actual file name

% Extract the columns as numeric arrays, handling cell types
time = data{:, 1}; % Column 1: TIME
altitude = data{:, 2}; % Column 2: ALTITUDE

% Convert to numeric if necessary
if iscell(time)
    time = cellfun(@str2double, time); % Convert cell array to numeric array
end

if iscell(altitude)
    altitude = cellfun(@str2double, altitude); % Convert cell array to numeric array
end

% Check for and remove NaN values if any conversions fail
time = time(~isnan(time));
altitude = altitude(~isnan(altitude));

% Adjust time column (subtract the first value and divide by 1000)
time_shifted = (time - time(1)) / 1000;

% Plot the data
figure;
plot(time_shifted, altitude);
xlabel('Time (seconds)');
ylabel('Altitude (m)');
title('Altitude vs Time');
grid on;


% % Load data from Excel file, skipping the first row
% data = readtable('DragDatalog.xlsx', 'ReadVariableNames', true); % Replace with your actual file name
% 
% % Extract the TIME column and convert it to numeric if necessary
% time = data{:, 1};
% if iscell(time)
%     time = cellfun(@str2double, time);
% end
% 
% % Ensure no NaNs are present in the TIME array due to conversion issues
% time = time(~isnan(time));
% 
% % Adjust the TIME column
% if ~isempty(time)
%     time_shifted = (time - time(1)) / 1000;
% else
%     error('The TIME column could not be properly converted. Please check the data format.');
% end
% 
% % Debug: display the time_shifted array
% disp('Time shifted array (first 10 values):');
% disp(time_shifted(1:min(10, end)));
% 
% % Plotting orientation (in degrees)
% figure;
% subplot(3, 1, 1);
% [orientationX, cleanTime] = cleanColumnData(data{:, 3}, time_shifted);
% disp('Orientation X (first 10 values):');
% disp(orientationX(1:min(10, end))); % Debug: display first 10 values of orientationX
% plot(cleanTime, orientationX, '-');
% xlabel('Time (seconds)');
% ylabel('Orientation X (degrees)');
% title('Orientation X vs Time');
% grid on;
% 
% subplot(3, 1, 2);
% [orientationY, cleanTime] = cleanColumnData(data{:, 4}, time_shifted);
% disp('Orientation Y (first 10 values):');
% disp(orientationY(1:min(10, end))); % Debug: display first 10 values of orientationY
% plot(cleanTime, orientationY, '-');
% xlabel('Time (seconds)');
% ylabel('Orientation Y (degrees)');
% title('Orientation Y vs Time');
% grid on;
% 
% subplot(3, 1, 3);
% [orientationZ, cleanTime] = cleanColumnData(data{:, 5}, time_shifted);
% disp('Orientation Z (first 10 values):');
% disp(orientationZ(1:min(10, end))); % Debug: display first 10 values of orientationZ
% plot(cleanTime, orientationZ, '-');
% xlabel('Time (seconds)');
% ylabel('Orientation Z (degrees)');
% title('Orientation Z vs Time');
% grid on;
% 
% % Function to clean column data
% function [cleanData, cleanTime] = cleanColumnData(columnData, timeData)
%     % This function takes a column of data and a time array, converts the column
%     % to numeric if needed, and returns both with non-numeric or NaN values removed.
% 
%     % Convert cell array to numeric array if necessary
%     if iscell(columnData)
%         columnData = cellfun(@str2double, columnData);
%     end
% 
%     % Debug: display the column before cleaning
%     disp('Column data before cleaning (first 10 values):');
%     disp(columnData(1:min(10, end)));
% 
%     % Find valid indices (non-NaN)
%     validIndices = ~isnan(columnData) & ~isnan(timeData);
% 
%     % Filter data and time based on valid indices
%     cleanData = columnData(validIndices);
%     cleanTime = timeData(validIndices);
% 
%     % Debug: display cleaned data
%     disp('Cleaned column data (first 10 values):');
%     disp(cleanData(1:min(10, end)));
% end
