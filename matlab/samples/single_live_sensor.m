%%
% Initialize
if cepton_sdk.is_initialized()
    cepton_sdk.deinitialize();
end
cepton_sdk.initialize();

%% Get sensor
sensors = cepton_sdk.get_sensors();
sensor_serial_numbers = sensors.keys();
sensor_serial_number = sensor_serial_numbers(1);
disp(sensor.information)

%% Get points
image_points = ...
    cepton_sdk.get_sensor_image_points(sensor_serial_number);
points = image_points.to_points();

% Plot
common.plot_points(points);
