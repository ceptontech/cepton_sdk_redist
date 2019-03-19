%%
% Initialize
if cepton_sdk.is_initialized()
    cepton_sdk.deinitialize();
end
cepton_sdk.initialize();
cepton_sdk.wait(3);

% Get sensor
sensors_dict = cepton_sdk.get_sensors();
sensors_list = values(sensors_dict);
sensor = sensors_list{1};
disp(sensor.information)

%% Get points
listener = cepton_sdk.SensorFramesListener(sensor.serial_number);
points_list = listener.get_points();
delete(listener);
points = points_list{1};

% Plot
cepton_sdk.plot_points(points);
