%%
% Variables
capture_path = common.get_sample_capture_path();

% Initialize
if cepton_sdk.is_initialized()
    cepton_sdk.deinitialize();
end
options = struct();
options.capture_path = capture_path;
cepton_sdk.initialize(options);

% Get sensor
sensors_dict = cepton_sdk.get_sensors();
sensors_list = values(sensors_dict);
sensor = sensors_list{1};
disp(sensor.information);

%% Get points
listener = cepton_sdk.SensorFramesListener(sensor.serial_number);
points_list = listener.get_points();
delete(listener);
points = points_list{1};

% Plot
cepton_sdk.plot_points(points);
