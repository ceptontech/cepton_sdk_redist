%%
% Variables
capture_path = common.get_sample_capture_path();
sensor_serial_number = 4165;

% Initialize
if cepton_sdk.is_initialized()
    cepton_sdk.deinitialize();
end
options = struct();
options.capture_path = capture_path;
cepton_sdk.initialize(options);

%% Get sensor
sensor = cepton_sdk.Sensor.create(sensor_serial_number);
disp(sensor.information);

%% Get points
image_points = ...
    cepton_sdk.get_sensor_image_points(sensor_serial_number);
points = image_points.to_points();

% Plot
common.plot_points(points);
