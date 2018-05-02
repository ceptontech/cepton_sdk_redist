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
sensors_dict = cepton_sdk.get_sensors();
sensor = sensors_dict(sensor_serial_number);
disp(sensor.information)

%% Get points
image_points_dict = cepton_sdk.get_image_points();
image_points_list = image_points_dict(sensor_serial_number);
image_points = image_points_list{1};
points = image_points.to_points();

% Plot
common.plot_points(points);
