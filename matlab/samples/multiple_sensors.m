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

% Get sensors
sensors_dict = cepton_sdk.get_sensors();

%% Get points
listener = cepton_sdk.ImageFramesListener();
image_points_dict = listener.get_points();
delete(listener);
image_points_list = values(image_points_dict);
image_points_list = image_points_list{1};
points = image_points_list{1}.to_points();

% Plot
cepton_sdk.plot_points(points);
