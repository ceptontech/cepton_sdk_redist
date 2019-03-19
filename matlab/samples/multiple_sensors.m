%%
% Variables
capture_path = common.get_sample_capture_path();

%%
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
listener = cepton_sdk.FramesListener();
points_dict = listener.get_points();
delete(listener);
points_list = values(points_dict);
points_list = points_list{1};
points = points_list{1};

% Plot
cepton_sdk.plot_points(points);
