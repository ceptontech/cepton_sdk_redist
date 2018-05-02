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

%%
% Get start time (seconds since epoch)
t_start = cepton_sdk.capture_replay.get_start_time()

% Get length (seconds)
t_length = cepton_sdk.capture_replay.get_length()

% Seek to relative position (seconds)
cepton_sdk.capture_replay.seek(1);

% Get relative position (seconds)
t = cepton_sdk.capture_replay.get_position()
