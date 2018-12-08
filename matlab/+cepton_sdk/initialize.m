function initialize(varargin)
%initialize(capture_path=None, control_flags=0)
%
%   Initializes SDK and starts capture replay.
%
%   Arguments:
%       control_flags: cepton_sdk.ControlFlag
    default_args = struct();
    default_args.capture_path = cepton_sdk.common.None;
    default_args.control_flags = 0;
    args = cepton_sdk.common.parse_args(default_args, varargin{:});

    control_flags = uint32(args.control_flags);
    if ~cepton_sdk.common.is_none(args.capture_path)
        control_flags = ...
            bitor(control_flags, cepton_sdk.ControlFlag.DISABLE_NETWORK);
    end
    cepton_sdk.c.call_and_check('cepton_sdk_matlab_initialize', 18, control_flags);

    if ~cepton_sdk.common.is_none(args.capture_path)
        cepton_sdk.open_replay(args.capture_path);
    end
end
