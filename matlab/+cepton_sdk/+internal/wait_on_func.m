function wait_on_func(func, varargin)
    default_args = struct();
    default_args.timeout = cepton_sdk.common.None;
    args = cepton_sdk.common.parse_args(default_args, varargin{:});

    if ~cepton_sdk.common.is_none(args.timeout)
        t_start = cepton_sdk.get_timestamp();
    end

    while ~func()
        cepton_sdk.wait();
        if ~cepton_sdk.common.is_none(args.timeout)
            if (cepton_sdk.get_timestamp() - t_start) > args.timeout
                error('Timed out!');
            end
        end
    end

end
