function wait_on_func(func, varargin)
    default_args = struct();
    default_args.timeout = cepton_sdk.common.None;
    args = cepton_sdk.common.parse_args(default_args, varargin{:});

    if ~cepton_sdk.common.is_none(args.timeout)
        t_start = posixtime(datetime());
    end

    while ~func()
        cepton_sdk.internal.wait();
        if ~cepton_sdk.common.is_none(args.timeout)
            if posixtime(datetime()) > args.timeout
                error('timed out');
            end
        end
    end

end
