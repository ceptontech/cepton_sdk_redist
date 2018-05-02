function check_error_code(error_code, varargin)
    default_args = struct();
    default_args.warning = false;
    args = cepton_sdk.common.parse_args(default_args, varargin{:});

    if ~error_code
        return
    end

    error_code_name = cepton_sdk.c.get_error_code_name(error_code);
    id = sprintf('cepton_sdk:%s', error_code_name);
    msg = error_code_name;
    if cepton_sdk.c.is_error_code & ~args.warning
        error(id, msg);
    else
        warning(id, msg);
    end
end
