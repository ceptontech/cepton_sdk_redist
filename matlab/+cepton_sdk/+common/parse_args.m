function args = parse_args(default_args, varargin)
    try
        args = struct(varargin{:});
    catch
        error_id = 'parse_args:ArgumentError';
        error_msg = sprintf('invalid arguments');
        error = MException(error_id, error_msg);
        throwAsCaller(error);
    end

    error = check_args(args, default_args);
    if ~isempty(error)
        throwAsCaller(error);
    end

    args = update_args(args, default_args);
end

function error = check_args(args, default_args)
    error = [];
    
    names = fieldnames(args);
    for i_field = 1:numel(names)
        name_tmp = names{i_field};
        if ~has_field(default_args, name_tmp)
            error_id = 'parse_args:ArgumentError';
            error_msg = sprintf('invalid argument: %s', name_tmp);
            error = MException(error_id, error_msg);
            return;
        end
    end
end

function args = update_args(args, default_args)
    names = fieldnames(default_args);
    for i_field = 1:numel(names)
        name_tmp = names{i_field};
        if ~has_field(args, name_tmp)
            args.(name_tmp) = default_args.(name_tmp);
        end
    end
end

function tf = has_field(s, name)
    tf = any(ismember(name, fieldnames(s)));
end
