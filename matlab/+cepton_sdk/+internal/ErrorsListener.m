classdef ErrorsListener

methods
    function update(self)
        while cepton_sdk.c.call('cepton_sdk_matlab_has_error')
            [error_code_tmp, handle, error_code, error_msg_size] = ...
                cepton_sdk.c.call('cepton_sdk_matlab_get_error', 0, 0, 0);
            cepton_sdk.c.check_error_code(error_code_tmp);

            options = struct();
            options.warning = true;
            cepton_sdk.c.check_error_code(error_code, options);
        end
    end
end

end
