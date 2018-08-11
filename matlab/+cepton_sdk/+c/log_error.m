function err = check_error(err)
    if ~err
        return;
    end

    exception = MException(err);
    warning(exception.msgID, exception.msgtext);
end
