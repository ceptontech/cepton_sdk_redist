function err = check_error(err)
    if ~err
        return;
    end

    exception = MException(err);
    if err.is_error()
        throwAsCaller(exception);
    else
        warning(exception.msgID, exception.msgtext);
    end
end
