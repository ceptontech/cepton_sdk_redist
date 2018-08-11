function wait(t_length_tmp)
    if nargin >= 1
        t_length = t_length_tmp;
    else
        t_length = 0.1;
    end

    if cepton_sdk.capture_replay.is_end()
        error('EOF');
    end

    t = cepton_sdk.get_time() + t_length;
    while cepton_sdk.get_time() < t
        
        if cepton_sdk.is_realtime()
            pause(0.1);
        else
            if cepton_sdk.capture_replay.is_end()
                break;
            end
            cepton_sdk.capture_replay.resume_blocking(0.1);
        end
        cepton_sdk.internal.update();
    end
end
