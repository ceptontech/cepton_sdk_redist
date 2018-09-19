function deinitialize()
	cepton_sdk.c.call_and_check('cepton_sdk_deinitialize');

	% HACK
    cepton_sdk.c.unload();
end
