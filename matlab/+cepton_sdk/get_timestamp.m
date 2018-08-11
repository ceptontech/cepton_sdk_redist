function result = get_timestamp()
    result = posixtime(datetime('now', 'TimeZone', 'UTC'));
end
