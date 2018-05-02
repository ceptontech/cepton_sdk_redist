function c = filled_cell(sz, value)
    c = cell(sz);
    if isa(value, 'function_handle')
        for i_cell = 1:numel(c)
            c{i_cell} = value();
        end
    else
        c(:) = {value};
    end
end
