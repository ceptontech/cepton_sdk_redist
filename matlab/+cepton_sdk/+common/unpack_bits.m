function result = unpack_bits(a, n)
    result = dec2bin(a, n) == '1';
end