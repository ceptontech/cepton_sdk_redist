function result = unpack_bits(a, n)
    result = fliplr(dec2bin(a, n) == '1');
end