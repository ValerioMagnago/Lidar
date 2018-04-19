function out = custom_cat_cell(a, b, dim)
if dim == 1 % cat along the row
    out = [a b];
else
    out = [a;b];
end