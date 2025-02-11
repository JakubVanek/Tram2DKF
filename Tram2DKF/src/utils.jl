function lerp(x0, y0, x1, y1, xq)
    return (xq - x0) / (x1 - x0) * (y1 - y0) + y0
end
function dlerp(x0, y0, x1, y1)
    return (y1 - y0) / (x1 - x0)
end
