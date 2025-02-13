"""
    lerp(x0, y0, x1, y1, xq)

Perform linear interpolation between two points.

The interpolation line is specified using the points `(x0, y0)` and `(x1, y1)`.
The function then returns the y-coordinate of the point with x-coordinate equal to `xq`.

See also [`dlerp`](@ref).
"""
function lerp(x0, y0, x1, y1, xq)
    return (xq - x0) / (x1 - x0) * (y1 - y0) + y0
end

"""
    dlerp(x0, y0, x1, y1)

Calculate the slope of a line going through points `(x0, y0)` and `(x1, y1)`.

This function is the derivative of [`dlerp`](@ref) with respect
to the `xq` argument.

See also [`dlerp`](@ref).
"""
function dlerp(x0, y0, x1, y1)
    return (y1 - y0) / (x1 - x0)
end
