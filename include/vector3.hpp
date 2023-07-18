#pragma once

#include <math.h>

struct vector3
{
    // displacements on a given axis
    float x, y, z;
    // returns the length of the vector
    float length()
    {
        float l = sqrt(x * x + y * y + z * z);
        return l;
    }

    vector3 operator+(const vector3 &other) const
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    vector3 operator-(const vector3 &other) const
    {
        return {x - other.x, y - other.y, z - other.z};
    }

    vector3 operator+=(const vector3 &other) const
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    bool operator==(const vector3 &other) const
    {
        return (x == other.x && y == other.y && z == other.z);
    }

    bool operator!=(const vector3 &other) const
    {
        return !(x == other.x && y == other.y && z == other.z);
    }
} nullVector = {0, 0, 0};