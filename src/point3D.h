/*UVOS*/

/* This file is part of TelemetrySystem.
 *
 * TelemetrySystem is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL) version 3 as published by
 * the Free Software Foundation.
 *
 * TelemetrySystem is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with TelemetrySystem.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include <stdint.h>
#include <cstring>

template <class T>
class Point3D
{
public:
    T x;
    T y;
    T z;
    
    
    Point3D()
    {
        x = 0;
        y = 0;
        z = 0;
    }
    
    int serialize(uint8_t* buffer, const unsigned length) const //todo: make this portable
    {
        if(length >= sizeof(T)*3)
        {
            std::memcpy(buffer, &x, sizeof(T));
            std::memcpy(buffer+sizeof(T), &y, sizeof(T));
            std::memcpy(buffer+sizeof(T)*2, &z, sizeof(T));
            return sizeof(T)*3;
        }
        else return -1;
    }
    
    Point3D operator + (const Point3D& param)
    {
        Point3D sum;
        sum.x = x+param.x;
        sum.y = y+param.y;
        sum.z = z+param.z;
        return sum;
    }
};
