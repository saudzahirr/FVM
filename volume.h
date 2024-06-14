/*
 * Author: Saud Zahir
 * Date: June 14, 2024
 * Description: Algorithms to compute the volume of a polyhedron.
                Implementation for polygon volume is based on Gauss Divergence Theorem.
*/

#pragma once

#include "area.h"


typedef Matrix<double, Dynamic, 3> Vertices;
typedef Matrix<int, Dynamic, Dynamic> Faces;

double computePolyhedronVolume(const Vertices &vertices, const Faces &faces);
