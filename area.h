/*
 * Author: Saud Zahir
 * Date: June 14, 2024
 * Description: Algorithm to compute the surface area (both scalar & vector) of a polyhedron.
                Implementation for polygon surface area is adapted from
                https://github.com/caoguolin/3D-Polygon-area
*/

#pragma once

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;


Vector3d computePolygonVectorArea(const std::vector<Vector3d> &points);

double computePolygonArea(const std::vector<Vector3d> &points);
