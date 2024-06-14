/*
 * Author: Saud Zahir
 * Date: June 14, 2024
 * Description: Algorithm to compute the surface area (both scalar & vector) of a polyhedron.
                Implementation for polygon surface area is adapted from
                https://github.com/caoguolin/3D-Polygon-area
*/

#include "area.h"


Vector3d computePolygonVectorArea(const vector<Vector3d> &points) {
  Vector3d p1 = points[0];
  Vector3d p2 = points[1];
  Vector3d p3 = points[2];

  // Normal vector on face of polygon
  Vector3d normal;
  normal = (p2 - p1).cross(p3 - p1);

  // double k = pow(normal.norm(), 2);

  double area =
      normal.z() * (points.back().x() * p1.y() - p1.x() * points.back().y()) +
      normal.x() * (points.back().y() * p1.z() - p1.y() * points.back().z()) +
      normal.y() * (points.back().z() * p1.x() - p1.z() * points.back().x());

  for (size_t i = 0; i < points.size() - 1; ++i) {
    const Vector3d &p1 = points[i];
    const Vector3d &p2 = points[i + 1];
    double ss = normal.z() * (p1.x() * p2.y() - p2.x() * p1.y()) +
                normal.x() * (p1.y() * p2.z() - p2.y() * p1.z()) +
                normal.y() * (p1.z() * p2.x() - p2.z() * p1.x());
    area += ss;
  }

  area = abs(area / 2.0);

  return area * normal;
}

double computePolygonArea(const vector<Vector3d> &points) {
  return sqrt(computePolygonVectorArea(points).dot(computePolygonVectorArea(points)));
}
