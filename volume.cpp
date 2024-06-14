/*
 * Author: Saud Zahir
 * Date: June 14, 2024
 * Description: Algorithms to compute the volume of a polyhedron.
                Implementation for polygon volume is based on Gauss Divergence Theorem.
*/

#include "volume.h"


double computePolyhedronVolume(const Vertices &vertices, const Faces &faces) {
  double volume = 0.0;

  // Iterate over each face
  for (int i = 0; i < faces.rows(); ++i) {
    Vector3d xc(0.0, 0.0, 0.0); // centroid of face i
    std::vector<Vector3d> points;

    for (int j = 0; j < faces.cols(); ++j) {
      Vector3d p = vertices.row(faces(i, j) -
                                1); // Adjust for 1-based to 0-based indexing
      points.push_back(p);
      xc += p;
    }

    // Centroid xc
    xc /= static_cast<double>(faces.cols());

    Vector3d p0 =
        vertices.row(faces(i, 0) - 1); // Adjust for 1-based to 0-based indexing
    Vector3d p1 = vertices.row(faces(i, 1) - 1);
    Vector3d p2 = vertices.row(faces(i, 2) - 1);

    // Normal vector to that face
    Vector3d normal = (p1 - p0).cross(p2 - p0);
    normal.normalize();

    // Compute the area of the face (polygon)
    double area = computePolygonArea(points);
    volume += std::abs(xc.dot(normal) * area);
  }

  return volume / 3.0;
}
