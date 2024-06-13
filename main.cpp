/*
 * Author: Saud Zahir
 * Date: June 13, 2024
 * Description: Algorithms to compute the volume & surface area of a polyhedra.
                Implementation for polygon surface area is adapted from
                https://github.com/caoguolin/3D-Polygon-area
*/

#include <Eigen/Dense>
#include <fmt/core.h>
#include <iostream>
#include <vector>

using namespace Eigen;

typedef Matrix<double, Dynamic, 3> Vertices;
typedef Matrix<int, Dynamic, Dynamic> Faces;

double computePolygonArea(const std::vector<Vector3d> &points) {
  Vector3d p1 = points[0];
  Vector3d p2 = points[1];
  Vector3d p3 = points[2];

  // Normal vector on face of polygon
  Vector3d normal;
  normal << (p2.y() - p1.y()) * (p3.z() - p1.z()) -
                (p3.y() - p1.y()) * (p2.z() - p1.z()),
      (p3.x() - p1.x()) * (p2.z() - p1.z()) -
          (p2.x() - p1.x()) * (p3.z() - p1.z()),
      (p2.x() - p1.x()) * (p3.y() - p1.y()) -
          (p3.x() - p1.x()) * (p2.y() - p1.y());

  // double k = pow(normal.norm(), 2);

  double signed_area =
      normal.z() * (points.back().x() * p1.y() - p1.x() * points.back().y()) +
      normal.x() * (points.back().y() * p1.z() - p1.y() * points.back().z()) +
      normal.y() * (points.back().z() * p1.x() - p1.z() * points.back().x());

  for (size_t i = 0; i < points.size() - 1; ++i) {
    const Vector3d &p1 = points[i];
    const Vector3d &p2 = points[i + 1];
    double ss = normal.z() * (p1.x() * p2.y() - p2.x() * p1.y()) +
                normal.x() * (p1.y() * p2.z() - p2.y() * p1.z()) +
                normal.y() * (p1.z() * p2.x() - p2.z() * p1.x());
    signed_area += ss;
  }

  signed_area = std::abs(signed_area / 2.0);

  return signed_area;
}

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

int main() {
    // Vertices and faces of a cube
    Vertices cubeVertices(8, 3);
    cubeVertices << 0, 0, 0,
                    1, 0, 0,
                    1, 0, 1,
                    0, 0, 1,
                    0, 1, 1,
                    1, 1, 1,
                    1, 1, 0,
                    0, 1, 0;

    Faces cubeFaces(6, 4);
    cubeFaces << 1, 2, 3, 4,
                 2, 3, 6, 7,
                 3, 4, 5, 6,
                 1, 4, 5, 8,
                 1, 2, 7, 8,
                 5, 6, 7, 8;

    for (int i = 0; i < cubeFaces.rows(); ++i) {
    std::vector<Vector3d> points;
    for (int j = 0; j < cubeFaces.cols(); ++j) {
        Vector3d p = cubeVertices.row(
            cubeFaces(i, j) - 1); // Adjust for 1-based to 0-based indexing
        points.push_back(p);
    }

    double faceArea = computePolygonArea(points);
    fmt::print("Area of face {} of the cube: {}\n", i + 1, faceArea);
    }

    double volume = computePolyhedronVolume(cubeVertices, cubeFaces);
    fmt::print("Volume of the cube: {}\n", volume);

    // // Golden ratio
    // const double phi = (1.0 + std::sqrt(5.0)) / 2.0;

    // // Vertices for a dodecahedron
    // Vertices dodecaVertices(20, 3);
    // dodecaVertices << 1, 1, 1,
    //                   1, 1, -1,
    //                   1, -1, 1,
    //                   1, -1, -1,
    //                   -1, 1, 1,
    //                   -1, 1, -1,
    //                   -1, -1, 1,
    //                   -1, -1, -1,
    //                   0, phi, 1 / phi,
    //                   0, phi, -1 / phi,
    //                   0, -phi, 1 / phi,
    //                   0, -phi, -1 / phi,
    //                   1 / phi, 0, phi,
    //                   1 / phi, 0, -phi,
    //                   -1 / phi, 0, phi,
    //                   -1 / phi, 0, -phi,
    //                   phi, 1 / phi, 0,
    //                   phi, -1 / phi, 0,
    //                   -phi, 1 / phi, 0,
    //                   -phi, -1 / phi, 0;

    // // Faces for a dodecahedron
    // Faces dodecaFaces(12, 5);
    // dodecaFaces << 1, 9, 10, 5, 17,
    //                1, 17, 18, 3, 13,
    //                13, 3, 11, 7, 14,
    //                2, 10, 9, 6, 15,
    //                2, 15, 16, 4, 12,
    //                12, 4, 20, 8, 19,
    //                19, 8, 7, 11, 3,
    //                20, 4, 16, 14, 7,
    //                18, 17, 5, 6, 9,
    //                18, 13, 14, 16, 15,
    //                19, 3, 18, 15, 2,
    //                19, 11, 7, 8, 20;

    // for (int i = 0; i < dodecaFaces.rows(); ++i) {
    // std::vector<Vector3d> points;
    // for (int j = 0; j < dodecaFaces.cols(); ++j) {
    //     Vector3d p = dodecaVertices.row(
    //         dodecaFaces(i, j) - 1); // Adjust for 1-based to 0-based indexing
    //     points.push_back(p);
    // }

    // double faceArea = computePolygonArea(points);
    // fmt::print("Area of face {} of the dodecahedron: {}\n", i + 1, faceArea);
    // }

    // double dodeca_volume = computePolyhedronVolume(dodecaVertices, dodecaFaces);
    // fmt::print("Volume of the dodecahedron: {}\n", dodeca_volume);

    // return 0;
}
