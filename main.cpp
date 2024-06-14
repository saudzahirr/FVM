/*
 * Author: Saud Zahir
 * Date: June 13, 2024
 * Description: Algorithm for computing the volume & surface area
                of CFD polyhedron mesh elements (like tetrahedron,
                pentahedron & hexahedron) in Finite Volume Method.
*/


// External Includes
#include <fmt/core.h>
#include <iostream>

// Internal Includes
#include "area.h"
#include "volume.h"

void display(const Vertices &vertices, const Faces &faces, const std::string type) {
    for (int i = 0; i < faces.rows(); ++i) {
        std::vector<Vector3d> points;
        for (int j = 0; j < faces.cols(); ++j) {
            Vector3d p = vertices.row(
                faces(i, j) - 1); // Adjust for 1-based to 0-based indexing
            points.push_back(p);
        }

        double faceArea = computePolygonArea(points);
        fmt::print("Area of face {} of the {}: {}\n", i + 1, type, faceArea);
    }

    double volume = computePolyhedronVolume(vertices, faces);
    fmt::print("Volume of the {}: {}\n\n", type, volume);
}

int main() {
    // Vertices and faces of a tetrahedron
    Vertices tetraVertices(4, 3);
    tetraVertices << 0, 0, 0,
                     1, 0, 0,
                     0.5, sqrt(3)/2, 0,
                     0.5, sqrt(3)/6, sqrt(6)/3;

    Faces tetraFaces(4, 3);
    tetraFaces << 1, 2, 3,
                        1, 2, 4,
                        1, 3, 4,
                        2, 3, 4;

    display(tetraVertices, tetraFaces, "tetrahedron");


    // Vertices and faces of a pentahedron
    // Vertices pentaVertices(5, 3);
    // pentaVertices << 0, 0, 0,
    //                  1, 0, 0,
    //                  0.5, sqrt(3)/2, 0,
    //                  0.5, sqrt(3)/6, sqrt(6)/3,
    //                  0.5, sqrt(3)/6, -sqrt(6)/3;

    // Faces pentaFaces(5, 4);
    // pentaFaces << 0, 1, 2, 0,
    //               3, 4, 5, 3,
    //               0, 1, 4, 3,
    //               1, 2, 5, 4,
    //               2, 0, 3, 5;

    // display(pentaVertices, pentaFaces, "pentahedron");

    // Vertices and faces of a hexahedron
    Vertices hexaVertices(8, 3);
    hexaVertices << 0, 0, 0,
                    1, 0, 0,
                    1, 0, 1,
                    0, 0, 1,
                    0, 1, 1,
                    1, 1, 1,
                    1, 1, 0,
                    0, 1, 0;

    Faces hexaFaces(6, 4);
    hexaFaces << 1, 2, 3, 4,
                 2, 3, 6, 7,
                 3, 4, 5, 6,
                 1, 4, 5, 8,
                 1, 2, 7, 8,
                 5, 6, 7, 8;

    display(hexaVertices, hexaFaces, "hexahedron");

}
