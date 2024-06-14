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
    for (size_t i = 0; i < faces.size(); ++i) {
        Vector3d xc(0.0, 0.0, 0.0); // centroid of face i
        vector<Vector3d> points;

        // Iterate over vertices of the current face
        for (size_t j = 0; j < faces[i].size(); ++j) {
            Vector3d p = vertices.row(faces[i](j) - 1); // Adjust for 1-based indexing
            points.push_back(p);
            xc += p;
        }

        // Centroid xc
        xc /= static_cast<double>(faces[i].size());

        // Select three points to define the plane of the face
        Vector3d p0 = vertices.row(faces  - 1);
        Vector3d p1 = vertices.row(faces  - 1);
        Vector3d p2 = vertices.row(faces  - 1);

        // Compute the normal vector to the face
        Vector3d normal = (p1 - p0).cross(p2 - p0);
        normal.normalize();

        // Compute the area of the face (polygon)
        double area = computePolygonArea(points);

        // Compute the volume contribution using the centroid and area
        volume += abs(xc.dot(normal) * area);
    }

    // Return one-third of the computed volume (since we computed 3 times the actual volume)
    return volume / 3.0;
}
