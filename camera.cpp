#include "camera.h"

Ray Camera::ray_direction(double x, double y, unsigned short* Xi) {
    double sx = s * (2 * x - 1);
    double sy = s * (2 * y - 1);
    // No depth
    // return Ray(eye, cv::normalize(forward + sx * right + sy * down));
    // With depth
    // Ray fray = Ray(eye, cv::normalize(forward + sx * right + sy * down));
    Vec3d orid = forward + sx * right + sy * down;
    double theta = 2 * M_PI * erand48(Xi);
    double randr = apeature * erand48(Xi);
    Vec3d move = cos(theta) * randr * right + sin(theta) * randr * down;
    return Ray(eye + Point3d(move), normalize(orid * focus - move));
}
