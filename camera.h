#ifndef CAMERA_H
#define CAMERA_H
#include <opencv2/opencv.hpp>
#include <cmath>
using cv::Vec3d;
using cv::Point3d;

class Ray {
public:
    Ray(const Point3d& p, const Vec3d& dd) : o(p), d(cv::normalize(dd)) {}
    Point3d o;
    Vec3d d;
    const Point3d p(double t) const {
        return o + Point3d(t * d);
    }
};

class Camera {
public:
    Camera(const Point3d& e, const Vec3d& f, const Vec3d& r, double fov, double apeature, double focus) : eye(e), forward(cv::normalize(f)), right(cv::normalize(r)), FOV(fov), apeature(apeature), focus(focus) {
        down = cv::normalize(forward.cross(right));
        s = tan(FOV / 2);
    }
    Ray ray_direction(double x, double y, unsigned short* Xi);
private:
    Point3d eye;
    Vec3d forward;
    Vec3d right;
    Vec3d down;
    double FOV;
    double s;
    double apeature;
    double focus;
};

#endif // CAMERA_H
