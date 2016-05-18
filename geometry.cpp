#include "geometry.h"
#include <cmath>
#include <iostream>

Vec3d Geometry::color(const Ray& ray, const double t, const Ray& out, const Vec3d& outI, const Vec3d& envI) const {
    Vec3d h = normalize(-ray.d - out.d);
    Vec3d nv = n_vec(ray, t);
    Vec3d tex = texture(ray, t);
    return (mate.Ka * envI + outI.mul( tex * (-out.d.dot(nv)) ) + outI * mate.Ks * pow(h.dot(nv), mate.ns));
}

double Plain::intersect(const Ray &ray) const {
    // n.dot(ray.o)+t*(n.dot(ray.d)) + d = 0;
    double divu = n.dot(ray.d);
    if (fabs(divu) < EPS) {
        return -1;
    }
    else {
        return (-d - n.dot(ray.o)) / divu;
    }
}

Vec3d Plain::n_vec(const Ray& ray, const double t) const {
    if (ray.d.dot(n) < 0)
        return n;
    else
        return (-n);
}

double Sphere::intersect(const Ray& ray) const {
    double td = (o - ray.o).dot(ray.d);
    double roo = norm(o - ray.o);
    if (roo > r && td < 0)
        return -1;
    double t2 = r * r - roo * roo + td * td;
    if (t2 < 0)
        return -1;
    else {
        double tt = sqrt(t2);
        if (td - tt > 0)
            return td - tt;
        else if (td + tt > 0)
            return td + tt;
        else
            return -1;
    }
}

Vec3d Sphere::n_vec(const Ray &ray, const double t) const {
    Vec3d nv = cv::normalize(Vec3d(ray.p(t) - this->o));
    if (ray.d.dot(nv) < 0)
        return nv;
    else
        return (-nv);
}

bool Sphere::go_in(const Ray &ray) const {
    const Point3d& o = ray.o;
    if (norm(o - this->o) > r)
        return true;
    else
        return false;
}
