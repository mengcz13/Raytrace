#include "geometry.h"
#include <cmath>
#include <iostream>


Vec3d Geometry::color(const Ray& ray, const double t, const Ray& out, const Vec3d& outI, const Vec3d& envI) const {
    Vec3d h = normalize(-ray.d - out.d);
    Vec3d nv = n_vec(ray, t);
    Vec3d tex = texture(ray, t);
    // return (mate.Ka * envI + outI.mul( tex * (-out.d.dot(nv)) ) + outI * mate.Ks * pow(h.dot(nv), mate.ns));
    return outI.mul( tex * (-out.d.dot(nv)) );
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

void Sphere::rxy(cv::Point3d p, double &x, double &y) const {
    Vec3d direction = normalize(Vec3d(p - o));
    double phi = asin(direction[2]);
    double theta = acos(direction[0] / sqrt(direction[1] * direction[1] + direction[0] * direction[0]));
    if (direction[1] < 0)
        theta = 2 * M_PI - theta;
    x = theta / (2 * M_PI);
    y = 0.5 - phi / M_PI;
}

Vec3d Sphere::texture(const Ray &ray, const double t) const {
    if (this->material().textype == PURE)
        return this->material().Kds;
    else {
        double rx = 0, ry = 0;
        rxy(ray.p(t), rx, ry);
        if (this->material().textype == SQUARE) {
            int tx = (int)(rx / 0.1), ty = (int)(ry / 0.1);
            if ((abs(tx) & 1) ^ (abs(ty) & 1)) {
                return Vec3d(0.01, 0.01, 0.01);
            }
            else {
                return Vec3d(1, 1, 1);
            }
        }
        else if (this->material().textype == PIC) {
            double col = rx * (this->picture.cols - 1), row = ry * (this->picture.rows - 1);
            return this->picture.ptr<Vec3d>((int)row)[(int)col];
        }
    }
}

double Rectangle::intersect(const Ray &ray) const {
    double divu = n.dot(ray.d);
    if (fabs(divu) < EPS) {
        return -1;
    }
    else {
        double d = -n.dot(o);
        double t = (-d - n.dot(ray.o)) / divu;
        double rx = 0, ry = 0;
        rxy(ray.p(t), rx, ry);
        if (0 < rx && rx < 1 && 0 < ry && ry < 1)
            return t;
        else
            return -1;
    }
}

Vec3d Rectangle::n_vec(const Ray &ray, const double t) const {
    if (ray.d.dot(n) < 0)
        return n;
    else
        return (-n);
}

Vec3d Rectangle::texture(const Ray &ray, const double t) const {
    if (this->material().textype == PURE)
        return this->material().Kds;
    else if (this->material().textype == SQUARE) {
        double rx = 0, ry = 0;
        rxy(ray.p(t), rx, ry);
        int xd = abs((int)(rx / 0.1)) & 1, yd = abs((int)(ry / 0.1)) & 1;
        if (xd ^ yd)
            return Vec3d(0.01, 0.01, 0.01);
        else
            return Vec3d(1, 1, 1);
    }
    else if (this->material().textype == PIC) {
        double rx = 0, ry = 0;
        rxy(ray.p(t), rx, ry);
        double col = rx * (this->picture.cols - 1), row = ry * (this->picture.rows - 1);
        return this->picture.ptr<Vec3d>((int)row)[(int)col];
    }
}

void Rectangle::rxy(cv::Point3d xy, double &x, double &y) const {
    Vec3d oxy = xy - o;
    x = oxy.dot(xaxis) / xrange;
    y = oxy.dot(yaxis) / yrange;
}
