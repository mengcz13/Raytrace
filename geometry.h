#ifndef GEOMETRY_H
#define GEOMETRY_H
#include "camera.h"
using cv::Point3d;
using cv::Vec3d;
using cv::normalize;

const double EPS = 1e-6;
enum TYPE {DIFF, SPEC, REFR};
enum TEXTURETYPE {PURE, SQUARE, PIC};
struct Material {
    double Ka, Kto, Kso, nf;
    Vec3d Kds;
    double Ks;
    double ns;
    Vec3d Kdt;
    double Kt;
    double nt;
    Vec3d selflight;
    TYPE type;
    TEXTURETYPE textype;
};

const Material BALL_material = {
    0, 0, 0.9, 1,
    Vec3d(0.99, 0.99, 0.99),
    0.01,
    2000,
    Vec3d(0, 0, 0),
    0,
    1,
    Vec3d(0, 0, 0),
    SPEC, PURE
};

const Material BALL_material_REFR = {
    0, 0, 0.9, 1.5,
    Vec3d(0.99, 0.99, 0.99),
    0.01,
    2000,
    Vec3d(0, 0, 0),
    0,
    1,
    Vec3d(0, 0, 0),
    REFR, PURE
};

const Material TEST_material = {
    0, 0, 0, 1,
    Vec3d(1, 1, 1) * 0.99,
    0.8,
    100,
    Vec3d(0, 0, 0),
    0,
    1,
    Vec3d(0, 0, 0),
    DIFF, PURE
};

const Material TEST_material2 = {
    0, 0, 0, 1,
    Vec3d(0.75, 0.25, 0.25),
    0.8,
    100,
    Vec3d(0, 0, 0),
    0,
    1,
    Vec3d(0, 0, 0),
    DIFF, PURE
};

const Material TEST_material3 = {
    0, 0, 0, 1,
    Vec3d(0.25, 0.25, 0.75),
    0.8,
    100,
    Vec3d(0, 0, 0),
    0,
    1,
    Vec3d(0, 0, 0),
    DIFF, PURE
};

const Material TEST_material4 = {
    0, 0, 0, 1,
    Vec3d(1, 1, 1) * 0.75,
    0.8,
    100,
    Vec3d(0, 0, 0),
    0,
    1,
    Vec3d(0, 0, 0),
    DIFF, PURE
};

const Material TEST_material_SQ = {
    0, 0, 0, 1,
    Vec3d(1, 1, 1) * 0.75,
    0.8,
    100,
    Vec3d(0, 0, 0),
    0,
    1,
    Vec3d(0, 0, 0),
    DIFF, SQUARE
};

const Material LIGHT_material = {
    0, 0, 0, 1,
    Vec3d(0.7, 0.7, 0.2),
    0.8,
    100,
    Vec3d(0, 0, 0),
    0,
    1,
    Vec3d(15, 25, 25),
    DIFF, PURE
};

class Geometry {
public:
    Geometry(const Material& ma) : mate(ma) {}
    virtual double intersect(const Ray& ray) const = 0;
    virtual Vec3d color(const Ray& ray, const double t, const Ray& out, const Vec3d& outI, const Vec3d& envI) const;
    virtual Vec3d n_vec(const Ray& ray, const double t) const = 0;
    virtual Vec3d texture(const Ray& ray, const double t) const {
        return mate.Kds;
    }
    const Material& material() const { return mate; }
    virtual bool go_in(const Ray& ray) const { return false; }
private:
    Material mate;
};

class Plain : public Geometry {
public:
    Plain(const Vec3d& v, const Point3d& p, const Material& mate) : n(normalize(v)), o(p), Geometry(mate) {
        d = -n.dot(o);
    }
    double intersect(const Ray& ray) const;
    Vec3d color(const Ray& ray, const double t, const Ray& out, const Ray& env) const;
    Vec3d n_vec(const Ray& ray, const double t) const;
    virtual Vec3d texture(const Ray& ray, const double t) const {
        if (this->material().textype == PURE)
            return this->material().Kds;
        else if (this->material().textype == SQUARE) {
            Point3d p = ray.p(t) - Point3d(-100, -100, -100);
            int xd = abs((int)(p.x)) & 1, yd = abs((int)(p.z)) & 1;
            if (xd ^ yd)
                return Vec3d(0, 0, 0);
            else
                return Vec3d(1, 1, 1);
        }
    }
private:
    Vec3d n;
    Point3d o;
    double d;
};

class Sphere : public Geometry {
public:
    Sphere(const Point3d& center, const double ra, const Material& mate) : o(center), r(ra), Geometry(mate) {}
    double intersect(const Ray& ray) const;
    Vec3d n_vec(const Ray &ray, const double t) const;
    bool go_in(const Ray &ray) const;
private:
    Point3d o;
    double r;
};

#endif // GEOMETRY_H
