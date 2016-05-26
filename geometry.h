#ifndef GEOMETRY_H
#define GEOMETRY_H
#include "camera.h"
#include "string.h"
using cv::Point3d;
using cv::Vec3d;
using cv::normalize;
using std::string;

const double EPS = 1e-6;
enum TYPE {DIFF, SPEC, REFR, MIXED};
enum TEXTURETYPE {PURE, SQUARE, PIC};
/* struct Material {
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
}; */

struct Material {
    double nf;
    Vec3d Kds; // Default color(if no texture is given)
    Vec3d selflight;
    TYPE type;
    TEXTURETYPE textype;
    Vec3d p_of_dsr;
};

const Material BALL_material_SPEC = {
    1,
    Vec3d(0.95, 0.95, 0.95),
    Vec3d(0, 0, 0),
    SPEC,
    PURE
};

const Material BALL_material_REFR = {
    1.5,
    Vec3d(0.95, 0.95, 0.95),
    Vec3d(0, 0, 0),
    MIXED,
    PURE,
    Vec3d(0.01, 0.01, 0.98)
};

const Material WHITE_WALL = {
    1,
    Vec3d(0.95, 0.95, 0.95),
    Vec3d(0, 0, 0),
    DIFF,
    PURE
};

const Material RED_WALL = {
    1,
    Vec3d(0, 0, 0.95),
    Vec3d(0, 0, 0),
    DIFF,
    PURE
};

const Material BLUE_WALL = {
    1,
    Vec3d(0.95, 0, 0),
    Vec3d(0, 0, 0),
    DIFF,
    PURE
};

const Material SQUARE_WALL = {
    1,
    Vec3d(0.95, 0.95, 0.95),
    Vec3d(0, 0, 0),
    DIFF,
    SQUARE
};

const Material PIC_WALL = {
    1,
    Vec3d(0.95, 0.95, 0.95),
    Vec3d(0, 0, 0),
    DIFF,
    PIC
};

const Material PIC_FLOOR = {
    1,
    Vec3d(0.95, 0.95, 0.95),
    Vec3d(0, 0, 0),
    DIFF,
    PIC
};

const Material PIC_BALL = {
    1,
    Vec3d(0.95, 0.95, 0.95),
    Vec3d(0, 0, 0),
    MIXED,
    PIC,
    Vec3d(0.8, 0.05, 0.15)
};

const Material LIGHT_material = {
    1,
    Vec3d(0.2, 0.7, 0.7),
    Vec3d(15, 25, 25),
    DIFF, PURE
};

class Geometry {
public:
    Geometry(const Material& ma, string picname = "") : mate(ma) {
        if (mate.textype == PIC) {
            cv::imread(picname).copyTo(picture);
            picture.convertTo(picture, CV_64FC3, 1.0 / 256.0);
        }
    }
    virtual double intersect(const Ray& ray) const = 0;
    virtual Vec3d color(const Ray& ray, const double t, const Ray& out, const Vec3d& outI, const Vec3d& envI) const;
    virtual Vec3d n_vec(const Ray& ray, const double t) const = 0;
    virtual Vec3d texture(const Ray& ray, const double t) const {
        return mate.Kds;
    }
    const Material& material() const { return mate; }
    const cv::Mat& pic() const { return picture; }
    virtual bool go_in(const Ray& ray) const { return false; }
    TYPE raytype(unsigned short* Xi) const {
        if (mate.type == MIXED) {
            double scale[3] = {0, 0, 0};
            scale[0] = mate.p_of_dsr[0];
            scale[1] = scale[0] + mate.p_of_dsr[1];
            scale[2] = scale[1] + mate.p_of_dsr[2];
            scale[0] /= scale[2]; scale[1] /= scale[2];  scale[2] = 1;
            double choose = erand48(Xi);
            if (choose < scale[0])
                return DIFF;
            else if (choose < scale[1])
                return SPEC;
            else
                return REFR;
        }
        else {
            return mate.type;
        }
    }
    Point3d randpointforlight(unsigned short* Xi) const {
        return Point3d(0, 0, 0);
    }

protected:
    Material mate;
    cv::Mat picture;
};

class Plain : public Geometry {
public:
    Plain(const Vec3d& v, const Point3d& p, const Material& mate) : n(normalize(v)), o(p), Geometry(mate) {
        d = -n.dot(o);
    }
    double intersect(const Ray& ray) const;
    // Vec3d color(const Ray& ray, const double t, const Ray& out, const Ray& env) const;
    Vec3d n_vec(const Ray& ray, const double t) const;
    virtual Vec3d texture(const Ray& ray, const double t) const {
        return this->material().Kds;
    }
private:
    Vec3d n;
    Point3d o;
    double d;
};

class Sphere : public Geometry {
public:
    Sphere(const Point3d& center, const double ra, const Material& mate, string picname = "") : o(center), r(ra), Geometry(mate, picname) {}
    double intersect(const Ray& ray) const;
    Vec3d n_vec(const Ray &ray, const double t) const;
    bool go_in(const Ray &ray) const;
    void rxy(Point3d p, double& x, double& y) const;
    virtual Vec3d texture(const Ray& ray, const double t) const;
private:
    Point3d o;
    double r;
};

class Rectangle : public Geometry {
public:
    Rectangle(const Point3d& p1, const Point3d& p2, const Point3d& p3, const Point3d& p4, const Material& mate, string picname = ""): o(p1), xaxis(p2 - p1), yaxis(p3 - p2), Geometry(mate, picname) {
        xrange = norm(xaxis);
        yrange = norm(yaxis);
        xaxis = normalize(xaxis);
        yaxis = normalize(yaxis);
        n = normalize(xaxis.cross(yaxis));
    }
    double intersect(const Ray &ray) const;
    Vec3d n_vec(const Ray &ray, const double t) const;
    Vec3d texture(const Ray &ray, const double t) const;
    void rxy(Point3d xy, double& x, double& y) const;
    Point3d randpointforlight(unsigned short *Xi) const {
        double dx = erand48(Xi); double dy = erand48(Xi);
        return (o + Point3d(dx * xrange * xaxis) + Point3d(dy * yrange * yaxis));
    }
private:
    Point3d o;
    Vec3d xaxis;
    Vec3d yaxis;
    double xrange;
    double yrange;
    Vec3d n;
};

#endif // GEOMETRY_H
