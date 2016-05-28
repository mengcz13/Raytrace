#ifndef GEOMETRY_H
#define GEOMETRY_H
#include "camera.h"
#include <cstring>
#include <string>
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

const Material RED_GLASS = {
    1.5,
    Vec3d(0.05, 0.05, 0.85),
    Vec3d(0, 0, 0),
    MIXED,
    PURE,
    Vec3d(0.05, 0.15, 0.8)
};

const Material MARBLE_BLOCK = {
    1,
    Vec3d(0.95, 0.95, 0.95),
    Vec3d(0, 0, 0),
    MIXED,
    PIC,
    Vec3d(0.6, 0.4, 0)
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
    Geometry(const Material* ma = NULL, string picname = "") : mate(ma) {
        if (mate) {
            if (mate->textype == PIC) {
                cv::imread(picname).copyTo(picture);
                picture.convertTo(picture, CV_64FC3, 1.0 / 256.0);
            }
        }
    }
    virtual double intersect(const Ray& ray, Geometry const ** unitgeo) const = 0;
    virtual Vec3d color(const Ray& ray, const Point3d& point, const Ray& out, const Vec3d& outI, const Vec3d& envI, const cv::Vec3d &nv, const Geometry *unitgeo) const;
    virtual Vec3d n_vec(const Ray& ray, const Point3d& point) const = 0;
    virtual Vec3d texture(const Point3d& point, Geometry const* unitgeo) const {
        return mate->Kds;
    }
    const Material* material() const { return mate; }
    const cv::Mat& pic() const { return picture; }
    virtual bool go_in(const Ray& ray) const { return false; }
    TYPE raytype(unsigned short* Xi) const {
        if (mate->type == MIXED) {
            double scale[3] = {0, 0, 0};
            scale[0] = mate->p_of_dsr[0];
            scale[1] = scale[0] + mate->p_of_dsr[1];
            scale[2] = scale[1] + mate->p_of_dsr[2];
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
            return mate->type;
        }
    }
    virtual Point3d randpointforlight(unsigned short* Xi) const {
        return Point3d(0, 0, 0);
    }

protected:
    const Material* mate;
    cv::Mat picture;
};

class Plain : public Geometry {
public:
    Plain(const Vec3d& v, const Point3d& p, const Material* mate = NULL, string picname = "") : n(normalize(v)), o(p), Geometry(mate, picname) {
        d = -n.dot(o);
    }
    virtual double intersect(const Ray& ray, Geometry const** unitgeo) const;
    Vec3d n_vec(const Ray& ray, const Point3d& point) const;
    virtual Vec3d texture(const Point3d& point, Geometry const* unitgeo) const {
        return this->mate->Kds;
    }
protected:
    Vec3d n;
    Point3d o;
    double d;
};

class Sphere : public Geometry {
public:
    Sphere(const Point3d& center, const double ra, const Material* mate = NULL, string picname = "") : o(center), r(ra), Geometry(mate, picname) {}
    double intersect(const Ray& ray, Geometry const** unitgeo) const;
    Vec3d n_vec(const Ray &ray, const Point3d& point) const;
    // bool go_in(const Ray &ray) const;
    void rxy(const Point3d& p, double& x, double& y) const;
    virtual Vec3d texture(const Point3d& point, Geometry const* unitgeo) const;
private:
    Point3d o;
    double r;
};

class Rectangle : public Plain {
public:
    Rectangle(const Point3d& p1, const Point3d& p2, const Point3d& p3, const Point3d& p4, const Material* mate = NULL, string picname = ""): xaxis(p2 - p1), yaxis(p3 - p2), Plain(Vec3d(p2 - p1).cross(Vec3d(p3 - p1)), p1, mate, picname) {
        xrange = norm(xaxis);
        yrange = norm(yaxis);
        xaxis = normalize(xaxis);
        yaxis = normalize(yaxis);
    }
    Rectangle(const Point3d& op, const Vec3d& xx, const Vec3d& yy, double xr, double yr, const Material* mate = NULL, string picname = ""): xaxis(normalize(xx)), yaxis(normalize(yy)), xrange(xr), yrange(yr), Plain(xx.cross(yy), op, mate, picname) {}
    Rectangle(): Plain(Point3d(0, 0, 0), Vec3d(0, 0, 0)) {}
    double intersect(const Ray &ray, Geometry const** unitgeo) const;
    // Vec3d n_vec(const Ray &ray, const Point3d& point) const;
    Vec3d texture(const Point3d& point, Geometry const* unitgeo) const;
    void rxy(const Point3d& xy, double& x, double& y) const;
    Point3d randpointforlight(unsigned short *Xi) const {
        double dx = erand48(Xi); double dy = erand48(Xi);
        return (o + Point3d(dx * xrange * xaxis) + Point3d(dy * yrange * yaxis));
    }
private:
    Vec3d xaxis;
    Vec3d yaxis;
    double xrange;
    double yrange;
};

class Block : public Geometry {
public:
    Block(const Point3d& p1, const Vec3d& xx, const Vec3d& yy, double xr, double yr, double zr, const Material* mate = NULL, string picname = ""): o(p1), xaxis(normalize(xx)), yaxis(normalize(yy)), zaxis(normalize(xx.cross(yy))), xrange(xr), yrange(yr), zrange(zr), Geometry(mate, picname) {
        face[0] = Rectangle(o, xaxis, yaxis, xrange, yrange);
        face[1] = Rectangle(o, xaxis, zaxis, xrange, zrange);
        face[2] = Rectangle(o, yaxis, zaxis, yrange, zrange);
        Point3d f2fo = (Vec3d)o + xaxis * xrange + yaxis * yrange + zaxis * zrange;
        face[3] = Rectangle(f2fo, - yaxis, - zaxis, yrange, zrange);
        face[4] = Rectangle(f2fo, - xaxis, - zaxis, xrange, zrange);
        face[5] = Rectangle(f2fo, - xaxis, - yaxis, xrange, yrange);
    }
    Block(): Geometry() {}
    double intersect(const Ray &ray, Geometry const** unitgeo) const;
    Vec3d n_vec(const Ray &ray, const Point3d& point) const { std::cerr << "Wrong!" << std::endl; return Vec3d(0, 0, 0); } // Never use it
    Vec3d texture(const Point3d& point, Geometry const* unitgeo) const;
// private:
    Point3d o;
    Vec3d xaxis;
    Vec3d yaxis;
    Vec3d zaxis;
    double xrange;
    double yrange;
    double zrange;
    Rectangle face[6];
};

class Triangle : public Geometry {
public:
    Triangle(const Point3d& p1, const Point3d& p2, const Point3d& p3, const Material* mate = NULL, string picname = "");
    double intersect(const Ray &ray, Geometry const** unitgeo) const;
    Vec3d n_vec(const Ray &ray, const Point3d& point) const;
    Vec3d texture(const Point3d& point, Geometry const* unitgeo) const;
    double axis_max(int i) {
        return corange[i][1];
    }
    double axis_min(int i) {
        return corange[i][0];
    }
    double mid_v(int i) {
        return midp[i];
    }

private:
    Point3d vertex[3];
    Vec3d midp;
    Vec3d n;
    double corange[3][2];
    double d;
};

struct KdTreeNode {
    Block box;
    std::vector<Triangle*> triangle_vec;
    std::vector<Triangle*> triangle_left;
    KdTreeNode* lc;
    KdTreeNode* rc;
    int depth; // 0: x 1: y 2: z
    KdTreeNode(): lc(NULL), rc(NULL), depth(0) {}
};

class ComplexObj : public Geometry {
public:
    ComplexObj(const Point3d& o, double scaleto, string objfile, const Material* mate, string picname = "");
    ~ComplexObj() {
        delete []nodepool;
    }
    double intersect(const Ray &ray, const Geometry **unitgeo) const;
    Vec3d n_vec(const Ray &ray, const cv::Point3d &point) const;
    Vec3d texture(const cv::Point3d &point, const Geometry *unitgeo) const;

private:
    std::vector<Point3d> vertex_vec;
    std::vector<std::vector<int> > vnum;
    std::vector<Triangle> face_vec;
    Point3d o; // center of object
    KdTreeNode* root;
    KdTreeNode* nodepool;
    int top;
    double prange[3][2];
    void parser(string objfile, double scaleto);
    void build_kdtree(KdTreeNode* node);
    double search_kdtree(const Ray& ray, const Geometry **unitgeo, const KdTreeNode* node) const;
};

struct Triangle_by_xmid {
    bool operator() (Triangle* ta, Triangle* tb) {
        return ta->mid_v(0) < tb->mid_v(0);
    }
};

struct Triangle_by_ymid {
    bool operator() (Triangle* ta, Triangle* tb) {
        return ta->mid_v(1) < tb->mid_v(1);
    }
};

struct Triangle_by_zmid {
    bool operator() (Triangle* ta, Triangle* tb) {
        return ta->mid_v(2) < tb->mid_v(2);
    }
};

#endif // GEOMETRY_H
