#ifndef SCENE_H
#define SCENE_H
#include "camera.h"
#include "geometry.h"
#include <vector>
#include <cstdlib>
#include <cmath>

static const double MINWEIGHT = 0.01;
static const int MAXTIME = 5;

class Scene {
public:
    Scene(const Point3d& lp) : lightpoint(lp) {}
    ~Scene() {
        for (int i = 0; i < geometry_vec.size(); ++i)
            delete geometry_vec[i];
    }

    Vec3d RayTracing(const Ray& ray, double weight = MINWEIGHT, int tracetime = MAXTIME);
    Vec3d MCRayTracing(const Ray& ray, int tracetime, unsigned short* Xi);
    void AddGeometry(const Geometry* geo) {
        geometry_vec.push_back(geo);
    }

private:
    std::vector<const Geometry*> geometry_vec;
    // Point of light
    Point3d lightpoint;
};

#endif // SCENE_H
