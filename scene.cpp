#include "scene.h"
#include <iostream>


Vec3d Scene::RayTracing(const Ray& ray, double weight, int tracetime) {
    if (tracetime == 0)
        return Vec3d(0, 0, 0);

    double mint = 1e10;
    const Geometry* mingeo = NULL;
    for (int i = 0; i < geometry_vec.size(); ++i) {
        double t = geometry_vec[i]->intersect(ray);
        if (t > 0 && t < mint) {
            mint = t;
            mingeo = geometry_vec[i];
        }
    }
    if (mingeo == NULL)
        return Vec3d(0, 0, 0);
    Ray out(lightpoint, Vec3d(ray.o) + mint*ray.d - Vec3d(lightpoint));
    double outt = norm(Vec3d(ray.o) + mint*ray.d - Vec3d(lightpoint));
    bool block = false;
    for (int i = 0; i < geometry_vec.size(); ++i) {
            double t = geometry_vec[i]->intersect(out);
            if (0 < t && t < outt - EPS) {
                block = true;
                break;
            }
    }
    Vec3d localcolor = Vec3d(0, 0, 0);
    if (!block)
        localcolor = mingeo->color(ray, mint, out, Vec3d(0.8, 0.8, 0.8), Vec3d(0, 0, 0));
    // return localcolor;
    // To be continued...
    mint -= EPS; // Avoid black points!!!
    Vec3d nv = mingeo->n_vec(ray, mint);
    Ray refray(ray.p(mint), ray.d - 2 * nv.dot(ray.d) * nv);
    Vec3d refcolor = Vec3d(0, 0, 0);
    if (mingeo->material().Kso > 0) {
        refcolor = RayTracing(refray, weight * mingeo->material().Kso, tracetime - 1);
    }
    return localcolor + mingeo->material().Kso * refcolor;
}

Vec3d Scene::MCRayTracing(const Ray &ray, int tracetime, unsigned short* Xi) {
    double mint = 1e10;
    const Geometry* mingeo = NULL;
    for (int i = 0; i < geometry_vec.size(); ++i) {
        double t = geometry_vec[i]->intersect(ray);
        if (t > 0 && t < mint) {
            mint = t;
            mingeo = geometry_vec[i];
        }
    }
    if (mingeo == NULL)
        return Vec3d(0, 0, 0);
    mint -= EPS;
    Vec3d pointcolor = mingeo->texture(ray, mint);
    if (tracetime > MAXTIME) {
        double r = erand48(Xi);
        double cp = 0;
        for (int i = 0; i < 3; ++i)
            if (pointcolor[i] > cp)
                cp = pointcolor[i];
        if (r > cp)
            return mingeo->material().selflight;
    }
    if (mingeo->material().type == DIFF) {
        double theta = 2 * M_PI * erand48(Xi), rxy = erand48(Xi), rxys = sqrt(rxy);
        Vec3d nv = mingeo->n_vec(ray, mint);
        Vec3d xaxis;
        if (fabs(nv[0]) > 0.1)
            xaxis = normalize(nv.cross(Vec3d(0, 1, 0)));
        else
            xaxis = normalize(nv.cross(Vec3d(1, 0, 0)));
        Vec3d yaxis = normalize(nv.cross(xaxis));
        Vec3d d = normalize(xaxis * rxys * cos(theta) + yaxis * rxys * sin(theta) + nv * sqrt(1 - rxy));
        return (mingeo->material().selflight + pointcolor.mul(MCRayTracing(Ray(ray.p(mint), d), tracetime + 1, Xi)));
    }
    else if (mingeo->material().type == SPEC) {
        Vec3d nv = mingeo->n_vec(ray, mint);
        Ray refray(ray.p(mint), ray.d - 2 * nv.dot(ray.d) * nv);
        return (mingeo->material().selflight + pointcolor.mul(MCRayTracing(refray, tracetime + 1, Xi)));
    }
    else if (mingeo->material().type == REFR) {
        Vec3d nv = mingeo->n_vec(ray, mint);
        Ray reflray(ray.p(mint), ray.d - 2 * nv.dot(ray.d) * nv);
        bool into = mingeo->go_in(ray);
        double nr = (into) ? 1.0/mingeo->material().nf : mingeo->material().nf;
        if (nr > 1) {
            double ndotr = nv.dot(ray.d);
            if (ndotr * ndotr + (1.0 / nr) * (1.0 / nr) <= 1)
                return (mingeo->material().selflight + pointcolor.mul(MCRayTracing(reflray, tracetime + 1, Xi)));
        }
        double cosin = -ray.d.dot(nv);
        Vec3d newd = normalize(nr * (ray.d + nv * cosin) - nv * sqrt(1 - nr * nr * (1 - cosin * cosin)));
        Point3d newo = ray.p(mint + 2 * EPS);
        double nn = mingeo->material().nf, f0 = (nn - 1) * (nn - 1) / ((nn + 1) * (nn + 1)), c = 1 - (into?cosin:(-newd.dot(nv)));
        double Frt = f0 + (1 - f0) * c*c*c*c*c;
        double chooser = erand48(Xi);
        if (chooser < Frt)
            return (mingeo->material().selflight + pointcolor.mul(MCRayTracing(reflray, tracetime + 1, Xi)));
        else
            return (mingeo->material().selflight + pointcolor.mul(MCRayTracing(Ray(newo, newd), tracetime + 1, Xi)));
    }
}
