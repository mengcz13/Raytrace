#include "scene.h"
#include <iostream>

/*
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
*/

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
    Vec3d basecolor = mingeo->material().selflight;

    if (tracetime > MAXTIME) {
        double r = erand48(Xi);
        double cp = 0;
        for (int i = 0; i < 3; ++i)
            if (pointcolor[i] > cp)
                cp = pointcolor[i];
        if (r > cp) {
            for (int i = 0; i < geometry_vec.size(); ++i) {
                if (geometry_vec[i]->material().selflight != Vec3d(0, 0, 0)) {
                    const Geometry* geo = geometry_vec[i];
                    Point3d plight = geo->randpointforlight(Xi);
                    double targett = norm(ray.p(mint) - plight);
                    Ray light(plight, ray.p(mint) - plight, ray.into);
                    bool blocked = false;
                    for (int j = 0; j < geometry_vec.size(); ++j) {
                        if ((geometry_vec[j]) != (geo)) {
                            double testlight = geometry_vec[j]->intersect(light);
                            if (testlight > 0 && testlight < targett) {
                                blocked = true;
                                break;
                            }
                        }
                    }
                    if (!blocked) {
                        basecolor += mingeo->color(ray, mint, light,(geo->material().selflight), Vec3d(0, 0, 0));
                    }
                }
            }
            return basecolor;
        }
    }
    TYPE raytype = mingeo->raytype(Xi);
    if (raytype == DIFF) {
        double theta = 2 * M_PI * erand48(Xi), rxy = erand48(Xi), rxys = sqrt(rxy);
        Vec3d nv = mingeo->n_vec(ray, mint);
        Vec3d xaxis;
        if (fabs(nv[0]) > 0.1)
            xaxis = normalize(nv.cross(Vec3d(0, 1, 0)));
        else
            xaxis = normalize(nv.cross(Vec3d(1, 0, 0)));
        Vec3d yaxis = normalize(nv.cross(xaxis));
        Vec3d d = normalize(xaxis * rxys * cos(theta) + yaxis * rxys * sin(theta) + nv * sqrt(1 - rxy));
        // Direct light item
//        Vec3d dirlight = Vec3d(0, 0, 0);
//        for (int i = 0; i < geometry_vec.size(); ++i) {
//            if (geometry_vec[i]->material().selflight != Vec3d(0, 0, 0)) {
//                const Geometry* geo = geometry_vec[i];
//                Point3d plight = geo->randpointforlight(Xi);
//                double targett = norm(ray.p(mint) - plight);
//                Ray light(plight, ray.p(mint) - plight, ray.into);
//                bool blocked = false;
//                for (int j = 0; j < geometry_vec.size(); ++j) {
//                    if ((geometry_vec[j]) != (geo)) {
//                        double testlight = geometry_vec[j]->intersect(light);
//                        if (testlight > 0 && testlight < targett) {
//                            blocked = true;
//                            break;
//                        }
//                    }
//                }
//                if (!blocked)
//                    dirlight += pointcolor.mul(geo->material().selflight);
//            }
//        }
        return (basecolor + pointcolor.mul(MCRayTracing(Ray(ray.p(mint), d, ray.into), tracetime + 1, Xi)));
    }
    else if (raytype == SPEC) {
        Vec3d nv = mingeo->n_vec(ray, mint);
        Ray refray(ray.p(mint), ray.d - 2 * nv.dot(ray.d) * nv, ray.into);
        return (basecolor + pointcolor.mul(MCRayTracing(refray, tracetime + 1, Xi)));
    }
    else if (raytype == REFR) {
        Vec3d nv = mingeo->n_vec(ray, mint);
        Ray reflray(ray.p(mint), ray.d - 2 * nv.dot(ray.d) * nv, ray.into);
        bool into = ray.into;
        double nr = (into) ? 1.0/mingeo->material().nf : mingeo->material().nf;
        if (nr > 1) {
            double ndotr = nv.dot(ray.d);
            if (ndotr * ndotr + (1.0 / nr) * (1.0 / nr) <= 1)
                return (basecolor + pointcolor.mul(MCRayTracing(reflray, tracetime + 1, Xi)));
        }
        double cosin = -ray.d.dot(nv);
        Vec3d newd = normalize(nr * (ray.d + nv * cosin) - nv * sqrt(1 - nr * nr * (1 - cosin * cosin)));
        Point3d newo = ray.p(mint + 2 * EPS);
        double nn = mingeo->material().nf, f0 = (nn - 1) * (nn - 1) / ((nn + 1) * (nn + 1)), c = 1 - (into?cosin:(-newd.dot(nv)));
        double Frt = f0 + (1 - f0) * c*c*c*c*c;
        double chooser = erand48(Xi);
        if (chooser < Frt)
            return (basecolor + pointcolor.mul(MCRayTracing(reflray, tracetime + 1, Xi)));
        else
            return (basecolor + pointcolor.mul(MCRayTracing(Ray(newo, newd, !ray.into), tracetime + 1, Xi)));
    }
}
