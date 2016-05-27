#include "scene.h"
#include <iostream>

Vec3d Scene::MCRayTracing(const Ray &ray, int tracetime, unsigned short* Xi) {
    double mint = 1e10;
    const Geometry* mingeo = NULL;
    const Geometry* fathergeo = NULL;
    for (int i = 0; i < geometry_vec.size(); ++i) {
        Geometry const* unitgeo = NULL;
        double t = geometry_vec[i]->intersect(ray, &unitgeo);
        if (t > 0 && t < mint) {
            mint = t;
            mingeo = unitgeo;
            fathergeo = geometry_vec[i];
        }
    }
    if (mingeo == NULL)
        return Vec3d(0, 0, 0);
    mint -= EPS;
    Point3d point = ray.p(mint); // Point where light hit object
    Vec3d pointcolor = fathergeo->texture(point, mingeo);
    Vec3d basecolor = fathergeo->material()->selflight;
    Vec3d nv = mingeo->n_vec(ray, point);

    if (tracetime > MAXTIME) {
        double r = erand48(Xi);
        double cp = 0;
        for (int i = 0; i < 3; ++i)
            if (pointcolor[i] > cp)
                cp = pointcolor[i];
        if (r > cp) {
//            for (int i = 0; i < geometry_vec.size(); ++i) {
//                if (geometry_vec[i]->material()->selflight != Vec3d(0, 0, 0)) {
//                    const Geometry* geo = geometry_vec[i];
//                    Point3d plight = geo->randpointforlight(Xi);
//                    double targett = norm(point - plight);
//                    Ray light(plight, point - plight, ray.into);
//                    bool blocked = false;
//                    for (int j = 0; j < geometry_vec.size(); ++j) {
//                        if ((geometry_vec[j]) != (geo)) {
//                            Geometry const* temp = NULL;
//                            double testlight = geometry_vec[j]->intersect(light, &temp);
//                            if (testlight > 0 && testlight < targett) {
//                                blocked = true;
//                                break;
//                            }
//                        }
//                    }
//                    if (!blocked) {
//                        basecolor += fathergeo->color(ray, point, light,(geo->material()->selflight), Vec3d(0, 0, 0), nv, mingeo);
//                    }
//                }
//            }
            return basecolor;
        }
    }
    TYPE raytype = fathergeo->raytype(Xi);
    if (raytype == DIFF) {
        double theta = 2 * M_PI * erand48(Xi), rxy = erand48(Xi), rxys = sqrt(rxy);
        Vec3d xaxis;
        if (fabs(nv[0]) > 0.1)
            xaxis = normalize(nv.cross(Vec3d(0, 1, 0)));
        else
            xaxis = normalize(nv.cross(Vec3d(1, 0, 0)));
        Vec3d yaxis = normalize(nv.cross(xaxis));
        Vec3d d = normalize(xaxis * rxys * cos(theta) + yaxis * rxys * sin(theta) + nv * sqrt(1 - rxy));
        return (basecolor + pointcolor.mul(MCRayTracing(Ray(point, d, ray.into), tracetime + 1, Xi)));
    }
    else if (raytype == SPEC) {
        Ray refray(point, ray.d - 2 * nv.dot(ray.d) * nv, ray.into);
        return (basecolor + pointcolor.mul(MCRayTracing(refray, tracetime + 1, Xi)));
    }
    else if (raytype == REFR) {
        Ray reflray(point, ray.d - 2 * nv.dot(ray.d) * nv, ray.into);
        bool into = ray.into;
        double nr = (into) ? 1.0/fathergeo->material()->nf : fathergeo->material()->nf;
        if (nr > 1) {
            double ndotr = nv.dot(ray.d);
            if (ndotr * ndotr + (1.0 / nr) * (1.0 / nr) <= 1)
                return (basecolor + pointcolor.mul(MCRayTracing(reflray, tracetime + 1, Xi)));
        }
        double cosin = -ray.d.dot(nv);
        Vec3d newd = normalize(nr * (ray.d + nv * cosin) - nv * sqrt(1 - nr * nr * (1 - cosin * cosin)));
        Point3d newo = ray.p(mint + 2 * EPS);
        double nn = fathergeo->material()->nf, f0 = (nn - 1) * (nn - 1) / ((nn + 1) * (nn + 1)), c = 1 - (into?cosin:(-newd.dot(nv)));
        double Frt = f0 + (1 - f0) * c*c*c*c*c;
        double chooser = erand48(Xi);
        if (chooser < Frt)
            return (basecolor + pointcolor.mul(MCRayTracing(reflray, tracetime + 1, Xi)));
        else
            return (basecolor + pointcolor.mul(MCRayTracing(Ray(newo, newd, !ray.into), tracetime + 1, Xi)));
    }
}
