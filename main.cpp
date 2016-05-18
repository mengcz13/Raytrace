#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "camera.h"
#include "geometry.h"
#include "scene.h"

using namespace std;
using namespace cv;

Vec3d truncvec(const Vec3d& vec) {
    return Vec3d(vec[0]>1?1:vec[0],vec[1]>1?1:vec[1],vec[2]>1?1:vec[2]);
}

int main(int argc, char** argv)
{
    Vec3d f = Vec3d(0, 1, 0);
    Vec3d right = f.cross(Vec3d(0, 0, 1));
    Camera camera(Point3d(0, -20, 0), f, right, M_PI / 4, 22.5/28.0, 22.5);
    Scene scene(Point3d(0, 0, 3));
    Plain* plain = new Plain(Vec3d(0, 1, 0), Point3d(0, 6, 0), TEST_material_SQ);
    Plain* plain2 = new Plain(Vec3d(0, 0, 1), Point3d(0, 0, 6), TEST_material4);
    Plain* plain3 = new Plain(Vec3d(0, 0, 1), Point3d(0, 0, -6), TEST_material4);
    Plain* plain4 = new Plain(Vec3d(1, 0, 0), Point3d(6, 0, 0), TEST_material2);
    Plain* plain5 = new Plain(Vec3d(1, 0, 0), Point3d(-6, 0, 0), TEST_material3);
    //Plain* plain6 = new Plain(Vec3d(0, 1, 0), Point3d(0, -10, 0), TEST_material4);
    Sphere* sphere = new Sphere(Point3d(2, -2, -6+2.3), 2.3, BALL_material_REFR);
    Sphere* sphere2 = new Sphere(Point3d(-3, 2.5, -4), 2, BALL_material);
    Sphere* spherelight = new Sphere(Point3d(0, 0, 6+100), 100+0.015, LIGHT_material);
    scene.AddGeometry(plain);
    scene.AddGeometry(plain2);
    scene.AddGeometry(plain3);
    scene.AddGeometry(plain4);
    scene.AddGeometry(plain5);
    //scene.AddGeometry(plain6);
    scene.AddGeometry(sphere);
    scene.AddGeometry(sphere2);
    scene.AddGeometry(spherelight);
    Mat pic(512, 512, CV_64FC3);
    const double dx[4] = {0, 0.5, 0, 0.5};
    const double dy[4] = {0, 0, 0.5, 0.5};
    const int samplenum = 5000; const double piecesample = 1.0 / samplenum;
#pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < pic.rows; ++i) {
        unsigned short Xi[3] = {0, 0, i * i * i};
        for (int j = 0; j < pic.cols; ++j) {
            Vec3d*p = pic.ptr<Vec3d>(i);
            p[j] = Vec3d(0, 0, 0);
            for (int k = 0; k < 4; ++k) {
                Vec3d tempv(0, 0, 0);
                double x = ((double)j + dx[k]) / pic.cols;
                double y = ((double)i + dy[k]) / pic.rows;
                for (int snum = 0; snum < samplenum; ++snum) {
                    double sx = x + (erand48(Xi) / (2 * pic.cols));
                    double sy = y + (erand48(Xi) / (2 * pic.rows));
                    tempv += ((scene.MCRayTracing(camera.ray_direction(sx, sy, Xi), 0, Xi)) * piecesample);
                }
                p[j] += truncvec(tempv) * 0.25;
            }
        }
    }
    Mat picpng;
    pic.convertTo(picpng, CV_16UC3, 65535.0);
    imwrite("result.png", picpng);
    imshow("sss", picpng);
    waitKey(0);
    return 0;
}

