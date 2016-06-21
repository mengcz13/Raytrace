#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <cstdlib>
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
    // Camera camera(Point3d(0, -20, 0), f, right, M_PI / 4, 22.5/32.0, 22.5);
    Camera camera(Point3d(0, -20, 0), f, right, M_PI / 4, 0, 22.5);
    Scene scene(Point3d(0, 0, 3));
    // Plain* plain = new Plain(Vec3d(0, 1, 0), Point3d(0, 6, 0), SQUARE_WALL);
    Rectangle* plain = new Rectangle(Point3d(-6, 6, 6), Point3d(6, 6, 6),Point3d(6, 6, -6), Point3d(-6, 6, -6), &PIC_WALL, "ussr2.png");
    Plain* plain2 = new Plain(Vec3d(0, 0, 1), Point3d(0, 0, 6), &WHITE_WALL);
    Rectangle* plain3 = new Rectangle(Point3d(-6, 6, -6), Point3d(6, 6, -6), Point3d(6, -20, -6), Point3d(-6, -20, -6), &PIC_FLOOR, "wood2.jpg");
    Plain* plain4 = new Plain(Vec3d(1, 0, 0), Point3d(6, 0, 0), &BLUE_WALL);
    Plain* plain5 = new Plain(Vec3d(1, 0, 0), Point3d(-6, 0, 0), &RED_WALL);
    Plain* plain6 = new Plain(Vec3d(0, 1, 0), Point3d(0, -25, 0), &WHITE_WALL);
    //Sphere* sphere = new Sphere(Point3d(2, -2, -6+4), 1.5, &BALL_material_REFR);
    //Sphere* sphere2 = new Sphere(Point3d(-3, 2.5, -6+2+5), 2, &PIC_BALL, "world-physical-map.jpg");
    Rectangle* spherelight = new Rectangle(Point3d(-2, 2, 5.999), Point3d(2, 2, 5.999),Point3d(2, -2, 5.999),Point3d(-2, -2, 5.999), &LIGHT_material);
    //Block* block = new Block(Point3d(-3, 2.5 - 2.0 / sqrt(2.0), -6), Vec3d(1, 1, 0), Vec3d(-1, 1, 0), 2, 2, 5, &MARBLE_BLOCK, "marble2.jpg");
    //Block* block2 = new Block(Point3d(2, -2 - 2.0 / sqrt(2.0), -6), Vec3d(1, 1, 0), Vec3d(-1, 1, 0), 2, 2, 2.5, &MARBLE_BLOCK, "marble2.jpg");
    //ComplexObj* cobj = new ComplexObj(Point3d(1, -2, -3), 8, "fixed.perfect.dragon.100K.0.07.obj", &RED_GLASS);
    ComplexObj* cobj = new ComplexObj(Point3d(1, -2, -3), 8, "dragonp5.obj", &RED_WALL);
//    ComplexObj* cobj = new ComplexObj(Point3d(0, 0, 0), 8, "dinosaur.2k.obj", &RED_GLASS);
    scene.AddGeometry(plain);
    scene.AddGeometry(plain2);
    scene.AddGeometry(plain3);
    scene.AddGeometry(plain4);
    scene.AddGeometry(plain5);
    scene.AddGeometry(plain6);
    //scene.AddGeometry(sphere);
    //scene.AddGeometry(sphere2);
    scene.AddGeometry(spherelight);
    //scene.AddGeometry(block);
    //scene.AddGeometry(block2);
    scene.AddGeometry(cobj);
    int picsize = atoi(argv[2]);
    Mat pic(picsize, picsize, CV_64FC3);
    const double dx[4] = {0, 0.5, 0, 0.5};
    const double dy[4] = {0, 0, 0.5, 0.5};
    const int samplenum = atoi(argv[1]); const double piecesample = 1.0 / samplenum;
    int finished_pixel = 0;
    timeval starttime, endtime; double timeuse = 0, timeleft = 0, percentf = 0; int hour = 0, minute = 0;
    gettimeofday(&starttime, NULL);
#pragma omp parallel for schedule(dynamic, 1) shared(finished_pixel, starttime, endtime, timeuse, timeleft, hour, minute)
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
//#pragma omp critical
//            {
//                ++finished_pixel;
//                if (finished_pixel % 10 == 0) {
//                    gettimeofday(&endtime, NULL);
//                    timeuse = endtime.tv_sec - starttime.tv_sec + (endtime.tv_usec - starttime.tv_usec) / 1000000.0;
//                    percentf = (double)finished_pixel / (double)(pic.rows * pic.cols);
//                    timeleft = timeuse * (1.0 / percentf - 1);
//                    printf("Finished %.2f%% ...    time usage: %dh%dm%ds    timeleft: %dh%dm%ds      \r", percentf * 100.0, (int)timeuse/3600, ((int)(timeuse)%3600)/60, ((int)(timeuse)%3600)%60, (int)timeleft/3600, ((int)(timeleft)%3600)/60, ((int)(timeleft)%3600)%60);
//                    fflush(stdout);
//                }
//            }
        }
    }
    Mat picpng;
    pic.convertTo(picpng, CV_16UC3, 65535.0);
    imwrite("result.png", picpng);
    imshow("sss", picpng);
    waitKey(0);
    return 0;
}

