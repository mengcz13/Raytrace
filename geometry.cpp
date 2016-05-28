#include "geometry.h"
#include <cmath>
#include <iostream>
#include <cstdio>
#include <algorithm>
using namespace std;


Vec3d Geometry::color(const Ray& ray, const Point3d& point, const Ray& out, const Vec3d& outI, const Vec3d& envI, const Vec3d& nv, Geometry const* unitgeo) const {
    Vec3d h = normalize(-ray.d - out.d);
    Vec3d tex = texture(point, unitgeo);
    return outI.mul( tex * (-out.d.dot(nv)) );
}


double Plain::intersect(const Ray &ray, Geometry const** unitgeo) const {
    // n.dot(ray.o)+t*(n.dot(ray.d)) + d = 0;
    double divu = n.dot(ray.d);
    (*unitgeo) = NULL;
    if (fabs(divu) < EPS) {
        return -1;
    }
    else {
        (*unitgeo) = this;
        return (-d - n.dot(ray.o)) / divu;
    }
}

Vec3d Plain::n_vec(const Ray& ray, const Point3d& point) const {
    if (ray.d.dot(n) < 0)
        return n;
    else
        return (-n);
}

double Sphere::intersect(const Ray& ray, Geometry const** unitgeo) const {
    double td = (o - ray.o).dot(ray.d);
    double roo = norm(o - ray.o);
    (*unitgeo) = NULL;
    if (roo > r && td < 0)
        return -1;
    double t2 = r * r - roo * roo + td * td;
    if (t2 < 0)
        return -1;
    else {
        double tt = sqrt(t2);
        if (td - tt > 0) {
            (*unitgeo) = this;
            return td - tt;
        }
        else if (td + tt > 0) {
            (*unitgeo) = this;
            return td + tt;
        }
        else
            return -1;
    }
}

Vec3d Sphere::n_vec(const Ray &ray, const Point3d& point) const {
    Vec3d nv = cv::normalize(Vec3d(point - this->o));
    if (ray.d.dot(nv) < 0)
        return nv;
    else
        return (-nv);
}

void Sphere::rxy(const Point3d& p, double &x, double &y) const {
    Vec3d direction = normalize(Vec3d(p - o));
    double phi = asin(direction[2]);
    double theta = acos(direction[0] / sqrt(direction[1] * direction[1] + direction[0] * direction[0]));
    if (direction[1] < 0)
        theta = 2 * M_PI - theta;
    x = theta / (2 * M_PI);
    y = 0.5 - phi / M_PI;
}

Vec3d Sphere::texture(const Point3d& point, Geometry const* unitgeo) const {
    if (this->mate->textype == PURE)
        return this->mate->Kds;
    else {
        double rx = 0, ry = 0;
        rxy(point, rx, ry);
        if (this->mate->textype == SQUARE) {
            int tx = (int)(rx / 0.1), ty = (int)(ry / 0.1);
            if ((abs(tx) & 1) ^ (abs(ty) & 1)) {
                return Vec3d(0.01, 0.01, 0.01);
            }
            else {
                return Vec3d(1, 1, 1);
            }
        }
        else if (this->mate->textype == PIC) {
            double col = rx * (this->picture.cols - 1), row = ry * (this->picture.rows - 1);
            return this->picture.ptr<Vec3d>((int)row)[(int)col];
        }
    }
}

double Rectangle::intersect(const Ray &ray, Geometry const** unitgeo) const {
    double t = this->Plain::intersect(ray, unitgeo);
    if (t < 0)
        return t;
    double rx = 0, ry = 0;
    rxy(ray.p(t), rx, ry);
    if (0 < rx && rx < 1 && 0 < ry && ry < 1) {
        (*unitgeo) = this;
        return t;
    }
    else
        return -1;
}

//Vec3d Rectangle::n_vec(const Ray &ray, const Point3d& point) const {
//    if (ray.d.dot(n) < 0)
//        return n;
//    else
//        return (-n);
//}

Vec3d Rectangle::texture(const Point3d& point, Geometry const* unitgeo) const {
    if (this->mate->textype == PURE)
        return this->mate->Kds;
    else if (this->mate->textype == SQUARE) {
        double rx = 0, ry = 0;
        rxy(point, rx, ry);
        int xd = abs((int)(rx / 0.1)) & 1, yd = abs((int)(ry / 0.1)) & 1;
        if (xd ^ yd)
            return Vec3d(0.01, 0.01, 0.01);
        else
            return Vec3d(1, 1, 1);
    }
    else if (this->mate->textype == PIC) {
        double rx = 0, ry = 0;
        rxy(point, rx, ry);
        double col = rx * (this->picture.cols - 1), row = ry * (this->picture.rows - 1);
        return this->picture.ptr<Vec3d>((int)row)[(int)col];
    }
}

void Rectangle::rxy(const Point3d& xy, double &x, double &y) const {
    Vec3d oxy = xy - o;
    x = oxy.dot(xaxis) / xrange;
    y = oxy.dot(yaxis) / yrange;
}

double Block::intersect(const Ray &ray, const Geometry **unitgeo) const {
    double tmm[3][2];
    int mark[3][2];
    for (int i = 0; i < 3; ++i) {
        double pair1 = face[i].Plain::intersect(ray, unitgeo);
        double pair2 = face[5 - i].Plain::intersect(ray, unitgeo);
        if (unitgeo == NULL) {
            tmm[i][0] = -1e11; tmm[i][1] = 1e11;
            mark[i][0] = mark[i][1] = -1;
        }
        else {
            if (pair1 < pair2) {
                tmm[i][0] = pair1; tmm[i][1] = pair2; mark[i][0] = i; mark[i][1] = 5 - i;
            }
            else {
                tmm[i][1] = pair1; tmm[i][0] = pair2; mark[i][0] = 5 - i; mark[i][1] = i;
            }
        }
    }
    double tmaxofmin = -1e10, tminofmax = 1e10; int fmin = -1, fmax = -1;
    for (int i = 0; i < 3; ++i) {
        if (tmm[i][0] > tmaxofmin) {
            tmaxofmin = tmm[i][0]; fmin = mark[i][0];
        }
        if (tmm[i][1] < tminofmax) {
            tminofmax = tmm[i][1]; fmax = mark[i][1];
        }
    }
    if (0 < tmaxofmin && tmaxofmin < tminofmax) {
        (*unitgeo) = &face[fmin];
        return tmaxofmin;
    }
    else if (tmaxofmin < 0 && 0 < tminofmax) {
        (*unitgeo) = &face[fmax];
        return tminofmax;
    }
    else {
        (*unitgeo) = NULL;
        return -1;
    }
}

Vec3d Block::texture(const Point3d& point, Geometry const* unitgeo) const {
    if (this->mate->textype != PIC) {
        return this->mate->Kds;
    }
    else {
        Rectangle const* select = NULL;
        for (int i = 0; i < 6; ++i) {
            if (&face[i] == unitgeo) {
                select = &face[i];
                break;
            }
        }
        double rx = 0, ry = 0;
        select->rxy(point, rx, ry);
        double col = rx * (this->picture.cols - 1), row = ry * (this->picture.rows - 1);
        return this->picture.ptr<Vec3d>((int)row)[(int)col];
    }
}

Triangle::Triangle(const cv::Point3d &p1, const cv::Point3d &p2, const cv::Point3d &p3, const Material* mate, string picname): Geometry(mate, picname) {
    vertex[0] = p1; vertex[1] = p2; vertex[2] = p3;
    n = normalize(Vec3d(p2 - p1).cross(Vec3d(p3 - p1)));
    for (int i = 0; i < 3; ++i) {
        corange[i][0] = 1e10;
        corange[i][1] = -1e10;
    }
    for (int i = 0; i < 3; ++i) {
        if (vertex[i].x < corange[0][0])
            corange[0][0] = vertex[i].x;
        if (vertex[i].x > corange[0][1])
            corange[0][1] = vertex[i].x;

        if (vertex[i].y < corange[1][0])
            corange[1][0] = vertex[i].y;
        if (vertex[i].y > corange[1][1])
            corange[1][1] = vertex[i].y;

        if (vertex[i].z < corange[2][0])
            corange[2][0] = vertex[i].z;
        if (vertex[i].z > corange[2][1])
            corange[2][1] = vertex[i].z;
    }
    d = -vertex[0].dot(n);
    midp = ((Vec3d)vertex[0] + (Vec3d)vertex[1] + (Vec3d)vertex[2]) / 3;
}

double Triangle::intersect(const Ray &ray, const Geometry **unitgeo) const {
    double divu = n.dot(ray.d);
    (*unitgeo) = NULL;
    if (fabs(divu) < EPS) {
        return -1;
    }
    else {
        (*unitgeo) = this;
        double t = (-d - n.dot(ray.o)) / divu;
        Point3d ppoint = ray.p(t);
        Vec3d pa = Vec3d(vertex[0] - ppoint);
        Vec3d pb = Vec3d(vertex[1] - ppoint);
        Vec3d pc = Vec3d(vertex[2] - ppoint);
        Vec3d ab = pa.cross(pb), bc = pb.cross(pc), ca = pc.cross(pa);
        double da = ab.dot(n), db = bc.dot(n), dc = ca.dot(n);
        if (da > 0 && db > 0 && dc > 0 || da < 0 && db < 0 && dc < 0) {
            (*unitgeo) = this;
            return t;
        }
        else
            return -1;
    }
}

Vec3d Triangle::n_vec(const Ray &ray, const cv::Point3d &point) const {
    if (ray.d.dot(n) < 0)
        return n;
    else
        return (-n);
}

Vec3d Triangle::texture(const cv::Point3d &point, const Geometry *unitgeo) const {
    return this->mate->Kds;
}

ComplexObj::ComplexObj(const cv::Point3d &o, double scaleto, string objfile, const Material* mate, string picname): o(o), Geometry(mate, picname) {
    for (int i = 0; i < 3; ++i) {
        prange[i][0] = 1e10;
        prange[i][1] = -1e10;
    }
    parser(objfile, scaleto);
    nodepool = new KdTreeNode[face_vec.size() * 32]; top = 0;
    root = &nodepool[top++];
    Point3d newo = o + Point3d(1, 0, 0) * prange[0][0] + Point3d(0, 1, 0) * prange[1][0] + Point3d(0, 0, 1) * prange[2][0];
    root->box = Block(newo, Vec3d(1, 0, 0), Vec3d(0, 1, 0), prange[0][1] - prange[0][0], prange[1][1] - prange[1][0], prange[2][1] - prange[2][0]);
    for (int i = 0; i < face_vec.size(); ++i)
        root->triangle_vec.push_back(&face_vec[i]);
    root->depth = 0;
    build_kdtree(root);
}

double ComplexObj::intersect(const Ray &ray, const Geometry **unitgeo) const {
    return search_kdtree(ray, unitgeo, root);
}

Vec3d ComplexObj::n_vec(const Ray &ray, const cv::Point3d &point) const {
    return Vec3d(0, 0, 0);
}

Vec3d ComplexObj::texture(const cv::Point3d &point, const Geometry *unitgeo) const {
    return mate->Kds;
}

void ComplexObj::parser(string objfile, double scaleto) {
    FILE* objf = fopen(objfile.c_str(), "r");
    char buffer[200];
    memset(buffer, 0, sizeof(char) * 200);
    while (!feof(objf)) {
        fgets(buffer, 200, objf);
        if (buffer[0] == 'v') {
            double p[3];
            sscanf(buffer, "v %lf %lf %lf", &p[0], &p[2], &p[1]); p[1] = -p[1];
            for (int k = 0; k < 3; ++k) {
                if (p[k] < prange[k][0])
                    prange[k][0] = p[k];
                if (p[k] > prange[k][1])
                    prange[k][1] = p[k];
            }
            vertex_vec.push_back(Point3d(p[0], p[1], p[2]));
        }
        else if (buffer[0] == 'f') {
            vector<int> temp(3, 0);
            sscanf(buffer, "f %d %d %d", &temp[0], &temp[1], &temp[2]);
            temp[0] -= 1; temp[1] -= 1; temp[2] -= 1;
            vnum.push_back(vector<int>(temp));
        }
    }
    fclose(objf);
    double maxaxis = -1e10;
    for (int i = 0; i < 3; ++i) {
        if (fabs(prange[i][1] - prange[i][0]) > maxaxis)
            maxaxis = fabs(prange[i][1] - prange[i][0]);
    }
    double rate = scaleto / maxaxis;
    for (int i = 0; i < 3; ++i) {
        prange[i][0] *= rate; prange[i][1] *= rate;
    }
    for (int i = 0; i < vertex_vec.size(); ++i) {
        vertex_vec[i] *= rate;
    }
    for (int i = 0; i < vnum.size(); ++i) {
        const vector<int>& v = vnum[i];
        face_vec.push_back(Triangle(o + vertex_vec[v[0]], o + vertex_vec[v[1]], o + vertex_vec[v[2]]));
    }
}

void ComplexObj::build_kdtree(KdTreeNode* node) {

    if (node->triangle_vec.size() < 8 || node->depth > 16) {
        // node->triangle_left = node->triangle_vec;
        return;
    }
    int divide_axis = node->depth % 3;
    node->lc = &nodepool[top++];
    node->rc = &nodepool[top++];
    node->lc->depth = node->rc->depth = node->depth + 1;
    Block& block = node->box;
    double pivot = 0;
    if (divide_axis == 0) {
        sort(node->triangle_vec.begin(), node->triangle_vec.end(), Triangle_by_xmid());
        pivot = node->triangle_vec[node->triangle_vec.size() / 2]->mid_v(divide_axis);
        //pivot = block.o.x + 0.5 * block.xrange;
        Vec3d moveax = Vec3d(1, 0, 0);
        node->lc->box = Block(block.o, block.xaxis, block.yaxis, pivot - block.o.x, block.yrange, block.zrange);
        node->rc->box = Block(block.o + Point3d(moveax * (pivot - block.o.x)), block.xaxis, block.yaxis, block.xrange - (pivot - block.o.x), block.yrange, block.zrange);
    }
    else if (divide_axis == 1) {
        sort(node->triangle_vec.begin(), node->triangle_vec.end(), Triangle_by_ymid());
        pivot = node->triangle_vec[node->triangle_vec.size() / 2]->mid_v(divide_axis);
        //pivot = block.o.y + 0.5 * block.yrange;
        Vec3d moveax = Vec3d(0, 1, 0);
        node->lc->box = Block(block.o, block.xaxis, block.yaxis, block.xrange, pivot - block.o.y, block.zrange);
        node->rc->box = Block(block.o + Point3d(moveax * (pivot - block.o.y)), block.xaxis, block.yaxis, block.xrange, block.yrange - (pivot - block.o.y), block.zrange);
    }
    else if (divide_axis == 2) {
        sort(node->triangle_vec.begin(), node->triangle_vec.end(), Triangle_by_zmid());
        pivot = node->triangle_vec[node->triangle_vec.size() / 2]->mid_v(divide_axis);
        //pivot = block.o.z + 0.5 * block.zrange;
        Vec3d moveax = Vec3d(0, 0, 1);
        node->lc->box = Block(block.o, block.xaxis, block.yaxis, block.xrange, block.yrange, pivot - block.o.z);
        node->rc->box = Block(block.o + Point3d(moveax * (pivot - block.o.z)), block.xaxis, block.yaxis, block.xrange, block.yrange, block.zrange - (pivot - block.o.z));
    }
    //std::cout << pivot << ' ' << divide_axis << std::endl;
    //std::cout << block.xrange << std::endl;
    for (int i = 0; i < node->triangle_vec.size(); ++i) {
        Triangle* pt = node->triangle_vec[i];
        if (pt->axis_max(divide_axis) < pivot)
            node->lc->triangle_vec.push_back(pt);
        else if (pt->axis_min(divide_axis) > pivot)
            node->rc->triangle_vec.push_back(pt);
        else {
            node->lc->triangle_vec.push_back(pt);
            node->rc->triangle_vec.push_back(pt);
        }
    }
    //std::cout << node->triangle_vec.size() << ' ' << node->lc->triangle_vec.size() << ' ' << node->rc->triangle_vec.size() << std::endl;
    build_kdtree(node->lc);
    build_kdtree(node->rc);
}

double ComplexObj::search_kdtree(const Ray &ray, const Geometry **unitgeo, const KdTreeNode *node) const {
    if (node == NULL) {
        *unitgeo = NULL;
        return -1;
    }
    const Geometry* temp;
    double t = node->box.intersect(ray, &temp);
    if (t < 0) {
        *unitgeo = NULL;
        return -1;
    }
    else if (node->lc == NULL && node->rc == NULL) {
        const Geometry* markp = NULL; double mint = 1e10; const Geometry* rec = NULL;
        for (int i = 0; i < node->triangle_vec.size(); ++i) {
            double tt = node->triangle_vec[i]->intersect(ray, &markp);
            if (tt > 0 && tt < mint) {
                mint = tt;
                rec = markp;
            }
        }
        if (rec == NULL) {
            *unitgeo = NULL;
            return -1;
        }
        else {
            *unitgeo = rec;
            return mint;
        }
    }
    else {
        const Geometry* lres = NULL; const Geometry* rres = NULL;
        double lt = search_kdtree(ray, &lres, node->lc);
        double rt = search_kdtree(ray, &rres, node->rc);
        if (lt > 0 && rt > 0) {
            if (lt < rt) {
                *unitgeo = lres;
                return lt;
            }
            else {
                *unitgeo = rres;
                return rt;
            }
        }
        else if (lt > 0) {
            *unitgeo = lres;
            return lt;
        }
        else if (rt > 0) {
            *unitgeo = rres;
            return rt;
        }
        else {
            *unitgeo = NULL;
            return -1;
        }
    }
//    else {
//        const Geometry* markp = NULL; double mint = 1e10; const Geometry* rec = NULL;
//        for (int i = 0; i < node->triangle_left.size(); ++i) {
//            double tt = node->triangle_left[i]->intersect(ray, &markp);
//            if (tt > 0 && tt < mint) {
//                mint = tt;
//                rec = markp;
//            }
//        }
//        if (rec == NULL) {
//            mint = -1;
//        }
//        const Geometry* lres = NULL; const Geometry* rres = NULL;
//        double lt = -1, rt = -1;
//        if (node->lc)
//            lt = search_kdtree(ray, &lres, node->lc);
//        if (node->rc)
//            rt = search_kdtree(ray, &rres, node->rc);
//        if (mint > 0 && (mint < lt || lt < 0) && (mint < rt || rt < 0)) {
//            *unitgeo = rec;
//            return mint;
//        }
//        else if (lt > 0 && (lt < mint || mint < 0) && (lt < rt || rt < 0)) {
//            *unitgeo = lres;
//            return lt;
//        }
//        else if (rt > 0 && (rt < mint || mint < 0) && (rt < lt || lt < 0)) {
//            *unitgeo = rres;
//            return rt;
//        }
//        else {
//            *unitgeo = NULL;
//            return -1;
//        }
//    }
}
