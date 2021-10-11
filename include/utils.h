#ifndef UTILS_H
#define UTILS_H
#define CONVHULL_3D_ENABLE

#include <cmath>
#include <ctime>
#include <tuple>
#include <vector>
#include <stack>
#include "ros/ros.h"

#include "convexhull_3d.h"

using namespace std;


const double INF = 1e9;
const double EPS = 1e-6;

double LIDAR_X = 0.0;
double LIDAR_Y = 0.0;
double LIDAR_Z = 1.7;
// double minX = -20.0;
// double maxX = 40.0;
// double minY = -30.0;
// double maxY = 30.0;
double minX = -20.0;
double maxX = 40.0;
double minY = -30.0;
double maxY = 30.0;
double minZ = 0.5;
double maxZ = 2.0;
double resolution_x = 1.0;
double resolution_y = 1.0;
double resolution_z = 0.1;

const int GRID_X = int((maxX-minX+EPS)/resolution_x);
const int GRID_Y = int((maxY-minY+EPS)/resolution_y);
const int GRID_Z = int((maxZ-minZ+EPS)/resolution_z);

const double ALPHA_L = 0.5;
const double ALPHA_W = 0.5;
const double ALPHA_H = 0.5;
const double ALPHA_P = 0.5;
const double ALPHA_O = 1.0;
const double ALPHA_F = 10.0;
const double ALPHA_D = 0.3;
const double INIT_LR = 5e-2;
const double DECAY_K = 5e-2;

struct pixel3d{
    int x, y, z;
    pixel3d(): pixel3d(0,0,0){}
    pixel3d(int x, int y, int z): x(x), y(y), z(z){}
};

struct point{
    double x,y,z;
    double theta;
    bool visited;
    vector<double> scores;
    double depth;
    int px, py;
    int cluster_id;
    point() : point(0,0,0){}
    point(double x, double y, double z): x(x), y(y), z(z), theta(0.0), visited(false){}
    void update(point p){
        theta = atan2(y-p.y,x-p.x);
    }
    bool operator <(const point& o){
        if(abs(theta-o.theta)>EPS) return theta<o.theta;
        if(abs(y-o.y)>EPS) return y<o.y;
        return x<o.x;
    }
    point operator +(const point &o) const{return point(x+o.x, y+o.y, z+o.z);}
    point operator -(const point &o) const{return point(x-o.x, y-o.y, z-o.z);}
    point operator -() const{return point(-x, -y, -z);}
    double operator * (const point &o) const{ return x*o.x + y*o.y + z*o.z;}
    point operator *(const double t) const{return point(x*t, y*t, z*t);}
};

point pixel2point(pixel3d p){
    return point(p.x * resolution_x + minX, p.y * resolution_y + minY, p.z * resolution_z + minZ);
}

pixel3d point2pixel(point p){
    return pixel3d((p.x - minX)/resolution_x, (p.y - minY)/resolution_y, (p.z - minZ)/resolution_z);
}

bool is_valid_pixel(pixel3d p){
    return p.x >= 0 && p.x < GRID_X && p.y >=0 && p.y < GRID_Y && p.z >=0 && p.z < GRID_Z;
}

double norm3d(point p){
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

point cross(point u, point v){
    return point(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x);
}

bool ccw(point A, point B, point C){
    return (B.x - A.x)*(C.y - A.y) - (C.x - A.x)*(B.y - A.y) > 0;
}

point normalize(point p){
    double mag = norm3d(p);
    return point(p.x / mag, p.y / mag, p.z / mag);
}


string zfill(int n){
    if(n==0) return "000000";
    int digit = log10(n) + 1;
    string rt = "";
    for(int i=0;i<6-digit;i++) rt += "0";
    return rt+to_string(n);
}

struct face{
    point A,B,C;
    point n;
    double a,b,c,d;
    double la, lb, lc;
    double area;

    face(point A, point B, point C):A(A), B(B), C(C){
        area = 0.5 * norm3d(cross(A-B, A-C));
        n = normalize(cross(A-B, A-C));
        // cout << "area: " << area << endl;
        // cout << "point" << endl;
        // cout << A.x << " " << A.y << " " << A.z << endl;
        // cout << B.x << " " << B.y << " " << B.z << endl;
        // cout << C.x << " " << C.y << " " << C.z << endl;
        // cout << "a-b and a-c" << endl;
        // cout << (A-B).x << " " << (A-B).y << " " << (A-B).z << endl;
        // cout << (A-C).x << " " << (A-C).y << " " << (A-C).z << endl;
        // cout << "normal vector" << endl;
        // cout << n.x << " " << n.y << " " << n.z << endl;
        a = n.x;
        b = n.y;
        c = n.z;
        d = (a * A.x + b * A.y + c * A.z);
        // ax+by+cz = d

        la = norm3d(B-C);
        lb = norm3d(C-A);
        lc = norm3d(A-B);
    }

    bool on_face(point p){
        // assume point p on the plane
        double parea = norm3d(cross(p-A, p-B)) + norm3d(cross(p-B, p-C)) + norm3d(cross(p-C, p-A));
        parea *= 0.5;
        if (abs(parea - area) < EPS) return true;
        return false;
    }

    double distance2face(point p){
        double t = a * p.x + b * p.y + c * p.z - d;
        point ph = point(p.x - a * t, p.y - b * t, p.z - c * t);

        if (on_face(ph)) return t;

        double tt = INF;
        tt = min<double>(tt, norm3d(ph - A));
        tt = min<double>(tt, norm3d(ph - B));
        tt = min<double>(tt, norm3d(ph - C));

        double tmp;
        tmp = (ph-B) * (A-B) / lc;
        if (tmp >= 0 && tmp <= lc) tt = min<double>(tt, norm3d(cross(ph-B, A-B))/lc);

        tmp = (ph-C) * (B-C) / la;
        if (tmp >= 0 && tmp <= la) tt = min<double>(tt, norm3d(cross(ph-C, B-C))/la);

        tmp = (ph-A) * (C-A) / lb;
        if (tmp >= 0 && tmp <= lb) tt = min<double>(tt, norm3d(cross(ph-A, C-A))/lb);

        return sqrt(t*t + tt * tt);    
    }
};

struct bounding_box{
    double x;
    double y;
    double z;
    double yaw;
    double l;
    double w;
    double h;
    int type;

    bounding_box(): bounding_box(1){}
    bounding_box(int type): type(type){
        this->yaw = 0.0;
        if(type == 0){
            this->l = 1.0;
            this->w = 1.0;
            this->h = 1.6;
        }
        if(type == 1){
            this->l = 3.0;
            this->w = 1.0;
            this->h = 1.0;
        }
        if(type == 2){
            this->l = 5.0;
            this->w = 2.0;
            this->h = 1.5;
        }
        if(type == 3){
            this->l = 3.0;
            this->w = 1.0;
            this->h = 1.0;
        }
        if(type == 5){
            this->l = 10.0;
            this->w = 2.0;
            this->h = 2.5;
        }
        if(type == 7){
            this->l = 8.0;
            this->w = 2.0;
            this->h = 2.5;
        }

    }
    bounding_box(point p, int type) : bounding_box(type){
        this->x = p.x;
        this->y = p.y;
        this->z = p.z;
    }

};

struct group{
    int N;
    int V;
    int F;
    int VF;
    int M;
    vector<point> pts;
    vector<vector<int>> faces;
    int class_type;
    double sum_projection_area;
    vector<bool> is_hull;
    vector<bool> occluded_face;
    vector<double> scores;
    bool has_polyhedron;
    point center;
    bounding_box box;
    int max_iter;
    double cur_loss;
    double prv_loss;
    double g_x;
    double g_y;
    double g_z;
    double g_t;
    double g_l;
    double g_w;
    double g_h;
    double loss_threshold_;
    double gt_l;
    double gt_w;
    double gt_h;
    double lr_init = INIT_LR;
    double decay_k = DECAY_K;


    group() {}
    group(vector<point> pts): pts(pts), N(pts.size()){
        V = 0;
        F = 0;
        VF = 0;
        M = pts[0].scores.size();
        scores = vector<double>(M, 0.0);
        center = point();
        for(const point& p : pts) center = center + p;
        center = center * (1.0 / N);
        max_iter = 100;
        loss_threshold_ = 0.1;
        has_polyhedron = false;
    }

    group(vector<point> pts, int class_type): pts(pts), N(pts.size()), class_type(class_type){
        V = 0;
        F = 0;
        VF = 0;
        M = pts[0].scores.size();
        scores = vector<double>(M, 0.0);
        center = point();
        for(const point& p : pts) center = center + p;
        center = center * (1.0 / N);
        max_iter = 100;
        loss_threshold_ = 0.1;
        has_polyhedron = false;
    }

    void calculate_confidence(){
        for(const point& p : pts){
            for(int i = 0; i < M; i++) scores[i] += p.scores[i];
        }
        for(int i=0;i<M;i++) cout << scores[i] / N << " ";
        cout << endl;
    }

    void add_confidence(const point& p){
        for(int i = 0; i < M; i++) scores[i] += p.scores[i] / N;
    }

    void print_confidence(){
        for(int i=0;i<M;i++) cout << scores[i] << " ";
        cout << endl;
    }

    void build_polyhedron(){
        // for (point p : pts){
        //     cout << p.x << " " << p.y << " " << p.z << endl;
        // }
        // cout << "***********************" << endl;
        ch_vertex* vertices;
        vertices = (ch_vertex*)malloc(N*sizeof(ch_vertex));
        for(int i=0;i<N;i++){
            vertices[i].x = pts[i].x;
            vertices[i].y = pts[i].y;
            vertices[i].z = pts[i].z;
            is_hull.push_back(false);
        }
        int* faceindices = NULL;
        convhull_3d_build(vertices, N, &faceindices, &F);
        for(int i=0; i<F;i++){
            faces.push_back({faceindices[3*i],faceindices[3*i+1],faceindices[3*i+2]});
            is_hull[faceindices[3*i]] = true;
            is_hull[faceindices[3*i+1]] = true;
            is_hull[faceindices[3*i+2]] = true;
        }
        for(int i=0;i<N;i++){
            if(!is_hull[i]) continue;
            V++;
            center.x += pts[i].x;
            center.y += pts[i].y;
            center.z += pts[i].z;
            // cout << pts[i].x << " " << pts[i].y << " " << pts[i].z << endl;
        }
        center.x /= V;
        center.y /= V;
        center.z /= V;

        for(int i=0;i<F;i++){
            // cout << "face " << i << ": " << faces[i][0] << " " << faces[i][1] << " " << faces[i][2] <<endl;
            face f = face(pts[faces[i][0]], pts[faces[i][1]], pts[faces[i][2]]);
            if(f.n * center > f.d){
                int tmp = faces[i][0];
                faces[i][0] = faces[i][1];
                faces[i][1] = tmp;
            }
            
        }
        int TF = 0;
        for(int i=0;i<F;i++) {
            if (is_occluded(i)) occluded_face.push_back(true);
            else {
                TF ++;
                face f = face(pts[faces[i][0]], pts[faces[i][1]], pts[faces[i][2]]);
                if (f.area < EPS)occluded_face.push_back(true);
                else{
                    VF ++;
                    occluded_face.push_back(false);
                    // cout << "area: " << f.area << endl;
                    // cout << "scale: " << sqrt(f.n.x * f.n.x + f.n.y * f.n.y) << endl;
                    // cout << "area: " << f.area << endl;
                    // cout << "point" << endl;
                    // cout << f.A.x << " " << f.A.y << " " << f.A.z << endl;
                    // cout << f.B.x << " " << f.B.y << " " << f.B.z << endl;
                    // cout << f.C.x << " " << f.C.y << " " << f.C.z << endl;
                    // cout << "a-b and a-c" << endl;
                    // cout << (f.A-f.B).x << " " << (f.A-f.B).y << " " << (f.A-f.B).z << endl;
                    // cout << (f.A-f.C).x << " " << (f.A-f.C).y << " " << (f.A-f.C).z << endl;
                    // cout << "normal vector" << endl;
                    // cout << f.n.x << " " << f.n.y << " " << f.n.z << endl;
                    sum_projection_area += f.area * sqrt(f.n.x * f.n.x + f.n.y * f.n.y);
                }
                
            }
        }
        // cout << "sum: " << sum_projection_area << endl;
        // cout << "# of faces: " << F << endl;
        // cout << "# of TF: " << TF << endl;
        // cout << "# of valid face: " << VF << endl;


        // cout << "Completed to build polyhedron" << endl;
        // cout << "N: " << N << endl;
        // cout << "V: " << V << endl;
        // cout << "F: " << F << endl;
        // cout << "center: " << center.x << " " << center.y << " " << center.z << endl;
        has_polyhedron = true;
    }

    bool is_occluded(int id){
        // check the i-th face is occluded from origin
        face f = face(pts[faces[id][0]], pts[faces[id][1]], pts[faces[id][2]]);
        return f.d > 0;
    }

    double point2cvh(point p){
        point origin = point(0,0,0);
        double rt = INF;
        for(int i=0;i<F;i++){
            face f = face(pts[faces[i][0]], pts[faces[i][1]], pts[faces[i][2]]);
            rt = min<double>(rt, f.distance2face(origin));
        }
        return rt;
    }


    vector<double> calculate_loss_and_grad(int type, double px, double py, double pz, double rx, double ry, double rz, double st, double ct){
        vector<double> values;
        values.push_back(abs(rx-box.l/2));
        values.push_back(abs(rx+box.l/2));
        values.push_back(abs(ry-box.w/2));
        values.push_back(abs(ry+box.w/2));
        values.push_back(abs(rz-box.h/2));
        values.push_back(abs(rz+box.h/2));
        vector<double> rt;
            
        if (type == 0){
            rt.push_back(values[type] * values[type]);
            rt.push_back(2.0 * (box.l/2 - rx) * ct);
            rt.push_back(2.0 * (box.l/2 - rx) * st);
            rt.push_back(0.0);
            rt.push_back(-2.0 * (box.l/2 - rx) * ((px - box.x) * (-st) + (py - box.y) * ct));
            rt.push_back(box.l/2 - rx);
            rt.push_back(0.0);
            rt.push_back(0.0);
            return rt;
        }
        if (type == 1){
            rt.push_back(values[type] * values[type]);
            rt.push_back(-2.0 * (box.l/2 + rx) * ct);
            rt.push_back(-2.0 * (box.l/2 + rx) * st);
            rt.push_back(0.0);
            rt.push_back(2.0 * (box.l/2 + rx) * ((px - box.x) * (-st) + (py - box.y) * ct));
            rt.push_back(box.l/2 + rx);
            rt.push_back(0.0);
            rt.push_back(0.0);
            return rt;
        }
        if (type == 2){
            rt.push_back(values[type] * values[type]);
            rt.push_back(-2.0 * (box.w/2 - ry) * st);
            rt.push_back(2.0 * (box.w/2 - ry) * ct);
            rt.push_back(0.0);
            rt.push_back(2.0 * (box.w/2 - ry) * ((px - box.x) * ct + (py - box.y) * st));
            rt.push_back(0.0);
            rt.push_back(box.w/2 - ry);
            rt.push_back(0.0);
            return rt;
        }
        if (type == 3){
            rt.push_back(values[type] * values[type]);
            rt.push_back(2.0 * (box.w/2 + ry) * st);
            rt.push_back(-2.0 * (box.w/2 + ry) * ct);
            rt.push_back(0.0);
            rt.push_back(-2.0 * (box.w/2 + ry) * ((px - box.x) * ct + (py - box.y) * st));
            rt.push_back(0.0);
            rt.push_back(box.w/2 + ry);
            rt.push_back(0.0);
            return rt;
        }
        if (type == 4){
            rt.push_back(values[type] * values[type]);
            rt.push_back(0.0);
            rt.push_back(0.0);
            rt.push_back(2.0 * (box.h/2 - rz));
            rt.push_back(0.0);
            rt.push_back(0.0);
            rt.push_back(0.0);
            rt.push_back(box.h/2 - rz);
            return rt;
        }
        if(type==5){
            rt.push_back(values[type] * values[type]);
            rt.push_back(0.0);
            rt.push_back(0.0);
            rt.push_back(-2.0 * (box.h/2 + rz));
            rt.push_back(0.0);
            rt.push_back(0.0);
            rt.push_back(0.0);
            rt.push_back(box.h/2 + rz);
            return rt;
        }
    }

    vector<double> calculate_box_loss(const point& p){
        double px = p.x;
        double py = p.y;
        double pz = p.z;
        double ct = cos(box.yaw);
        double st = sin(box.yaw);
        double rx = (px - box.x) * ct + (py - box.y) * st;
        double ry = - (px - box.x) * st + (py - box.y) * ct;
        double rz = pz - box.z;
            

        if (abs(rx)<box.l/2 && abs(ry) <box.w/2 && abs(rz) < box.h/2){
            vector<double> values;
            values.push_back(abs(rx-box.l/2));
            values.push_back(abs(rx+box.l/2));
            values.push_back(abs(ry-box.w/2));
            values.push_back(abs(ry+box.w/2));
            values.push_back(abs(rz-box.h/2));
            values.push_back(abs(rz+box.h/2));
            int type = 0;
            double value = values[0];
            for(int i=1;i<=5;i++){
                if(values[i] < value){
                    type = i;
                    value = values[i];
                }
            }
            return calculate_loss_and_grad(type, px, py, pz, rx, ry, rz, st, ct);
        }
        else{
            double rt_loss = 0.0;
            double rt_grad_x = 0.0;
            double rt_grad_y = 0.0;
            double rt_grad_z = 0.0;
            double rt_grad_theta = 0.0;
            double rt_grad_l = 0.0;
            double rt_grad_w = 0.0;
            double rt_grad_h = 0.0;
            if(rx>box.l/2){
                vector<double> rt = calculate_loss_and_grad(0, px, py, pz, rx, ry, rz, st, ct);
                rt_loss += rt[0];
                rt_grad_x += rt[1];
                rt_grad_y += rt[2];
                rt_grad_z += rt[3];
                rt_grad_theta += rt[4];
                rt_grad_l += rt[5];
                rt_grad_w += rt[6];
                rt_grad_h += rt[7];
            }
            if(rx<-box.l/2){
                vector<double> rt = calculate_loss_and_grad(1, px, py, pz, rx, ry, rz, st, ct);
                rt_loss += rt[0];
                rt_grad_x += rt[1];
                rt_grad_y += rt[2];
                rt_grad_z += rt[3];
                rt_grad_theta += rt[4];
                rt_grad_l += rt[5];
                rt_grad_w += rt[6];
                rt_grad_h += rt[7];
            }
            if(ry>box.w/2){
                vector<double> rt = calculate_loss_and_grad(2, px, py, pz, rx, ry, rz, st, ct);
                rt_loss += rt[0];
                rt_grad_x += rt[1];
                rt_grad_y += rt[2];
                rt_grad_z += rt[3];
                rt_grad_theta += rt[4];
                rt_grad_l += rt[5];
                rt_grad_w += rt[6];
                rt_grad_h += rt[7];
            }
            if(ry<-box.w/2){
                vector<double> rt = calculate_loss_and_grad(3, px, py, pz, rx, ry, rz, st, ct);
                rt_loss += rt[0];
                rt_grad_x += rt[1];
                rt_grad_y += rt[2];
                rt_grad_z += rt[3];
                rt_grad_theta += rt[4];
                rt_grad_l += rt[5];
                rt_grad_w += rt[6];
                rt_grad_h += rt[7];
            }
            if(rz>box.h){
                vector<double> rt = calculate_loss_and_grad(4, px, py, pz, rx, ry, rz, st, ct);
                rt_loss += rt[0];
                rt_grad_x += rt[1];
                rt_grad_y += rt[2];
                rt_grad_z += rt[3];
                rt_grad_theta += rt[4];
                rt_grad_l += rt[5];
                rt_grad_w += rt[6];
                rt_grad_h += rt[7];
            }
            if(rz<-box.h/2){
                vector<double> rt = calculate_loss_and_grad(5, px, py, pz, rx, ry, rz, st, ct);
                rt_loss += rt[0];
                rt_grad_x += rt[1];
                rt_grad_y += rt[2];
                rt_grad_z += rt[3];
                rt_grad_theta += rt[4];
                rt_grad_l += rt[5];
                rt_grad_w += rt[6];
                rt_grad_h += rt[7];
            }
            vector<double> final_rt;
            final_rt.push_back(ALPHA_O * rt_loss);
            final_rt.push_back(ALPHA_O * rt_grad_x);
            final_rt.push_back(ALPHA_O * rt_grad_y);
            final_rt.push_back(ALPHA_O * rt_grad_z);
            final_rt.push_back(ALPHA_O * rt_grad_theta);
            final_rt.push_back(ALPHA_O * rt_grad_l);
            final_rt.push_back(ALPHA_O * rt_grad_w);
            final_rt.push_back(ALPHA_O * rt_grad_h);
            return final_rt;
        }
    }

    vector<double> calculate_face_loss(const face& f){
        vector<double> rt;
        double parea = f.area * sqrt(f.n.x * f.n.x + f.n.y * f.n.y);
        double ct = cos(box.yaw);
        double st = sin(box.yaw);

        double px = (point(ct, st, 0.0) * f.n);
        double py = (point(-st, ct, 0.0) * f.n);

        if(abs(px) > abs(py)){
            if(px > 0){
                rt.push_back(parea / sum_projection_area * (1.0 - px) * (1.0 - px));
                rt.push_back(0.0);
                rt.push_back(0.0);
                rt.push_back(0.0);
                rt.push_back(parea / sum_projection_area * -2.0 * (1.0 - px) * py);
                rt.push_back(0.0);
                rt.push_back(0.0);
                rt.push_back(0.0);
                return rt;
            }
            else{
                rt.push_back(parea / sum_projection_area * (1.0 + px) * (1.0 + px));
                rt.push_back(0.0);
                rt.push_back(0.0);
                rt.push_back(0.0);
                rt.push_back(parea / sum_projection_area * 2.0 * (1.0 + px) * py);
                rt.push_back(0.0);
                rt.push_back(0.0);
                rt.push_back(0.0);
                return rt;
            }
        }
        else{
            if(py > 0){
                rt.push_back(parea / sum_projection_area * (1.0 - py) * (1.0 - py));
                rt.push_back(0.0);
                rt.push_back(0.0);
                rt.push_back(0.0);
                rt.push_back(parea / sum_projection_area * 2.0 * (1.0 - py) * px);
                rt.push_back(0.0);
                rt.push_back(0.0);
                rt.push_back(0.0);
                return rt;
            }
            else{
                rt.push_back(parea / sum_projection_area * (1.0 + py) * (1.0 + py));
                rt.push_back(0.0);
                rt.push_back(0.0);
                rt.push_back(0.0);
                rt.push_back(parea / sum_projection_area * -2.0 * (1.0 - py) * px);
                rt.push_back(0.0);
                rt.push_back(0.0);
                rt.push_back(0.0);
                return rt;
            }
        }
    }

    void solve(){
        box = bounding_box(center, class_type);
        gt_l = box.l;
        gt_w = box.w;
        gt_h = box.h;

        // classify occluded faces


        // measure origin to polyhedron
        double d_cvh = point2cvh(point(0,0,0));

        for(int iter = 0; iter < max_iter; iter++){
            box.yaw = box.yaw - M_PI * 2 * int(box.yaw / M_PI / 2);
            if (box.yaw < - M_PI) box.yaw += M_PI;
            if (box.yaw > M_PI) box.yaw += M_PI;
            cur_loss = 0.0;
            g_x = 0.0;
            g_y = 0.0;
            g_z = 0.0;
            g_t = 0.0;
            g_l = 0.0;
            g_w = 0.0;
            g_h = 0.0;

            double lr = lr_init / (1+decay_k*iter);

            // calculate gradient
            // model loss
            cur_loss += ALPHA_L * (box.l - gt_l) * (box.l - gt_l);
            g_l += ALPHA_L * 2 * (box.l - gt_l);

            cur_loss += ALPHA_W * (box.w - gt_w) * (box.w - gt_w);
            g_w += ALPHA_W * 2 * (box.w - gt_w);
            
            cur_loss += ALPHA_H * (box.h - gt_h) * (box.h - gt_h);
            g_h += ALPHA_H * 2 * (box.h - gt_h);

            // point loss
            for(int i = 0; i < N; i++){
                if(!is_hull[i]) continue;
                vector<double> rt = calculate_box_loss(pts[i]);

                cur_loss += ALPHA_P / V * rt[0];
                g_x += ALPHA_P / V * rt[1];
                g_y += ALPHA_P / V * rt[2];
                g_z += ALPHA_P / V * rt[3];
                g_t += ALPHA_P / V * rt[4];
                g_l += ALPHA_P / V * rt[5];
                g_w += ALPHA_P / V * rt[6];
                g_h += ALPHA_P / V * rt[7];
            }

            // face loss
            for(int i = 0; i < F; i++){
                if(occluded_face[i]) continue;
                vector<double> rt = calculate_face_loss(face(pts[faces[i][0]], pts[faces[i][1]], pts[faces[i][2]]));

                cur_loss += ALPHA_F * rt[0];
                g_x += ALPHA_F * rt[1];
                g_y += ALPHA_F * rt[2];
                g_z += ALPHA_F * rt[3];
                g_t += ALPHA_F * rt[4];
                g_l += ALPHA_F * rt[5];
                g_w += ALPHA_F * rt[6];
                g_h += ALPHA_F * rt[7];
            }

            // dist loss
            vector<double> cvh_loss = calculate_box_loss(point(0.0, 0.0, 0.0));
            double d = sqrt(cvh_loss[0]);
            if (d > 1.0){
                double f = (d -d_cvh) / d;
                cur_loss += ALPHA_D * f * f * cvh_loss[0];
                g_x += ALPHA_D * f * cvh_loss[1];
                g_y += ALPHA_D * f * cvh_loss[2];
                g_z += ALPHA_D * f * cvh_loss[3];
                g_t += ALPHA_D * f * cvh_loss[4];
                g_l += ALPHA_D * f * cvh_loss[5];
                g_w += ALPHA_D * f * cvh_loss[6];
                g_h += ALPHA_D * f * cvh_loss[7];
            }

            // update
            box.x -= lr * g_x;
            box.y -= lr * g_y;
            box.z -= lr * g_z;
            box.yaw -= lr * g_t;
            box.l -= lr * g_l;
            box.w -= lr * g_w;
            box.h -= lr * g_h;

            // if loss is similar to previous, break
            if (iter>0 && abs(cur_loss - prv_loss) < loss_threshold_) break;
            prv_loss = cur_loss;
        }
        // cout << "center: " << center.x << " " << center.y << " " << center.z << endl;
        // cout << "x: " << box.x << endl;
        // cout << "y: " << box.y << endl;
        // cout << "z: " << box.z << endl;
        // cout << "yaw: " << box.yaw << endl;
        // cout << "l: " << box.l << endl;
        // cout << "w: " << box.w << endl;
        // cout << "h: " << box.h << endl;
    }
};

double gjk(const group& A, const group& B){
    return 0.0;
}

group merge_groups(const vector<group>& groups){
    return group();
}


#endif