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
double LIDAR_Z = 2.5;
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

const double ALPHA_L = 1.0;
const double ALPHA_W = 1.0;
const double ALPHA_H = 1.0;
const double ALPHA_P = 1.0;
const double ALPHA_F = 1.0;
const double ALPHA_D = 1.0;

struct pixel3d{
    int x, y, z;
    pixel3d(): pixel3d(0,0,0){}
    pixel3d(int x, int y, int z): x(x), y(y), z(z){}
};

struct point{
    double x,y,z;
    double theta;
    bool visited;
    point() : point(0,0){}
    point(double x1, double y1): x(x1),y(y1), theta(0.0), visited(false){}
    point(double x, double y, double z): x(x), y(y), z(z), theta(0.0), visited(false){}
    void update(point p){
        theta = atan2(y-p.y,x-p.x);
    }
    bool operator <(const point& o){
        if(abs(theta-o.theta)>EPS) return theta<o.theta;
        if(abs(y-o.y)>EPS) return y<o.y;
        return x<o.x;
    }
    point operator +(const point &o) const{return point(x+o.x, y+o.y);}
    point operator -(const point &o) const{return point(x-o.x, y-o.y);}
    point operator -() const{return point(-x, -y);}
    double operator * (const point &o) const{ return x*o.x + y*o.y;}
    point operator *(const double t) const{return point(x*t, y*t);}
};

struct face{
    int a,b,c;

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
        if(type == 1){
            this->x = 0.0;
            this->y = 0.0;
            this->z = 0.0;
            this->yaw = 0.0;
            this->l = 5.0;
            this->w = 2.5;
            this->h = 1.5;
        }
    }
    bounding_box(point p, int type) : bounding_box(type){
        this->x = p.x;
        this->y = p.y;
        this->z = p.z;
    }

};

struct group{
    vector<point> pts;
    // vector<int> vertices;
    vector<vector<int>> faces;
    int N;
    int V;
    int F;
    vector<bool> is_hull;
    vector<bool> occluded_face;
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


    group() {}
    group(vector<point> pts): pts(pts), N(pts.size()){
        V = 0;
        F = 0;
        center = point();
        max_iter = 100;
        loss_threshold_ = 0.1;
        has_polyhedron = false;
    }
    void build_polyhedron(){
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
        }
        center.x /= V;
        center.y /= V;
        center.z /= V;
        
        for(int i=0;i<F;i++) occluded_face.push_back(is_occluded(i));


        cout << "Completed to build polyhedron" << endl;
        cout << "N: " << N << endl;
        cout << "V: " << V << endl;
        cout << "F: " << F << endl;
        cout << "center: " << center.x << " " << center.y << " " << center.z << endl;
        has_polyhedron = true;
    }

    bool is_occluded(int id){
        // check the i-th face is occluded from origin
        return false;
    }

    double point2cvh(point p){
        return 1.0;
    }

    void calculate_box_loss(){

    }

    void solve(){
        box = bounding_box(center, 1);
        gt_l = 5.0;
        gt_w = 2.5;
        gt_h = 1.5;

        // classify occluded faces


        // measure origin to polyhedron
        double d_cvh = point2cvh(point(0,0,0));

        for(int iter = 0; iter < max_iter; iter++){
            cur_loss = 0.0;
            g_x = 0.0;
            g_y = 0.0;
            g_z = 0.0;
            g_t = 0.0;
            g_l = 0.0;
            g_w = 0.0;
            g_h = 0.0;

            // calculate gradient
            // model loss
            cur_loss += ALPHA_L * (box.l - gt_l) * (box.l - gt_l);
            g_l += ALPHA_L * 2 * (box.l - gt_l);

            cur_loss += ALPHA_W * (box.w - gt_w) * (box.w - gt_w);
            g_w += ALPHA_W * 2 * (box.w - gt_w);
            
            cur_loss += ALPHA_H * (box.h - gt_h) * (box.h - gt_h);
            g_h += ALPHA_H * 2 * (box.h - gt_h);

            // point loss
            

            // face loss

            // dist loss


            // update

            // if loss is similar to previous, break
            if (iter>0 && abs(cur_loss - prv_loss) < loss_threshold_) break;
            prv_loss = cur_loss;
        }

    }
};

struct object{

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

double gjk(const group& A, const group& B){
    return 0.0;
}

group merge_groups(const vector<group>& groups){
    return group();
}

string zfill(int n){
    if(n==0) return "000000";
    int digit = log10(n) + 1;
    string rt = "";
    for(int i=0;i<6-digit;i++) rt += "0";
    return rt+to_string(n);
}




#endif