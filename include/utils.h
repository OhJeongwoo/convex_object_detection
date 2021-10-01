#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <ctime>
#include <tuple>
#include <vector>
#include <stack>
#include "ros/ros.h"

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

struct group{
    vector<point> pts;
    vector<int> vertices;
    vector<vector<int>> faces;
    int N;
    int V;
    int F;
    bool has_polyhedron;

    group() {}
    group(vector<point> pts): pts(pts), N(pts.size()){
        V = -1;
        F = -1;
        has_polyhedron = false;
    }
    void build_polyhedron(){
        has_polyhedron = true;
    }

    void solve(){
        
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