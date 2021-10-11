#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ctime>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"


#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/ModelCoefficients.h>
#include <pcl-1.8/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "utils.h"


using namespace std;

typedef pair<pixel3d, int> pixel_id;
typedef pair<double, point> depth_point;

/********hyperparameter setting********/
int cluster_threshold_ = 10;
double group_threshold_ = 0.8;
double distance_threshold_ = 1.0;
double score_threshold_ = 0.7;
double min_score_threshold_ = 0.2;
int pixel_interval = 10;
int max_image_size = 1500;
const int H = max_image_size / pixel_interval;
const int W = max_image_size / pixel_interval; 
double depth_threshold_ = 1.0;

////////////////////////////////////////

bool compare(const point& a, const point& b){
    return a.depth < b.depth;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "fusion_object_detector");
    clock_t begin = clock();
    
    clock_t end = clock();
    cout << double(end-begin) / CLOCKS_PER_SEC << endl;
    // initialization
    string data_name = "traj3";
    stringstream data_path;
    data_path << ros::package::getPath("convex_object_detection") << "/data/" << data_name << "/";
    string spc_path = data_path.str() + "spc/";
    string obj_path = data_path.str() + "object/";
    int n_data = 260;
    int offset = 181;
    for(int seq = offset; seq < offset + n_data; seq++){
        // load spc file

        // load pcd file
        string spc_file = spc_path + zfill(seq) + ".txt";
        string obj_file = obj_path + zfill(seq) + ".txt";

        const char delimiter =  ' ';
        string in_line;
        ifstream in(spc_file);
        int N = -1;
        int M = -1;
        int MM = -1;
        vector<int> valid_list;
        vector<int> class_array;
        vector<int> instance_class;
        vector<point> grid_map[GRID_X][GRID_Y][GRID_Z];
        int line_num = 0;
        int root = 5;
        int n_points = 0;
        while(getline(in, in_line)){
            stringstream ss(in_line);
            string token;
            vector<string> tokens;
            line_num ++;
            while(getline(ss,token,delimiter)) tokens.push_back(token);
            if (line_num == 1) {
                N = stoi(tokens[0]);
            }
            else if(line_num == 2){
                MM = stoi(tokens[0]);
            }
            else if(line_num == 3){
                for(int i=0;i<MM;i++) class_array.push_back(stoi(tokens[i]));
            }
            else if(line_num == 4){
                M = 0;
                for(int i=0;i<MM;i++){
                    if(stoi(tokens[i]) == 1) {
                        M++;
                        valid_list.push_back(i);
                        instance_class.push_back(class_array[i]);
                    }
                }
            }
            else{
                double x = stod(tokens[0]) + LIDAR_X;
                double y = stod(tokens[1]) + LIDAR_Y;
                double z = stod(tokens[2]) + LIDAR_Z;
                if(x < minX + EPS || x > maxX - EPS || y < minY + EPS || y > maxY - EPS || z < minZ + EPS || z > maxZ - EPS) continue;
                point p = point(x, y, z);
                vector<double> score_list;
                double sum = 0.0;
                for(int i : valid_list) {
                    double score = stod(tokens[3+i]);
                    score_list.push_back(score);
                    sum += score;
                }
                if (sum < 0.1) continue;
                p.scores = score_list;
                p.px = stoi(tokens[3+MM]);
                p.py = stoi(tokens[4+MM]);
                p.depth = stod(tokens[5+MM]);
                pixel3d pixel = point2pixel(p);
                grid_map[pixel.x][pixel.y][pixel.z].push_back(p);
            }
        }
        end = clock();
        cout << double(end-begin) / CLOCKS_PER_SEC << endl;
        // // first clustering
        int G = 0;
        vector<group> clusters;
        vector<point> entire_points;
        for(int gx = 0; gx < GRID_X; gx++){
            for(int gy = 0; gy < GRID_Y; gy++){
                for(int gz = 0; gz < GRID_Z; gz++){
                    int sz = grid_map[gx][gy][gz].size();
                    for(int k = 0; k < sz; k++){
                        if(grid_map[gx][gy][gz][k].visited) continue;
                        vector<point> pts;
                        queue<pixel_id> q;
                        q.push({pixel3d(gx, gy, gz), k});
                        while(!q.empty()){
                            pixel3d pixel = q.front().first;
                            int id = q.front().second;
                            q.pop();
                            if(grid_map[pixel.x][pixel.y][pixel.z][id].visited) continue;
                            grid_map[pixel.x][pixel.y][pixel.z][id].visited = true;
                            point cur = grid_map[pixel.x][pixel.y][pixel.z][id];
                            cur.cluster_id = G;
                            double d = norm3d(cur);
                            pts.push_back(cur);
                            double rx = max<double>(0.10, d/100.0);
                            double ry = max<double>(0.10, d/100.0);
                            double rz = max<double>(0.20, d/50.0);

                            int RX = int(rx/resolution_x) + 1;
                            int RY = int(ry/resolution_y) + 1;
                            int RZ = int(rz/resolution_z) + 1;
                            for(int ngx = pixel.x - RX; ngx <= pixel.x + RX; ngx++){
                                for(int ngy = pixel.y - RY; ngy <= pixel.y + RY; ngy++){
                                    for(int ngz = pixel.z - RZ; ngz <= pixel.z + RZ; ngz++){
                                        if(!is_valid_pixel(pixel3d(ngx, ngy, ngz))) continue;
                                        int nsz = grid_map[ngx][ngy][ngz].size();
                                        for(int i = 0; i < nsz; i++){
                                            if(grid_map[ngx][ngy][ngz][i].visited) continue;
                                            point nxt = grid_map[ngx][ngy][ngz][i];
                                            if(abs(cur.x-nxt.x) < rx && abs(cur.y-nxt.y) && abs(cur.z-nxt.z)) q.push({pixel3d(ngx, ngy, ngz), i});
                                        }
                                    }
                                }
                            }
                        }
                        if (pts.size()<cluster_threshold_) continue;
                        clusters.push_back(group(pts));
                        G++;
                    }
                }
            }
        }
        end = clock();
        cout << double(end-begin) / CLOCKS_PER_SEC << endl;
        for(int i = 0; i < G; i++) entire_points.insert(entire_points.end(), clusters[i].pts.begin(), clusters[i].pts.end());
        sort(entire_points.begin(), entire_points.end(), compare);
        end = clock();
        cout << double(end-begin) / CLOCKS_PER_SEC << endl;
        double depth_info[H][W] = {-1,};
        for(const point& p : entire_points){
            double d = p.depth;
            int px = p.px;
            int py = p.py;
            int cluster_id = p.cluster_id;
            if(d<0) continue;
            if(depth_info[px/pixel_interval][py/pixel_interval] < EPS){
                depth_info[px/pixel_interval][py/pixel_interval] = d;
            }
            else{
                if(d>depth_info[px/pixel_interval][py/pixel_interval] + depth_threshold_) continue;
            }
            clusters[cluster_id].add_confidence(p);
        }
        end = clock();
        cout << double(end-begin) / CLOCKS_PER_SEC << endl;
        // calculate group confidence
        // int G = clusters.size();
        cout << "# of clusters: " << G << endl; 

        vector<int> chief_cluster_index(M, -1);
        vector<double> chief_cluster_score(M, min_score_threshold_);
        for(int i = 0; i < G; i++) {
            for(int j = 0; j < M; j++){
                if(chief_cluster_score[j] < clusters[i].scores[j]){
                    chief_cluster_index[j] = i;
                    chief_cluster_score[j] = clusters[i].scores[j];
                }
            }
        }


        vector<vector<int>> cluster_index_list;
        for(int i = 0; i < M; i++){
            vector<int> cluster_index;
            int K = chief_cluster_index[i];
            if (K == -1) {
                cout << "no cluster" << endl;
                cluster_index_list.push_back(cluster_index);
                continue;
            }
            point center = clusters[K].center;
            for(int j = 0; j < G; j++){
                if (j == K) cluster_index.push_back(j);
                else{
                    if(clusters[j].scores[i] > score_threshold_  && norm3d(clusters[j].center - center) < distance_threshold_) cluster_index.push_back(j);
                }
            }
            cluster_index_list.push_back(cluster_index);
        }
        end = clock();
        cout << double(end-begin) / CLOCKS_PER_SEC << endl;
        vector<group> box_list;
        for(int i = 0; i < M; i++){
            if(cluster_index_list[i].size() == 0) continue;
            vector<point> pts;
            for(int index : cluster_index_list[i]) {
                pts.insert(pts.end(), clusters[index].pts.begin(), clusters[index].pts.end());
                cout << index << endl;
            }
            cout << pts.size() << endl;
            box_list.push_back(group(pts, instance_class[i]));
        }

        G = box_list.size();
        cout << "# of box: " << G << endl;
        for(int i = 0; i < G; i++) box_list[i].build_polyhedron();
        for(int i = 0; i < G; i++) box_list[i].solve();
        end = clock();
        cout << double(end-begin) / CLOCKS_PER_SEC << endl;

        ofstream out(obj_file.c_str());
        for(int i = 0; i< G; i++){
            out << to_string(box_list[i].cur_loss) << " " 
                << to_string(box_list[i].box.x) << " "
                << to_string(box_list[i].box.y) << " " 
                << to_string(box_list[i].box.z) << " " 
                << to_string(box_list[i].box.yaw) << " " 
                << to_string(box_list[i].box.l) << " " 
                << to_string(box_list[i].box.w) << " "
                << to_string(box_list[i].box.h) << endl;
        }
        out.close();

        cout << seq << "-th data is saved" << endl;
    }
    end = clock();
    cout << double(end-begin) / CLOCKS_PER_SEC << endl;
}