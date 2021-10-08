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

/********hyperparameter setting********/
int cluster_threshold_ = 10;
double group_threshold_ = 1.0;


////////////////////////////////////////



int main(int argc, char **argv){
    ros::init(argc, argv, "object_detector");
    clock_t begin = clock();
    // initialization
    string data_name = "RouteScenario_0";
    stringstream data_path;
    data_path << ros::package::getPath("convex_object_detection") << "/data/" << data_name << "/";
    string pcd_path = data_path.str() + "pcd/";
    string obj_path = data_path.str() + "object/";
    int n_data = 1000;
    for(int seq = 0; seq < n_data; seq++){
        // load pcd file
        string pcd_file = pcd_path + zfill(seq) + ".pcd";
        string obj_file = obj_path + zfill(seq) + ".txt";

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *cloud) == -1) {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return (-1);
        }
        

        // build voxel grid map
        vector<point> grid_map[GRID_X][GRID_Y][GRID_Z];
        // cout << cloud->points.size() << endl;
        for(const auto& p : *cloud) {
            double x = p.x + LIDAR_X + 0.03 * (double)rand() / (RAND_MAX);
            double y = p.y + LIDAR_Y + 0.03 * (double)rand() / (RAND_MAX);
            double z = p.z + LIDAR_Z + 0.03 * (double)rand() / (RAND_MAX);
            if(x < minX + EPS || x > maxX - EPS || y < minY + EPS || y > maxY - EPS || z < minZ + EPS || z > maxZ - EPS) continue;
            point t = point(x,y,z);
            pixel3d pixel = point2pixel(t);
            grid_map[pixel.x][pixel.y][pixel.z].push_back(t);
        }
        
        // // first clustering
        vector<group> clusters;
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
                    }
                }
            }
        }

        /* visualization of first clustering result */
        // int n_points = 0;
        // for(const auto& g : clusters) n_points += g.pts.size();
        // pcl::PointCloud<pcl::PointXYZRGB> debug_cloud;
        // debug_cloud.width = n_points;
        // debug_cloud.height = 1;
        // debug_cloud.is_dense = false;
        // debug_cloud.points.resize(debug_cloud.width * debug_cloud.height);
        // int index = 0;
        // int cluster_num = 0;
        // int n_clusters = clusters.size();
        // cout << n_clusters << endl;
        // cout << "debug" << endl;
        // for(const auto& g : clusters) {
        //     int rv = rand()%256;
        //     int gv = rand()%256;
        //     int bv = rand()%256;
        //     for(const auto& p : g.pts){
        //         pcl::PointXYZRGB pt;
        //         pt.x = p.x;
        //         pt.y = p.y;
        //         pt.z = p.z;
        //         pt.r = rv;
        //         pt.g = gv;
        //         pt.b = bv;
        //         debug_cloud.points[index] = pt;
        //         index++;
        //     }
        // }
        // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
        // viewer.showCloud(debug_cloud.makeShared());
        // while (!viewer.wasStopped ())
        // {
        // }

        // build convex polyhedron
        int G = clusters.size();
        // int G = 1;
        // cout << "build polyhedron" << endl;
        for(int i = 0; i < G; i++) clusters[i].build_polyhedron();
        for(int i = 0; i < G; i++) clusters[i].solve();

        // ofstream out(obj_file.c_str());
        // for(int i = 0; i< G; i++){
        //     out << to_string(clusters[i].cur_loss) << " " 
        //         << to_string(clusters[i].box.x) << " "
        //         << to_string(clusters[i].box.y) << " " 
        //         << to_string(clusters[i].box.z) << " " 
        //         << to_string(clusters[i].box.yaw) << " " 
        //         << to_string(clusters[i].box.l) << " " 
        //         << to_string(clusters[i].box.w) << " "
        //         << to_string(clusters[i].box.h) << endl;
        // }
        // out.close();

        // cout << seq << "-th data is saved" << endl;



        // second clustering
        // vector<vector<int>> adj(G);
        // for(int i=0;i<G;i++){
        //     vector<int> tmp;
        //     for(int j=i+1;j<G;j++){
        //         if(gjk(clusters[i], clusters[j]) < group_threshold_) {
        //             adj[i].push_back(j);
        //             adj[j].push_back(i);
        //         }
        //     }
        // }

        // vector<vector<int>> group_clusters;
        // vector<bool> group_visited(G, false);
        // for(int i=0;i<G;i++){
        //     if(group_visited[i]) continue;
        //     vector<int> cluster;
        //     queue<int> q;
        //     q.push(i);
        //     while(!q.empty()){
        //         int cur = q.front();
        //         q.pop();
        //         if(group_visited[cur]) continue;
        //         cluster.push_back(cur);
        //         for(int nxt : adj[cur]){
        //             if(group_visited[nxt]) continue;
        //             q.push(nxt);
        //         }
        //     }
        //     group_clusters.push_back(cluster);
        // }

        // // merge convex polyhedron
        // vector<group> objects;
        // for(int i=0;i<group_clusters.size();i++){
        //     vector<group> merged_groups;
        //     for(int id : group_clusters[i]) merged_groups.push_back(clusters[id]);
        //     objects.push_back(merge_groups(merged_groups));
        // }

        // // assign class for each convex polyhedron
        // int n_objects = objects.size();
        

        // // find parameters for each convex polyhedron
        // for(int i=0;i<n_objects;i++) objects[i].solve();

    }
    clock_t end = clock();
    cout << double(end-begin) / CLOCKS_PER_SEC;
}