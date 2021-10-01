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
int cluster_threshold_ = 5;
double group_threshold_ = 1.0;


////////////////////////////////////////



int main(int argc, char **argv){
    ros::init(argc, argv, "object_detector");

    // initialization
    string data_name = "RouteScenario_0";
    stringstream data_path;
    data_path << ros::package::getPath("convex_object_detection") << "/data/" << data_name << "/";
    string pcd_path = data_path.str() + "pcd/";
    cout << pcd_path << endl;
    int n_data = 1;
    for(int seq = 0; seq < n_data; seq++){
        seq = 630;
        // load pcd file
        string pcd_file = pcd_path + zfill(seq) + ".pcd";

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *cloud) == -1) {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return (-1);
        }
        

        // build voxel grid map
        vector<point> grid_map[GRID_X][GRID_Y][GRID_Z];
        cout << cloud->points.size() << endl;
        for(const auto& p : *cloud) {
            double x = p.x + LIDAR_X;
            double y = p.y + LIDAR_Y;
            double z = p.z + LIDAR_Z;
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
                        cout << pts.size() << endl;
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
        // cout << "debug" << endl;
        // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
        // viewer.showCloud(debug_cloud.makeShared());
        // cout << "debug" << endl;
        // while (!viewer.wasStopped ())
        // {
        // }

        // build convex polyhedron
        int G = clusters.size();
        for(int i = 0; i < G; i++) clusters[i].build_polyhedron();

        // second clustering
        vector<vector<int>> adj(G);
        for(int i=0;i<G;i++){
            vector<int> tmp;
            for(int j=i+1;j<G;j++){
                if(gjk(clusters[i], clusters[j]) < group_threshold_) {
                    adj[i].push_back(j);
                    adj[j].push_back(i);
                }
            }
        }

        vector<vector<int>> group_clusters;
        vector<bool> group_visited(G, false);
        for(int i=0;i<G;i++){
            if(group_visited[i]) continue;
            vector<int> cluster;
            queue<int> q;
            q.push(i);
            while(!q.empty()){
                int cur = q.front();
                q.pop();
                if(group_visited[cur]) continue;
                cluster.push_back(cur);
                for(int nxt : adj[cur]){
                    if(group_visited[nxt]) continue;
                    q.push(nxt);
                }
            }
            group_clusters.push_back(cluster);
        }

        // merge convex polyhedron
        vector<group> objects;
        for(int i=0;i<group_clusters.size();i++){
            vector<group> merged_groups;
            for(int id : group_clusters[i]) merged_groups.push_back(clusters[id]);
            objects.push_back(merge_groups(merged_groups));
        }

        // assign class for each convex polyhedron
        int n_objects = objects.size();
        

        // find parameters for each convex polyhedron
        for(int i=0;i<n_objects;i++) objects[i].solve();

    }

//     for(int i = 0; i<NN;i++){
//         if (i<101) continue;
//         string data_name = data_name_list[i];
//         stringstream data_path;
//         data_path << ros::package::getPath("sensor_decoder") << "/data/" << data_name << "/";
//         cout << "[" << i + 1 << "/" << NN << ", " << double(end - begin) / CLOCKS_PER_SEC << "] Start to object detection for data named " << data_name << endl; 
        
//         string pcd_path = data_path.str() + "pcd/";
//         string obj_path = data_path.str() + "object/";
//         int st = seq_list[i].first;
//         int en = seq_list[i].second;
//         int MM = en - st;

//         for(int load_seq = 1; load_seq < MM + 1; load_seq++){
//             int seq = load_seq + st;
//             string pcd_file = pcd_path + zfill(seq) + ".pcd";
//             string obj_file = obj_path + zfill(seq) + ".txt";
            
//             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//             if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *cloud) == -1) {
//                 PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//                 return (-1);
//             }

//             vector<point> grid_map[GRID][GRID];
//             for(const auto& p : *cloud) {
//                 double x = p.x;
//                 double y = p.y;
//                 double z = p.z;
//                 if(x < minX + EPS || x > maxX - EPS || y < minY + EPS || y > maxY - EPS || z < minZ || z > maxZ) continue;
//                 point t;
//                 t.x = x;
//                 t.y = y;
//                 t.valid = false;
//                 t.occluded = false;
//                 pii pixel = xy_to_pixel(t);
//                 grid_map[pixel.first][pixel.second].push_back(t);
//             }
//             // clustering
//             vector<vector<point>> clusters;
//             for(int i = 0; i < GRID; i++){
//                 for(int j = 0; j < GRID; j++){
//                     int sz = grid_map[i][j].size();
//                     for(int k = 0; k < sz; k++){
//                         if(grid_map[i][j][k].valid) continue;
//                         double point_threshold = max(10.0, 50 - 1.5 * (abs(grid_map[i][j][k].x) + abs(grid_map[i][j][k].y)));
//                         vector<point> cluster;
//                         queue<piii> q;
//                         q.push({{i,j},k});
//                         while(!q.empty()){
//                             int px = q.front().first.first;
//                             int py = q.front().first.second;
//                             int id = q.front().second;
//                             point cur = grid_map[px][py][id];
//                             q.pop();
//                             if(grid_map[px][py][id].valid) continue;
//                             grid_map[px][py][id].valid = true;
//                             double dist_threshold = min(1.0, (abs(cur.x) + abs(cur.y))/30.0) * cluster_threshold;
//                             cluster.push_back(grid_map[px][py][id]);
//                             for(int dx = -R; dx <= R; dx++){
//                                 for(int dy = -R; dy <= R; dy++){
//                                     int nx = px + dx;
//                                     int ny = py + dy;
//                                     if(nx < 0 || nx >= GRID || ny < 0 || ny >= GRID) continue;
//                                     int nsz = grid_map[nx][ny].size();
//                                     for(int nk = 0; nk < nsz; nk++){
//                                         if(grid_map[nx][ny][nk].valid) continue;
//                                         if(dist(cur, grid_map[nx][ny][nk]) < dist_threshold) q.push({{nx, ny}, nk});
//                                     }
//                                 }
//                             }
//                         }
//                         if(cluster.size() > point_threshold) {
//                             // cout << cluster.size() << endl;
//                             clusters.push_back(cluster);
//                         }
//                     }
//                 }
//             }
//             //build convex hull for each cluster
//             vector<ConvexHull> cvh_list;
//             for(const vector<point>& v : clusters) cvh_list.push_back(ConvexHull(v));

//             // build adj matrix with gjk algorithm
//             int N = cvh_list.size();
//             // cout << "# of cvh before merge: " << N << endl;
//             vector<vector<int>> adj(N);
//             for(int i = 0; i < N; i++){
//                 for(int j = i+1; j < N; j++){
//                     if(gjk(cvh_list[i].p, cvh_list[j].p) < convex_hull_threshold){
//                         adj[i].push_back(j);
//                         adj[j].push_back(i);
//                     }
//                 }
//             }
//             // merge convex hull
//             vector<bool> convex_visited(N, false);
//             vector<ConvexHull> obstacles;
//             for(int i=0;i<N;i++){
//                 if(convex_visited[i]) continue;
//                 vector<ConvexHull> cluster;
//                 queue<int> q;
//                 q.push(i);
//                 while(!q.empty()){
//                     int cur = q.front();
//                     q.pop();
//                     if(convex_visited[cur]) continue;
//                     cluster.push_back(cvh_list[cur]);
//                     convex_visited[cur] =  true;
//                     for(int next : adj[cur]){
//                         if(convex_visited[next]) continue;
//                         q.push(next);
//                     }
//                 }
//                 obstacles.push_back(UnionConvexHull(cluster));
//             }

//             // cout << "# of cvh after merge : " << obstacles.size() << endl;

//             // for each cluster, find optimal solution
//             ofstream out(obj_file.c_str());
//             vector<point> origin;
//             origin.push_back(point());
//             vector<BoundingBox> boxes;
//             for(int i=0;i<obstacles.size();i++) {
//                 vector<pdi> pa;
//                 for(int j=0;j<obstacles[i].size;j++) pa.push_back({atan2(obstacles[i].p[i].y, obstacles[i].p[i].x), j});
//                 sort(pa.begin(), pa.end());
//                 pdi s = pa[0];
//                 pdi e = pa[obstacles[i].size-1];
//                 for(int j=0;j<obstacles[i].size-1;j++){
//                     if(pa[j+1].first - pa[j].first > M_PI){
//                         s = pa[j];
//                         e = pa[j+1];
//                         break;
//                     }
//                 }
//                 ConvexHull c = ConvexHull({point(0,0), obstacles[i].p[s.second], obstacles[i].p[e.second]});
//                 for(int j=0;j<obstacles[i].size;j++){
//                     if(j==s.second || j == e.second) obstacles[i].p[j].occluded = false;
//                     else if(isIncludeConvexHull(c, obstacles[i].p[j])) obstacles[i].p[j].occluded = false;
//                     else obstacles[i].p[j].occluded = true;
//                 }
//             }
//             for(const ConvexHull& cvh : obstacles){
//                 BoundingBox b = BoundingBox(cvh.center.x, cvh.center.y, 0.0, 5.0, 2.5);
//                 b.optimize(cvh.p, gjk(cvh.p, origin));
//                 boxes.push_back(b);
//             }
//             sort(boxes.begin(), boxes.end());
//             for(const BoundingBox& b : boxes){
//                 out << to_string(b.loss) << " " << to_string(b.box_loss) << " " << to_string(b.x) << " " << to_string(b.y) << " " << to_string(b.theta) << " " << to_string(b.l) << " " << to_string(b.w) << endl;
//             }
//             out.close();
//             end = clock();
//             if (load_seq % 100 == 0) cout << "[" << load_seq << "/" << MM << ", " << double(end - begin) / CLOCKS_PER_SEC << "] In progress to object detection" << endl;
//         }

//     }
        

//   return (0);
}