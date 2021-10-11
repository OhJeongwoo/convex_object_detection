#include <iostream>

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>


using namespace std;
using namespace cv;


string zfill(int n){
    if(n==0) return "000000";
    int digit = log10(n) + 1;
    string rt = "";
    for(int i=0;i<6-digit;i++) rt += "0";
    return rt+to_string(n);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "driving_video_generator");

    string path;

    string data_name = "traj3";
    stringstream result_path;
    result_path << ros::package::getPath("convex_object_detection") << "/data/" << data_name << "/";
    
    string image_raw_path = result_path.str() + "rgb/";
    string local_map_path = result_path.str() + "out/";

    VideoWriter writer;
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
    double fps = 10.0;                          // framerate of the created video stream
    string filename = result_path.str() + "video.avi";             // name of the output video file
    writer.open(filename, codec, fps, cv::Size(600,300));

    int n_images = 260;
    int offset = 181;
    for(int i= offset;i<offset+n_images;i++){
        string cur_image_raw_path = image_raw_path + zfill(i) + ".png";
        string cur_local_map_path = local_map_path + zfill(i) + ".png";

        Mat image_raw = cv::imread(cur_image_raw_path);
        Mat local_map = cv::imread(cur_local_map_path);
        if(image_raw.size().height == 0 || local_map.size().height == 0) continue;
        Mat out;

        cv::resize(image_raw, image_raw, Size(300,300), 0, 0);
        cv::resize(local_map, local_map, Size(300,300), 0, 0);

        Mat arr[2] = {image_raw, local_map};
        cv::hconcat(arr,2,out);
        writer.write(out);

        if(i % 100 == 0) cout << "checkpoint: " << i << "/" << n_images << endl;
    }
    writer.release();
}