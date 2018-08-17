#ifndef __CREATE_MAP_HPP__
#define __CREATE_MAP_HPP__

#include <opencv2/opencv.hpp>
#include <sl_zed/Camera.hpp>

typedef struct {
    int count;
    float buffer[16];
}DepthData;



void writeMatToFile(sl::Mat& m, const char* filename);
void transformPose(sl::Transform &pose, float tx);
void create_map(sl::Mat& point_cloud, sl::Transform& camera_pose);

void process_globalbuffer(DepthData &buffer);

#endif
