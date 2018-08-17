#include "CreateMap.hpp"
#include <fstream>
#include <iostream>

using namespace std;

// 4m*4m, so resolution=1cm/grid
#define GLOBALMAP_REALSIZE 4.0f     //caution define as float
#define GLOBALMAP_SIZE   400
#define GLOBALMAP_RESOLUTION (GLOBALMAP_REALSIZE/GLOBALMAP_SIZE)
DepthData globalmap_buffer[GLOBALMAP_SIZE][GLOBALMAP_SIZE];

float globalmap[GLOBALMAP_SIZE][GLOBALMAP_SIZE];


#define MAX(a,b) ((a)>(b))?(a):(b)
#define LIMIT_MIN(x,MIN)  ((x)<(MIN)?(MIN):(x))
#define LIMIT_MAX(x,MAX)  ((x)>(MAX)?(MAX):(x))

void create_map(sl::Mat& point_cloud, sl::Transform& camera_pose)
{
    sl::float3 rotation = camera_pose.getEulerAngles();//rad
    sl::float3 translation = camera_pose.getTranslation();//m
    float yaw = - rotation.z;
//    float x = translation.x;
//    float y = translation.y;


    sl::float4 point_cloud_value;
    uint32_t col = point_cloud.getHeight();//caution here
    uint32_t row = point_cloud.getWidth();
    for(int i=0; i<row; i++) {
        for(int j=0; j<col; j++) {
            point_cloud.getValue(i, j, &point_cloud_value);
            float depthVal = point_cloud_value.z;
            if(isValidMeasure(depthVal))
            {
                if(depthVal > -2.0) //obstacle
                {
                    // transform from current frame to world frame, others discard
                    // first to 1st frame camera frame
                    float data_1[4]={point_cloud_value.x, point_cloud_value.y, point_cloud_value.z, 1.0f};
                    cv::Mat point_curFrame(4,1,CV_32FC1,data_1);
                    float data_2[16]={camera_pose.r00,camera_pose.r01,camera_pose.r02,translation.x,
                                      camera_pose.r10,camera_pose.r11,camera_pose.r12,translation.y,
                                      camera_pose.r20,camera_pose.r21,camera_pose.r22,translation.z,
                                      0.0f, 0.0f, 0.0f,1.0f
                                     };
                    //if only rotate arount z
                    //data_2[16]={cos(yaw),-sin(yaw),0.0f,x,   sin(yaw),cos(yaw),0.0f,y,  0.0f,0.0f,1.0f,z,  0.0f,0.0f,0.0f,1.0f};
                    cv::Mat camera_pose_(4,4,CV_32FC1,data_2);
                    cv::Mat point_camFrame = camera_pose_ * point_curFrame;
//                    cout<<"data_2:"<<camera_pose_<<endl;
                    //cout<<"pCamFrame:"<<point_camFrame<<endl;

                    // then to world frame
                    // to do

                    // drop these points to corresponding grids in global occupancy map buffer
                    int grid_i = ceil(point_camFrame.at<float>(0,0)/GLOBALMAP_RESOLUTION) + GLOBALMAP_SIZE/2;//cartisian frame to image frame
                    if((grid_i < 0) || (grid_i >= GLOBALMAP_SIZE))
                        continue;

                    int grid_j = GLOBALMAP_SIZE/2 - ceil(point_camFrame.at<float>(1,0)/GLOBALMAP_RESOLUTION);
                    if((grid_j < 0) || (grid_j >= GLOBALMAP_SIZE))
                        continue;

                    //cout<<"grid:"<<grid_i<<","<<grid_j<<endl;

                    int grid_buffer_count = globalmap_buffer[grid_i][grid_j].count;
                    if(grid_buffer_count == 16)
                        continue;

                    globalmap_buffer[grid_i][grid_j].buffer[grid_buffer_count] = point_camFrame.at<float>(2,0);
                    globalmap_buffer[grid_i][grid_j].count++;
                    //cout<<"count:"<<grid_buffer_count<<endl;

                }

            }
            else
            {
                //cout<<"unvalid:"<<i<<","<<j<<","<<depthVal<<endl;
            }

        }
    }


    // update global map
    for(int i=0; i<GLOBALMAP_SIZE; i++) {
        for(int j=0; j<GLOBALMAP_SIZE; j++) {
            int count = globalmap_buffer[i][j].count;
            float sum = 0.0f;
            for(int k=0;k<count;k++)
            {
                sum += globalmap_buffer[i][j].buffer[k];
            }

            globalmap[i][j] = sum/count;//maybe buffer defined as queue
            //cout<<"globalmap:"<<i<<","<<j<<","<<count<<","<<sum<<endl;

        }
    }

    ofstream fout(string("./globalmap.txt"));
    for(int i=0; i<GLOBALMAP_SIZE; i++){
        for(int j=0; j<GLOBALMAP_SIZE; j++){
            if(j != (GLOBALMAP_SIZE-1))
                fout << globalmap[i][j]<<",";
            else
                fout << globalmap[i][j]<<endl;
        }
    }
    fout.close();

}


void process_globalbuffer(DepthData &index)
{
}



// The generic formula used here is: Pose(new reference frame) = M.inverse() * Pose (camera frame) * M,
// where M is the transform between two frames.
void transformPose(sl::Transform &pose, float tx)
{
    sl::Transform transform_;
    transform_.setIdentity();
    // Move the tracking frame by tx along the X axis
    transform_.tx = tx;
    // Apply the transformation
    pose = sl::Transform::inverse(transform_) * pose * transform_;
}


void writeMatToFile(sl::Mat& m, const char* filename)
{
    std::ofstream fout(filename);

    if(!fout) {
        std::cout<<"File Not Opened"<<std::endl;
        return;
    }
    sl::float1 depthVal;
    int col = m.getHeight();
    int row = m.getWidth();
    for(int i=0; i<row; i++) {
        for(int j=0; j<col; j++) {
            m.getValue(i, j, &depthVal);
            if (j != (col-1))
                fout << depthVal <<",";
            else
                fout << depthVal <<endl;


//            if(isValidMeasure(depthVal))
//                fout<<depthVal<<std::endl;
//            else
//                cout<<"unvalid"<<endl;

        }
    }

    fout.close();
}

