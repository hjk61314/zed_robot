#include <sl_zed/Camera.hpp>
#include <opencv2/opencv.hpp>
#include "SaveDepth.hpp"
#include "CreateMap.hpp"

#include <time.h>
#include <pthread.h>

using namespace std;

cv::Mat slMat2cvMat(sl::Mat& input);
void printHelp();
void* test_thread(void* args);


int main(int argc, char **argv) {

    sl::Camera zed;

    // Set configuration parameters
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION_HD720;
    init_params.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = sl::UNIT_METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
        
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::SUCCESS) {
        std::cout << sl::toString(err) << std::endl;
        zed.close();
        return 1; // Quit if an error occurred
    }

    // Set positional tracking parameters
    sl::TrackingParameters trackingParameters;
    trackingParameters.initial_world_transform = sl::Transform::identity();
    trackingParameters.enable_spatial_memory = true;
    // Start motion tracking
    zed.enableTracking(trackingParameters);

    printHelp();

    // Prepare new image size to retrieve half-resolution images
    sl::Resolution image_size = zed.getResolution();
    int new_width = image_size.width / 2;
    int new_height = image_size.height / 2;

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    sl::Mat image_zed(new_width, new_height, sl::MAT_TYPE_8U_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);
    sl::Mat depth_image_zed(new_width*2, new_height*2, sl::MAT_TYPE_8U_C4);
    cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);

    sl::Mat depth_map;
    sl::Mat point_cloud;


    sl::Pose camera_pose;

//    pthread_t tidMap;
//    int ret = pthread_create(&tidMap, NULL, test_thread, NULL);
//    if(ret != 0){
//        cout << "pthread_create error: error_code = "<< ret << endl;
//    }


    // Get the distance between the center of the camera and the left eye
    float translation_left_to_center = zed.getCameraInformation().calibration_parameters.T.x * 0.5f;//6cm
    char text_rotation[128];
    char text_translation[128];
    char key = ' ';
    while (key != 'q') {

        if (zed.grab() == sl::SUCCESS) {

            // Retrieve the left image, depth image in half-resolution
            zed.retrieveImage(image_zed, sl::VIEW_LEFT, sl::MEM_CPU, new_width, new_height);
            zed.retrieveImage(depth_image_zed, sl::VIEW_DEPTH, sl::MEM_CPU);//, new_width, new_height
            zed.retrieveMeasure(depth_map, sl::MEASURE_DEPTH,sl::MEM_CPU);

            // Retrieve the RGBA point cloud in half-resolution
            zed.retrieveMeasure(point_cloud, sl::MEASURE_XYZRGBA, sl::MEM_CPU, new_width, new_height);

            // Display image and depth using cv:Mat which share sl:Mat data
            cv::imshow("Image", image_ocv);
            cv::imshow("Depth", depth_image_ocv);

            sl::TRACKING_STATE tracking_state = zed.getPosition(camera_pose, sl::REFERENCE_FRAME_WORLD);
            if (tracking_state == sl::TRACKING_STATE_OK) {
                // Get the pose at the center of the camera (baseline/2 on X axis)
                transformPose(camera_pose.pose_data, translation_left_to_center);

                // Get the pose of the camera relative to the world frame
                sl::float3 rotation = camera_pose.getEulerAngles();
                sl::float3 translation = camera_pose.getTranslation();
//                snprintf(text_rotation, 128, "%3.2f; %3.2f; %3.2f", rotation.x, rotation.y, rotation.z);
//                snprintf(text_translation, 128, "%3.2f; %3.2f; %3.2f", translation.x, translation.y, translation.z);
//                cout<<"rotation:"<<text_rotation<<endl;
//                cout<<"translation:"<<text_translation<<endl;
            }


            key = cv::waitKey(10);
            if(key == 'h'){
                cout << depth_map.getInfos() <<endl;
                //cv::imwrite(string("./depthMap.png"),depth_image_zed);

//                clock_t start = clock();
//                writeMatToFile(depth_map,"./myDepthMap.txt");
//                double duration = (double)(clock() - start) / CLOCKS_PER_SEC;
//                std::cout<<"save time :"<<duration<<std::endl;

                create_map(point_cloud,camera_pose.pose_data);
            }

            processKeyEvent(zed, key);
        }
    }
    zed.close();
    return 0;
}











void* test_thread(void* args){

    while(1){
        sleep(1);
        std::cout<<"create map running"<<std::endl;
    }
}




// Conversion function between sl::Mat and cv::Mat
// Mapping between MAT_TYPE and CV_TYPE
cv::Mat slMat2cvMat(sl::Mat& input) {
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

// This function displays help in console
void printHelp() {
    std::cout << " Press 's' to save Side by side images" << std::endl;
    std::cout << " Press 'p' to save Point Cloud" << std::endl;
    std::cout << " Press 'd' to save Depth image" << std::endl;
    std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
    std::cout << " Press 'n' to switch Depth format" << std::endl;
}
