#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag36artoolkit.h>
#include <curl/curl.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <unordered_map>
#include <math.h>

using namespace cv;
using std::vector;

#define TAG_SIZE 6.0f

int main(int argc, char** argv){

    std::unordered_map<int, Mat> known_tags;

    // Display usage
    if (argc < 3) {
        printf("Usage: %s <basestation url> [cameras...]\n", argv[0]);
        return -1;
    }

   int id = atoi(argv[1]);
    VideoCapture device(id);

    if (!device.isOpened()) {
        std::cerr << "Failed to open video capture device " << id << std::endl;
    }

    std::ifstream fin;
    fin.open(argv[1]);
    if (fin.fail()) {
        std::cerr << "Failed to open file " << argv[1] << std::endl;
    }

    int size;
    Mat camera_matrix, dist_coeffs, transform_matrix;
    std::string line2;
    // TODO Error checking
    while (std::getline(fin, line2)) {
        std::stringstream line_stream(line2);
        std::string key, equals;
        line_stream >> key >> equals;
        if (key == "camera_matrix") {
            vector<double> data;
            for (int i = 0; i < 9; i++) {
                double v;
                line_stream >> v;
                data.push_back(v);
            }
            camera_matrix = Mat(data, true).reshape(1, 3);
        }

        else if (key == "dist_coeffs") {
            vector<double> data;
            for (int i = 0; i < 5; i++) {
                double v;
                line_stream >> v;
                data.push_back(v);
            }
            dist_coeffs = Mat(data, true).reshape(1, 1);
        }

        else if (key == "size"){
            line_stream >> size;
        }

        else if (key == "known_tags"){
            for(int i = 0; i < size; i++) {
                int id;
                line_stream >> id;
                vector<double> data;
                for (int i = 0; i < 16; i++) {
                    double v;
                    line_stream >> v;
                    data.push_back(v);

                }
                transform_matrix = Mat(data, true).reshape(1,4);
                known_tags.insert({id, transform_matrix});
            }
        }

        else {
            std::cerr << "Unrecognized key '" << key << "' in file " << argv[1] << std::endl;
        }
    };

    if (camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        std::cerr << "Error reading camera_matrix in file " << argv[1] << std::endl;
    }

    if (dist_coeffs.rows != 1 || dist_coeffs.cols != 5) {
        std::cerr << "Error reading dist_coeffs in file " << argv[1] << std::endl;
    }
    if (transform_matrix.rows != 4 || transform_matrix.cols != 4){
            std::cerr << "Error reading transform_matrix in file " << argv[1] << std::endl;
    }

    device.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    device.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    device.set(CV_CAP_PROP_FPS, 30);

    // Initialize detector
    apriltag_family_t* tf = tag36h11_create();
    tf->black_border = 1;

    apriltag_detector_t* td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 1.0;
    td->quad_sigma = 0.0;
    td->nthreads = 4;
    td->debug = 0;
    td->refine_edges = 1;
    td->refine_decode = 0;
    td->refine_pose = 0;

    int key = 0;
    Mat frame, gray;
    int trial = 0;
    double oldX = 0;
    double oldY = 0;
    double oldZ = 0;
    while (key != 27) { // Quit on escape keypress
        if (!device.isOpened()) {
            continue;
        }
        device >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        image_u8_t im = {
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t* detections = apriltag_detector_detect(td, &im);


        vector<Point2f> img_points(4);
        vector<Point3f> obj_points(4);
        Mat rvec(3, 1, CV_64FC1);
        Mat tvec(3, 1, CV_64FC1);
        for (int j = 0; j < zarray_size(detections); j++) {
            // Get the ith detection
            apriltag_detection_t *det;
            zarray_get(detections, j, &det);

            // Draw onto the frame
            line(frame, Point(det->p[0][0], det->p[0][1]),
                        Point(det->p[1][0], det->p[1][1]),
                        Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                        Point(det->p[3][0], det->p[3][1]),
                        Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                        Point(det->p[2][0], det->p[2][1]),
                        Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                        Point(det->p[3][0], det->p[3][1]),
                        Scalar(0xff, 0, 0), 2);

            // Compute transformation using PnP
            img_points[0] = Point2f(det->p[0][0], det->p[0][1]);
            img_points[1] = Point2f(det->p[1][0], det->p[1][1]);
            img_points[2] = Point2f(det->p[2][0], det->p[2][1]);
            img_points[3] = Point2f(det->p[3][0], det->p[3][1]);


            obj_points[0] = Point3f(-TAG_SIZE * 0.5f, -TAG_SIZE * 0.5f, 0.f);
            obj_points[1] = Point3f( TAG_SIZE * 0.5f, -TAG_SIZE * 0.5f, 0.f);
            obj_points[2] = Point3f( TAG_SIZE * 0.5f,  TAG_SIZE * 0.5f, 0.f);
            obj_points[3] = Point3f(-TAG_SIZE * 0.5f,  TAG_SIZE * 0.5f, 0.f);

            solvePnP(obj_points, img_points, camera_matrix,
                    dist_coeffs, rvec, tvec);
            Matx33d r;



            Rodrigues(rvec,r);
            //std::cout << "rvec: " << rvec << std::endl;
            //std::cout << "tvec: " << tvec << std::endl;
            vector<double> data;
            data.push_back(r(0,0));
            data.push_back(r(0,1));
            data.push_back(r(0,2));
            data.push_back(tvec.at<double>(0));
            data.push_back(r(1,0));
            data.push_back(r(1,1));
            data.push_back(r(1,2));
            data.push_back(tvec.at<double>(1));
            data.push_back(r(2,0));
            data.push_back(r(2,1));
            data.push_back(r(2,2));
            data.push_back(tvec.at<double>(2));
            data.push_back(0);
            data.push_back(0);
            data.push_back(0);
            data.push_back(1);
            Mat tag2cam = Mat(data,true).reshape(1, 4);
            Mat cam2tag = tag2cam.inv();


            vector<double> data2;
            data2.push_back(0);
            data2.push_back(0);
            data2.push_back(0);
            data2.push_back(1);
            Mat genout = Mat(data2,true).reshape(1, 4);

            //Mat tag2camXYZ = tag2cam * genout;
            //printf("%i :: %d :: % 3.3f % 3.3f % 3.3f\n",
            //        id, det->id,
            //        tag2camXYZ.at<double>(0), tag2camXYZ.at<double>(1), tag2camXYZ.at<double>(2));

            Mat cam2tagXYZ = cam2tag * genout;
            //printf("%i :: %d :: % 3.3f % 3.3f % 3.3f\n",
            //       id, det->id,
            //        cam2tagXYZ.at<double>(0), cam2tagXYZ.at<double>(1), cam2tagXYZ.at<double>(2));


            //std::cout << "tag2cam: " << det -> id<< " :: " << tag2cam << std::endl;
            //std::cout << "cam2tag: " << det -> id<< " :: " << cam2tag << std::endl;

            Mat cam2orig = known_tags[det->id] * cam2tag;
            Mat tagXYZS = cam2orig * genout;

            //std::cout << "cam2orig: " << det -> id<< " :: " << cam2orig << std::endl;

            //double yaw = atan(tag2cam.at<double>(1,0)/tag2cam.at<double>(0,0));
            //double roll = atan(tag2cam.at<double>(2,1)/tag2cam.at<double>(2,2));
            //double pitch = atan((tag2cam.at<double>(2,0)*-1)/ sqrt(tag2cam.at<double>(2,1)*tag2cam.at<double>(2,1) +  tag2cam.at<double>(2,2)*tag2cam.at<double>(2,2)));

            //std::cout << "roll: " << roll << std::endl;
            //std::cout << "pitch: " << pitch << std::endl;
            //std::cout << "yaw: " << yaw << std::endl;

            double magnitude = std::sqrt((oldX - tagXYZS.at<double>(0))*(oldX - tagXYZS.at<double>(0)) +
               (oldY - tagXYZS.at<double>(1))*(oldY - tagXYZS.at<double>(1)) + (oldZ - tagXYZS.at<double>(2))*(oldZ - tagXYZS.at<double>(2)));
            //std::cout << magnitude << std:: endl;

            if (magnitude <= 20 || trial == 0){
                //std::cout << "magnitude is less than 15" << std:: endl;
                printf("%i :: %d :: % 3.3f % 3.3f % 3.3f\n",
                    id, det->id,
                    tagXYZS.at<double>(0), tagXYZS.at<double>(1), tagXYZS.at<double>(2));

                oldX = tagXYZS.at<double>(0);
                oldY = tagXYZS.at<double>(1);
                oldZ = tagXYZS.at<double>(2);
                trial ++;
                /*std::cout << oldX << std:: endl;
                std::cout << oldY << std:: endl;
                std::cout << oldZ << std:: endl;

                std::cout << trial << std:: endl;*/
            }

        }

        zarray_destroy(detections);
        imshow(std::to_string(0), frame);
        key = waitKey(16);
    }
}