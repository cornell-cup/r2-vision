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

using namespace cv;
using std::vector;

#define TAG_SIZE 6.5f

int main(int argc, char** argv) {
    vector<double> origin_tag = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    std::unordered_map<int, Mat> known_tags = {
        {0, Mat(origin_tag, true).reshape(1,4)}
    };

    // Display usage
    if (argc < 2) {
        printf("Usage: %s [cameras...]\n", argv[0]);
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

    Mat camera_matrix, dist_coeffs;
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
        else {
            std::cerr << "Unrecognized key '" << key << "' in file " << argv[1] << std::endl;
        }
    }

    if (camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        std::cerr << "Error reading camera_matrix in file " << argv[1] << std::endl;
    }

    if (dist_coeffs.rows != 1 || dist_coeffs.cols != 5) {
        std::cerr << "Error reading dist_coeffs in file " << argv[1] << std::endl;
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

    while (key != 27) { //Quit on escape key press
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

        if (key == 's'){
            zarray_t* detections = apriltag_detector_detect(td, &im);

            vector<Point2f> img_points(4);
            vector<Point3f> obj_points(4);
            Mat rvec(3, 1, CV_64FC1);
            Mat tvec(3, 1, CV_64FC1);

            int origin_id = -1; // known tag id being used as the "origin" tag
            //locate_origin
            for (int j = zarray_size(detections) - 1; j >= 0; j--) {
                // Get the ith detection
                apriltag_detection_t *det;
                zarray_get(detections, j, &det);

                //sees if we can use the tag detected as the origin
                if (known_tags.find(det -> id) != known_tags.end()) {
                    origin_id = det -> id;

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
                    /*
                    img_points[0] = Point2f(243.168, 115.075);
                    img_points[1] = Point2f(268.68, 114.811);
                    img_points[2] = Point2f(269.206, 88.5634);
                    img_points[3] = Point2f(243.38, 88.9512);
                    img_points[0] = Point2f(243.151, 115.079);
                    img_points[1] = Point2f(268.709, 114.815);
                    img_points[2] = Point2f(269.532, 88.5649);
                    img_points[3] = Point2f(243.411, 88.9758);
                    */

                    img_points[0] = Point2f(det->p[0][0], det->p[0][1]);
                    img_points[1] = Point2f(det->p[1][0], det->p[1][1]);
                    img_points[2] = Point2f(det->p[2][0], det->p[2][1]);
                    img_points[3] = Point2f(det->p[3][0], det->p[3][1]);


                    obj_points[0] = Point3f(-TAG_SIZE * 0.5f, -TAG_SIZE * 0.5f, 0.f);
                    obj_points[1] = Point3f( TAG_SIZE * 0.5f, -TAG_SIZE * 0.5f, 0.f);
                    obj_points[2] = Point3f( TAG_SIZE * 0.5f,  TAG_SIZE * 0.5f, 0.f);
                    obj_points[3] = Point3f(-TAG_SIZE * 0.5f,  TAG_SIZE * 0.5f, 0.f);


                }
            }

            if (origin_id == -1){
                printf("Did not find a known tag");
            }

            solvePnP(obj_points, img_points, camera_matrix, dist_coeffs, rvec, tvec);

            std::cout << rvec << std::endl;
            std::cout << tvec << std::endl;

            Matx33d r;
            Rodrigues(rvec,r);

            // Construct the origin to camera matrix
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
            Mat origin2cam = Mat(data,true).reshape(1,4);

            Mat cam2origin = origin2cam.inv();

            // DEBUG Generate the location of the camera
            vector<double> data3;
            data3.push_back(0);
            data3.push_back(0);
            data3.push_back(0);
            data3.push_back(1);
            Mat genout = Mat(data3,true).reshape(1,4);
            Mat camcoords = cam2origin * genout;


            printf("%i :: filler :: % 3.3f % 3.3f % 3.3f\n", id,
                    camcoords.at<double>(0,0), camcoords.at<double>(1,0), camcoords.at<double>(2,0));



            //locate_tags
            for (int j = 0; j < zarray_size(detections); j++) {
                // Get the ith detection
                apriltag_detection_t *det;
                zarray_get(detections, j, &det);

                if (known_tags.find(det -> id) == known_tags.end()){
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

                    vector<double> data2;
                    data2.push_back(0);
                    data2.push_back(0);
                    data2.push_back(0);
                    data2.push_back(1);
                    Mat genout = Mat(data2,true).reshape(1, 4);

                    Mat tag2orig = cam2origin * tag2cam;

                    Mat trans_rot;
                    trans_rot = (origin_id == 0) ? tag2orig : known_tags[origin_id] * tag2orig;

                    Mat tagXYZS = trans_rot * genout;
                    printf("%i :: %d :: % 3.3f % 3.3f % 3.3f\n",
                        id, det->id,
                        tagXYZS.at<double>(0), tagXYZS.at<double>(1), tagXYZS.at<double>(2));

                    known_tags.insert({det->id, trans_rot});

                    //for (auto it : known) {
                    //    std::cout << " " << it.first << ":" << it.second;
                    //}

                    //write known_tags to a calibration file
                    printf("written to camera \n");
                    std::ofstream fout;
                    fout.open(std::to_string(id) + ".calib", std::ofstream::out);
                    fout << "camera_matrix =";
                    for (int r = 0; r < camera_matrix.rows; r++) {
                        for (int c = 0; c < camera_matrix.cols; c++) {
                            fout << " " << camera_matrix.at<double>(r, c);
                        }
                    }
                    fout << std::endl;
                    fout << "dist_coeffs =";
                    for (int r = 0; r < dist_coeffs.rows; r++) {
                        for (int c = 0; c < dist_coeffs.cols; c++) {
                            fout << " " << dist_coeffs.at<double>(r, c);
                        }
                    }
                    fout << std::endl;
                    /*fout << "known_tags =";
                    for (std::pair<int, Mat> element : known_tags) {
                        fout << element.first << " :: " << element.second << std::endl;
                    }
                    fout << std::endl;
                    */
                    fout.close();
                }
            }

            zarray_destroy(detections);
        }
        imshow(std::to_string(0), frame);
        key = waitKey(16);
    }
}

//write to calibration file
//optional: more testing
//documentation