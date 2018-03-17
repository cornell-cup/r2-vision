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
    vector<double> origin_tag = {0,0,0,0,0,0,0,0,0};

    std::unordered_map<int, Mat> known_tags = {
        {0, Mat(origin_tag, true).reshape(1,3)};
    };

    //setup the hardware part

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

        zarray_t* detections = apriltag_detector_detect(td, &im);

        vector<Point2f> img_points(4);
        vector<Point3f> obj_points(4);
        Mat rvec(3, 1, CV_64FC1);
        Mat tvec(3, 1, CV_64FC1);

        //locate_origin
        for (int j = zarray_size(detections) - 1; j >= 0; j--) {
                // Get the ith detection
                apriltag_detection_t *det;
                zarray_get(detections, j, &det);

                //sees if we can use the tag detected as the origin
                if (known_tags.find(det -> id) != known_tags.end()) {
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

                    obj_points[0] = Point3f(-0.5f * TAG_SIZE, -0.5f * TAG_SIZE, 0.f);
                    obj_points[1] = Point3f( 0.5f * TAG_SIZE, -0.5f * TAG_SIZE, 0.f);
                    obj_points[2] = Point3f( 0.5f * TAG_SIZE,  0.5f * TAG_SIZE, 0.f);
                    obj_points[3] = Point3f(-0.5f * TAG_SIZE,  0.5f * TAG_SIZE, 0.f);
                }
            }
        }

        solvePnP(obj_points, img_points, device_camera_matrix[i],
        device_dist_coeffs[i], rvec, tvec);

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

        //DEBUG Generate the location of the camera?

        //locate_tags

        //make a new zarray with only detected unknown tag IDs
        unknown_tags


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

            solvePnP(obj_points, img_points, device_camera_matrix[i],
                    device_dist_coeffs[i], rvec, tvec);
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

            Mat tag2orig = device_transform_matrix[i] * tag2cam;
            Mat tagXYZS = tag2orig * genout;


            //f we are using another tag as the origin, need to compute again
            printf("%zu :: %d :: % 3.3f % 3.3f % 3.3f\n",
                    i, det->id,
                    tagXYZS.at<double>(0), tagXYZS.at<double>(1), tagXYZS.at<double>(2));

            //write it to a calibration file

            // Send data to basestation
            sprintf(postDataBuffer, "{\"id\":%d,\"x\":%f,\"y\":%f,\"z\":%f}",
                    det->id, tagXYZS.at<double>(0), tagXYZS.at<double>(1), tagXYZS.at<double>(2));
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postDataBuffer);
            // TODO Check for error response
            curl_easy_perform(curl);
        }

        zarray_destroy(detections);

        imshow(std::to_string(i), frame);
        key = waitKey(16);
    }
    curl_easy_cleanup(curl);
}