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


}