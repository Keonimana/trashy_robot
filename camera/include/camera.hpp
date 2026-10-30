#include <zmq.hpp>
#include <iostream>
#include <cstdio>
#include <string>
#include <thread>
#include <future>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

struct CommandPoint {
    float x, y, z;
};