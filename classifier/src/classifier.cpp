#include "classifier.hpp"

using namespace std;
using namespace cv;

int main() {

    Mat image, hsvImage, mask;
    vector<vector<Point>> contours;

    Scalar lowerBound(160, 50, 50); 
    Scalar upperBound(180, 255, 255); 

    zmq::context_t context(1);

    zmq::socket_t img_subscriber(context, zmq::socket_type::sub);
    img_subscriber.connect("tcp://localhost:5553");
    img_subscriber.set(zmq::sockopt::subscribe, "");

    zmq::socket_t publisher(context, zmq::socket_type::pub);
    publisher.bind("tcp://*:5555");

    cout << "Starting!" << endl;

    while (true) {
        zmq::message_t imageMessage;
        img_subscriber.recv(imageMessage, zmq::recv_flags::none);

        // Decode image
        const char* messageData = static_cast<const char*>(imageMessage.data());
        vector<uchar> imageData(messageData, messageData + imageMessage.size());
        image = imdecode(imageData, IMREAD_COLOR);
        if (image.empty()) {
            continue;
        }

        // Convert to HSV
        cvtColor(image, hsvImage, COLOR_BGR2HSV);
        inRange(hsvImage, lowerBound, upperBound, mask);

        // Find contours
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Find the largest object
        double maxArea = 0;
        int largestContourIndex = -1;

        for (size_t i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                largestContourIndex = i;
            }
        }

        // Find center of object in frame
        Point2f centerPoint(-1, -1);

        if (largestContourIndex != -1) {
            Moments moments = cv::moments(contours[largestContourIndex]);
            if (moments.m00 > 0) {
                centerPoint = Point2f(static_cast<float>(moments.m10 / moments.m00), static_cast<float>(moments.m01 / moments.m00));
            }
        }

        CommandPoint objective = {centerPoint.x, centerPoint.y, 0.0};

        zmq::message_t msg(sizeof(CommandPoint));
        memcpy(msg.data(), &objective, sizeof(CommandPoint));
        publisher.send(msg, zmq::send_flags::dontwait);
    }

    return 0;
}
