/*
 * main.cpp
 *
 *  Created on: Jan 26, 2018
 *      Author: conner
 */
#include <vector>
#include <queue>
#include <algorithm>
#include <iostream>
#include <utility>
#include <future>
#include <chrono>
#include <cmath>
#include <errno.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <mosquitto.h>

const double CUBE_WIDTH = 13;
const double FOV = 56.9;

constexpr double toRadians(const double angle) {
	return (angle * M_PI) / 180.0;
}

constexpr double toDegrees(const double angle) {
	return (angle * 180.0) / M_PI;
}

void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, cv::Scalar c) {
	cv::Point2f points[4];
	rect.points(points);
	cv::circle(img, rect.center, 3, c);
	for (int j = 0; j < 4; j++)
		cv::line(img, points[j], points[(j + 1) % 4], c, 1, 8);
}

void my_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str) {
#	ifndef NDEBUG
	std::cerr << "Mosquitto(" << level << "): " << str << std::endl;
#	endif
}

std::tuple<double, double, cv::Mat, cv::Mat> process(const cv::Mat& frame) {
	//cv::GaussianBlur(frame, frame, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);

	cv::Mat conv;
	cv::cvtColor(frame, conv, cv::COLOR_BGR2HSV);

	cv::Mat binary;
	cv::inRange(conv, cv::Scalar(25, 80, 25), cv::Scalar(35, 255, 255), binary);

	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(binary, contours, cv::RETR_EXTERNAL,
			cv::CHAIN_APPROX_TC89_KCOS);

	auto cube = std::max_element(contours.begin(), contours.end(),
		[](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
			return cv::contourArea(a) < cv::contourArea(b);
		});
	if (cube != contours.end() && cv::contourArea(*cube)) {
		cv::RotatedRect cubeRect = cv::minAreaRect(*cube);
		double distance = (frame.cols * CUBE_WIDTH)/ (2.0 * cubeRect.size.width * std::tan(toRadians(FOV / 2.0)));

		cv::Point2f frameCenter = cv::Point2f(frame.cols / 2.0, frame.rows / 2.0);
		cv::Point2f centerRel = cubeRect.center - frameCenter;
		double angle = toDegrees(std::atan(centerRel.x / (frameCenter.x / std::tan(toRadians(FOV / 2.0)))));

		return std::make_tuple(distance, angle, frame, binary);
	}

	return std::make_tuple(-1, -1, frame, binary);
}

int main() {
	using namespace std::chrono_literals;

	mosquitto_lib_init();
	mosquitto* mqtt = mosquitto_new(NULL, true, NULL);
	if (!mqtt) {
		std::cerr << "Failed to init mosquitto: " << strerror(errno) << std::endl;
		return -1;
	}

	mosquitto_log_callback_set(mqtt, my_log_callback);
	mosquitto_loop_start(mqtt);
	mosquitto_connect_async(mqtt, "127.0.0.1", 1883, 5);

	cv::VideoCapture camera(-1);

	std::future<cv::Mat> frameFuture = std::async([&camera] () {
		cv::Mat frame;
		camera >> frame;
		return frame;
	});

	while (camera.isOpened()) {

		double distance, angle;
		cv::Mat frame = frameFuture.get(), binary;

		frameFuture = std::async([&camera] () {
			cv::Mat frame;
			camera >> frame;
			return frame;
		});

		std::tie(distance, angle, frame, binary) = process(frame);

		std::string distStr = std::to_string(distance);
		std::string angleStr = std::to_string(angle);

		mosquitto_publish(mqtt, NULL, "distance", distStr.length(), distStr.c_str(), 1, false);
		mosquitto_publish(mqtt, NULL, "angle", angleStr.length(), angleStr.c_str(), 1, false);

#		ifndef NDEBUG
#		ifndef RPI
		cv::imshow("Frame", frame);
		cv::imshow("Binary", binary);
		cv::waitKey(1);
		//std::cout << "d=" << distance << ", a=" << angle << std::endl;
#		endif
#		endif
	}

	mosquitto_disconnect(mqtt);
	mosquitto_loop_stop(mqtt, true);
	mosquitto_destroy(mqtt);
	mosquitto_lib_cleanup();

	return 0;
}

