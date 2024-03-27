/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * inrange_stage.cpp - image inrange effect
 */

#include <libcamera/stream.h>

#include "core/rpicam_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "iostream"

using namespace cv;
using namespace std;
using Stream = libcamera::Stream;

class InrangeStage : public PostProcessingStage
{
public:
	InrangeStage(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	Stream *stream_;
	
	int low_hue1, low_hue2, high_hue1, high_hue2, low_saturation, low_value;
	Scalar color;
};

#define NAME "inrange"

char const *InrangeStage::Name() const
{
	return NAME;
}

void InrangeStage::Read(boost::property_tree::ptree const &params)
{
        int b = params.get<int>("color.b", 0);
        int g = params.get<int>("color.g", 0);
        int r = params.get<int>("color.r", 255);
        color = Scalar(b, g, r);
		low_saturation = params.get<int>("low_saturation", 50);
		low_value = params.get<int>("low_value", 50);
}

void InrangeStage::Configure()
{
	stream_ = app_->GetMainStream();

	Mat rgb_color = Mat(1, 1, CV_8UC3, color);
	Mat hsv_color;
	cvtColor(rgb_color, hsv_color, COLOR_BGR2HSV);

	int hue = (int)hsv_color.at<Vec3b>(0, 0)[0];
	// int saturation = (int)hsv_color.at<Vec3b>(0, 0)[1];
	// int value = (int)hsv_color.at<Vec3b>(0, 0)[2];

	int low_hue = hue - 10;
	int high_hue = hue + 10;
	low_hue1 = 0;
	low_hue2 = 0;
	high_hue1 = 0;
	high_hue2 = 0;

	if (low_hue < 10 ) {
		high_hue1 = 180;
		low_hue1 = low_hue + 180;
		high_hue2 = high_hue;
		low_hue2 = 0;
	}
	else if (high_hue > 170) {
		high_hue1 = low_hue;
		low_hue1 = 180;
		high_hue2 = high_hue - 180;
		low_hue2 = 0;
	}
	else {
		low_hue1 = low_hue;
		high_hue1 = high_hue;
	}
}

bool InrangeStage::Process(CompletedRequestPtr &completed_request)
{
	StreamInfo info = app_->GetStreamInfo(stream_);
	BufferWriteSync w(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	uint8_t *ptr = (uint8_t *)buffer.data();

	Mat src = Mat(info.height, info.width, CV_8U, ptr, info.stride);
	
	StreamInfo rgb_info;
	rgb_info.width = info.width;
	rgb_info.height = info.height;
	rgb_info.stride = info.width * 3;
	vector<uint8_t> rgb_image = Yuv420ToRgb(buffer.data(), info, rgb_info);
	Mat imageMat(info.height, info.width, CV_8UC3, rgb_image.data());
	cvtColor(imageMat, imageMat, COLOR_RGB2HSV);

	Mat img_mask, img_mask1, img_mask2;
	inRange(imageMat, Scalar(low_hue1, low_saturation, low_value), Scalar(high_hue1, 255, 255), img_mask1);
	inRange(imageMat, Scalar(low_hue2, low_saturation, low_value), Scalar(high_hue2, 255, 255), img_mask2);
	img_mask = img_mask1 | img_mask2;

	erode(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	Mat img_labels, stats, centroids;
	int numOfLables = connectedComponentsWithStats(img_mask, img_labels, stats, centroids, 8, CV_32S);

	int max = -1, idx = 0;
	for (int j = 1; j < numOfLables; j++) {
		int area = stats.at<int>(j, CC_STAT_AREA);
		if (max < area)	{
			max = area;
			idx = j;
		}
	}

	if (max != -1){
		int left = stats.at<int>(idx, CC_STAT_LEFT);
		int top = stats.at<int>(idx, CC_STAT_TOP);
		int width = stats.at<int>(idx, CC_STAT_WIDTH);
		int height = stats.at<int>(idx, CC_STAT_HEIGHT);

		rectangle(src, Point(left, top), Point(left + width, top + height), Scalar(255, 255, 255), 2);
	}

	resize(img_mask, img_mask, Size(info.width / 4, info.height / 4), 0, 0, INTER_LINEAR);
	imshow("Mask Image", img_mask);
	waitKey(10);

	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new InrangeStage(app);
}

static RegisterStage reg(NAME, &Create);
