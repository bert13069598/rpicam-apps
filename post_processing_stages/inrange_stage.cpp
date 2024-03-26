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

	void Read(boost::property_tree::ptree const &params) override {}

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	Stream *stream_;
};

#define NAME "inrange"

char const *InrangeStage::Name() const
{
	return NAME;
}

void InrangeStage::Configure()
{
	stream_ = app_->GetMainStream();
}

bool InrangeStage::Process(CompletedRequestPtr &completed_request)
{
	StreamInfo info = app_->GetStreamInfo(stream_);
	BufferWriteSync w(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	
	StreamInfo rgb_info;
	rgb_info.width = info.width;
	rgb_info.height = info.height;
	rgb_info.stride = info.width * 3;
	vector<uint8_t> rgb_image = Yuv420ToRgb(buffer.data(), info, rgb_info);
	Mat imageMat(info.height, info.width, CV_8UC3, rgb_image.data());
	cvtColor(imageMat, imageMat, COLOR_RGB2BGR);

	int left=10, top=10, width=300, height=300;
	rectangle(imageMat, Point(left, top), Point(left + width, top + height), Scalar(255, 0, 0), 1);

	imshow("RGB Image", imageMat);
	waitKey(20);

	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new InrangeStage(app);
}

static RegisterStage reg(NAME, &Create);
