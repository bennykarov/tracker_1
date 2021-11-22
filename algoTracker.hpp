#pragma once

#include <stdio.h>      /* printf, scanf, NULL */
#include <stdlib.h>     /* malloc, free, rand */


#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"

#include "mog.hpp"


class CRoi2frame {
public:
	cv::Rect   bbox;
	int frameNum;
};

class CTrack {
public:
	void init(int w, int h, int imgSize, float scaleDisplay = 0.5);
	int process(void *dataTemp);
	int processFrame(cv::Mat frame);

	int show();

private:
	std::vector<cv::KeyPoint> findBySimpleBlob(cv::Mat img);
	std::vector <cv::Rect> findByContours(cv::Mat bgMask);
	int matchObjects(std::vector<cv::Rect> newROIs);
	void consolidateDetection();

private:
	int m_width = 0;
	int m_height = 0;
	void *m_data = NULL;
	int m_frameNum = 0;
	float m_scale = 0.5;
	float m_scaleDisplay = 1.;


	cv::Mat m_frame;
	cv::Mat m_display;


	CBGSubstruct   m_bgSeg;
	int status=0;
	int m_colorDepth=4;

	// Tracker members
	cv::Rect m_trackerROI;
	// object "class":
	std::vector<CTracker>  m_trackers;
	std::vector<CObject>       m_objects;
	std::vector <CRoi2frame>  m_roiList;
	Config m_params;

};
