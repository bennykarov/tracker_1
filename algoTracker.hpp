#pragma once

#include <stdio.h>      /* printf, scanf, NULL */
#include <stdlib.h>     /* malloc, free, rand */


#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"

#include "mog.hpp"
#include "CObject.hpp"


class CRoi2frame {
public:
	cv::Rect   bbox;
	int frameNum;
};


enum AGE {
	BORN = 0,
	STARTER, // 1,
	FINE,	// 2
	STABLE, // 3
	TRACKED, // 4
	HIDDEN	// 5
};


class CTrack {
public:
	void init(int w, int h, int imgSize, float scaleDisplay = 0.5);
	int process(void *dataTemp);
	int processFrame(cv::Mat &frame);
	void draw(cv::Mat &img, float scale);
	int draw();

private:
	std::vector<cv::KeyPoint> detectBySimpleBlob(cv::Mat img);
	std::vector <cv::Rect> detectByContours(cv::Mat bgMask);
	int detectByTracker(cv::Mat frame);
	bool detectObjByOpticalFlow(cv::Mat frame, int objInd);
	int detectByTracker_OLD(cv::Mat frame);

	bool checkAreStability(std::vector <cv::Rect>, int len);

	int matchObjects(std::vector<cv::Rect> newROIs);
	void consolidateDetection();
	void removeShadows(float shadowClockDirection);
	void removeShadows(std::vector<cv::Rect>  &newROIs, std::vector<LABEL> labels, float shadowClockDirection);
	void removeShadow(CObject &obj, float shadowClockDirection);
	std::vector<LABEL>   classify(cv::Mat img, cv::Mat bgMask, std::vector <cv::Rect>  rois);


private:
	int m_width = 0;
	int m_height = 0;
	void *m_data = NULL;
	int m_frameNum = 0;
	//float m_calcScale = 0.5;
	float m_scaleDisplay = 1.;// 0.7;


	cv::Mat m_frameOrg; // Original image
	cv::Mat m_frame; // working image
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
