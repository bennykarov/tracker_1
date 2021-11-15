#pragma once

#include <stdio.h>      /* printf, scanf, NULL */
#include <stdlib.h>     /* malloc, free, rand */


#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"

#include "mog.hpp"

class CTrack {
public:
	void init(int w, int h, float scaleDisplay = 0.5);
	int process(void *dataTemp);
	int show();


private:
	int m_width = 0;
	int m_height = 0;
	void *m_data = NULL;
	int m_frameNum = 0;
	float m_scaleDisplay = 1.;


	cv::Mat m_frame;
	cv::Mat m_display;


	CBGSubstruct   m_bgSeg;
	int status=0;
};
