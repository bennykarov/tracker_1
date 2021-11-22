#pragma once 

//int main_mog(std::string videoName);
#include "opencv2/opencv_modules.hpp"


class CBGSubstruct {
public:
	int main(std::string videoName);
	void init(int History = 50, double varThreshold = 30.0, bool detectShadows = true, int emphasize=0);
	void setLearnRate(double rate) { m_learningRate = rate;}  // Negative parameter value makes the algorithm to use some automatically chosen learning rate
	cv::Mat process(cv::Mat img);
	void startLearn() { m_learningRate = -1.;}  // Negative parameter value makes the algorithm to use some automatically chosen learning rate
	void pauseLearn() { m_learningRate = 0;}   // 0 means that the background model is not updated at all, 

	void setDebugLevel(int level) { m_debugLevle = level; }


private:
	void emphasizeMask(cv::Mat &mask,int enlargeDepth=1);
private:
    cv::Ptr<cv::BackgroundSubtractor> m_pBackSub;
    double m_learningRate = -1;// DDEBUG -1;
	int m_debugLevle = 0;
	int m_frameNum = 0;
	int m_emphasize = 1;
    //int learning_frame = 0;
};
