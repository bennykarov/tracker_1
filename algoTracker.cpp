#include <thread>
#include <mutex>
#include <iostream>
#include <chrono>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"


#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp> 



#include "utils.hpp"
#include "config.hpp"
#include "trackerBasic.hpp"
#include "mog.hpp"

/*
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
*/
#include "algoTracker.hpp"




#ifdef _DEBUG
#pragma comment(lib, "opencv_core430d.lib")
#pragma comment(lib, "opencv_highgui430d.lib")
#pragma comment(lib, "opencv_imgproc430d.lib")
#pragma comment(lib, "opencv_tracking430d.lib")
#pragma comment(lib, "opencv_videoio430d.lib")
#pragma comment(lib, "opencv_video430d.lib")
/*
#pragma comment(lib, "opencv_videoio430d.lib")
#pragma comment(lib, "opencv_imgcodecs430d.lib")
#pragma comment(lib, "opencv_bgsegm430d.lib")
*/
//#pragma comment(lib, "opencv_calib3d430d.lib")
//#pragma comment(lib, "opencv_bgsegm430d.lib")


#else
#pragma comment(lib, "opencv_core430.lib")
#pragma comment(lib, "opencv_highgui430.lib")
//#pragma comment(lib, "opencv_imgcodecs430.lib")
#pragma comment(lib, "opencv_imgproc430.lib")
#endif


/*---------------------------------------------------------------------------------------------
								U T I L S
---------------------------------------------------------------------------------------------*/
class CRoi2frame {
public:
	cv::Rect   bbox;
	int frameNum;
};


class CObject {
public:
	CObject(cv::Rect  r)
	{
		m_bboxes.push_back(r);
		classify();
	}

	int len() { return (int)m_bboxes.size(); }

	void set(int motionFrames, int motionDist)
	{
		m_motionFrames = motionFrames;
		m_motionDist = motionDist;
	}

	void add(cv::Rect p)
	{
		m_bboxes.push_back(p);
	}

	// Add last points to RECT - check for the box  dimensios 
	bool isMove(int dist = 10)
	{
		if (m_bboxes.size() < m_motionFrames)
			return true;

		cv::Rect motionBox;
		for (int i = (int)m_bboxes.size() - 1; i > (int)m_bboxes.size() - m_motionFrames; i--) {
			motionBox = extendBBox(motionBox, centerOf(m_bboxes[i]));
		}

		if (motionBox.width > dist || motionBox.height > dist)
			return true;
		else
			return false;
	}

	int classify( /*cv::Mat img*/)
	{
		if (m_bboxes.back().width < 70)
			m_label = 1;
		else
			m_label = 2;

		return m_label;

	}

	int       m_label = 0;
	std::vector<cv::Rect>   m_bboxes;
	int m_motionFrames = 1 * CONSTANTS::FPS; // DDEBUG CONST 
	int m_motionDist;
};


std::vector <CRoi2frame>  readROISfile(std::string fname)
{
	std::vector <CRoi2frame>  rois2frames;
	//int frameNum;
	ifstream roisFile(fname);

	if (!roisFile.is_open())
		return std::vector <CRoi2frame>();

	CRoi2frame newBBox2f;
	int index;
	while (!roisFile.eof()) {
		roisFile >> index >> newBBox2f.frameNum >> newBBox2f.bbox.x >> newBBox2f.bbox.y >> newBBox2f.bbox.width >> newBBox2f.bbox.height;
		if (index > 0 && !roisFile.eof())
			rois2frames.push_back(newBBox2f);
	}

	return rois2frames;
}



bool readConfigFile(std::string ConfigFName, Config &conf)
{
	if (!FILE_UTILS::file_exists(ConfigFName))
	{
		std::cout << "WARNING : Can't find Config.ini file, use default values \n";
		return false;
	}

	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(ConfigFName, pt);
	// [GENERAL]
	conf.videoName = pt.get<std::string>("GENERAL.video", conf.videoName);
	conf.roisName = pt.get<std::string>("GENERAL.rois", conf.roisName);
	conf.trackerType = pt.get<int>("GENERAL.tracker", conf.trackerType);
	conf.waitKeyTime = pt.get<int>("GENERAL.wait-key", conf.waitKeyTime);
	conf.displayScale = pt.get<float>("GENERAL.scale", conf.displayScale);
	conf.record = pt.get<int>("GENERAL.record", conf.record);
	conf.demoMode = pt.get<int>("GENERAL.demo", conf.demoMode);


	return true;
}


/*---------------------------------------------------------------------------------------------
 ---------------------------------------------------------------------------------------------*/

void CTrack::init(int w, int h, float scaleDisplay)
	{
		m_width = w;
		m_height = h;

		Config conf;
		readConfigFile("./config.ini", conf);
		// Read RIO's from a file 
		std::vector <CRoi2frame>  roiList;
		if (!conf.roisName.empty())
			roiList = readROISfile(conf.roisName);

	}

	int CTrack::process(void *dataTemp)
	{

		size_t sizeTemp(m_width * m_height * 3); // 24 bit
		if (m_data == NULL)
			m_data  = malloc(sizeTemp);


		memcpy( m_data, dataTemp, sizeTemp); // buffering NOT optimized
		m_frame = cv::Mat(m_height, m_width, CV_8UC3, m_data);
		if (m_frame.empty())
			std::cout << "read() got an EMPTY frame\n";

		return m_frameNum++;
	}


	int CTrack::show()
	{
		cv::resize(m_frame, m_display, cv::Size(0, 0), m_scaleDisplay, m_scaleDisplay);
		cv::Point textPos(50, 60); // frameNum display
		cv::putText(m_display, std::to_string(m_frameNum), textPos, cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar(0, 200, 50), 2);

		cv::imshow("frame", m_display);

		return cv::waitKey(10);
	}


