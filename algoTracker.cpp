#include <thread>
#include <mutex>
#include <iostream>
#include <chrono>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/videoio.hpp"

//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"


#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp> 



#include "utils.hpp"
#include "config.hpp"
#include "trackerBasic.hpp"
#include "mog.hpp"
#include "CObject.hpp"

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
#pragma comment(lib, "opencv_features2d430d.lib")
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
#pragma comment(lib, "opencv_imgproc430.lib")
#pragma comment(lib, "opencv_tracking430.lib")
#pragma comment(lib, "opencv_videoio430.lib")
#pragma comment(lib, "opencv_video430.lib")
#endif


/*---------------------------------------------------------------------------------------------
								U T I L S
---------------------------------------------------------------------------------------------*/

#if 0
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
#endif 

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
	if (!FILE_UTILS::file_exists(ConfigFName))  {
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
	conf.displayScale = pt.get<float>("GENERAL.scaleDisplay", conf.displayScale);
	conf.scale = pt.get<float>("GENERAL.scale", conf.scale);
	conf.record = pt.get<int>("GENERAL.record", conf.record);
	conf.demoMode = pt.get<int>("GENERAL.demo", conf.demoMode);
	// ALGO:
	conf.MHistory = pt.get<int>("ALGO.MHistory", conf.MHistory);
	conf.MvarThreshold = pt.get<float>("ALGO.MvarThreshold", conf.MvarThreshold);
	conf.MlearningRate = pt.get<float>("ALGO.MlearningRate", conf.MlearningRate);


	return true;
}


/*---------------------------------------------------------------------------------------------
 ---------------------------------------------------------------------------------------------*/

void CTrack::init(int w, int h, int imgSize , float scaleDisplay)
	{
		m_width = w;
		m_height = h;
		m_colorDepth = imgSize / (w*h);

		readConfigFile("config.ini", m_params);
		// Read RIO's from a file 
		if (!m_params.roisName.empty())
			m_roiList = readROISfile(m_params.roisName);
		if (m_scale != 1.)
			for (auto &roi : m_roiList)
				roi.bbox = scaleBBox(roi.bbox, m_scale);

		// MOG2 
		bool detectShadows = false;
		int emphasize = 1;
		m_bgSeg.init(m_params.MHistory , m_params.MvarThreshold ,detectShadows, emphasize);
		m_bgSeg.setLearnRate(m_params.MlearningRate);

	}


	int depth2cvType(int depth)
	{
		
		switch (depth) {
		case 1:
			return  CV_8UC1;
			break;
		case 2:
			return  CV_8UC2;
			break;
		case 3:
			return  CV_8UC3;
			break;
		case 4:
			return  CV_8UC4;
			break;
		}
	}

	int CTrack::processFrame(cv::Mat frame)
	{
		int tracked_count = 0;

		// Find new motion 
		if (m_frameNum % 2 == 0) {
			cv::Mat sFrame;
			cv::resize(frame, sFrame, cv::Size(0, 0), 0.5, 0.5);
			cv::Mat bgMask = m_bgSeg.process(sFrame);
			if (!bgMask.empty())
				cv::imshow("mog", bgMask);

			

			// -1- Find  blobs (objects)
			//---------------------------
			if (!bgMask.empty()) {
				//std::vector<cv::KeyPoint> myBlobs = findBySimpleBlob(bgMask);
				std::vector<cv::Rect>    newROIs = findByContours(bgMask);
				matchObjects(newROIs);
			}
		}

		consolidateDetection();

		// Remove un-detected objects (fade)
		for (int i = 0; i < m_objects.size();i++) {
			int hiddenLenAllowed = m_objects[i].m_bboxes.size() < 4 ? 3 : 10;
			if (m_objects[i].m_lastDetected + hiddenLenAllowed < m_frameNum) 
				m_objects.erase(m_objects.begin() + i--);
		}



		// init tracker from file's list 
		//int ind=0;
		for (auto &roi : m_roiList) {
			// Add the new tracker
			if (m_frameNum == roi.frameNum) {
				CTracker *newTracker = new CTracker;
				m_trackers.push_back(*newTracker);
				m_trackers.back().init(m_params.trackerType, 0);
				m_trackers.back().setROI(frame, roi.bbox);
				roi.frameNum *= -1; // sign as nonactive 
				// Add additionla obj data
				int objLabel = roi.bbox.width > 70 ? 2 : 1; // classifies obj by ROI size + DDEBUG CONST  
				m_objects.push_back(CObject(roi.bbox, m_frameNum));
				//roiList.erase(roiList.begin()+ind); // remove from init list 
			}
		}

		// Track 
		int ind = 0;
		for (auto tracker : m_trackers)
		{
			if (m_params.demoMode == 1 && ind == 6 && m_frameNum == 140)  m_trackers.erase(m_trackers.begin() + ind); // DDEBUG 
			if (m_params.demoMode == 2 && ind == 3 && m_frameNum == 350)  m_trackers.erase(m_trackers.begin() + ind); // DDEBUG 

			//tracker.reset();
			if (tracker.isActive())
				if (tracker.track(frame)) {
					m_trackerROI = tracker.getBBox();
					m_objects[ind].add(m_trackerROI);
					// Set rect color:
					cv::Scalar color;
					if (!m_objects[ind].isMove(10))
						color = cv::Scalar(100, 200, 250);
					else
						color = m_objects[ind].m_label == 1 ? cv::Scalar(0, 200, 100) : cv::Scalar(0, 100, 200);

					// DDEBUG - Expand display of too nerrow tracker-box  
					if (m_trackerROI.width < 30) {
						m_trackerROI.x -= (30 - m_trackerROI.width) / 2;
						m_trackerROI.width = 30;
					}

					/*
					cv::rectangle(frame, m_trackerROI, color, 3);
					// tracker.reset();
					// color = cv::Scalar(0, 0, 0);

					// Add time on screen 
					int seconds = int((float)m_objects[ind].len() / (float)CONSTANTS::FPS);
					cv::Point text_pos = cv::Point(m_trackerROI.x, m_trackerROI.y - 10);
					cv::putText(frame, std::to_string(seconds + 1), text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
					*/

					tracked_count++;
				}
			ind++;
		}

#if 0
		cv::Mat display;
		if (m_params.displayScale != 1.)
			cv::resize(frame, display, cv::Size(0, 0), m_params.displayScale, m_params.displayScale);
		else
			display = frame;

		cv::Point textPos(50, 60); // frameNum display
		cv::putText(display, std::to_string(m_frameNum), textPos, cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar(0, 200, 50), 2);


		imshow("video", display);

		/*
		if (m_params.record)
			demoRecorder.write(display);
		*/

		int key = cv::waitKey(10);
#endif 
		return tracked_count;


	}

	int CTrack::process(void *dataTemp)
	{
		cv::Mat frameIn;
		size_t sizeTemp(m_width * m_height * m_colorDepth); 
		if (m_data == NULL)
			m_data  = malloc(sizeTemp);

		memcpy( m_data, dataTemp, sizeTemp); // buffering NOT optimized
		frameIn = cv::Mat(m_height, m_width, depth2cvType(m_colorDepth), m_data);
		if (frameIn.empty())
			std::cout << "read() got an EMPTY frame\n";
		else
			cv::resize(frameIn, m_frame, cv::Size(0, 0), m_scale, m_scale);

		processFrame(m_frame);
		return m_frameNum++;
	}



	int CTrack::show()
	{
		static int key = 0;
		cv::Mat display;
		if (m_params.displayScale != 1.)
			cv::resize(m_frame, display, cv::Size(0, 0), m_params.displayScale, m_params.displayScale);
		else
			display = m_frame;

		cv::Point textPos(50, 60); // frameNum display
		cv::putText(display, std::to_string(m_frameNum), textPos, cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar(0, 200, 50), 2);

		// Draw detection rects
		for (int i = 0; i < m_objects.size();i++) {
			//if (m_objects[i].m_bboxes.size() < 10)
			if (m_objects[i].m_detectionType < 1)
				continue;
			cv::Scalar color(0, 0, 250, 2);
			cv::rectangle(display, enlargeBBox(m_objects[i].m_bboxes.back(),0.6) , color);
			cv::Point text_pos = cv::Point(m_objects[i].m_bboxes.back().x, m_objects[i].m_bboxes.back().y - 10);
			cv::putText(display, std::to_string(i), text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
			/*
			// Add time on screen 
			int seconds = int((float)obj.len() / (float)CONSTANTS::FPS);
			cv::Point text_pos = cv::Point(obj.m_bboxes.back().x, obj.m_bboxes.back().y - 10);
			cv::putText(display, std::to_string(seconds + 1), text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
			*/
		}


		imshow("video", display);

		/*
		if (m_params.record)
			demoRecorder.write(display);
		*/

		key = cv::waitKey(10);
		if (m_frameNum == 1) // DDEBUG 
			key = 'p';
		if (key == 'p')
			key = cv::waitKey(-1);

		return key;

	}



	std::vector<cv::KeyPoint> CTrack::findBySimpleBlob(cv::Mat bgMask)
	{
		std::vector<cv::KeyPoint> myBlobs;
		// Find blobs using SIMPLEBOBS
		cv::SimpleBlobDetector::Params params;
		params.minDistBetweenBlobs = 10.0;  // minimum 10 pixels between blobs

		params.filterByCircularity = false;
		params.filterByColor = false;
		params.filterByConvexity = false;
		params.filterByInertia = false;

		params.filterByArea = true;         // filter my blobs by area of blob
		params.minArea = 2 * 2;
		params.maxArea = 100 * 100;

		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

		detector->detect(bgMask, myBlobs);

		return myBlobs;

		/*
		cv::Mat im_with_keypoints;
		if (0)
			cv::drawKeypoints(sFrame, myBlobs, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		else {
			for (auto blob : myBlobs)
				cv::circle(sFrame, blob.pt, 4, cv::Scalar(0, 0, 255));
		}
		*/
	}



	std::vector <cv::Rect> CTrack::findByContours(cv::Mat bgMask)
	{
		// Find blobs using CONTOURS 
		std::vector < std::vector<cv::Point>> contours, good_contours;
		//std::vector <cv::Rect> eyeROIs;
		std::vector<cv::Rect>    newROIs;
		std::vector<cv::Vec4i> hierarchy;

		findContours(bgMask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		const int MIN_CONT_AREA = 4;
		const int MAX_CONT_AREA = 80 * 80;

		// Contour analysis
		float min_area = MIN_CONT_AREA;
		//float max_area = MAX_ROI_AREA; 
		float max_area = MAX_CONT_AREA;
		float goodAspectRatio = 0.5;
		float aspectRatioTolerance = 0.2;

		//m_objects.clear(); // DDEBUG 

		for (auto cont : contours) {
			int area = contourArea(cont, false);
			if (area < MIN_CONT_AREA || area > MAX_CONT_AREA)
				continue;
			cv::Rect box = cv::boundingRect(cont);
			//rbox = cv::minAreaRect(cont);
			/*
			attrib.aspectRatio = attrib.rbox.size.height / attrib.rbox.size.width;
			attrib.perimeter = cv::arcLength(cv::Mat(contours[i]), true);
			attrib.thickness = attrib.perimeter / attrib.area;
			attrib.close = (hierarchy[i][2] < 0 && hierarchy[i][3] < 0) ? -1 : 1;
			attrib.len = MAX(attrib.rbox.size.width, attrib.rbox.size.height);
			attrib.topLevel = hierarchy[i][3] == -1; // no paretn
			*/
			cv::Rect debug = scaleBBox(box, 1. / 0.5);
			newROIs.push_back(debug);
		}

		return newROIs;
	}



	/*-----------------------------------------------------------
	 * Match RECTs to exiting m_objects
	 * Add new to m_objects if no match was found
	 * Return number of new objects 
	 *-----------------------------------------------------------*/
	int CTrack::matchObjects(std::vector<cv::Rect> newROIs)
	{
		/*-------------------------------------------
		 * match new objects to the new ones
		 * Check box overlapping to match new & old
		 -------------------------------------------*/
		std::vector <int> match(newROIs.size(), 0);
		//match.assign(newROIs.size(), 0);
		for (int i = 0; i < newROIs.size(); i++)
			for (auto &oldObj : m_objects)
				if ((newROIs[i] & oldObj.m_bboxes.back()).area() > 0
					//&&  bboxRatio(newROIs[i], oldObj.m_bboxes.back()) < 0.7
					) {
					oldObj.m_bboxes.push_back(newROIs[i]);
					match[i] = 1; // Mark this ROI as matched
					oldObj.m_lastDetected = m_frameNum;
					break;
				}

		for (int i = 0; i < match.size(); i++) {
			if (match[i] == 0) {
				// A new object
				m_objects.push_back(CObject(newROIs[i], m_frameNum));
			}
		}

		return (int)match.size() - cv::countNonZero(match);

	}


	void CTrack::consolidateDetection()
	{
		const int warmupLen = 10;
		const int stableLen = CONSTANTS::FPS * 3;
		for (auto &obj : m_objects) {
			obj.m_detectionType = 0;
			if (obj.len() > warmupLen) {
				obj.m_detectionType = 1;
				if (obj.len() > stableLen) {
					obj.m_detectionType = 2;
					// Check for rect stability 
					int areaDiff = 0;
					int ind = obj.len() - 1;
					for (int i = 0; i > 4; ind--)
						areaDiff += obj.m_bboxes[ind].area() - obj.m_bboxes[ind - 1].area();
					areaDiff = abs(areaDiff);
					float areaRatio = (float)areaDiff / (float)obj.m_bboxes.back().area();
					if (areaRatio > 1.)
						areaRatio = 1. / areaRatio;

					if (areaRatio > 0.7) 
						obj.m_detectionType = 3;
				}
			}
		}

	}