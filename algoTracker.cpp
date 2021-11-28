#include <thread>
#include <mutex>
#include <iostream>
#include <chrono>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/videoio.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp> 


#include "utils.hpp"
#include "config.hpp"
#include "mog.hpp"
#include "trackerBasic.hpp"

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
#pragma comment(lib, "opencv_features2d430.lib")
#endif


/*---------------------------------------------------------------------------------------------
								U T I L S
---------------------------------------------------------------------------------------------*/

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
	/*
	conf.displayScale = pt.get<float>("GENERAL.scaleDisplay", conf.displayScale);
	conf.calcScale = pt.get<float>("GENERAL.scale", conf.calcScale);
	*/
	conf.record = pt.get<int>("GENERAL.record", conf.record);
	conf.demoMode = pt.get<int>("GENERAL.demo", conf.demoMode);
	conf.shadowclockDirection = pt.get<float>("GENERAL.shadowHand", conf.shadowclockDirection);
	// ALGO:
	conf.MHistory = pt.get<int>("ALGO.MHistory", conf.MHistory);
	conf.MvarThreshold = pt.get<float>("ALGO.MvarThreshold", conf.MvarThreshold);
	conf.MlearningRate = pt.get<float>("ALGO.MlearningRate", conf.MlearningRate);
	conf.useTracker = pt.get<float>("ALGO.tracker", conf.useTracker);


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

		// MOG2 
		bool detectShadows = false;
		int emphasize = 2;
		m_bgSeg.init(m_params.MHistory , m_params.MvarThreshold ,detectShadows, emphasize);
		m_bgSeg.setLearnRate(m_params.MlearningRate);

		// DDEBUG DDEBUG : Read RIO's from a file 
		if (!m_params.roisName.empty())
			m_roiList = readROISfile(m_params.roisName);
		if (m_calcScale != 1.)
			for (auto &roi : m_roiList)
				roi.bbox = scaleBBox(roi.bbox, m_calcScale);
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

		if (m_frameNum % CONSTANTS::DetectionFPS != 0)
			return 0;

		// -1- Track tracked objects
		//------------------------------------
		if (m_params.useTracker)
			tracked_count = detectByTracker(frame);

		// -1- Find new motion 
		//------------------------------------
		if (m_frameNum % CONSTANTS::motionDetectionFPS == 0) {
			cv::Mat sFrame;
			//cv::resize(frame, sFrame, cv::Size(0, 0), m_calcScale2, m_calcScale2);
			sFrame = frame;

			cv::Mat bgMask = m_bgSeg.process(sFrame);
			if (!bgMask.empty()) {
				cv::imshow("mog", bgMask);
				//bgMask.setTo(0, bgMask < 255); // = when shadow filter is on 
			}


			// -3- Find  motion's blobs (objects)
			//------------------------------------
			if (!bgMask.empty()) {
				//std::vector<cv::KeyPoint> myBlobs = detectBySimpleBlob(bgMask);
				std::vector<cv::Rect>    newROIs = detectByContours(bgMask);

				std::vector<LABEL>  lables = classify(sFrame, bgMask, newROIs);

				removeShadows(newROIs, lables, m_params.shadowclockDirection);
				matchObjects(newROIs); // match blobs to existing objects 
			}
		}

		// -3- consolidate detection
		//------------------------------------
		//removeShadows(2.0);
		consolidateDetection();


#if 0
		////>>>> tracking by ROI file:
		// DDEBUG : init tracker from file's list 
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

		tracked_count = detectByTracker(frame);
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
			cv::resize(frameIn, m_frame, cv::Size(0, 0), m_calcScale, m_calcScale);

		processFrame(m_frame);
		return m_frameNum++;
	}


	cv::Scalar  setColorByDetection(int type)
	{
		switch (type) {
		case (int)AGE::BORN:
		case (int)AGE::STARTER:
			return cv::Scalar(0, 0, 0);
		case (int)AGE::FINE:
			return cv::Scalar(255, 0, 0);
		case (int)AGE::STABLE:
			return cv::Scalar(0, 255, 0);
		case (int)AGE::TRACKED:
			return cv::Scalar(0, 0, 255);
		case (int)AGE::HIDDEN:
			return cv::Scalar(255, 255, 255);

		}
	}

	cv::Scalar  setColorByLabel(LABEL l)
	{
		switch (l) {
		case (int)LABEL::HUMAN:
			return cv::Scalar(0, 255, 0);
		case (int)LABEL::VEHICLE:
			return cv::Scalar(0, 0, 255);
		default:
			return cv::Scalar(0, 0, 0);
		}
	}

	int CTrack::show()
	{
		static int key = 0;
		cv::Mat display = m_frame.clone();

		cv::Point textPos(50, 60); // frameNum display
		cv::putText(display, std::to_string(m_frameNum), textPos, cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar(0, 200, 50), 2);

		// Draw detection rects
		for (int i = 0; i < m_objects.size();i++) {
			//if (m_objects[i].m_bboxes.size() < 10)
			if (m_objects[i].m_detectionType < AGE::FINE)
				continue;
			//cv::Scalar color(0, 0, 250, 2);
			cv::Scalar color;
			if (0)
				color = setColorByDetection((int)m_objects[i].m_detectionType);
			else
				color = setColorByLabel(LABEL(m_objects[i].m_label));

			//cv::rectangle(display, resizeBBox(m_objects[i].m_bboxes.back(),0.6) , color);
			cv::rectangle(display, m_objects[i].m_bboxes.back(), color);

			cv::Point text_pos = cv::Point(m_objects[i].m_bboxes.back().x, m_objects[i].m_bboxes.back().y - 10);
			cv::putText(display, std::to_string(i), text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
			/*
			// Add time on screen 
			int seconds = int((float)obj.len() / (float)CONSTANTS::FPS);
			cv::Point text_pos = cv::Point(obj.m_bboxes.back().x, obj.m_bboxes.back().y - 10);
			cv::putText(display, std::to_string(seconds + 1), text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
			*/
			if (m_scaleDisplay != 1.)
				cv::resize(display, display, cv::Size(0, 0), m_scaleDisplay, m_scaleDisplay);

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



	std::vector<cv::KeyPoint> CTrack::detectBySimpleBlob(cv::Mat bgMask)
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
	}



	std::vector <cv::Rect> CTrack::detectByContours(cv::Mat bgMask)
	{
		// Find blobs using CONTOURS 
		std::vector < std::vector<cv::Point>> contours, good_contours;
		//std::vector <cv::Rect> eyeROIs;
		std::vector<cv::Rect>    newROIs;
		std::vector<cv::Vec4i> hierarchy;

		findContours(bgMask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		const int MIN_CONT_AREA = 10*5;
		const int MAX_CONT_AREA = 200 * 120;

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
			//cv::Rect debug = scaleBBox(box, 1. / 0.5);
			newROIs.push_back(box);
		}

		return newROIs;
	}



	/*-----------------------------------------------------------
	 * Match RECTs to exiting m_objects
	 * If newROI matched update BBOX
	 * unless this bbox already updated (by tracker)
	 * If no match was found:  Add a new to m_objects 
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
		for (int i = 0; i < newROIs.size(); i++) {
			for (auto &oldObj : m_objects)
				if ((newROIs[i] & oldObj.m_bboxes.back()).area() > 0 &&
					bboxRatio(newROIs[i], oldObj.m_bboxes.back()) > 0.7) { // DANGEROUSE? 
					if (oldObj.m_lastDetected != m_frameNum) {	// already detected by other algo (tracker etc.)
						oldObj.add(newROIs[i], m_frameNum);
						match[i] = 1; // Mark this ROI as matched
						break;
					}
				}
		}

		// A new objects if no match found
		for (int i = 0; i < match.size(); i++) {
			if (match[i] == 0) {
				m_objects.push_back(CObject(newROIs[i], m_frameNum));
			}
		}

		return (int)match.size() - cv::countNonZero(match);

	}


	void CTrack::consolidateDetection()
	{
		const int starterLen = 10;   // DDEBUG CONST 
		const int stableLen = CONSTANTS::FPS * 1; // DDEBUG CONST 
		for (auto &obj : m_objects) {
			obj.m_detectionType = AGE::STARTER;
			if (obj.isTracked())
				obj.m_detectionType = AGE::TRACKED;
			else if (obj.len() > starterLen) {
				obj.m_detectionType = AGE::FINE;
				if (obj.len() > stableLen) {
					if (checkAreStability(obj.m_bboxes, 4))
						obj.m_detectionType = AGE::STABLE;
				}
			}
		}

		// Remove un-detected objects (fade)
		for (int i = 0; i < m_objects.size(); i++) {
			int hiddenLenAllowed = m_objects[i].m_bboxes.size() < 4 ? 3 : 10;
			if (m_objects[i].m_lastDetected + hiddenLenAllowed < m_frameNum)
				m_objects.erase(m_objects.begin() + i--);
		}


	}


	int CTrack::detectByTracker(cv::Mat frame)
	{
		int tracked_count = 0;
		int ind = 0;
		cv::Rect trackerROI;

		for (auto obj : m_objects) { 
				if (obj.m_tracker == NULL && obj.m_detectionType == AGE::STABLE) {
					// Set a new tracker 
					obj.m_tracker = new CTracker;
					obj.m_tracker->init(m_params.trackerType, 0);
					//obj.m_tracker->setROI(frame, resizeBBox(obj.m_bboxes.back(),0.7));
					obj.m_tracker->setROI(frame, obj.m_bboxes.back());
					obj.m_detectionType = AGE::TRACKED;
				}

				if (obj.isTracked()) {
					if (obj.m_tracker->track(frame)) {
						trackerROI = obj.m_tracker->getBBox();
						//obj.add(resizeBBox(trackerROI, 0.7), m_frameNum);
						obj.add(trackerROI, m_frameNum);
					}
					else {
						obj.m_detectionType = AGE::HIDDEN;
					}
					tracked_count++;
				}
		}

		return tracked_count;
	}

	int CTrack::detectByTracker_OLD(cv::Mat frame)
	{
		int tracked_count = 0;
		int ind = 0;
		for (auto tracker : m_trackers)
		{
			if (m_params.demoMode == 1 && ind == 6 && m_frameNum == 140)  m_trackers.erase(m_trackers.begin() + ind); // DDEBUG 
			if (m_params.demoMode == 2 && ind == 3 && m_frameNum == 350)  m_trackers.erase(m_trackers.begin() + ind); // DDEBUG 

			//tracker.reset();
			if (tracker.isActive())
				if (tracker.track(frame)) {
					m_trackerROI = tracker.getBBox();
					m_objects[ind].add(m_trackerROI, m_frameNum);
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
					tracked_count++;
				}
			ind++;
		}

		return tracked_count;
	}



	bool CTrack::checkAreStability(std::vector <cv::Rect> bboxes, int len)
	{

		// Check for rect stability 
		int areaDiff = 0;
		int ind = bboxes.size() - 1;
		for (int i = 0; i < len; i++,ind--)
			areaDiff += bboxes[ind].area() - bboxes[ind - 1].area();
		areaDiff = abs(areaDiff);
		float areaRatio = (float)areaDiff / (float)bboxes.back().area();

		return areaRatio < 0.3;
	}


	/*----------------------------------------------------------------------------------------
	 * Shrink MOG area (bbox) according to shadow angle in image
	 * We do that by the clockDirection - were the shadow point (as clock hour):
	 * We DONT touch tracker boxes 
	----------------------------------------------------------------------------------------*/
	void CTrack::removeShadows(float shadowClockDirection)
	{
		// missing >> : calc 'cut' by clockDirection 
		float cut = 0.4; // 0.5 of image from left side
		for (auto &obj : m_objects) {
			if (obj.m_lastDetected == m_frameNum && obj.m_detectionType < AGE::TRACKED) {
				int trimSize = int((float)obj.m_bboxes.back().width * (1. - fabs(cut)));
				obj.m_bboxes.back().width -= trimSize;
				if (cut < 0)
					obj.m_bboxes.back().x += trimSize;
			}
		}
	}

	void CTrack::removeShadows(std::vector<cv::Rect>  &rois, std::vector<LABEL> labels, float shadowClockHand)
	{
		float cut;
		if (shadowClockHand > 1.5  && shadowClockHand < 4.5)
			cut = 0.5;
		else if (shadowClockHand > 7.5  && shadowClockHand < 10.5)
			cut = -0.5;
		else
			return;

		for (int i = 0; i < rois.size();i++) {
			if (labels[i] == LABEL::HUMAN) {
				int trimSize = int((float)rois[i].width * fabs(cut));
				rois[i].width -= trimSize;
				if (cut < 0)
					rois[i].x += trimSize;
			}
		}
	}

	void CTrack::removeShadow(CObject &obj, float shadowClockDirection)
	{
		cv::Rect sBbox;
		// missing >> : calc 'cut' by clockDirection 
		float cut = 0.4; // 0.5 of image from left side
		//if (obj.m_lastDetected == m_frameNum && obj.m_detectionType < AGE::TRACKED) 
		{
			int trimSize = int((float)obj.m_bboxes.back().width * (1. - fabs(cut)));
			obj.m_bboxes.back().width -= trimSize;
			if (cut < 0)
				obj.m_bboxes.back().x += trimSize;
		}
	}



	std::vector<LABEL>   CTrack::classify(cv::Mat img, cv::Mat bgMask, std::vector <cv::Rect>  rois)
	{
		const float filllingRatio = 0.6;
		std::vector <LABEL> labels;
		 

		for (int i = 0; i < rois.size(); i++) {
			if (rois[i].width < int((float)SIZES::minVehicle * m_calcScale))
				labels.push_back(LABEL::HUMAN);
			else if (rois[i].width >= int((float)SIZES::maxHuman * m_calcScale))
				labels.push_back(LABEL::VEHICLE);
			else  { // between 70..140 possible humnan + shadow  OR a Car 
				int activePixels = cv::countNonZero(bgMask(rois[i]));
				if ((float)activePixels / (float)rois[i].area() < filllingRatio) // probbaly human with shadow \ others
					labels.push_back(LABEL::HUMAN);
				else 
					labels.push_back(LABEL::VEHICLE);
			}
		}


		return labels;

	}
