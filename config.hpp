#pragma once


namespace CONSTANTS {
	int const FPS = 30;

	/*
	int const KEEP_ALIVE_SEC = 5;
	int const MAX_OBJECTS = 20;
	int const NO_MOTION_TOLERANCE = 10; // in pixels
	int const DETECTION_REFRASH_RATE = 7; //FPS * 3; // 7 seconds
	int const MAX_AFTER_MOTION_COUNTER = 10*6;
	float const DAY_CAM_PROCESS_SCALE = 0.25;
	float const DAY_CAM_DISPLAY_SCALE = 1.;
	*/
};


enum SHAPE_TYPE {
	hamburger=1,
	stake,
	fillingLevel,
	miniBamburger
};

struct Config
{
	std::string videoName;
	std::string roisName;
	int trackerType = 2;
	int debugLevel = 0;
	float scale = 1.;
	float displayScale = 1.;
	int waitKeyTime=10;
	int record = 0;
	int demoMode=0;
	// MOG2 params:
	int MHistory = 30;
	float MvarThreshold = 20.0;
	float MlearningRate = -1.;

	};
