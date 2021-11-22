#pragma once 
#include <Windows.h>
#include <string>
#include <vector>


enum ANGLES {
	PAN_IDX=0, // Heading 
	TILT_IDX,  // Attiude
	ROLL_IDX   // Bank
};

#define   DEG2RAD(angle)		(angle * 0.0174532925)
#define   RAD2DEG(angle)        (angle / 0.0174532925)
inline cv::Point3f  RAD2DEG_vec3f(cv::Point3f radVec) { return cv::Point3f(RAD2DEG(radVec.x), RAD2DEG(radVec.y), RAD2DEG(radVec.z)); }
inline std::vector <float> RAD2DEG_vec(std::vector <float>  radVec) { std::vector <float>  degVec;  for (auto &rad : radVec) degVec.push_back(RAD2DEG(rad)); return degVec; }

inline cv::Point centerOf(cv::Rect r) { return (r.br() + r.tl())*0.5; }
inline bool isEmpty(cv::RotatedRect rr) { return (rr.size.width == 0 || rr.size.height == 0); }
inline bool isEmpty(cv::Rect r) { return (r.width == 0 || r.height == 0); }


#define TICKFREQUENCY 1000.


class CTimer {
public:
	double  start() { m_start_time = m_cur_time = (double)GetTickCount(); return m_start_time / TICKFREQUENCY; }
	double  sample() { m_prev_time = m_cur_time;  m_cur_time = (double)GetTickCount(); return (m_cur_time - m_prev_time)  / TICKFREQUENCY; }
	double  durationFromStart() { return (m_cur_time - m_start_time) / TICKFREQUENCY; } // in sec
	void printDuration(std::string msg) 
	{
		double duration = sample(); std::cout << msg << "( duration = " << duration << " )";
	}


private:
	double m_start_time;
	double m_prev_time;
	double m_cur_time;
};


 using namespace std;



 class FILE_UTILS {
 public:
	 static std::vector <std::string>  list_files_in_folder(const std::string path, bool reverseOrder = false);
	 static std::vector <std::string>  list_sorted_files_in_folder(std::string path);

	 static bool is_folder(std::string filename);
	 static bool is_image_extension(std::string fname);

	 static std::string find_path(const std::string  filename);
	 static std::string  find_fname(const std::string  filename);

	 static std::string change_extension(const std::string & filename, const std::string & extension);
	 static std::string remove_extension(const std::string  filename);
	 static std::string find_extension(const std::string & filename);
	 static std::string add_soffix(const std::string & filename, const std::string postfix);
	 static bool file_exists(const std::string & filename);
	 static bool folder_exists(const std::string & filename);

 };


 namespace UTILS {
	 vector<cv::Rect>  alignRois(vector<cv::Rect> rois);
	 bool drawCorners(cv::Mat panoImg, const vector<cv::Rect> rois);
 };


 // Find value in mat
 template <class T>
 bool findValue(const cv::Mat &mat, T value, int &i, int &j) {
	 for (int r = 0; r < mat.rows; r++) {		 const T* row = mat.ptr<T>(r);		 const T *pos = std::find(row, row + mat.cols, value);		 if (pos != row + mat.cols) {			 i = pos - row;			 j = r;			 return true;		 }
	 }	 return false; }
 std::wstring stringToWstring(const std::string& t_str);
 int debug_imshow(std::string title, cv::Mat img, double scale = 0, int waitTime = -1);



#if 0
 //opencv
 cv::Mat rot2euler_CV(const cv::Mat & rotationMatrix);
 cv::Mat euler2rot_CV(cv::Vec3f euler3);

 cv::Vec3f rot2euler_1(const cv::Mat_<float> & rotationMatrix);
 cv::Mat   euler2rot_1(cv::Vec3f euler3);

 // learnOpencv
 cv::Vec3f rot2euler_3(cv::Mat &R);
 cv::Mat   euler2rot_3(cv::Vec3f &theta);
#endif 

 namespace GEO_UTILS {

	 //cv::Vec3f rot2euler(cv::Mat_<float> R);
	 cv::Vec3f rot2euler(cv::Mat R);
	 cv::Mat   euler2rot(cv::Vec3f theta);

	 cv::Mat shiftRotationMatrix(cv::Mat_<float>  rotationMatrix, cv::Vec3f shiftRot);


	 // Y version 
	 cv::Mat euler2rot_2(cv::Vec3f euler3);

	 // DDEBUG DDEBUG TEMP
	 /*
	 cv::Mat rot2euler_CV(cv::Mat_<float>  rotationMatrix);
	 cv::Mat euler2rot_CV(cv::Vec3f euler3);
	 cv::Vec3f rot2euler_1(cv::Mat_<float>  rotationMatrix);
	 cv::Mat   euler2rot_1(cv::Vec3f euler3);
	 // learnOpencv
	 cv::Vec3f rot2euler_3(cv::Mat_<float> R);
	 cv::Mat   euler2rot_3(cv::Vec3f theta);
	  */
 }


 // RECT utilities 
 bool isIn(cv::Point hitPixel, cv::Rect  roi);
 cv::Rect extendBBox(cv::Rect rect_, cv::Point p);
 cv::Rect scaleBBox(cv::Rect rect, float scale);
 float    bboxRatio(cv::Rect r1, cv::Rect r2);
 cv::Rect enlargeBBox(cv::Rect rect, float scale);
