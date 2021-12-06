#pragma once 
#include <Windows.h>
#include <string>
#include <vector>

// EXCEPTION HANDLING
#define CHECK_assertion(COND)   ((COND) ? (static_cast<void>(0)) : assertion_fail(# COND, __FILE__, __LINE__))
#define CHECK_exception(COND, MSG)   ((COND) ? (static_cast<void>(0)) : assertion_fail(# COND, __FILE__, __LINE__, MSG))


inline  void assertion_fail(const char* expr, const char* file, int line, const char* msg)
{
	std::string error = msg;
	error.append(";    in expr" + std::string(expr) + ", file: " + file + "(" + std::to_string(line) + ")");
	MessageBoxA(0, error.c_str(), "EXCEPTION!", MB_OK);
	std::exit(1);
}
inline  void assertion_fail(const char* expr, const char* file, int line)
{
	std::string error = ";Error    in expr : " + std::string(expr) + ", file: " + file + "(" + std::to_string(line) + ")";
	MessageBoxA(0, error.c_str(), "EXCEPTION!", MB_OK);
	std::exit(1);
}

template<typename T>
T sqr(T x)
{
	return x * x;
}

template<typename T>
inline double distanceSQR(T a, const T b)
{
	return sqr(a.x - b.x) + sqr(a.y - b.y);
}


template<typename T>
inline double distance(T a, const T b)
{
	return sqrt(sqr(a.x - b.x) + sqr(a.y - b.y));
}


enum ANGLES {
	PAN_IDX=0, // Heading 
	TILT_IDX,  // Attiude
	ROLL_IDX   // Bank
};

#define   DEG2RAD(angle)		(angle * 0.0174532925)
#define   RAD2DEG(angle)        (angle / 0.0174532925)
inline cv::Point3f  RAD2DEG_vec3f(cv::Point3f radVec) { return cv::Point3f(RAD2DEG(radVec.x), RAD2DEG(radVec.y), RAD2DEG(radVec.z)); }
inline std::vector <float> RAD2DEG_vec(std::vector <float>  radVec) { std::vector <float>  degVec;  for (auto &rad : radVec) degVec.push_back(RAD2DEG(rad)); return degVec; }

cv::Point centerOf(cv::Rect r);// { return (r.br() + r.tl())*0.5; }
cv::Rect  moveByCenter(cv::Rect r, cv::Point center);
inline cv::Point2f centerOf(cv::Rect2f r) { return (r.br() + r.tl())*0.5; }
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


 class UTILS {
	 public:
	 vector<cv::Rect>  alignRois(vector<cv::Rect> rois);
	 static bool drawCorners(cv::Mat &img, const vector<cv::Rect> rois);
	 static void drawCorss(cv::Mat &img, cv::Rect r, cv::Scalar color, int thickness = 1)
	 {
		 cv::line(img, r.tl(), r.br(), color, thickness);
		 cv::line(img, cv::Point(r.br().x, r.tl().y), cv::Point(r.tl().x, r.br().y), color, thickness);
	 };

	 static void   checkBounderies(cv::Rect  &box, cv::Size imgSize)
	 {
		 if (box.x < 0)
			 box.x = 0;
		 if (box.y < 0)
			 box.y = 0;
		 if (box.x + box.width >= imgSize.width)
			 box.width = imgSize.width - box.x - 1;
		 if (box.y + box.height >= imgSize.height)
			 box.height = imgSize.height - box.y - 1;
	 }
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
 cv::Rect resizeBBox(cv::Rect rect, float scale);
 cv::Rect resizeBBox(cv::Rect rect, cv::Size size, float scale);
 float bboxesOverlapping(cv::Rect r1, cv::Rect r2); // Ratio of overlapping 

