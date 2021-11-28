#include <filesystem>
#include <iostream>
#include <fstream>
#include <stdarg.h>
#include <string.h>
#include <vector>
#include <codecvt>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "Utils.hpp"



namespace fs = std::experimental::filesystem;


  std::vector <std::string>  FILE_UTILS::list_files_in_folder(std::string path, bool reverseOrder)
  {
	  std::vector <std::string> filesList, filesList_reverseOrder;
	  //std::string path = "/path/to/directory";
	  for (const auto & entry : fs::directory_iterator(path))
		  filesList.push_back(entry.path().string());
	  //std::cout << entry.path() << std::endl;

	  if (!reverseOrder) 
		return filesList;

	  // REVERSE ORDER 
	  for (int r = filesList.size() - 1; r >= 0; r--)
		  filesList_reverseOrder.push_back(filesList[r]);

	  return filesList_reverseOrder;
  }

  std::vector <std::string>  FILE_UTILS::list_sorted_files_in_folder(std::string path)
  {
	  std::vector <std::string> filesList, filesList_reverseOrder;
	  //std::string path = "/path/to/directory";
	  for (const auto & entry : fs::directory_iterator(path))
		  filesList.push_back(entry.path().string());

	  // Sort file names
	  std::sort(filesList.begin(), filesList.end(), [](const std::string  & a, const std::string & b) -> bool { return a < b; });

	  return filesList;

  }
  
  
  
  std::string FILE_UTILS::change_extension(const std::string & filename, const std::string & extension)
  {
    char path[_MAX_PATH];
    char drive[_MAX_DRIVE];
    char dir[_MAX_DIR];
    char fname[_MAX_FNAME];
    char ext[_MAX_EXT];
    
    _splitpath_s(filename.c_str(), drive, dir, fname, ext);
    _makepath_s(path, drive, dir, fname, extension.c_str());
    
    return path;
  }

std::string FILE_UTILS::remove_extension(const std::string  filename)
  {
    char path[_MAX_PATH];
    char drive[_MAX_DRIVE];
    char dir[_MAX_DIR];
    char fname[_MAX_FNAME];
    char ext[_MAX_EXT];
    
    _splitpath_s(filename.c_str(), drive, dir, fname, ext);
    _makepath_s(path, drive, dir, fname, NULL);
    
    return path;
  }

std::string FILE_UTILS::find_path(const std::string  filename)
  {
    char path[_MAX_PATH];
    char drive[_MAX_DRIVE];
    char dir[_MAX_DIR];
    char fname[_MAX_FNAME];
    char ext[_MAX_EXT];
    
    _splitpath_s(filename.c_str(), drive, dir, fname, ext);
    _makepath_s(path, drive, dir, NULL , NULL);
    
    return std::string(path);
  }

std::string FILE_UTILS::find_fname(const std::string  filename)
{
	char path[_MAX_PATH];
	char drive[_MAX_DRIVE];
	char dir[_MAX_DIR];
	char fname[_MAX_FNAME];
	char ext[_MAX_EXT];

	_splitpath_s(filename.c_str(), drive, dir, fname, ext);
	_makepath_s(path, drive, dir, NULL, NULL);

	return std::string(fname);
}

    std::string FILE_UTILS::find_extension(const std::string  &filename)
  {
    char path[_MAX_PATH];
    char drive[_MAX_DRIVE];
    char dir[_MAX_DIR];
    char fname[_MAX_FNAME];
    char ext[_MAX_EXT];
    
    _splitpath_s(filename.c_str(), drive, dir, fname, ext);

	std::string extStr(ext);
	extStr.erase(std::remove(extStr.begin(), extStr.end(), '.'), extStr.end());

	return extStr;
  }
 std::string FILE_UTILS::add_soffix(const std::string & filename, const std::string postfix)
  {
    char path[_MAX_PATH];
    char drive[_MAX_DRIVE];
    char dir[_MAX_DIR];
    char fname[_MAX_FNAME];
    char ext[_MAX_EXT];
    
    _splitpath_s(filename.c_str(), drive, dir, fname, ext);
	strcat_s(fname ,postfix.c_str());
    _makepath_s(path, drive, dir, fname, ext);
    
    return path;
  }



 std::wstring stringToWstring(const std::string& t_str)
 {
	 //setup converter
	 typedef std::codecvt_utf8<wchar_t> convert_type;
	 std::wstring_convert<convert_type, wchar_t> converter;

	 //use converter (.to_bytes: wstr->str, .from_bytes: str->wstr)
	 return converter.from_bytes(t_str);
 }


  bool FILE_UTILS::file_exists(const std::string & filename)
  {
    return GetFileAttributes(stringToWstring(filename).c_str()) != (DWORD)-1;
  }

  bool FILE_UTILS::folder_exists(const std::string & filename)
  {
    //return GetFileAttributes((LPCWSTR)filename.c_str()) != (DWORD)-1;
	  DWORD ftype = GetFileAttributes(stringToWstring(filename).c_str());
	  if (INVALID_FILE_ATTRIBUTES == ftype) 
		  return false;
	  return (FILE_ATTRIBUTE_DIRECTORY  & GetFileAttributes(stringToWstring(filename).c_str()));
  }
 

  bool FILE_UTILS::is_folder(std::string filename)
  {
		  return (GetFileAttributes(stringToWstring(filename).c_str()) == FILE_ATTRIBUTE_DIRECTORY);
  }

  bool FILE_UTILS::is_image_extension(const std::string fname_)
  {
	  std::string fname = fname_;

	  if (is_folder(fname))
		  return false;


	  std::transform(fname.begin(), fname.end(), fname.begin(), ::tolower);

	  if (find_extension(fname) == "jpg" ||
		  find_extension(fname) == "png" ||
		  find_extension(fname) == "gif" ||
		  find_extension(fname) == "tiff" ||
		  find_extension(fname) == "tif" ||
		  find_extension(fname) == "bmp" )
		  return true;

	  return false;
}


  vector<cv::Rect>  UTILS::alignRois(vector<cv::Rect> rois)
  {
	  vector<cv::Rect> rois_local;
	  int minX = INT_MAX;
	  int minY = INT_MAX;

	  for (auto r : rois) {
		  minX = MIN(r.x, minX);
		  minY = MIN(r.y, minY);

		  rois_local.push_back(r);
	  }
	  for (int i = 0; i < rois.size(); i++) {
		  rois_local[i].x = rois[i].x - minX;
		  rois_local[i].y = rois[i].y - minY;
	  }

	  return rois_local;
  }

  bool UTILS::drawCorners(cv::Mat panoImg, const vector<cv::Rect> rois_local)
  {
	  //vector<cv::Rect> rois_local;
	  cv::Mat display;
	  panoImg.convertTo(display, CV_8UC3);

	  //rois_local = allgnRois(rois); // allign to start on (0,0)
	  /*
	  int minX = INT_MAX;
	  int minY = INT_MAX;

	  for (auto r : rois) {
		  minX = MIN(r.x, minX);
		  minY = MIN(r.y, minY);

		  rois_local.push_back(r);
	  }
	  for (int i = 0; i < rois.size();i++) {
		  rois_local[i].x = rois[i].x - minX;
		  rois_local[i].y = rois[i].y - minY;
	  }
	  */

	  for (auto r : rois_local)
		  cv::rectangle(display, r, cv::Scalar(0, 0, 255), 5);

	  debug_imshow("rois", display, 0.5, -1);

	  return true;

  }


  

  int debug_imshow(std::string title, cv::Mat img, double scale, int waitTime)
  {
	  int key;
	  if (scale == 0)
		  scale = 1240. / (double)img.cols;

	  cv::Mat display;
	  cv::resize(img, display, cv::Size(), scale, scale);
	  cv::imshow(title, display);
	  key = cv::waitKey(waitTime);
	  if (waitTime < 0)
		  cv::destroyWindow(title);

	  return key;
  }


  bool isIn(cv::Point hitPixel, cv::Rect  roi)
  {
	  return (hitPixel.x > roi.x   && hitPixel.x < roi.br().x &&
		  hitPixel.y > roi.y   && hitPixel.y < roi.br().y);
  }

  //===========================================================================================
  //   G E O    U T I L S
  //===========================================================================================

  namespace GEO_UTILS {

	  /*--------------------------------------------------------------------------------
	  * https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
	  * Return Pan , Tilt and Roll radians
	  *--------------------------------------------------------------------------------*/


	  //template <typename T>
	  cv::Vec3f rot2euler_1(cv::Mat_<float> rotationMatrix)
	  {

		  float m00 = rotationMatrix.at<float>(0, 0);
		  float m02 = rotationMatrix.at<float>(0, 2);
		  float m10 = rotationMatrix.at<float>(1, 0);
		  float m11 = rotationMatrix.at<float>(1, 1);
		  float m12 = rotationMatrix.at<float>(1, 2);
		  float m20 = rotationMatrix.at<float>(2, 0);
		  float m22 = rotationMatrix.at<float>(2, 2);

		  float bank, attitude, heading;

		  // Assuming the angles are in radians.
		  if (m10 > 0.998) { // singularity at north pole
			  bank = 0;
			  attitude = CV_PI / 2;
			  heading = atan2(m02, m22);
		  }
		  else if (m10 < -0.998) { // singularity at south pole
			  bank = 0;
			  attitude = -CV_PI / 2;
			  heading = atan2(m02, m22);
		  }
		  else
		  {
			  bank = atan2(-m12, m11);
			  attitude = asin(m10);
			  heading = atan2(-m20, m00);
		  }

		  cv::Vec3f  eulers;

		  /*
		  eulers[0] = bank;	 	 // 0 : actual : Attiude (~'tilt') b.k.
		  eulers[1] = attitude;  // 1 : actual : Bank    (~'rol') b.k.
		  eulers[2] = heading;   // 2 : actual : Heading (~'pan') b.k.
		  */
		  // format : Pan, Tilt , Roll
		  eulers[TILT_IDX] = bank;	 	 // 0 : actual : Attiude (~'tilt') b.k.
		  eulers[ROLL_IDX] = attitude;  // 1 : actual : Bank    (~'rol') b.k.
		  eulers[PAN_IDX] = heading;   // 2 : actual : Heading (~'pan') b.k.

		  return eulers;
	  }

	  // Converts a given Euler angles to Rotation Matrix
	  // Convention used is Y-Z-X Tait-Bryan angles
	  // Reference:
	  // https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
	  cv::Mat euler2rot_1(const cv::Mat_<float> & euler)
	  {
		  cv::Mat rotationMatrix(3, 3, CV_32F);

		  float bank = euler.at<float>(0);
		  float attitude = euler.at<float>(1);
		  float heading = euler.at<float>(2);

		  // Assuming the angles are in radians.
		  float ch = cos(heading);
		  float sh = sin(heading);
		  float ca = cos(attitude);
		  float sa = sin(attitude);
		  float cb = cos(bank);
		  float sb = sin(bank);

		  float m00, m01, m02, m10, m11, m12, m20, m21, m22;

		  m00 = ch * ca;
		  m01 = sh * sb - ch * sa*cb;
		  m02 = ch * sa*sb + sh * cb;
		  m10 = sa;
		  m11 = ca * cb;
		  m12 = -ca * sb;
		  m20 = -sh * ca;
		  m21 = sh * sa*cb + ch * sb;
		  m22 = -sh * sa*sb + ch * cb;

		  rotationMatrix.at<float>(0, 0) = m00;
		  rotationMatrix.at<float>(0, 1) = m01;
		  rotationMatrix.at<float>(0, 2) = m02;
		  rotationMatrix.at<float>(1, 0) = m10;
		  rotationMatrix.at<float>(1, 1) = m11;
		  rotationMatrix.at<float>(1, 2) = m12;
		  rotationMatrix.at<float>(2, 0) = m20;
		  rotationMatrix.at<float>(2, 1) = m21;
		  rotationMatrix.at<float>(2, 2) = m22;

		  return rotationMatrix;
	  }


	  cv::Mat euler2rot_1(cv::Vec3f euler3)
	  {
		  cv::Mat_<float>  eulerM;

		  /*
		  eulerM.push_back(euler3[0]);
		  eulerM.push_back(euler3[1]);
		  eulerM.push_back(euler3[2]);
		  */
		  eulerM.push_back(euler3[TILT_IDX]); // [0]=bank		: actual tilt b.k.
		  eulerM.push_back(euler3[ROLL_IDX]); // [1]= attitude : actual rol b.k.
		  eulerM.push_back(euler3[PAN_IDX]);  // [2] = heading	: actual pan b.k.

		  return euler2rot_1(eulerM);
	  }

	  //function rotation_mtx = compute_rotation_mtx(ax, ay, az)
	  cv::Mat euler2rot_2(cv::Vec3f euler3)
	  {
		  //% ax, ay, az are rotation angles in radians around axes x, y, z (respectively)
		  //% assumption: order of applying angles is pan->tilt->roll
		  float ax, ay, az;
		  ax = euler3[0];
		  ay = euler3[1];
		  az = euler3[2];

		  float cx = cos(ax);
		  float cy = cos(ay);
		  float cz = cos(az);
		  float sx = sin(ax);
		  float sy = sin(ay);
		  float sz = sin(az);

		  //rx = [1 0 0 ; 0 cx sx ; 0 -sx cx];
		  //ry = [cy 0 -sy ; 0 1 0 ; sy 0 cy];
		  //rz = [cz sz 0 ; -sz cz 0 ; 0 0 1];
		  float rx_data[] = { 1.0, 0, 0, 0, cx, sx, 0, -sx, cx };
		  float ry_data[] = { cy, 0, -sy, 0, 1.0, 0, sy, 0, cy };
		  float rz_data[] = { cz, sz, 0, -sz, cz, 0, 0, 0, 1.0 };
		  cv::Mat rx(3, 3, CV_32F, rx_data);

		  /*
		  cv::Mat ry(3, 3, CV_32F, ry_data);
		  cv::Mat rz(3, 3, CV_32F, rz_data);
		  */
		  cv::Mat ry(3, 3, CV_32F, ry_data);
		  cv::Mat rz(3, 3, CV_32F, rz_data);

		  cv::Mat rotation_mtx = rz * rx * ry;
		  return rotation_mtx;
	  }

	  // Checks if a matrix is a valid rotation matrix.
	  bool isRotationMatrix(cv::Mat &R)
	  {
		  cv::Mat Rt;
		  transpose(R, Rt);
		  cv::Mat shouldBeIdentity = Rt * R;
		  cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

		  return  cv::norm(I, shouldBeIdentity) < 1e-6;

	  }


	  // Calculates rotation matrix given euler angles.
	  // https://www.learnopencv.com/rotation-matrix-to-euler-angles/
	  cv::Mat euler2rot_3(cv::Vec3f &theta)
	  {
		  // Calculate rotation about x axis
		  cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
			  1, 0, 0,
			  0, cos(theta[0]), -sin(theta[0]),
			  0, sin(theta[0]), cos(theta[0])
			  );

		  // Calculate rotation about y axis
		  cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
			  cos(theta[1]), 0, sin(theta[1]),
			  0, 1, 0,
			  -sin(theta[1]), 0, cos(theta[1])
			  );

		  // Calculate rotation about z axis
		  cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
			  cos(theta[2]), -sin(theta[2]), 0,
			  sin(theta[2]), cos(theta[2]), 0,
			  0, 0, 1);

		  // Combined rotation matrix
		  cv::Mat R = R_z * R_y * R_x;

		  return R;
	  }

	  // https://www.learnopencv.com/rotation-matrix-to-euler-angles/
	  cv::Vec3f rot2euler_3(cv::Mat_<float> R)
	  {

		  assert(isRotationMatrix(R));

		  float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

		  bool singular = sy < 1e-6; // If

		  float x, y, z;
		  if (!singular)
		  {
			  x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
			  y = atan2(-R.at<double>(2, 0), sy);
			  z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
		  }
		  else
		  {
			  x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
			  y = atan2(-R.at<double>(2, 0), sy);
			  z = 0;
		  }
		  return cv::Vec3f(x, y, z);
	  }



	  /*------------------------------------------------------------------------------------------------------------------------------------------
	   * Converts a given Rotation Matrix to Euler angles
	   * opencv original : https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation/src/Utils.h
	   *
	   * measured_eulers.at<double>(0);      // roll
	   * measured_eulers.at<double>(1);      // pitch
	   * measured_eulers.at<double>(2);      // yaw
	   *-------------------------------------------------------------------------------------------------------------------------------------------*/
	  cv::Vec3f  rot2euler_CV(cv::Mat  rotationMatrix)
	  {

		  double m00 = rotationMatrix.at<double>(0, 0);
		  double m02 = rotationMatrix.at<double>(0, 2);
		  double m10 = rotationMatrix.at<double>(1, 0);
		  double m11 = rotationMatrix.at<double>(1, 1);
		  double m12 = rotationMatrix.at<double>(1, 2);
		  double m20 = rotationMatrix.at<double>(2, 0);
		  double m22 = rotationMatrix.at<double>(2, 2);

		  double x, y, z;

		  // Assuming the angles are in radians.
		  if (m10 > 0.998) { // singularity at north pole
			  x = 0;
			  y = CV_PI / 2;
			  z = atan2(m02, m22);
		  }
		  else if (m10 < -0.998) { // singularity at south pole
			  x = 0;
			  y = -CV_PI / 2;
			  z = atan2(m02, m22);
		  }
		  else
		  {
			  x = atan2(-m12, m11);
			  y = asin(m10);
			  z = atan2(-m20, m00);
		  }

		  cv::Vec3f euler3;
		  euler3[0] = x;
		  euler3[1] = y;
		  euler3[2] = z;

		  return euler3;
	  }

	  // Converts a given Euler angles to Rotation Matrix
	  // opencv original : https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation/src/Utils.h
	  //---------------------------------------------------------------------------------------------------------------------------------------
	  cv::Mat euler2rot_CV(cv::Vec3f euler3)
	  {
		  cv::Mat rotationMatrix(3, 3, CV_64F);

		  double x = euler3[0];
		  double y = euler3[1];
		  double z = euler3[2];

		  // Assuming the angles are in radians.
		  double ch = cos(z);
		  double sh = sin(z);
		  double ca = cos(y);
		  double sa = sin(y);
		  double cb = cos(x);
		  double sb = sin(x);

		  double m00, m01, m02, m10, m11, m12, m20, m21, m22;

		  m00 = ch * ca;
		  m01 = sh * sb - ch * sa*cb;
		  m02 = ch * sa*sb + sh * cb;
		  m10 = sa;
		  m11 = ca * cb;
		  m12 = -ca * sb;
		  m20 = -sh * ca;
		  m21 = sh * sa*cb + ch * sb;
		  m22 = -sh * sa*sb + ch * cb;

		  rotationMatrix.at<double>(0, 0) = m00;
		  rotationMatrix.at<double>(0, 1) = m01;
		  rotationMatrix.at<double>(0, 2) = m02;
		  rotationMatrix.at<double>(1, 0) = m10;
		  rotationMatrix.at<double>(1, 1) = m11;
		  rotationMatrix.at<double>(1, 2) = m12;
		  rotationMatrix.at<double>(2, 0) = m20;
		  rotationMatrix.at<double>(2, 1) = m21;
		  rotationMatrix.at<double>(2, 2) = m22;

		  return rotationMatrix;
	  }



	  // FINAL GOOD FUNCTION :
	  //cv::Vec3f  rot2euler(cv::Mat_<float>  rotationMatrix)
	  cv::Vec3f  rot2euler(cv::Mat  rotationMatrix)
	  {
		  return rot2euler_1(rotationMatrix);
	  }

	  cv::Mat euler2rot(cv::Vec3f euler3)
	  {
		  return euler2rot_1(euler3);
	  }

	  /*-----------------------------------
		Shift (opencv) rot matrix by 'shiftRot' angle
		Note: all angles in RAD
		-----------------------------------*/
		//cv::Mat shiftRotationMatrix(cv::Mat_<float>  rotationMatrix, cv::Vec3f shiftRot)
	  cv::Mat shiftRotationMatrix(cv::Mat rotationMatrix, cv::Vec3f shiftRot)
	  {
		  cv::Vec3f orgRot = rot2euler_1(rotationMatrix);
		  orgRot += shiftRot;
		  return euler2rot_1(orgRot);
	  }
  }



  cv::Rect extendBBox(cv::Rect rect_, cv::Point p)
  {
	  if (rect_.empty())
		  return cv::Rect(p.x - 1, p.y - 1, 3, 3);

	  cv::Rect rect = rect_;
	  if (p.x < rect.x) {
		  rect.width += rect.x - p.x;
		  rect.x = p.x;
	  }
	  else if (p.x > rect.x + rect.width) {
		  rect.width = p.x - rect.x + 1;
	  }


	  if (p.y < rect.y) {
		  rect.height += rect.y - p.y;
		  rect.y = p.y;
	  }
	  else if (p.y > rect.y + rect.height) {
		  rect.height = p.y - rect.y + 1;
	  }

	  return rect;

  }



  cv::Rect scaleBBox(cv::Rect rect, float scale)
  {
	  cv::Rect sBBox;

	  sBBox.width = int((float)rect.width * scale);
	  sBBox.height = int((float)rect.height* scale);
	  sBBox.x = int((float)rect.x * scale);
	  sBBox.y = int((float)rect.y * scale);

	  return sBBox;

  }

  cv::Rect resizeBBox(cv::Rect rect, float scale)
  {
	  cv::Rect sBBox;

	  sBBox = rect;
	  int wDiff = int((float)rect.width * (1. - scale));
	  int hDiff = int((float)rect.height * (1. - scale));
	  sBBox.width -= wDiff;
	  sBBox.height -= hDiff;
	  sBBox.x += int((float)wDiff / 2.);
	  sBBox.y += int((float)hDiff / 2.);

	  return sBBox;

  }


  float bboxRatio(cv::Rect r1, cv::Rect r2)
  {
	  int area1 = r1.area();
	  int area2 = r2.area();

	  return area1 > area2 ? (float)area2 / (float)area1 : (float)area1 / (float)area2;
  }
