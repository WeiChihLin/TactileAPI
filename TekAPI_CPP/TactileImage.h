#include <opencv2/core/core.hpp>    
#include <opencv2/highgui/highgui.hpp>    
#include <opencv/cv.h>

class TactileImage{

public:
	void TactileImageCenter(cv::Mat, cv::Mat);
	cv::Mat _img1, _img2, img_gray;
	cv::Size _a;
	double m00, m10, m01;
	CvMoments moment;
	cv::Point pt, pt1, pt2, pt3, pt4, pt_tmp;
	cv::Point pt_palm, pt_thumb, pt_first, pt_mid, pt_ring, pt_little;
	
	void TactileInterpolated(cv::Mat, cv::Mat, cv::Size);
	void ImageCenter(cv::Mat, int);
};