#include "TactileImage.h"

void TactileImage::TactileImageCenter(cv::Mat img1, cv::Mat img2)
{
	_img1 = img1.clone();
	_img2 = img2.clone();

	TactileInterpolated(_img1, _img2, _a);
	ImageCenter(_img2, 0);		////算全圖中心
}

void TactileImage::TactileInterpolated(cv::Mat img1, cv::Mat img2, cv::Size a)	// image resize
{
	a.height = img1.rows*200;	a.width = img1.cols*200;

	resize(img1, img2, a, 0, 0, cv::INTER_CUBIC);

	_img2 = img2.clone();

    //namedWindow("Test"); 
    //imshow("Test",_img2);
}

void TactileImage::ImageCenter(cv::Mat img, int ty)
{
	//cvtColor(img, img_gray, CV_BGR2GRAY);
	moment = moments(img, 0);
	//moment = moments(img_gray, 0);

	m00 = cvGetSpatialMoment(&moment,0,0);
	m10 = cvGetSpatialMoment(&moment,1,0);
	m01 = cvGetSpatialMoment(&moment,0,1);

	pt_tmp.x = m10 / m00;
	pt_tmp.y = m01 / m00;

	pt1.x = pt_tmp.x + 8;	pt1.y = pt_tmp.y;
	pt2.x = pt_tmp.x - 8;	pt2.y = pt_tmp.y;
	pt3.x = pt_tmp.x;	pt3.y = pt_tmp.y + 8;
	pt4.x = pt_tmp.x;	pt4.y = pt_tmp.y - 8;

	if (m00 != 0)
	{
		if (ty == 0)
			pt = pt_tmp;
		else if (ty == 1)
			pt_palm = pt_tmp;
		else if (ty == 2)
			pt_thumb = pt_tmp;
		else if (ty == 3)
			pt_first = pt_tmp;
		else if (ty == 4)
			pt_mid = pt_tmp;
		else if (ty == 5)
			pt_ring = pt_tmp;
		else if (ty == 6)
			pt_little = pt_tmp;
		else
		{
		}
	}

	//cv::Mat normal_img;

	//normalize(img, normal_img, 1, 0, cv::NORM_MINMAX);

	//std::cout << normal_img;

	//cv::namedWindow("Test", 1);
	
	//circle(normal_img,pt,1,255, 1);
/*
	line(normal_img,pt1, pt2, 255);
	line(normal_img,pt3, pt4, 255);
    imshow("Test",normal_img);*/

 /*   line(_img2,pt1, pt2, 255);
	line(_img2,pt3, pt4, 255);
	imshow("Test",_img2);*/
}