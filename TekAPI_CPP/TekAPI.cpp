#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
//#include <msclr/marshal.h>        // .NET string to C-style string
//#include <msclr/marshal_cppstd.h> // .NET string to STL string
#include <math.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <Windows.h> 

#include "C:\opencv\build\include\opencv\cv.h";
#include "C:\opencv\build\include\opencv\highgui.h";

#include "3rdparty\Eigen\Dense";

#include "TactileImage.h";

#include "Driver.h"
#include "Control\SlipControl.h"

//using namespace cv;
using namespace System;
//using namespace System::Collections::Generic;
//using namespace System::Linq;
//using namespace System::Text;
//using namespace System::ComponentModel;
//using namespace System::Threading::Tasks;

//using namespace msclr::interop;
using namespace TekAPI;

using namespace Eigen;

using namespace sp;


int main()
{
	TactileImage test;
	SC::SlipControl SControl;	// Slip Control class
	SC::PID AngleRef;			// Caculate the Angle references

	//// 控制參數初始化
	InitPar();

	initMega();

	int HandFlag = 1;			// 0: right, 1: left

	//if (mega.setComPort("\\\\.\\COM20"))
	//	std::cout << "Arduino Mega 2560 Connected..." << std::endl;
	//else{
	//	std::cout << "Error!! Arduino Mega 2560 Connected" << std::endl;
	//	return 0;
	//}

	/* Load a calibration/equilibration (optional)
	* Return types are CTekEquilibration and CTekCalibration objects,
	* respectively. These objects provide a function allowing for calibration
	* and equilibration of arrays of captured frame data and can also be passed
	* as parameters to functions that save recordings or apply calibrations and
	* equilibrations to .fsx files. 
	*/

    String^ mapFileDirectory = "C:\\Tekscan\\TekAPI\\Samples\\"; // must set map file directory to load a calibration
    CTekAPI::TekSetMapFileDirectory(mapFileDirectory);
    String^ equilibrationFilePath("C:\\Tekscan\\TekAPI\\Samples\\SampleEquil.equ");
    String^ calibrationFilePath ("D:\\Tactile Sensor Program\\TekAPI_CPP_160316\\TekAPI_CPP\\right.cal");
	//String^ calibrationFilePath ("C:\\Tekscan\\TekAPI\\Samples\\SampleCal.cal");
	//String^ calibrationFilePath ("C:\\Tekscan\\TekAPI\\Samples\\TestLevi.cal");

    CTekEquilibration^ equilibration = CTekAPI::TekLoadEquilibration(equilibrationFilePath);
    CTekCalibration^ calibration = CTekAPI::TekLoadCalibration(calibrationFilePath);

    // Find and claim available hardware
    // Required as first step before any hardware will be recognized.
    CTekAPI::TekInitializeHardware();

    // Get list of available serial numbers, returned as System.String[].
    array<System::String^, 1>^ availableSerialNumbers;
    int errorCode = CTekAPI::TekEnumerateHandles(availableSerialNumbers);

    // Get lowest serial number to use as an identifier for future calls.
    String^ serialNumber = availableSerialNumbers[0]; // sensor with lowest serial number

	String^ availableSerialNumbers1;
	availableSerialNumbers1 = availableSerialNumbers[0];
    // The map file used here should match the type of sensor you are using.
	String^ mapFilePath("D:\\Tactile Sensor Program\\TekAPI_CPP_160316\\TekAPI_CPP\\null2.mp");
	//CTekAPI::TekClaimSensor(availableSerialNumbers, mapFilePath);
	if (HandFlag == 0)
	{
		CTekAPI::TekClaimRightSideSensor(availableSerialNumbers1, mapFilePath);
	}
	else
	{
		CTekAPI::TekClaimLeftSideSensor(availableSerialNumbers1, mapFilePath);
	}
	
    // Set up the selected sensor
    long framePeriod = 10000; // controls the period in microseconds of data collection (1/frequency)
	CTekAPI::TekInitializeSensor(serialNumber, framePeriod); // framePeriod in microseconds: 10000 microseconds = 100 Hz


	#pragma region 待確認
 //   // Sensitivity can  be set using an integer 1-40 (matching IScan
 //   // slider levels) 
	//errorCode = CTekAPI::TekSetSensitivityLevel(serialNumber, 20);
	//printf("%s\n", CTekAPI::TekGetLastError());
	////
	#pragma endregion
    
	/* OR
        * Sensitivity can also be set to match a calibration file's setting. This
        * method is highly recommended if you plan to apply calibrations to collected
        * data. While these operations will complete even if the sensitivity of the
        * recording and calibration do not match, an error code will be produced
        * when saving recordings with calibration or applying calibrations to recordings.
        */
    errorCode = CTekAPI::TekSetCalibratedSensitivity(serialNumber, calibration);

    // Get details about the sensor (optional)
    int rows, columns;
    double rowSpacing, columnSpacing;
    errorCode = CTekAPI::TekGetSensorRows(serialNumber, rows);
    errorCode = CTekAPI::TekGetSensorColumns(serialNumber, columns);
    rowSpacing = CTekAPI::TekGetSensorRowSpacing(serialNumber, rowSpacing);
    columnSpacing = CTekAPI::TekGetSensorColumnSpacing(serialNumber, columnSpacing);

    // Get frame data in real time
    int timeOut = 100; // time-out in milliseconds
    array<Byte,1>^ frameData;

	std::vector<MatrixXd> HandID;
	std::vector<double> AveragePress;
	for (int i = 0; i < 17; i++)
	{
		HandID.push_back(MatrixXd());
		AveragePress.push_back(double());
	}
	MatrixXd m_raw(rows, columns);

	cv::Point before_point;
	before_point.x = 0;	before_point.y = 0;
	int slip_count = 0;

	while (true)
	{
		errorCode = CTekAPI::TekCaptureDataFrame(serialNumber, timeOut, frameData);
		using namespace cv;
		//float a[9] = { 0, 10, 20, 10, 80, 20, 40, 10, 100 };
		cv::Mat b(rows, columns, CV_8UC1);
		
#pragma region 繪製壓力陣列與內插
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				b.at<byte>(i, j) = frameData[i * columns + j];

				if (frameData[i * columns + j]==1)
				{
					m_raw(i, j) = 0;
				}
				else
				{
					m_raw(i, j) = frameData[i * columns + j];
				}
				
			}
		}

		resize(b, b, cv::Size(480, 640), cv::INTER_CUBIC);
		//normalize(b, b, 1, 0, cv::NORM_MINMAX);
		cv::Mat c(b.rows, b.cols, CV_8UC3, Scalar(0));
		cv::Mat d(b.rows, b.cols, CV_8UC1, Scalar(0));

		for (int i = 0; i < b.rows; ++i){
			for (int j = 0; j < b.cols; ++j){
				//cout << b.at<float>(i, j)<<endl;

				if (b.at<byte>(i, j) == 1)
				{
					d.at<uchar>(i, j) = 0;

					c.at<Vec3b>(i, j)[0] = 0;
					c.at<Vec3b>(i, j)[1] = 0;
					c.at<Vec3b>(i, j)[2] = 0;
				}
				else if (b.at<byte>(i, j) == 0)
				{
					d.at<uchar>(i, j) = 0;
					
					c.at<Vec3b>(i, j)[0] = 255 ;
					c.at<Vec3b>(i, j)[1] = 255 ;
					c.at<Vec3b>(i, j)[2] = 255;
				}
				else if (b.at<byte>(i, j) < 128)
				{
					//cout << "x";
					float temp = b.at<byte>(i, j);

					d.at<uchar>(i, j) = b.at<byte>(i, j);

					c.at<Vec3b>(i, j)[0] = 255 - 255 * (temp / 128);
					c.at<Vec3b>(i, j)[1] = 255 * (temp / 128);
					c.at<Vec3b>(i, j)[2] = 0;
				}
				else if (b.at<byte>(i, j) >= 128){
					float temp = b.at<byte>(i, j) - 128;

					d.at<uchar>(i, j) = b.at<byte>(i, j);

					c.at<Vec3b>(i, j)[0] = 0;
					c.at<Vec3b>(i, j)[1] = 255 - 255 * (temp / 128);
					c.at<Vec3b>(i, j)[2] = 255 * (temp / 128);
				}
			}
		}
		test.ImageCenter(d, 0);

		cv::circle(c,test.pt,1,cv::Scalar(255,255,100), 1);
			
		cv::line(c, test.pt1, test.pt2, cv::Scalar(0, 200, 255), 5);
		cv::line(c, test.pt1, test.pt2, cv::Scalar(255, 0, 0), 2);
		cv::line(c, test.pt3, test.pt4, cv::Scalar(0, 200, 255), 5);
		cv::line(c, test.pt3, test.pt4, cv::Scalar(255, 0, 0), 2);
#pragma endregion

#pragma region 各手指區域重心位置
		MatrixXd m_palm(16, 20);
		MatrixXd m_thumb(8, 4);
		MatrixXd m_other(12, 4);
		MatrixXd StartAxisFingerBlock(6, 2);
		if (HandFlag == 0)
		{
			StartAxisFingerBlock << 13, 5,
				15, 0,
				0, 6,
				0, 11,
				0, 16,
				0, 21;
		}
		else
		{
			StartAxisFingerBlock << 13, 0,
				15, 21,
				0, 15,
				0, 10,
				0, 5,
				0, 0;
		}

		cv::Mat m_palm_cv(16, 20, CV_8UC1, Scalar(0));
		cv::Mat m_thumb_cv(8, 4, CV_8UC1, Scalar(0));
		cv::Mat m_first_cv(12, 4, CV_8UC1, Scalar(0));
		cv::Mat m_mid_cv(12, 4, CV_8UC1, Scalar(0));
		cv::Mat m_ring_cv(12, 4, CV_8UC1, Scalar(0));
		cv::Mat m_little_cv(12, 4, CV_8UC1, Scalar(0));

		for (int i = 0; i < 6; i++)
		{
			if (i == 0)
			{
				m_palm = m_raw.block<16, 20>(StartAxisFingerBlock(i, 0), StartAxisFingerBlock(i, 1));
				for (int j = 0; j < 16; j++)
				{
					for (int z = 0; z < 20; z++)
					{
						m_palm_cv.at<uchar>(j, z) = m_palm(j, z);
					}
				}
				test.ImageCenter(m_palm_cv, 1);
			}
			else if (i == 1)
			{
				m_thumb = m_raw.block<8, 4>(StartAxisFingerBlock(i, 0), StartAxisFingerBlock(i, 1));
				for (int j = 0; j < 8; j++)
				{
					for (int z = 0; z < 4; z++)
					{
						m_thumb_cv.at<uchar>(j, z) = m_thumb(j, z);
					}
				}
				test.ImageCenter(m_thumb_cv, 2);
			}
			else
			{
				m_other = m_raw.block<12, 4>(StartAxisFingerBlock(i, 0), StartAxisFingerBlock(i, 1));
				for (int j = 0; j < 12; j++)
				{
					for (int z = 0; z < 4; z++)
					{
						m_first_cv.at<uchar>(j, z) = m_other(j, z);
					}
				}
				if (i == 2)
				{
					m_first_cv = m_first_cv.clone();
					test.ImageCenter(m_first_cv, 3);
				}
				else if (i == 3)
				{
					m_mid_cv = m_first_cv.clone();
					test.ImageCenter(m_mid_cv, 4);
				}
				else if (i == 4)
				{
					m_ring_cv = m_first_cv.clone();
					test.ImageCenter(m_ring_cv, 5);
				}
				else if (i == 5)
				{
					m_little_cv = m_first_cv.clone();
					test.ImageCenter(m_little_cv, 6);
				}
				else
				{
				}
			}
		}
		std::cout << test.pt_thumb.x << '\t' << test.pt_thumb.y << std::endl;
#pragma endregion

#pragma region 滑動偵測與資料轉換
		//// slip detection by center moving
		if (sqrt(pow((test.pt.x - before_point.x),2)+pow((test.pt.y - before_point.y),2)) > 4 && slip_count != 0)
		{
			cv::putText(c, "Slip", Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255));
		}
		//std::cout << sqrt(pow((test.pt.x - before_point.x), 2) + pow((test.pt.y - before_point.y), 2)) << std::endl;
		slip_count = 1;
		before_point = test.pt;
		////

		cv::imshow("hand", c);
		cv::imshow("gray", d);
		cv::waitKey(10);

		//// Apply calibration and equilibration to frame data (optional)
		equilibration->TekEquilibrate(frameData); // passed by reference, data is now equilibrated

		array<double, 1>^ frameDataCalibrated;
		
		errorCode = calibration->TekCalibrate(frameData, frameDataCalibrated);
		////
		MatrixXd m(rows, columns);
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				m(i, j) = frameDataCalibrated[i * columns + j];
			}
		}
#pragma endregion

#pragma region 取得各區塊壓力數值 以及 各手指區域重心位置
		MatrixXd mA(4, 4);
		MatrixXd mB(3, 4);
		MatrixXd mC(5, 9);
		MatrixXd mD(9, 8);
		MatrixXd mE(4, 19);

		MatrixXd StartAxis(17, 2);
		if (HandFlag == 0)
		{
			StartAxis << 20, 17,
				24, 5,
				13, 6,
				20, 0,
				15, 0,
				9, 6,
				5, 6,
				0, 6,
				9, 11,
				5, 11,
				0, 11,
				9, 16,
				5, 16,
				0, 16,
				9, 21,
				5, 21,
				0, 21;
		}
		else
		{
			StartAxis << 20, 0,
				24, 11,
				13, 0,
				20, 21,
				15, 21,
				9, 15,
				5, 15,
				0, 15,
				9, 10,
				5, 10,
				0, 10,
				9, 5,
				5, 5,
				0, 5,
				9, 0,
				5, 0,
				0, 0;
		}

		for (int i = 0; i < 17; i++)
		{
			if (i == 4 || i==7 || i==10 || i==13 || i==16)
			{
				mA = m.block<4, 4>(StartAxis(i, 0), StartAxis(i, 1));
				HandID[i] = mA;
				AveragePress[i] = mA.mean();
			}
			else if (i == 3 || i == 5 || i == 6 || i == 8 || i == 9 || i == 11 || i == 12 || i == 14 || i == 15)
			{
				mB = m.block<3, 4>(StartAxis(i, 0), StartAxis(i, 1));
				HandID[i] = mB;
				AveragePress[i] = mB.mean();
			}
			else if (i == 1)
			{
				mC = m.block<5, 9>(StartAxis(i, 0), StartAxis(i, 1));
				HandID[i] = mC;
				AveragePress[i] = mC.mean();
			}
			else if (i == 0)
			{
				mD = m.block<9, 8>(StartAxis(i, 0), StartAxis(i, 1));
				HandID[i] = mD;
				AveragePress[i] = mD.sum();
				AveragePress[i] += 20;
				AveragePress[i] = AveragePress[i] / 52;
			}
			else if (i == 2)
			{
				mE = m.block<4, 19>(StartAxis(i, 0), StartAxis(i, 1));
				HandID[i] = mE;
				AveragePress[i] = mE.mean();
			}
			else
			{
			}
		}

#pragma endregion

#pragma region control_theory
		//// 7個指令 大拇指 大拇指下指節、上指節 食指下指節、上指節 中指下指節、上指節
		//// 對應需要少3 去除手掌中心3塊
		//// Pressure Reference 只放7個數值，第一個無意義，後續分別對應 ID: 3, 4, 5, 7, 8, 10 (參考ID設定)
		for (int i = 1; i < 7; i++)
		{
			if (abs(AveragePress[SControl.ID[i - 1]] - sp::Pressure_References[i]) > 0.0001)
			{
				SControl.SlipRule(AveragePress[SControl.ID[i-1]], sp::Pressure_References[i], i);
				SControl.update_Presure(AveragePress[SControl.ID[i-1]], i);
			}
			else{
				sp::Pressure_References[i] = sp::Pressure_References[i];  // 維持力量不變
			}
			AngleRef.Angle_PID(sp::path_Ref[i], AveragePress[SControl.ID[i-1]], sp::Pressure_References[i]);
			////修改新參考角度
		}
		std::cout << sp::Pressure_References[1] << std::endl;
		sp::pathR2E_1D(sp::path_Ref, sp::ePath_L);
#pragma endregion

		//megaSendPath();
	}
    // Print frame to console in space-delimited format
    // The array returned when capturing a frame contains columns from the sensor appended in sequence
    
	//marshal_context^ context = gcnew marshal_context();
	//const char* c_s = context->marshal_as<const unsigned char*>(frameData[i]);

    //// Apply calibration and equilibration to frame data (optional)
    //equilibration->TekEquilibrate(frameData); // passed by reference, data is now equilibrated
	
    //array<double, 1>^ frameDataCalibrated;
	
    //errorCode = calibration->TekCalibrate(frameData, frameDataCalibrated);

    //// Take a recording
    //CTekAPI::TekStartRecording(5); // 5 second recording
    //printf("Recording...");

    //// Total number frames that will be collected
    //int framesToRecord;
    //errorCode = CTekAPI::TekGetFramesToRecord(serialNumber, framesToRecord);

    //// Wait until the recording is complete
    //int framesRecorded;
    //while (CTekAPI::TekIsRecording() == 0)
    //{
    //    // Can also get the number of frames recorded so far
    //    errorCode = CTekAPI::TekGetFramesRecorded(serialNumber, framesRecorded);
    //    System::Threading::Thread::Sleep(100);
    //}
    //printf("Recording complete.");

    //// Or, can manually stop the recording at any point
    //CTekAPI::TekStopRecording();

    //// Save the recording
    //String^ recordingPath ("C:\\Tekscan\\TekAPI\\Samples\\MySampleRecording.fsx");
    //CTekAPI::TekSaveRecording(serialNumber, recordingPath);
    //printf("Recording saved.");
    //// OR
    //// Recordings can also be saved with calibrations and equilibrations
    //// CTekAPI.TekSaveRecording(serialNumber, ref recordingPath, equilibration);
    //// CTekAPI.TekSaveRecording(serialNumber, ref recordingPath, calibration);
    //// CTekAPI.TekSaveRecording(serialNumber, ref recordingPath, equilibration, calibration);

    /* IMPORTANT: Release hardware resources
        * Failure to do this could leave hardware in an unusable state, requiring
        * the handle/hub to be disconnected and reconnected to cycle power. Placing
        * this code in the catch block of a try-catch can ensure that errors in
        * other parts of the program do not prevent these statements from executing.
        */
    CTekAPI::TekReleaseSensor(serialNumber);
    CTekAPI::TekDeinitializeHardware();

	//system("PAUSE");
	return 0;
}