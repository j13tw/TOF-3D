/**
* @file			TofIrViewer.cpp
* @brief		Sample program for Hitachi-LG Data Storage (HLDS)'s TOF Motion Sensor
* @author		Hitachi-LG Data Storage, Inc. (HLDS)
* @date			2018.02.13
* @version		v1.1.0
* @copyright	Hitachi-LG Data Storage,Inc.
*
* @par Change History:
* - 2016.05.04 New
* - 2016.05.06 v1.0.1
*					- Delete amplification of IR data
* - 2016.06.03 v1.0.2
*					- Delete unnecessary comment
* - 2018.02.13 v1.1.0
*					- Add close window function
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <Windows.h>

#include "tof.h"

#include <opencv2/opencv.hpp>
//#include <opencv2/opencv_lib.hpp>

using namespace std;
using namespace hlds;

void main(void)
{
	//IR Gain
	int irgain = 8;

	// Create TofManager
	TofManager tofm;

	// Open TOF Manager (Read tof.ini file)
	if (tofm.Open() != Result::OK){
		std::cout << "TofManager Open Error (may not be tof.ini file)" << endl;
		system("pause");
		return;
	}

	// Get number of TOF sensor and TOF information list
	const TofInfo * ptofinfo = nullptr;
	int numoftof = tofm.GetTofList(&ptofinfo);

	if (numoftof == 0){
		std::cout << "No TOF Sensor" << endl;
		system("pause");
		return;
	}

	// Decide number of multi displays depending on number of TOF sensor
	int screen_row;
	int screen_col;
	screen_row = screen_col = (int)ceil(sqrt(numoftof));

	// Create Tof instances for TOF sensors
	Tof * tof = new Tof[numoftof];

	// Flags for enable/disable of TOF sensors
	bool * tofenable = new bool[numoftof];

	// Open all Tof instances (Set TOF information)
	for (int tofno = 0; tofno < numoftof; tofno++){
		if (tof[tofno].Open(ptofinfo[tofno]) != Result::OK){
			std::cout << "TOF ID " << ptofinfo[tofno].tofid << " Open Error" << endl;

			tofenable[tofno] = false;
		}
		else{
			tofenable[tofno] = true;
		}
	}

	// Once Tof instances are started, TofManager is not necessary and closed
	if (tofm.Close() != Result::OK){
		std::cout << "TofManager Close Error" << endl;
		system("pause");
		return;
	}

	// Start all Tof instances (Start data transferring)
	for (int tofno = 0; tofno < numoftof; tofno++){

		// Start only enable TOF sensor
		if (tofenable[tofno] == true){

			// Set camera mode as IR mode
			if (tof[tofno].SetCameraMode(CameraMode::CameraModeIr) != Result::OK){
				std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Set Camera Mode Error" << endl;
				system("pause");
				return;
			}

			// Set IR gain
			if (tof[tofno].SetIrGain(irgain) != Result::OK){
				std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Set IR Gain Error" << endl;
				system("pause");
				return;
			}

			// Start(Start data transferring)
			std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Run OK" << endl;
			if (tof[tofno].Run() != Result::OK){
				std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Run Error" << endl;
				system("pause");
				return;
			}
			std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Run OK" << endl;
		}
	}
	// Create windows for display as resizable
	cv::namedWindow("TOF IR Viewer", CV_WINDOW_NORMAL);

	// Matrix for display data(a parent display if multi displays)
	int sub_width = IMAGE_MAX_WIDTH;
	int sub_height = IMAGE_MAX_HEIGHT;
	cv::Mat screen(sub_height * screen_row, sub_width * screen_col, CV_16UC1);
	//cv::Mat screen(sub_height * screen_row, sub_width * screen_col, CV_8UC3);

	// Create instances for reading frames
	FrameIr * frame = new FrameIr[numoftof];

	// For measure FPS
	struct Timer {
		float fps;
		clock_t start;
		int framecount;
	};
	Timer * timer = new Timer[numoftof];
	for (int tofno = 0; tofno < numoftof; tofno++){
		timer[tofno].fps = 0;
		timer[tofno].framecount = -1;
	}

	// Flags for display modes
	bool isFlip = false;	// Default off
	bool isInfo = true;		// Default on

	// For frame timestamp
	TimeStamp* ts = new TimeStamp[numoftof];
	for (int tofno = 0; tofno < numoftof; tofno++){
		ZeroMemory(&ts[tofno], sizeof(TimeStamp));
	}

	bool isCloseWindow = false;

	// Main loop(Until q key pushed)
	while (1){

		// Loop for each TOF sensor
		for (int tofno = 0; tofno < numoftof; tofno++){

			if (tofenable[tofno] == true){

				// Get the latest frame number
				long frameno;
				TimeStamp timestamp;
				tof[tofno].GetFrameStatus(&frameno, &timestamp);

				if (frameno != frame[tofno].framenumber){
					// Read a new frame only if frame number is changed(Old data is shown if it is not changed.)

					// Read a frame of depth data
					tof[tofno].ReadFrame(&frame[tofno]);

					// Measure FPS(every 1 sec.)
					if (timer[tofno].framecount != -1){
						timer[tofno].framecount++;
						clock_t diff = clock() - timer[tofno].start;
						if (diff / CLOCKS_PER_SEC >= 1){
							timer[tofno].fps = (float)timer[tofno].framecount * CLOCKS_PER_SEC / (float)diff;
							timer[tofno].framecount = -1;
						}
					}
					if (timer[tofno].framecount == -1){
						timer[tofno].framecount = 0;
						timer[tofno].start = clock();
					}

					// Get timestamp
					memcpy(&ts[tofno], &frame[tofno].timestamp, sizeof(TimeStamp));
				}

				// Create picture
				std::vector<unsigned short> buf;
				buf.resize(frame[tofno].width * frame[tofno].height);

				if (isFlip == false){
					// Reverse(Mirror) mode
					for (int i = 0; i < frame[tofno].height; i++){
						for (int j = 0; j < frame[tofno].width; j++){
							buf[(i * frame[tofno].width + j)] = frame[tofno].databuf[i * frame[tofno].width + (frame[tofno].width - j - 1)];
						}
					}
				}
				else{
					// Normal(Camera view) mode
					for (int i = 0; i < frame[tofno].width * frame[tofno].height; i++){
						buf[i] = frame[tofno].databuf[i];
					}
				}

				// Make OpenCV matrix (16bits 1 color) from IR data
				cv::Mat sub(frame[tofno].height, frame[tofno].width, CV_16UC1, &buf[0]);

				// Set ROI to the position in multi display
				int col = tofno % screen_col;
				int row = tofno / screen_row;
				cv::Mat roi = screen(cv::Rect(col * sub_width, row * sub_height, sub_width, sub_height));

				// Copy and adjust size the matrix of depth data to ROI(display position)
				cv::resize(sub, roi, roi.size(), cv::INTER_LINEAR);

				string text;

				// Display operations
				text = "t: info, r: flip, q: quit";
				cv::putText(roi, text, cv::Point(30, sub_height - 10), cv::FONT_HERSHEY_TRIPLEX, 0.8, 65535, 2, CV_AA);

				if (isInfo){
					// Display TOF ID and IP address
					text = "TOF ID:" + tof[tofno].tofinfo.tofid + "   IP:" + tof[tofno].tofinfo.tofip;
					cv::putText(roi, text, cv::Point(30, 30), cv::FONT_HERSHEY_TRIPLEX, 0.8, 65535, 2, CV_AA);

					// Display FPS and timestamp
					text = std::to_string(timer[tofno].fps) + "fps  " + std::to_string(ts[tofno].month) + "/" + std::to_string(ts[tofno].day) + " "
						+ std::to_string(ts[tofno].hour) + ":" + std::to_string(ts[tofno].minute) + ":" + std::to_string(ts[tofno].second) + "." + std::to_string(ts[tofno].msecond);
					cv::putText(roi, text, cv::Point(30, 70), cv::FONT_HERSHEY_TRIPLEX, 0.8, 65535, 2, CV_AA);
				}
			}
			else{
				// Create gray picture
				std::vector<unsigned short> buf;
				buf.resize(frame[tofno].width * frame[tofno].height);

				for (int i = 0; i < frame[tofno].width * frame[tofno].height; i++){
					buf[i] = 100 << 8;	// 8 bit -> 16 bit
				}

				// Make OpenCV matrix(16 bits 1 color)
				cv::Mat sub(frame[tofno].height, frame[tofno].width, CV_16UC1, &buf[0]);

				// Set ROI to the position in multi display
				int col = tofno % screen_col;
				int row = tofno / screen_row;
				cv::Mat roi = screen(cv::Rect(col * sub_width, row * sub_height, sub_width, sub_height));

				// Copy and adjust size the matrix of depth data to ROI(display position)
				cv::resize(sub, roi, roi.size(), cv::INTER_LINEAR);

				string text;

				// Display operations
				text = "t: info, r: flip, q: quit";
				cv::putText(roi, text, cv::Point(30, sub_height - 10), cv::FONT_HERSHEY_TRIPLEX, 0.8, 65535, 2, CV_AA);


				if (isInfo){
					// Display TOF ID and IP address
					text = "TOF ID:" + tof[tofno].tofinfo.tofid + "   IP:" + tof[tofno].tofinfo.tofip;
					cv::putText(roi, text, cv::Point(30, 30), cv::FONT_HERSHEY_TRIPLEX, 0.8, 65535, 2, CV_AA);

					// Display Error
					text = "Not Connected";
					cv::putText(roi, text, cv::Point(30, 70), cv::FONT_HERSHEY_TRIPLEX, 0.8, 65535, 2, CV_AA);
				}
			}
		}


		// Display after all multi display are ready
		if (NULL == cvGetWindowHandle("TOF IR Viewer")){
			isCloseWindow = true;
		}
		else{
			cv::imshow("TOF IR Viewer", screen);
		}

		// Quit if q key is pushed
		auto key = cv::waitKey(1);
		if (key == 'q' || isCloseWindow == true){
			printf("Stopping program...");
			break;
		}
		else if (key == 'r')		//Reverse if r key is pushed
		{
			isFlip = !isFlip;
		}
		else if (key == 't')		//Display TOF ID, FPS, timestamp if i key is pushed
		{
			isInfo = !isInfo;
		}
		else if (key == 2490368)	//Å™
		{
			if (irgain < 15){
				irgain++;
				for (int tofno = 0; tofno < numoftof; tofno++){
					// Set IR gain
					if (tofenable[tofno] == true){
						if (tof[tofno].SetIrGain(irgain) != Result::OK){
							std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Set IR Gain Error" << endl;
							system("pause");
							return;
						}
					}
				}
			}
		}
		else if (key == 2621440)	//Å´
		{
			if (irgain > 1){
				irgain--;
				for (int tofno = 0; tofno < numoftof; tofno++){
					// Set IR gain
					if (tofenable[tofno] == true){
						if (tof[tofno].SetIrGain(irgain) != Result::OK){
							std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Set IR Gain Error" << endl;
							system("pause");
							return;
						}
					}
				}
			}
		}
	}

	// Stop and close all TOF sensors
	bool berror = false;
	for (int tofno = 0; tofno < numoftof; tofno++){
		if (tofenable[tofno] == true){
			if (tof[tofno].Stop() != Result::OK){
				std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Stop Error" << endl;
				berror = true;
			}
		}
	}


	for (int tofno = 0; tofno < numoftof; tofno++){
		if (tofenable[tofno] == true){
			if (tof[tofno].Close() != Result::OK){
				std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Close Error" << endl;
				berror = true;
			}
		}
	}

	delete[] timer;
	delete[] frame;
	delete[] tof;
	delete[] tofenable;
	cv::destroyAllWindows();

	if (berror){
		system("pause");
	}
}