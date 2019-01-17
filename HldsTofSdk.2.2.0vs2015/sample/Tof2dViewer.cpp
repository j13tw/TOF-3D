/**
* @file			Tof2dViewer.cpp
* @brief		Sample program for Hitachi-LG Data Storage (HLDS)'s TOF Motion Sensor
* @author		Hitachi-LG Data Storage, Inc. (HLDS)
* @date			2018.02.07
* @version		v1.2.0
* @copyright	Hitachi-LG Data Storage,Inc.
*
* @par Change History:
* - 2016.04.22 New
* - 2016.05.27 v1.0.1
*					- Add camera mode setting
* - 2017.05.25 v1.0.2
*					- Add edge noise reduction
* - 2017.10.25 v1.1.0
*					- Add mouse point function
* - 2018.02.07 v1.2.0
*					- Add close window function
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <Windows.h>

#include "tof.h""

#include "opencv2\opencv.hpp"
//#include <opencv2/opencv.hpp>
//#include <opencv2/opencv_lib.hpp>

using namespace std;
using namespace hlds;

// Mouse Information  
struct {
	int x;		// X coodinate
	int y;		// Y coodinate
	int event;	// event
	int flags;	// flag
} mouse;

void MouseCallBack(int event, int x, int y, int flags, void* userdata)
{
	mouse.x = x;
	mouse.y = y;
	mouse.event = event;
	mouse.flags = flags;
}

void main(void)
{
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

			// Set camera mode(Depth)
			if (tof[tofno].SetCameraMode(CameraMode::CameraModeDepth) != Result::OK){
				std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Set Camera Mode Error" << endl;
				system("pause");
				return;
			}

			// Set camera resolution
			if (tof[tofno].SetCameraPixel(CameraPixel::w320h240) != Result::OK){
				//		if (tof[tofno].SetCameraPixel((CameraPixel)(tofno % 7)) != Result::OK){
				std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Set Camera Pixel Error" << endl;
				system("pause");
				return;
			}

			//Edge noise reduction
			if (tof[tofno].SetEdgeSignalCutoff(EdgeSignalCutoff::Enable) != Result::OK){
				std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Edge Noise Reduction Error" << endl;
				system("pause");
				return;
			}

			// Start(Start data transferring)
			std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Run OK" << endl;
			Result ret2 = Result::OK;
			ret2 = tof[tofno].Run();
			if (ret2 != Result::OK){
				std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Run Error" << endl;
				printf("ret: %d\n", ret2);
				system("pause");
				return;
			}
			std::cout << "TOF ID " << tof[tofno].tofinfo.tofid << " Run OK" << endl;
		}
	}
	// Create windows for display as resizable
	cv::namedWindow("TOF 2D Viewer", CV_WINDOW_NORMAL);

	// Callback Setting for mouse point
	int mouse_x = 0;
	int mouse_y = 0;
	memset(&mouse, 0, sizeof(mouse));
	cv::setMouseCallback("TOF 2D Viewer", MouseCallBack, &mouse.event);

	// Matrix for display data(a parent display if multi displays)
	int sub_width = IMAGE_MAX_WIDTH;
	int sub_height = IMAGE_MAX_HEIGHT;
	//	cv::Mat screen(sub_height * screen_row, sub_width * screen_col, CV_16UC1);
	cv::Mat screen(sub_height * screen_row, sub_width * screen_col, CV_8UC3);

	// Create instances for reading frames
	FrameDepth * frame = new FrameDepth[numoftof];

	// Set color information in each frame
	for (int tofno = 0; tofno < numoftof; tofno++){
		if (tofenable[tofno] == true){
			frame[tofno].CreateColorTable(0, 65530);
		}
	}

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
	bool isFlip = false;		// Default off
	bool isGraph = false;		// Default off
	bool isPoint = false;		// Default off
	bool isInfo = true;			// Default on
	bool isTracking = true;		// Default on

	// Graph information
	int graph_cnt = sub_width / 2;
	float graph_min = (float)(sub_height * 0.25);
	float graph_max = (float)(sub_height * 0.75);

	// Create and initialize buffer for graph (for each TOF sensors)
	float ** graph = new float*[numoftof];
	for (int tofno = 0; tofno < numoftof; tofno++){
		graph[tofno] = new float[graph_cnt];
		for (int i = 0; i < graph_cnt; i++){
			graph[tofno][i] = 0;
		}
	}

	// For frame timestamp
	TimeStamp* ts = new TimeStamp[numoftof];
	for (int tofno = 0; tofno < numoftof; tofno++){
		ZeroMemory(&ts[tofno], sizeof(TimeStamp));
	}

	bool berror = false;
	bool isCloseWindow = false;

	try {

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
						if (tof[tofno].ReadFrame(&frame[tofno]) != Result::OK){
							std::cout << "Tof ReadFrame Error" << endl;
							berror = true;
							break;
						}

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

					// Create color picture
					std::vector<unsigned char> buf;
					buf.resize(frame[tofno].width * frame[tofno].height * COLOR_CH_NUM);

					if (isFlip == false){
						// Reverse(Mirror) mode
						for (int i = 0; i < frame[tofno].height; i++){
							for (int j = 0; j < frame[tofno].width; j++){
								for (int ch = 0; ch < COLOR_CH_NUM; ch++){
									buf[(i * frame[tofno].width + j)*COLOR_CH_NUM + ch] = frame[tofno].ColorTable[ch][frame[tofno].databuf[i * frame[tofno].width + (frame[tofno].width - j - 1)]];
								}
							}
						}
					}
					else{
						// Normal(Camera view) mode
						for (int i = 0; i < frame[tofno].width * frame[tofno].height; i++){
							for (int ch = 0; ch < COLOR_CH_NUM; ch++){
								buf[i * COLOR_CH_NUM + ch] = frame[tofno].ColorTable[ch][frame[tofno].databuf[i]];
							}
						}
					}

					// Make OpenCV matrix (8bits 3 colors) from depth data
					cv::Mat sub(frame[tofno].height, frame[tofno].width, CV_8UC3, &buf[0]);

					// Set ROI to the position in multi display
					int col = tofno % screen_col;
					int row = tofno / screen_row;
					cv::Mat roi = screen(cv::Rect(col * sub_width, row * sub_height, sub_width, sub_height));

					// Copy and adjust size the matrix of depth data to ROI(display position)
					cv::resize(sub, roi, roi.size(), cv::INTER_LINEAR);

					string text;

					// Display operations
					text = "t:info, g:graph, p:point, r:flip, q:quit";
					cv::putText(roi, text, cv::Point(30, sub_height - 10), cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(0, 0, 0), 2, CV_AA);


					if (isInfo){
						// Display TOF ID and IP address
						text = "TOF ID:" + tof[tofno].tofinfo.tofid + "   IP:" + tof[tofno].tofinfo.tofip;
						cv::putText(roi, text, cv::Point(30, 30), cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(255, 0, 0), 2, CV_AA);

						// Display FPS and timestamp
						text = std::to_string(timer[tofno].fps) + "fps  " + std::to_string(ts[tofno].month) + "/" + std::to_string(ts[tofno].day) + " "
							+ std::to_string(ts[tofno].hour) + ":" + std::to_string(ts[tofno].minute) + ":" + std::to_string(ts[tofno].second) + "." + std::to_string(ts[tofno].msecond);
						cv::putText(roi, text, cv::Point(30, 70), cv::FONT_HERSHEY_TRIPLEX, 0.7, cv::Scalar(255, 0, 0), 1.2, CV_AA);
					}
				}
				else{
					// Create gray picture
					std::vector<unsigned char> buf;
					buf.resize(frame[tofno].width * frame[tofno].height * COLOR_CH_NUM);

					for (int i = 0; i < frame[tofno].width * frame[tofno].height; i++){
						for (int ch = 0; ch < COLOR_CH_NUM; ch++){
							buf[i * COLOR_CH_NUM + ch] = 100;
						}
					}

					// Make OpenCV matrix(8 bits 3 colors)
					cv::Mat sub(frame[tofno].height, frame[tofno].width, CV_8UC3, &buf[0]);

					// Set ROI to the position in multi display
					int col = tofno % screen_col;
					int row = tofno / screen_row;
					cv::Mat roi = screen(cv::Rect(col * sub_width, row * sub_height, sub_width, sub_height));

					// Copy and adjust size the matrix of depth data to ROI(display position)
					cv::resize(sub, roi, roi.size(), cv::INTER_LINEAR);

					string text;

					// Display operations
					text = "t:info, g:graph, p:point, r:flip, q:quit";
					cv::putText(roi, text, cv::Point(30, sub_height - 10), cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(0, 0, 0), 2, CV_AA);


					if (isInfo){
						// Display TOF ID and IP address
						text = "TOF ID:" + tof[tofno].tofinfo.tofid + "   IP:" + tof[tofno].tofinfo.tofip;
						cv::putText(roi, text, cv::Point(30, 30), cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(255, 0, 0), 2, CV_AA);

						// Display Error
						text = "Not Connected";
						cv::putText(roi, text, cv::Point(30, 70), cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(255, 0, 0), 2, CV_AA);
					}
				}
			}

			// Display depth[mm] at mouse point
			// Left click, update point
			if (isTracking){
				mouse_x = mouse.x;
				mouse_y = mouse.y;
				if (mouse.event == cv::EVENT_LBUTTONDOWN){
					isTracking = !isTracking;
				}
			}
			else{
				if (mouse.event == cv::EVENT_LBUTTONDOWN){
					mouse_x = mouse.x;
					mouse_y = mouse.y;
				}
				// Right click, mouse point
				if (mouse.event == cv::EVENT_RBUTTONDOWN){
					isTracking = true;
				}
			}

			if (isGraph || isPoint){
				if ((mouse_x > 0) && (mouse_y > 0) && (mouse_x < (screen.cols / screen_col)) && (mouse_y < (screen.rows / screen_row))){
					int mx = (mouse_x * frame[0].width) / (screen.cols / screen_col);
					int my = (mouse_y * frame[0].height) / (screen.rows / screen_row);
					float depth = 0;

					if (isFlip){
						depth = frame[0].CalculateLength(frame[0].databuf[frame[0].width * my + mx]);
					}
					else{
						depth = frame[0].CalculateLength(frame[0].databuf[frame[0].width * my + frame[0].width - mx]);
					}

					if (depth < 0){
						depth = 0;
					}

					// Display X mark at mouse point
					cv::line(screen, cv::Point(mouse_x - 5, mouse_y - 5), cv::Point(mouse_x + 5, mouse_y + 5),
						cv::Scalar(255, 0, 0), 2);
					cv::line(screen, cv::Point(mouse_x - 5, mouse_y + 5), cv::Point(mouse_x + 5, mouse_y - 5),
						cv::Scalar(255, 0, 0), 2);

					// Display graph at mouse point
					if (isGraph){
						// Display distance at center
						string text = std::to_string(depth / 1000) + "m";
						cv::putText(screen, text, cv::Point(sub_width / 2 + 10, sub_height / 2 + 8), cv::FONT_HERSHEY_TRIPLEX,
							0.8, cv::Scalar(255, 0, 0), (int)1.5, CV_AA);

						// Graph scroll
						float data_max = depth;
						float data_min = depth;
						for (int x = 0; x < graph_cnt - 1; x++){
							graph[0][x] = graph[0][x + 1];
							if (data_max < graph[0][x]){
								data_max = graph[0][x];
							}
							if ((graph[0][x] > 0) && (data_min > graph[0][x])){
								data_min = graph[0][x];
							}
						}
						graph[0][graph_cnt - 1] = depth;

						// Display graph
						if (data_max != data_min){
							float zoom = (graph_max - graph_min) / (data_max - data_min);
							if (zoom > 0.5){
								zoom = 0.5;
							}
							for (int x = 0; x < graph_cnt - 1; x++){
								if ((graph[0][x] > 0) && (graph[0][x + 1] > 0)){
									cv::line(screen, cv::Point(x, (int)(sub_height - ((graph[0][x] - data_min) * zoom + graph_min))),
										cv::Point(x + 1, (int)(sub_height - ((graph[0][x + 1] - data_min) * zoom + graph_min))), cv::Scalar(0, 0, 255), 2);
								}
							}
						}

						// Display max bar and max value of graph
						cv::line(screen, cv::Point(0, sub_height - graph_max), cv::Point(sub_width / 2, sub_height - graph_max),
							cv::Scalar(0, 0, 255), 1);
						text = std::to_string(data_max / 1000) + "m";
						cv::putText(screen, text, cv::Point(sub_width / 2 + 10, sub_height - graph_max + 8), cv::FONT_HERSHEY_TRIPLEX,
							0.8, cv::Scalar(0, 0, 255), (int)1.5, CV_AA);

						// Display min bar and min value of graph
						cv::line(screen, cv::Point(0, sub_height - graph_min), cv::Point(sub_width / 2, sub_height - graph_min),
							cv::Scalar(0, 0, 255), 1.0);
						text = std::to_string(data_min / 1000) + "m";
						cv::putText(screen, text, cv::Point(sub_width / 2 + 10, sub_height - graph_min + 8), cv::FONT_HERSHEY_TRIPLEX,
							0.8, cv::Scalar(0, 0, 255), (int)1.5, CV_AA);
					}

					// Display depth[mm] at mouse point
					if (isPoint){
						// Black
						cv::rectangle(screen, cv::Point(500, 45), cv::Point(640, 75), cv::Scalar(0, 0, 0), -1, CV_AA);

						string text = to_string((int)depth) + "mm";
						cv::putText(screen, text, cv::Point(510, 70), cv::FONT_HERSHEY_TRIPLEX, 0.7, cv::Scalar(255, 255, 255), 0.8, CV_AA);
					}
				}
			}

			if (NULL == cvGetWindowHandle("TOF 2D Viewer")){
				isCloseWindow = true;
			}
			else{
				// Display after all multi display are ready
				cv::imshow("TOF 2D Viewer", screen);
			}

			// Quit if q key is pushed
			TCHAR key = cv::waitKey(1);
			if (key == 'q' || isCloseWindow == true){
				printf("Stopping program...");
				break;
			}
			else if (key == 'r')	//Reverse if r key is pushed
			{
				isFlip = !isFlip;
			}
			else if (key == 'g')	//Display graph if g key is pushed
			{
				isGraph = !isGraph;
				isPoint = false;
				if (isTracking == true){
					//If traccking is on, set to distance point at center.
					mouse_x = sub_width / 2;
					mouse_y = sub_height / 2;
					isTracking = false;
				}
			}
			else if (key == 'p')	//Display depth at mouse point if p key is pushed
			{
				isPoint = !isPoint;
				isGraph = false;
			}
			else if (key == 't')	//Display TOF ID, FPS, timestamp if i key is pushed
			{
				isInfo = !isInfo;
			}
		}
	}
	catch (std::exception& ex){
		std::cout << ex.what() << std::endl;
	}

	// Stop and close all TOF sensors
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
	delete[] graph;
	cv::destroyAllWindows();

	if (berror){
		system("pause");
	}
}
