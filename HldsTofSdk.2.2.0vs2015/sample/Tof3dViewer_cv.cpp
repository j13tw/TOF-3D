/**
* @file			Tof3dViewer_cv.cpp
* @brief		Sample program for Hitachi-LG Data Storage (HLDS)'s TOF Motion Sensor
* @author		Hitachi-LG Data Storage, Inc. (HLDS)
* @date			2018.02.13
* @version		v1.1.0
* @copyright	Hitachi-LG Data Storage,Inc.
*
* @par Change History:
* - 2016.04.22 New
* - 2016.05.23 v1.0.1
*					- Add camera mode setting
* - 2017.05.25 v1.0.2
*					- Add edge noise reduction
* - 2018.01.05 v1.0.3
*					- Correct x,y range of frame3d
* - 2018.02.13 v1.1.0
*					- Add close window function
*/

#define _CRT_SECURE_NO_WARNINGS

#include <Windows.h>
#include <opencv2/opencv.hpp>
#include <fstream>

#include "tof.h"

using namespace std;
using namespace hlds;

#define M_PI 3.14159265358979 /* Pi */
#define deg2rad(a) ( (a) / 180.0 * M_PI ) /* Macro to convert deg to rad */

#define DISPLAY_WIDTH		(640)		//Width of Main Display
#define DISPLAY_HEIGHT		(480)		//Height of Main Display

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

	// Create a Tof instance for a TOF sensor
	Tof tof;

	// Open Tof instance (Set TOF information)
	if (tof.Open(ptofinfo[0]) != Result::OK){
		std::cout << "TOF ID " << ptofinfo[0].tofid << " Open Error" << endl;
		system("pause");
		return;
	}

	// Once Tof instance is started, TofManager is not necessary and closed
	if (tofm.Close() != Result::OK){
		std::cout << "TofManager Close Error" << endl;
		system("pause");
		return;
	}

	// Start Tof instance (Start data transferring)

	// Set camera mode(Depth)
	if (tof.SetCameraMode(CameraMode::CameraModeDepth) != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Mode Error" << endl;
		system("pause");
		return;
	}

	// Set camera resolution
	if (tof.SetCameraPixel(CameraPixel::w320h240) != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Pixel Error" << endl;
		system("pause");
		return;
	}

	//Edge noise reduction
	if (tof.SetEdgeSignalCutoff(EdgeSignalCutoff::Enable) != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Edge Noise Reduction Error" << endl;
		system("pause");
		return;
	}

	// Start(Start data transferring)
	std::cout << "TOF ID " << tof.tofinfo.tofid << " Run OK" << endl;
	if (tof.Run() != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Run Error" << endl;
		system("pause");
		return;
	}
	std::cout << "TOF ID " << tof.tofinfo.tofid << " Run OK" << endl;

	// Create instance for reading frame
	FrameDepth frame;

	// Create instance for frame after 3D conversion
	Frame3d frame3d;

	// Make color table
	frame.CreateColorTable(0, 65530);

	bool berror = false;

	try {

		// Create windows for display as resizable
		cv::namedWindow("TOF 3D Viewer with OpenCV", CV_WINDOW_NORMAL);

		// Z buffer (to recognize front/back position)
		cv::Mat z_buffer(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_16UC1);

		// Display image
		cv::Mat img(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC3);

		// Initialize
		TCHAR mode;
		float angle_x = 0;		// X angle(degree)
		float angle_y = 0;		// Y angle(degree)
		float dx = 0;			// X shift (mm)
		float dy = 0;			// Y shift (mm)
		float min_depth = 500;	// Min distance between sensor(mm)
		float max_depth = 2000;	// Max distance between sensor(mm)
		float min_z = -1000;	// Min Z distance level on display(mm)	
		float max_z = 1000;		// Max Z distance level on display(mm)

		bool brun = true;
		while (brun){

			// Get the latest frame number
			long frameno;
			TimeStamp timestamp;
			tof.GetFrameStatus(&frameno, &timestamp);

			if (frameno != frame.framenumber){
				// Read a new frame only if frame number is changed(Old data is shown if it is not changed.)

				// Read a frame of depth data
				if (tof.ReadFrame(&frame) != Result::OK){
					std::cout << "Tof ReadFrame Error" << endl;
					berror = true;
					break;
				}

				// Convert to 3D(with lens correction)
				frame3d.Convert(&frame);

				// Initialize matrix
				z_buffer = cv::Mat::zeros(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_16UC1);
				img = cv::Mat::zeros(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC3);

				for (int y = 0; y < DISPLAY_HEIGHT; y++){
					for (int x = 0; x < DISPLAY_WIDTH; x++){

						int tofx = x * frame3d.width / DISPLAY_WIDTH;
						int tofy = y * frame3d.height / DISPLAY_HEIGHT;

						if ((frame3d.frame3d[tofy * frame3d.width + tofx].z >= min_depth) && (frame3d.frame3d[tofy * frame3d.width + tofx].z <= max_depth)){
							// Valid data(Only data is valid that distance between sensor is shorter than specific length)

							cv::Point3f point0;	// Coordinate before rotate
							cv::Point3f point;	// Coordinate after rotate

							// Get a point after 3D conversion
							point0.x = frame3d.frame3d[tofy * frame3d.width + tofx].x;
							point0.y = frame3d.frame3d[tofy * frame3d.width + tofx].y;
							point0.z = frame3d.frame3d[tofy * frame3d.width + tofx].z;

							// Rote X-axis
							point.x = point0.x;
							point.y = point0.y * cos(deg2rad(angle_x)) - point0.z * sin(deg2rad(angle_x));
							point.z = point0.y * sin(deg2rad(angle_x)) + point0.z * cos(deg2rad(angle_x));

							point0 = point;

							// Rote Y-axis
							point.x = point0.x * cos(deg2rad(angle_y)) + point0.z * sin(deg2rad(angle_y));
							point.y = point0.y;
							point.z = -1 * point0.x * sin(deg2rad(angle_y)) + point0.z * cos(deg2rad(angle_y));

							// X/Y direction shift on display
							point.x = point.x + dx;
							point.y = point.y + dy;

							// Adjust X/Y origin to center of display
							point.x = point.x / 10 + DISPLAY_WIDTH / 2;
							point.y = point.y / 10 + DISPLAY_HEIGHT / 2;

							if ((point.x >= 0) && (point.x < DISPLAY_WIDTH) &&
								(point.y >= 0) && (point.y < DISPLAY_HEIGHT)){

								if ((point.z >= min_z) && (point.z <= max_z)){
									// Within Z scope(the scope can be changed by h/l key)

									if ((z_buffer.at<unsigned short>(point.y, point.x) == 0) ||
										(z_buffer.at<unsigned short>(point.y, point.x) > point.z)){
										// The point is more front than Z buffer point

										// Register to Z buffer
										z_buffer.at<unsigned short>(point.y, point.x) = point.z;

										// Register color to display(x,y of coordinate is corresponded to 640 x 480 = 640mm x 480mm)
										cv::Vec3b v;
										v.val[0] = frame.ColorTable[0][frame.databuf[tofy * frame.width + tofx]];
										v.val[1] = frame.ColorTable[1][frame.databuf[tofy * frame.width + tofx]];
										v.val[2] = frame.ColorTable[2][frame.databuf[tofy * frame.width + tofx]];
										img.at<cv::Vec3b>(point.y, point.x) = v;
									}
								}
							}
						}
					}
				}

				// Display information
				string text = "q key : Quit, x key : Move, r key :Reset";
				cv::putText(img, text, cv::Point(30, 30), cv::FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(255, 255, 255), 1.5, CV_AA);
				text = "a key : Angle x=" + std::to_string((int)angle_x) + "[degree] y=" + std::to_string((int)angle_y) + "[degree]";
				cv::putText(img, text, cv::Point(30, 50), cv::FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(255, 255, 255), 1.5, CV_AA);
				text = "h/l key : Filter High=" + std::to_string((int)max_z) + "[mm] Low=" + std::to_string((int)min_z) + "[mm]";
				cv::putText(img, text, cv::Point(30, 70), cv::FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(255, 255, 255), 1.5, CV_AA);

				if (NULL == cvGetWindowHandle("TOF 3D Viewer with OpenCV")){
					brun = false;
				}
				else{
					cv::imshow("TOF 3D Viewer with OpenCV", img);
				}
			}

			auto key = cv::waitKey(10);
			switch (key){
			case 'a':
				mode = 'a';
				break;
			case 'x':
				mode = 'x';
				break;
			case 'h':
				mode = 'h';
				break;
			case 'l':
				mode = 'l';
				break;
			case 'r':
				angle_x = 0;
				angle_y = 0;
				dy = 0;
				dx = 0;
				max_z = 1000;
				min_z = -1000;
				break;
			case 2490368:	//Å™
				switch (mode){
				case 'a':
					angle_x += 5;
					break;
				case 'x':
					dy -= 100;
					break;
				case 'h':
					max_z += 10;
					break;
				case 'l':
					min_z += 10;
					break;
				}
				break;
			case 2621440:	//Å´
				switch (mode){
				case 'a':
					angle_x -= 5;
					break;
				case 'x':
					dy += 100;
					break;
				case 'h':
					max_z -= 10;
					break;
				case 'l':
					min_z -= 10;
					break;
				}
				break;
			case 2555904:	//Å®
				switch (mode){
				case 'a':
					angle_y += 5;
					break;
				case 'x':
					dx += 100;
					break;
				case 'h':
					max_z += 1;
					break;
				case 'l':
					min_z += 1;
					break;
				}
				break;
			case 2424832:	//Å©
				switch (mode){
				case 'a':
					angle_y -= 5;
					break;
				case 'x':
					dx -= 100;
					break;
				case 'h':
					max_z -= 1;
					break;
				case 'l':
					min_z -= 1;
					break;
				}
				break;
			case  'q':
				brun = false;
				break;
			}
		}
	}
	catch (std::exception& ex){
		std::cout << ex.what() << std::endl;
	}

	// Stop and close TOF sensor
	if (tof.Stop() != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Stop Error" << endl;
		berror = true;
	}

	Sleep(2000);

	if (tof.Close() != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Close Error" << endl;
		berror = true;
	}

	cv::destroyAllWindows();

	if (berror){
		system("pause");
	}
}