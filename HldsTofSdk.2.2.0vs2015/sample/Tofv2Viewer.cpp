/**
* @file			Tofv2Viewer.cpp
* @brief		Sample program for Hitachi-LG Data Storage (HLDS)'s TOFv2 Motion Sensor
* @author		Hitachi-LG Data Storage, Inc. (HLDS)
* @date			2018.02.13
* @version		v1.1.0
* @copyright	Hitachi-LG Data Storage,Inc.
*
* @par Change History:
* - 2016.08.29 New
* - 2017.05.25 v1.0.1
*					- Add edge noise reduction
* - 2018.02.13 v1.1.0
*					- Add close window function
*/


#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <Windows.h>

#include "tof.h"

#include <opencv2/opencv.hpp>
//#include <opencv2/opencv_lib.hpp>

using namespace std;
using namespace hlds;

//Main Display
#define MAIN_DISPLAY_WIDTH		(640 * 2)		//Width of Main Display
#define MAIN_DISPLAY_HEIGHT		(480 * 2)		//Height of Main Display

//Sub Display
#define SUB_DISPLAY_POS_MAX		(4)				//Number of positions of Sub Display
#define SUB_DISPLAY_X_0			(10)			//X-coordinate of lower-left of Sub Display
#define SUB_DISPLAY_Y_0			(590)			//Y-coordinate of lower-left of Sub Display
#define SUB_DISPLAY_X_1			(790)			//X-coordinate of lower-right of Sub Display
#define SUB_DISPLAY_Y_1			(590)			//Y-coordinate of lower-right of Sub Display
#define SUB_DISPLAY_X_2			(790)			//X-coordinate of upper-right of Sub Display
#define SUB_DISPLAY_Y_2			(50)			//Y-coordinate of upper-right of Sub Display
#define SUB_DISPLAY_X_3			(10)			//X-coordinate of upper-left of Sub Display
#define SUB_DISPLAY_Y_3			(50)			//Y-coordinate of upper-left of Sub Display
#define SUB_DISPLAY_WIDTH		(480)			//Width of Sub Display
#define SUB_DISPLAY_HEIGHT		(360)			//Height of Sub Display

int SubDisplayPos = 0;
int SubDisplayX[SUB_DISPLAY_POS_MAX] = { SUB_DISPLAY_X_0, SUB_DISPLAY_X_1, SUB_DISPLAY_X_2, SUB_DISPLAY_X_3 };
int SubDisplayY[SUB_DISPLAY_POS_MAX] = { SUB_DISPLAY_Y_0, SUB_DISPLAY_Y_1, SUB_DISPLAY_Y_2, SUB_DISPLAY_Y_3 };

//Display Image
cv::Mat img(MAIN_DISPLAY_HEIGHT, MAIN_DISPLAY_WIDTH, CV_8UC3);

//Key Input Mode
TCHAR mode;

//Tof Instance
Tof tof;

//Instances to read frame
FrameDepth frame1;
FrameDepth frame2;
FrameIr frameir;

//Capture Related
string capturefile = "TofCapture.bin";		//Capture file name
clock_t capturetime;						//Capture(Record/Replay) start time
#define MAX_CAPTURE_DURATION	(3600.0f)	//Capture(Record) max time (Sec.)

//Flags
bool bSubDisplay = true;			//Sub Display ON
bool bReverseMainSub = false;		//Reverse Sub Display and Main Display
bool bCapture = false;				//Capturing(Recording)
bool bSimulation = false;			//Emulation(Replay) of Capture File
bool bNoSensor = false;				//No TOF Sensor mode(Only emulation)
bool bRepeat = true;				//Repeat playback mode

//Dual Display Setting
int cammode = (int)CameraMode::Depth_Ir;
bool bDepthIr = true;				//Depth and IR display mode
string display1 = "Depth";
string display2 = "IR";

//Background Subtraction Setting
int bginterval = (int)BgInterval::bg1min;
int bgquantity = (int)BgQuantity::bgLv1;

string bgintervalstr[9] = {
	"",
	"1 minute",
	"3 minutes",
	"5 minutes",
	"10 minutes",
	"30 minutes",
	"1 hour",
	"2 hours",
	"5 hours"
};

string bgquantitystr[12] = {
	"",
	"None",
	"Level1",
	"Level2",
	"Level3",
	"Level4",
	"Level5",
	"Level6",
	"Level7",
	"Level8",
	"Level9",
	"Full"
};

//Color of text
cv::Scalar white(255, 255, 255);
cv::Scalar blue(255, 0, 0);
cv::Scalar red(0, 0, 255);
cv::Scalar color = white;

//Save File Name
string savefile;	//Screenshot file

//Repeat
bool brun = true;

void StartCapture(Tof* ptof)
{
	Result rtn = Result::OK;
	CaptureInfo capinf;
	capinf.path = "";
	capinf.filename = capturefile;

	if (bNoSensor){
		std::cout << "Cannot be started in eTOF mode" << endl;
		return;
	}

	ptof->capturetime = MAX_CAPTURE_DURATION;
	rtn = ptof->CreateCaptureFile(capinf);
	if (rtn == Result::OK){

		rtn = ptof->Capture(true);
		if (rtn == Result::OK){
			bCapture = true;

			//Capture start time
			capturetime = clock();
		}
		else {
			std::cout << "Capture Start Error (" << (int)rtn << ")" << endl;
		}
	}
	else {
		std::cout << "Create Capture Error (File:" << capinf.filename << ")" << endl;
	}
}

void StopCapture(Tof* ptof)
{
	Result rtn = Result::OK;

	rtn = ptof->Capture(false);
	if (rtn != Result::OK){
		std::cout << "Capture Stop Error (" << (int)rtn << ")" << endl;
	}

	bCapture = false;
}

void CheckCaptureStatus(Tof* ptof)
{
	CaptureStatus capstatus;
	if (ptof->GetCaptureStatus(&capstatus) != Result::OK){
		std::cout << "Check Capture Status Error" << endl;
	}

	if (capstatus != CaptureStatus::Run){
		//Not capturing
		bCapture = false;
	}
}

void StartSimulation(Tof** ptof)
{
	Result rtn = Result::OK;
	CaptureInfo capinf;
	capinf.path = "";
	capinf.filename = capturefile;
	Tof* etof = new Tof;

	//Start eTOF(Emulated TOF) in Emulation Mode
	rtn = etof->Open(capinf);
	if (rtn != Result::OK){
		std::cout << "eTOF Open Error (" << (int)rtn << ")" << endl;
		return;
	}

	rtn = etof->Run(RunMode::FrameEmulation);
	if (rtn != Result::OK){
		std::cout << "eTOF Run Error (" << (int)rtn << ")" << endl;
		return;
	}

	//Change TOF to eTOF
	*ptof = etof;

	//Capture start time
	capturetime = clock();

	bSimulation = true;
}

void StopSimulation(Tof** ptof, Tof* rtof)
{
	Result rtn = Result::OK;

	bSimulation = false;

	Tof* etof = *ptof;

	rtn = etof->Stop();
	if (rtn != Result::OK){
		std::cout << "eTOF Stop Error (" << (int)rtn << ")" << endl;
		return;
	}

	rtn = etof->Close();
	if (rtn != Result::OK){
		std::cout << "eTOF Close Error (" << (int)rtn << ")" << endl;
		return;
	}

	//Delete eTOF
	delete etof;

	if (bNoSensor){
		//In case of no sensor mode

		if (bRepeat){
			//Repleat playback mode

			StartSimulation(ptof);
		}
		else {
			brun = false;
		}
	}
	else{
		//Change eTOF to TOF
		*ptof = rtof;
	}
}

void CheckSimulationStatus(long frameno, Tof** ptof, Tof* rtof)
{
	if (frameno == -2){
		//End of capture file

		//Stop emulation
		StopSimulation(ptof, rtof);
	}
}

void ChangeCameraMode(void)
{
	cammode++;
	if (cammode > (int)CameraMode::Background_Ir){
		cammode = (int)CameraMode::Depth_Motion;
	}

	switch (cammode){
	case (int)CameraMode::Depth_Motion:
		//Deoth and Motion
		bDepthIr = false;
		display1 = "Depth";
		display2 = "Motion";
		break;
	case (int)CameraMode::Depth_Background:
		//Depth and Background
		bDepthIr = false;
		display1 = "Depth";
		display2 = "Background";
		break;
	case (int)CameraMode::Depth_Ir:
		//Depth and IR
		bDepthIr = true;
		display1 = "Depth";
		display2 = "IR";
		break;
	case (int)CameraMode::Motion_Background:
		//Motion and Background
		bDepthIr = false;
		display1 = "Motion";
		display2 = "Background";
		break;
	case (int)CameraMode::Motion_Ir:
		//Motion and IR
		bDepthIr = true;
		display1 = "Motion";
		display2 = "IR";
		break;
	case (int)CameraMode::Background_Ir:
		//Background and IR
		bDepthIr = true;
		display1 = "Background";
		display2 = "IR";
		break;
	}

	if (tof.Stop() != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Stop Error" << endl;
		system("pause");
		return;
	}

	if (tof.SetCameraMode((CameraMode)cammode) != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Mode Error" << endl;
		system("pause");
		return;
	}

	if (tof.Run() != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Run Error" << endl;
		system("pause");
		return;
	}
}

bool SaveFile(void){

	//Make file name with the current time
	char buff[16];
	time_t now = time(NULL);
	struct tm *pnow = localtime(&now);
	sprintf(buff, "%04d%02d%02d%02d%02d%02d",
		pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday,
		pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
	savefile = buff;
	savefile += ".png";

	//Save
	return cv::imwrite(savefile, img);
}


void main(void)
{
	//Create TofManager
	TofManager tofm;

	//Open TOF Manager (Read tof.ini file)
	if (tofm.Open() != Result::OK){
		std::cout << "TofManager Open Error (may not be tof.ini file)" << endl;
		system("pause");
		return;
	}

	//Get number of TOF sensor and TOF information list
	const TofInfo * ptofinfo = nullptr;
	int numoftof = tofm.GetTofList(&ptofinfo);

	if (numoftof == 0){
		std::cout << "No TOF Sensor" << endl;
		bNoSensor = true;
	}

	//Make TOF instance as a pointer to switch real TOF and eTOF dynamically
	Tof *ptof = &tof;

	if (!bNoSensor){
		//Real sensor mode


		//Open #0 Tof instances (Set TOF information)
		if (tof.Open(ptofinfo[0]) != Result::OK){
			std::cout << "TOF ID " << ptofinfo[0].tofid << " Open Error" << endl;
			system("pause");
			return;
		}

		//Once Tof instances are started, TofManager is not necessary and closed
		if (tofm.Close() != Result::OK){
			std::cout << "TofManager Close Error" << endl;
			system("pause");
			return;
		}

		//Check TOF version
		if ((tof.tofinfo.tofver == TofVersion::TOFv1) ||
			(tof.tofinfo.tofver == TofVersion::Unknown)){
			//Not TOFv2
			std::cout << "This application is only for TOFv2 sensor" << endl;
			system("pause");
			return;
		}

		//Set camera mode(Depth + IR)
		if (tof.SetCameraMode((CameraMode)cammode) != Result::OK){
			std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Mode Error" << endl;
			system("pause");
			return;
		}

		//Set camera resolution
		if (tof.SetCameraPixel(CameraPixel::w320h240) != Result::OK){
			//	if (tof.SetCameraPixel(CameraPixel::w160h120) != Result::OK){
			std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Pixel Error" << endl;
			system("pause");
			return;
		}

		//Noise reduction(Low Signal Cutoff)
#ifdef LOW_SIGNAL_CUTOFF
		int lowsignalcutoff = 20;
#else
		int lowsignalcutoff = 0;
#endif
		if (tof.SetLowSignalCutoff(lowsignalcutoff) != Result::OK){
			std::cout << "TOF ID " << tof.tofinfo.tofid << " Low Signal Cutoff Error" << endl;
			system("pause");
			return;
		}

		//Background subtraction setting(Interval)
		if (tof.SetBackgroundInterval((BgInterval)bginterval) != Result::OK){
			std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Backgound Interval Error" << endl;
			system("pause");
			return;
		}

		//Background subtraction setting(Quantity)
		if (tof.SetBackgroundQuantity((BgQuantity)bgquantity) != Result::OK){
			std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Backgound Quantity Error" << endl;
			system("pause");
			return;
		}

		//Edge noise reduction
		if (tof.SetEdgeSignalCutoff(EdgeSignalCutoff::Enable) != Result::OK){
			std::cout << "TOF ID " << tof.tofinfo.tofid << " Edge Noise Reduction Error" << endl;
			system("pause");
			return;
		}

		//Start(Start data transferring)
		if (tof.Run() != Result::OK){
			std::cout << "TOF ID " << tof.tofinfo.tofid << " Run Error" << endl;
			system("pause");
			return;
		}
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Run OK" << endl;
	}
	else {
		//Only emulation mode
		//Start
		StartSimulation(&ptof);

		if (!bSimulation){
			//Cannot start emulation mode
			system("pause");
			return;
		}
	}

	//Create color tables
	frame1.CreateColorTable(0, 0xfeff);
	frame2.CreateColorTable(0, 0xfeff);

	//Create windows for display as resizable
	cv::namedWindow("Tofv2Viewer", CV_WINDOW_NORMAL);

	//Sub Display
	cv::Mat subdisplay(SUB_DISPLAY_HEIGHT, SUB_DISPLAY_WIDTH, CV_8UC3);

	//To caluculate FPS
	clock_t frametime = clock();

	//Valid frame number(Only multiple number of this number is used)
#ifdef FRAME_RATE_10FPS
	int frameperiod = 3;
#else
	int frameperiod = 1;
#endif

	while (brun){

		//Get the latest frame number
		long frameno;
		TimeStamp timestamp;
		ptof->GetFrameStatus(&frameno, &timestamp);

		if (bCapture){
			//Capturing
			CheckCaptureStatus(ptof);
		}

		if (bSimulation){
			//Emulating
			CheckSimulationStatus(frameno, &ptof, &tof);
		}

		long framediff;
		if (frameno >= frame1.framenumber){
			framediff = frameno - frame1.framenumber;
		}
		else {
			framediff = frameno + 0x7fffffff - frame1.framenumber;
		}

		if (framediff >= frameperiod){
			//Read a new frame only if frame number is changed(Old data is shown if it is not changed.)

			Result ret = Result::OK;
			if (bDepthIr){
				//Read Depth/Motion/Background and IR simultaneously
				ret = ptof->ReadFrame(&frame1, &frameir);
			}
			else {
				//Read Depth/Motion/Background and Depth/Motion/Background simultaneously
				ret = ptof->ReadFrame(&frame1, &frame2);
			}
			if (ret != Result::OK) {
				std::cout << "readframe error" << endl;
				break;
			}

			//Initialize screen
			img = cv::Mat::zeros(MAIN_DISPLAY_HEIGHT, MAIN_DISPLAY_WIDTH, CV_8UC3);

			if (bSubDisplay){
				//Initialize Sub Display
				subdisplay = cv::Mat::zeros(SUB_DISPLAY_HEIGHT, SUB_DISPLAY_WIDTH, CV_8UC3);
			}

			for (int y = 0; y < MAIN_DISPLAY_HEIGHT; y++){
				for (int x = 0; x < MAIN_DISPLAY_WIDTH; x++){

					long datano = (y * frame1.height / MAIN_DISPLAY_HEIGHT) * frame1.width + x * frame1.width / MAIN_DISPLAY_WIDTH;

					cv::Vec3b v_main;
					cv::Vec3b v_sub;

					cv::Vec3b v_depth1;
					v_depth1.val[0] = frame1.ColorTable[0][frame1.databuf[datano]];
					v_depth1.val[1] = frame1.ColorTable[1][frame1.databuf[datano]];
					v_depth1.val[2] = frame1.ColorTable[2][frame1.databuf[datano]];

					if (bDepthIr){
						cv::Vec3b v_ir;
						v_ir.val[0] = frameir.databuf[datano] / 256;
						v_ir.val[1] = v_ir.val[0];
						v_ir.val[2] = v_ir.val[0];

						if (!bReverseMainSub){
							v_main = v_depth1;
							v_sub = v_ir;
						}
						else {
							v_main = v_ir;
							v_sub = v_depth1;
						}

					}
					else {
						cv::Vec3b v_depth2;
						v_depth2.val[0] = frame2.ColorTable[0][frame2.databuf[datano]];
						v_depth2.val[1] = frame2.ColorTable[1][frame2.databuf[datano]];
						v_depth2.val[2] = frame2.ColorTable[2][frame2.databuf[datano]];

						if (!bReverseMainSub){
							v_main = v_depth1;
							v_sub = v_depth2;
						}
						else {
							v_main = v_depth2;
							v_sub = v_depth1;
						}
					}

					img.at<cv::Vec3b>(y, x) = v_main;

					if (bSubDisplay){
						//Sub Display
						subdisplay.at<cv::Vec3b>(y * SUB_DISPLAY_HEIGHT / MAIN_DISPLAY_HEIGHT,
							x * SUB_DISPLAY_WIDTH / MAIN_DISPLAY_WIDTH) = v_sub;
					}
				}
			}

			if (bSubDisplay){
				//Sub Display
				cv::Mat roi = img(cv::Rect(SubDisplayX[SubDisplayPos], SubDisplayY[SubDisplayPos], SUB_DISPLAY_WIDTH, SUB_DISPLAY_HEIGHT));
				cv::resize(subdisplay, roi, roi.size(), cv::INTER_LINEAR);
			}

			//Information display
			int tx = 10;
			int ty = 80;
			int tdy = 40;

			string maindisplay = display1;
			string subdisplay = display2;
			if (bReverseMainSub){
				maindisplay = display2;
				subdisplay = display1;
			}
			cv::putText(img, maindisplay, cv::Point(1060, 30), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
			if (bSubDisplay){
				cv::putText(img, subdisplay, cv::Point(SubDisplayX[SubDisplayPos] + 20, SubDisplayY[SubDisplayPos] + 30), cv::FONT_HERSHEY_TRIPLEX, 0.8, color, 2, CV_AA);
			}

			string text;

			if ((bCapture || bSimulation) &&
				((((int)(clock() - capturetime) * 2 / CLOCKS_PER_SEC) % 2) == 0)){
				//Blink every 0.5sec.
				int capsec = (clock() - capturetime) / CLOCKS_PER_SEC;
				string time_txt = std::to_string(capsec / 60) + ":";
				if (capsec % 60 < 10){
					time_txt += "0";
				}
				time_txt += std::to_string(capsec % 60);
				if (bCapture){
					text = "Capture " + time_txt;
					cv::putText(img, text, cv::Point(900, 80), cv::FONT_HERSHEY_TRIPLEX, 1.2, red, 2, CV_AA);
				}
				if (bSimulation){
					text = "Replay " + time_txt;
					cv::putText(img, text, cv::Point(900, 80), cv::FONT_HERSHEY_TRIPLEX, 1.2, blue, 2, CV_AA);
				}
			}

			text = "q key for Quit, m key for Menu";
			cv::putText(img, text, cv::Point(tx, 30), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
			switch (mode){
			case 'p':
				text = "Key 1: Sub Display ";
				if (bSubDisplay){
					text += "ON";
				}
				else {
					text += "OFF";
				}
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key 2: Change Sub Display Position";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key 3: Switch Main/Sub Display";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key 4: Change Camera Mode";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				break;
			case 'b':
				text = "Key Up/Down : Update Interval : " + bgintervalstr[bginterval];
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key Left/Right : Update Quantity : " + bgquantitystr[bgquantity];
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key r: Initialize (Reset) Background";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				break;
			case 'f':
				if (savefile != ""){
					text = "Saved to " + savefile;
				}
				else {
					text = "Save Failed !";
				}
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				break;
			case 'q':
				text = "Quit ? y: yes";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				break;
			case 'm':
				text = "Key q: Quit";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key p: Display";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key b: Background Subtraction";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key o: Capture (Max 10 minutes)";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key i: Replay";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key t: Change Text Color";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key f: Save Screen";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key m: Menu";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;

				break;
			}

			//Show FPS
			clock_t lastframetime = frametime;
			frametime = clock();
			//0除算チェック
			if (frametime > lastframetime)
			{
				int fps = CLOCKS_PER_SEC / (frametime - lastframetime);
				text = to_string(fps) + "fps";
				cv::putText(img, text, cv::Point(1150, 950), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
			}

			//Show timestamp
			text = to_string(frame1.timestamp.year) + "/" + to_string(frame1.timestamp.month) + "/" + to_string(frame1.timestamp.day) + " "
				+ to_string(frame1.timestamp.hour) + ":" + to_string(frame1.timestamp.minute) + ":" + to_string(frame1.timestamp.second) + "." + to_string(frame1.timestamp.msecond);
			cv::putText(img, text, cv::Point(650, 950), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
			if (NULL == cvGetWindowHandle("Tofv2Viewer")){
				brun = false;
			}
			else{
				cv::imshow("Tofv2Viewer", img);
			}
		}

		auto key = cv::waitKey(10);
		switch (key){
		case 'p':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 'b':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 'm':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 'o':

			if (!bCapture){
				//Start capture
				StartCapture(ptof);
			}
			else {
				//Stop capture
				StopCapture(ptof);
			}
			break;
		case 'i':

			if (!bSimulation){

				if (bCapture){
					StopCapture(ptof);
				}
				//Start replay
				StartSimulation(&ptof);
			}
			else {
				//Stop replay
				StopSimulation(&ptof, &tof);
			}
			break;
		case 'r':
			switch (mode){
			case 'b':
				if (tof.ResetBackground() != Result::OK){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Reset Backgound Error" << endl;
					system("pause");
					return;
				}
				break;
			}
			break;
		case 't':
			if (color == white){
				color = blue;
			}
			else {
				color = white;
			}
			break;
		case 'f':
			if (mode == key){
				mode = 0;
			}
			else {
				//Save screen
				if (!SaveFile()){
					//Save failed
					savefile = "";
				}
				mode = key;
			}
			break;
		case '1':
			if (mode == 'p'){
				bSubDisplay = !bSubDisplay;
			}
			break;
		case '2':
			if (mode == 'p'){
				SubDisplayPos++;
				if (SubDisplayPos >= SUB_DISPLAY_POS_MAX){
					SubDisplayPos = 0;
				}
			}
			break;
		case '3':
			if (mode == 'p'){
				bReverseMainSub = !bReverseMainSub;
			}
			break;
		case '4':
			if (mode == 'p'){
				ChangeCameraMode();
			}
			break;
		case 2490368:	//↑
			switch (mode){
			case 'b':
				bginterval++;
				if (bginterval > (int)BgInterval::bg300min){
					bginterval = (int)BgInterval::bg1min;
				}
				//Background subtraction setting(Interval)
				if (tof.SetBackgroundInterval((BgInterval)bginterval) != Result::OK){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Backgound Interval Error" << endl;
					system("pause");
					return;
				}
				break;
			}
			break;
		case 2621440:	//↓
			switch (mode){
			case 'b':
				bginterval--;
				if (bginterval < (int)BgInterval::bg1min){
					bginterval = (int)BgInterval::bg300min;
				}
				//Background subtraction setting(Interval)
				if (tof.SetBackgroundInterval((BgInterval)bginterval) != Result::OK){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Backgound Interval Error" << endl;
					system("pause");
					return;
				}
				break;
			}
			break;
		case 2555904:	//→
			switch (mode){
			case 'b':
				bgquantity++;
				if (bgquantity > (int)BgQuantity::Full){
					bgquantity = (int)BgQuantity::None;
				}
				//Background subtraction setting(Quantity)
				if (tof.SetBackgroundQuantity((BgQuantity)bgquantity) != Result::OK){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Backgound Quantity Error" << endl;
					system("pause");
					return;
				}
				break;
			}
			break;
		case 2424832:	//←
			switch (mode){
			case 'b':
				bgquantity--;
				if (bgquantity < (int)BgQuantity::None){
					bgquantity = (int)BgQuantity::Full;
				}
				//Background subtraction setting(Quantity)
				if (tof.SetBackgroundQuantity((BgQuantity)bgquantity) != Result::OK){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Backgound Quantity Error" << endl;
					system("pause");
					return;
				}
				break;
			}
			break;
		case  'q':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case  'y':
			if (mode == 'q'){
				brun = false;
			}
			mode = 0;
			break;
		default:
			break;
		}
	}

	//Stop capture
	if (bCapture){
		StopCapture(ptof);
	}

	//Stop emulation
	if (bSimulation){
		bRepeat = false;
		StopSimulation(&ptof, &tof);
	}

	if (!bNoSensor){
		//real sensor mode

		//Stop and close TOF sensor
		bool berror = false;
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
}
