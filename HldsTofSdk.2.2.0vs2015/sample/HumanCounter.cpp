/**
* @file			HumanCounter.cpp
* @brief		Sample program for Hitachi-LG Data Storage (HLDS)'s TOF Motion Sensor
* @author		Hitachi-LG Data Storage, Inc. (HLDS)
* @date			2018.03.07
* @version		v2.2.0
* @copyright	Hitachi-LG Data Storage,Inc.
*
* @par Change History:
* - 2016.06.30 New
* - 2017.04.28 v1.1.0
*					- Add hand detection function
*					- Add edge noise reduction
*					- Support playing capture data
* - 2018.03.07 v2.0.0
*					- Support vertical position of sensor (HLS-LFOM5 is vertical in its default)
*					- Change rotation order from X-Y-Z to Z-Y-X
*					- Transpose sub display in case of vertical position
* - 2018.02.13 v2.1.0
*					- Add close window function
* - 2018.03.07 v2.2.0
*					- Add Enable Area
*					- Avoid negative value for angles
*					- Change LowSignalCutoff value from 20 to 10
*					- Add Side/Front View for calibration
*/

#define _CRT_SECURE_NO_WARNINGS

#include <Windows.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <time.h>

#include "tof.h"

using namespace std;
using namespace hlds;

#define VERSION "Ver.2.2.0"

//Compile option
#define HUMAN_COLOR		//Different color for each human
//#define NO_FOOTPRINT	//Footprint is not displayed
//#define NO_HUMAN_CURSOR	//Human cursor is not displayed

#define M_PI (3.14159265358979) //Pi
#define deg2rad(d) ( (d) / 180.0 * M_PI )	//Macro function to convert deg to rad

#define COUNT_NO	(-1)	//Not counted
#define COUNT_UP	(0)		//Move to up direction
#define COUNT_RIGHT	(1)		//Move to right direction
#define COUNT_DOWN	(2)		//Move to down direction
#define COUNT_LEFT	(3)		//Move to left direction

//Parameter to display human
#define HUMAN_CURSOR_SIZE		(600.0f)		//Size of human cursor(Length of one side from L to L)
#define MAX_TRACKS				(100)			//Max points in a track

//Parameter to display hand height
#define HAND_INDICATOR_MIN		(0)				//Min height of indicator
#define HAND_INDICATOR_MAX		(2000)			//Max height of indicator

//sub display
#define SUB_DISPLAY_X			(10)			//X-coordinate of upper left of sub display
#define SUB_DISPLAY_Y			(710)			//Y-coordinate of upper left of sub display
#define SUB_DISPLAY_WIDTH		(320)			//Width of sub display
#define SUB_DISPLAY_HEIGHT		(240)			//Height of sub display

#define ANGLE_ADJUSTMENT_DEGREE		(1)			//Unit of angle adjustment(degree)

//Section Display
#define SIDE_VIEW_X				(850)			//X-coordinate of side view
#define SIDE_VIEW_Y				(100)			//Y-coordinate of side view
#define SIDE_VIEW_WIDTH			(400)			//Width of side view
#define SIDE_VIEW_HEIGHT		(300)			//Height of side view
#define FRONT_VIEW_X			(850)			//X-coordinate of front view
#define FRONT_VIEW_Y			(500)			//Y-coordinate of front view
#define FRONT_VIEW_WIDTH		(400)			//Width of front view
#define FRONT_VIEW_HEIGHT		(300)			//Height of front view
#define SIDE_VIEW_RANGE			(5000)			//Range (distance) of side view [mm]
#define FRONT_VIEW_RANGE		(6000)			//Range (width) of front view [mm]
#define SECTION_HEIGHT_MIN		(-500)			//Min height of side/front view [mm]
#define SECTION_HEIGHT_MAX		(2000)			//Max height of side/front view [mm]

//Human count
struct {
	int Enter[4];			//Human count who enter to the area from each direction(Use COUNT_XXX macro)
	int Exit[4];			//Human count who exit from the area to each direction(Use COUNT_XXX macro)
	int TotalEnter;			//Total number of humans entering
	int TotalExit;			//Total number of humans exiting
	int InArea;				//Total number of humans in count area

	//Count area(Counted when a human exits from the area)
	struct {
		float left_x;
		float top_y;
		float right_x;
		float bottom_y;
	} Square;
} Count;

//Enable Area
struct {
	float left_x;
	float top_y;
	float right_x;
	float bottom_y;
} EnableArea;

//Input key mode
TCHAR mode;

//Display image
cv::Mat img(480 * 2, 640 * 2, CV_8UC3);

//Background image
cv::Mat back(480 * 2, 640 * 2, CV_8UC3);

//Z-buffer(to understand before and behind)
float z_buffer[640 * 2][480 * 2];

//ini file
LPCTSTR inifilename = L"./HumanCounter.ini";
LPCTSTR inisection = L"Settings";

//Save file name
string savefile;	//Image save file

//Initial settings
float angle_x = 90.0f;				//Angle to rotate around X-axis(degree)
float angle_y = 0.0f;				//Angle to rotate around Y-axis(degree)
float angle_z = 0.0f;				//Angle to rotate around Z-axis(degree)
float height = 1000.0f;				//Height from floor(mm)
float dx = 600.0f;					//Shift length to x-axis positive direction(mm)
float dy = 900.0f;					//Shift length to y-axis positive direction(mm)
float zoom = 0.12f;					//Zoom ratio
bool bPoint = true;					//Mode to display points
bool bBack = false;					//Mode to display footprint on background
bool bCount = true;					//Mode to display human count
bool bBoxShift = true;				//true: shift, false: change size in count area setting mode
bool bSubDisplay = true;			//Mode to display sub display
bool bEnableArea = false;			//Valid/Invalid Enable Area
bool bEnableAreaShift = true;		//In Enable Area setting mode... true: Whole box shift, false: Box size change

//Human information
struct AppHuman {

	long id;					//Human ID managed in HumanDetect function of SDK
	int appid;					//Human ID managed in application(HumanCounter.cpp)
	bool bEnable;				//false : Candidate(Not recognized as a human yet)
	HumanStatus status;			//Status

	float x;					//X-coordinate of center of gravity of the human
	float y;					//Y-coordinate of center of gravity of the human
	float prex;					//Previous X-coordinate of center of gravity of the human
	float prey;					//Previous Y-coordinate of center of gravity of the human
	float direction;			//Direction of body(0 degree to 359 degree) --> Positive direction of Y-axis is 0 degree, and clockwise
	float headheight;			//Head height from floor(mm)
	float handheight;			//Hand height from floor(mm)(Enable when bHand is true)

	//Track data(Ring queue)
	struct {
		float x;				//X-cordinate of one point in the track
		float y;				//Y-cordinate of one point in the track
	} track[MAX_TRACKS];
	int nexttrack;				//Next inserting point in ring buffer
	int trackcnt;				//Number of points in the track

	int enterdir;				//Direction entering to the count area(COUNT_XXX macro)
	int exitdir;				//Direction exiting from the count area(COUNT_XXX macro)
};

//Array for humans in application
vector<AppHuman> apphumans;

//Human ID managed in application
int apphumanid = 0;

//Save ini file
bool SaveIniFile(void)
{
	BOOL ret;
	TCHAR strBuffer[1024];

	swprintf_s(strBuffer, TEXT("%f"), angle_x);
	ret = WritePrivateProfileString(inisection, L"ANGLE_X", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%f"), angle_y);
	ret = WritePrivateProfileString(inisection, L"ANGLE_Y", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%f"), angle_z);
	ret = WritePrivateProfileString(inisection, L"ANGLE_Z", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%f"), dx);
	ret = WritePrivateProfileString(inisection, L"SHIFT_X", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%f"), dy);
	ret = WritePrivateProfileString(inisection, L"SHIFT_Y", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%f"), height);
	ret = WritePrivateProfileString(inisection, L"HEIGHT", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%f"), zoom);
	ret = WritePrivateProfileString(inisection, L"ZOOM", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%f"), Count.Square.left_x);
	ret = WritePrivateProfileString(inisection, L"COUNT_LEFT_X", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%f"), Count.Square.top_y);
	ret = WritePrivateProfileString(inisection, L"COUNT_TOP_Y", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%f"), Count.Square.right_x);
	ret = WritePrivateProfileString(inisection, L"COUNT_RIGHT_X", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%f"), Count.Square.bottom_y);
	ret = WritePrivateProfileString(inisection, L"COUNT_BOTTOM_Y", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%d"), bEnableArea);
	ret = WritePrivateProfileString(inisection, L"ENABLE_AREA", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%f"), EnableArea.left_x);
	ret = WritePrivateProfileString(inisection, L"ENABLE_LEFT_X", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%f"), EnableArea.top_y);
	ret = WritePrivateProfileString(inisection, L"ENABLE_TOP_Y", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%f"), EnableArea.right_x);
	ret = WritePrivateProfileString(inisection, L"ENABLE_RIGHT_X", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	swprintf_s(strBuffer, TEXT("%f"), EnableArea.bottom_y);
	ret = WritePrivateProfileString(inisection, L"ENABLE_BOTTOM_Y", (LPCTSTR)strBuffer, inifilename);
	if (ret != TRUE){
		return false;
	}

	return true;
}

//Load ini file
bool LoadIniFile(void){

	DWORD ret;
	TCHAR strBuffer[1024];

	ret = GetPrivateProfileString(inisection, L"ANGLE_X", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		angle_x = stof(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"ANGLE_Y", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		angle_y = stof(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"ANGLE_Z", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		angle_z = stof(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"HEIGHT", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		height = stof(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"SHIFT_X", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		dx = stof(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"SHIFT_Y", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		dy = stof(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"ZOOM", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		zoom = stof(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"COUNT_LEFT_X", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		Count.Square.left_x = stof(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"COUNT_TOP_Y", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		Count.Square.top_y = stof(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"COUNT_RIGHT_X", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		Count.Square.right_x = stof(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"COUNT_BOTTOM_Y", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		Count.Square.bottom_y = stof(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"ENABLE_AREA", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		bEnableArea = stod(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"ENABLE_LEFT_X", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		EnableArea.left_x = stof(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"ENABLE_TOP_Y", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		EnableArea.top_y = stof(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"ENABLE_RIGHT_X", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		EnableArea.right_x = stof(strBuffer);
	}

	ret = GetPrivateProfileString(inisection, L"ENABLE_BOTTOM_Y", 0, strBuffer, 1024, inifilename);
	if (ret != 0){
		EnableArea.bottom_y = stof(strBuffer);
	}

	return true;
}

//Initialize humans information
void InitializeHumans(void)
{
	//Initialize humans in application
	apphumans.clear();

	//Human ID managed in application
	apphumanid = 0;
}

bool InCountArea(float x, float y)
{
	if ((x >= Count.Square.left_x) && (x <= Count.Square.right_x) &&
		(y >= Count.Square.top_y) && (y <= Count.Square.bottom_y)){
		return true;
	}
	return false;
}

bool InEnableArea(float x, float y)
{
	if ((x >= EnableArea.left_x) && (x <= EnableArea.right_x) &&
		(y >= EnableArea.top_y) && (y <= EnableArea.bottom_y)){
		return true;
	}
	return false;
}

int CountDirection(float x, float y)
{
	//Linear equation from upper left to lower right of count areaFy = a1 * x + b1
	float a1 = (Count.Square.bottom_y - Count.Square.top_y) / (Count.Square.right_x - Count.Square.left_x);
	float b1 = Count.Square.top_y - a1 * Count.Square.left_x;
	float y1 = a1 * x + b1;

	//Linear equation from lower left to upper right of count areaFy = a2 * x + b2
	float a2 = (Count.Square.bottom_y - Count.Square.top_y) / (Count.Square.left_x - Count.Square.right_x);
	float b2 = Count.Square.top_y - a2 * Count.Square.right_x;
	float y2 = a2 * x + b2;

	int dir = COUNT_UP;

	if ((y <= y1) && (y <= y2)){
		dir = COUNT_UP;
	}
	else if ((y >= y1) && (y >= y2)){
		dir = COUNT_DOWN;
	}
	else if ((y >= y1) && (y <= y2)){
		dir = COUNT_LEFT;
	}
	else if ((y <= y1) && (y >= y2)){
		dir = COUNT_RIGHT;
	}

	return dir;
}

void CountHumans(void)
{
	Count.InArea = 0;

	for (unsigned int ahno = 0; ahno < apphumans.size(); ahno++){

		if (InCountArea(apphumans[ahno].x, apphumans[ahno].y)){
			//In count area

			//countup
			Count.InArea++;

			if (!InCountArea(apphumans[ahno].prex, apphumans[ahno].prey)){
				//It was outside last time(Outside to inside)

				if (apphumans[ahno].enterdir != COUNT_NO){
					//Cancel previous entering count if already counted
					Count.Enter[apphumans[ahno].enterdir]--;
				}

				//Entering direction
				int dir = CountDirection(apphumans[ahno].prex, apphumans[ahno].prey);

				//countup
				Count.Enter[dir]++;

				//Register countup
				apphumans[ahno].enterdir = dir;

			}
		}
		else {
			//In outside of count area

			if (InCountArea(apphumans[ahno].prex, apphumans[ahno].prey)){
				//It was inside last time(Inside to outside)

				if (apphumans[ahno].exitdir != COUNT_NO){
					//Cancel previous exiting count if already counted
					Count.Exit[apphumans[ahno].exitdir]--;
				}

				//Exiting direction
				int dir = CountDirection(apphumans[ahno].x, apphumans[ahno].y);

				//countup
				Count.Exit[dir]++;

				//Register countup
				apphumans[ahno].exitdir = dir;

			}
		}
	}

	//Total number of human
	Count.TotalEnter = 0;
	Count.TotalExit = 0;
	for (int dir = 0; dir < 4; dir++){
		Count.TotalEnter += Count.Enter[dir];
		Count.TotalExit += Count.Exit[dir];
	}
}

void InitializeCount(void)
{
	memset(Count.Enter, 0, sizeof(Count.Enter));
	memset(Count.Exit, 0, sizeof(Count.Exit));
	Count.InArea = 0;
}

//Draw humans
void DrawHumans(void)
{
	//11 colors if different colors are assigned for each human
	cv::Scalar color[11] = { cv::Scalar(0, 0, 255),
		cv::Scalar(0, 255, 0),
		cv::Scalar(0, 255, 255),
		cv::Scalar(255, 0, 0),
		cv::Scalar(255, 0, 255),
		cv::Scalar(255, 255, 0),
		cv::Scalar(0, 127, 255),
		cv::Scalar(0, 255, 127),
		cv::Scalar(255, 0, 127),
		cv::Scalar(255, 127, 0),
		cv::Scalar(127, 0, 255),
	};

	//Draw each human
	for (unsigned int ahno = 0; ahno < apphumans.size(); ahno++){

		//Colors
		cv::Scalar backcolor = cv::Scalar(255, 255, 255);		//Footprint on background : White
		cv::Scalar footcolor = cv::Scalar(255, 255, 0);			//Tracking line : Light blue
		cv::Scalar color_el = cv::Scalar(0, 255, 0);			//L of human cursor : Yellow-green
		cv::Scalar color_plus = cv::Scalar(0, 255, 255);		//Circle of human cursor : Yellow

#ifdef HUMAN_COLOR
		//Change color depending on ID
		cv::Scalar idcolor = color[apphumans[ahno].id % 11];	//Different color for each ID
		backcolor = idcolor;
		footcolor = idcolor;
		color_el = idcolor;
#endif //HUMAN_COLOR

#ifndef NO_FOOTPRINT

		//Draw footprint on background
		cv::line(back, cv::Point((int)(apphumans[ahno].prex * zoom + dx), (int)(apphumans[ahno].prey * zoom + dy)),
			cv::Point((int)(apphumans[ahno].x * zoom + dx), (int)(apphumans[ahno].y * zoom + dy)), backcolor, 1, CV_AA, 0);

		//Draw tracking line
		int tno = 0;
		if (apphumans[ahno].trackcnt == MAX_TRACKS){
			tno = apphumans[ahno].nexttrack;
		}
		int prex;
		int prey;
		for (int tcnt = 0; tcnt < apphumans[ahno].trackcnt; tcnt++){
			int x = (int)(apphumans[ahno].track[tno].x * zoom + dx);
			int y = (int)(apphumans[ahno].track[tno].y * zoom + dy);
			if (tcnt > 0){
				cv::line(img, cv::Point(prex, prey), cv::Point(x, y), footcolor, 2, CV_AA, 0);
			}
			prex = x;
			prey = y;

			tno++;
			if (tno == MAX_TRACKS){
				tno = 0;
			}
		}
#endif	//NO_FOOTPRINT

#ifndef NO_HUMAN_CURSOR

		//Draw human cursor
		int hx = (int)(apphumans[ahno].x * zoom + dx);
		int hy = (int)(apphumans[ahno].y * zoom + dy);
		cv::Point point1;
		cv::Point point2;

		if ((apphumans[ahno].status == HumanStatus::Crouch) || (apphumans[ahno].status == HumanStatus::CrouchHand)){
			//Crouching
			color_plus = cv::Scalar(0, 128, 255);	//Orange
		}

		//L of upper left
		point1.x = hx - (int)(HUMAN_CURSOR_SIZE * zoom / 2);
		point1.y = hy - (int)(HUMAN_CURSOR_SIZE * zoom / 2);
		point2.x = point1.x + (int)(HUMAN_CURSOR_SIZE * zoom / 3);
		point2.y = point1.y;
		cv::line(img, point1, point2, color_el, 2, CV_AA, 0);

		point2.x = point1.x;
		point2.y = point1.y + (int)(HUMAN_CURSOR_SIZE * zoom / 3);
		cv::line(img, point1, point2, color_el, 2, CV_AA, 0);

		//L of upper right
		point1.x = hx + (int)(HUMAN_CURSOR_SIZE * zoom / 2);
		point1.y = hy - (int)(HUMAN_CURSOR_SIZE * zoom / 2);
		point2.x = point1.x - (int)(HUMAN_CURSOR_SIZE * zoom / 3);
		point2.y = point1.y;
		cv::line(img, point1, point2, color_el, 2, CV_AA, 0);

		point2.x = point1.x;
		point2.y = point1.y + (int)(HUMAN_CURSOR_SIZE * zoom / 3);
		cv::line(img, point1, point2, color_el, 2, CV_AA, 0);

		//L of lower right
		point1.x = hx + (int)(HUMAN_CURSOR_SIZE * zoom / 2);
		point1.y = hy + (int)(HUMAN_CURSOR_SIZE * zoom / 2);
		point2.x = point1.x - (int)(HUMAN_CURSOR_SIZE * zoom / 3);
		point2.y = point1.y;
		cv::line(img, point1, point2, color_el, 2, CV_AA, 0);

		point2.x = point1.x;
		point2.y = point1.y - (int)(HUMAN_CURSOR_SIZE * zoom / 3);
		cv::line(img, point1, point2, color_el, 2, CV_AA, 0);

		//L of lower left
		point1.x = hx - (int)(HUMAN_CURSOR_SIZE * zoom / 2);
		point1.y = hy + (int)(HUMAN_CURSOR_SIZE * zoom / 2);
		point2.x = point1.x + (int)(HUMAN_CURSOR_SIZE * zoom / 3);
		point2.y = point1.y;
		cv::line(img, point1, point2, color_el, 2, CV_AA, 0);

		point2.x = point1.x;
		point2.y = point1.y - (int)(HUMAN_CURSOR_SIZE * zoom / 3);
		cv::line(img, point1, point2, color_el, 2, CV_AA, 0);

		point1.x = hx - (int)(2 * HUMAN_CURSOR_SIZE * zoom / 6 * sin(deg2rad(apphumans[ahno].direction)));
		point1.y = hy + (int)(2 * HUMAN_CURSOR_SIZE * zoom / 6 * cos(deg2rad(apphumans[ahno].direction)));
		point2.x = hx - (int)(HUMAN_CURSOR_SIZE * zoom / 6 * sin(deg2rad(apphumans[ahno].direction)));
		point2.y = hy + (int)(HUMAN_CURSOR_SIZE * zoom / 6 * cos(deg2rad(apphumans[ahno].direction)));
		cv::line(img, point1, point2, color_plus, 2, CV_AA, 0);

		//Display hand
		if ((apphumans[ahno].status == HumanStatus::StandHand) || (apphumans[ahno].status == HumanStatus::CrouchHand)){
			point1.x = hx - (int)(3 * HUMAN_CURSOR_SIZE * zoom / 6 * sin(deg2rad(apphumans[ahno].direction)));
			point1.y = hy + (int)(3 * HUMAN_CURSOR_SIZE * zoom / 6 * cos(deg2rad(apphumans[ahno].direction)));
			point2.x = hx - (int)(HUMAN_CURSOR_SIZE * zoom / 6 * sin(deg2rad(apphumans[ahno].direction)));
			point2.y = hy + (int)(HUMAN_CURSOR_SIZE * zoom / 6 * cos(deg2rad(apphumans[ahno].direction)));
			cv::line(img, point1, point2, color_plus, 4, CV_AA, 0);

			//Indicator box
			if (apphumans[ahno].direction < 180){
				point1.x = hx - HUMAN_CURSOR_SIZE * zoom / 2 - HUMAN_CURSOR_SIZE * zoom / 6 - 10;
			}
			else {
				point1.x = hx + HUMAN_CURSOR_SIZE * zoom / 2 + 10;
			}
			point1.y = hy - HUMAN_CURSOR_SIZE * zoom / 2;
			point2.x = point1.x + HUMAN_CURSOR_SIZE * zoom / 6;
			point2.y = point1.y + HUMAN_CURSOR_SIZE * zoom;
			cv::rectangle(img, point1, point2, color_plus, 2, CV_AA, 0);

			//Indicator
			int handh = (int)apphumans[ahno].handheight;
			if (handh < HAND_INDICATOR_MIN){
				handh = HAND_INDICATOR_MIN;
			}
			else if (handh > HAND_INDICATOR_MAX){
				handh = HAND_INDICATOR_MAX;
			}
			point1.y = point2.y - HUMAN_CURSOR_SIZE * zoom * (handh - HAND_INDICATOR_MIN) / (HAND_INDICATOR_MAX - HAND_INDICATOR_MIN);
			cv::rectangle(img, point1, point2, color_plus, CV_FILLED, CV_AA, 0);
		}

		point1.x = hx;
		point1.y = hy;
		cv::circle(img, point1, (int)(HUMAN_CURSOR_SIZE * zoom / 6), color_plus, 2);

#endif //NO_HUMAN_CURSOR
	}
}

void DrawCount(void)
{
	//Display count area
	float x = Count.Square.left_x * zoom + dx;
	float y = Count.Square.top_y * zoom + dy;
	float lx = Count.Square.right_x * zoom + dx - x;
	float ly = Count.Square.bottom_y * zoom + dy - y;
	cv::rectangle(img, cv::Rect((int)x, (int)y, (int)lx, (int)ly), cv::Scalar(255, 255, 255), 2, CV_AA);

	string text;
	int tx1 = 850;
	int tx2 = 1070;
	int ty = 200;
	int tdy = 40;

	text = "IN AREA";
	cv::putText(img, text, cv::Point(tx1, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);

	text = ": " + std::to_string(Count.InArea);
	cv::putText(img, text, cv::Point(tx2, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
	ty += tdy;
	ty += tdy;

	text = "ENTER COUNTER";
	cv::putText(img, text, cv::Point(tx1, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
	ty += tdy;

	text = "UP";
	cv::putText(img, text, cv::Point(tx1, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);

	text = ": " + std::to_string(Count.Enter[COUNT_UP]);
	cv::putText(img, text, cv::Point(tx2, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
	ty += tdy;

	text = "DOWN";
	cv::putText(img, text, cv::Point(tx1, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);

	text = ": " + std::to_string(Count.Enter[COUNT_DOWN]);
	cv::putText(img, text, cv::Point(tx2, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
	ty += tdy;

	text = "LEFT";
	cv::putText(img, text, cv::Point(tx1, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);

	text = ": " + std::to_string(Count.Enter[COUNT_LEFT]);
	cv::putText(img, text, cv::Point(tx2, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
	ty += tdy;

	text = "RIGHT";
	cv::putText(img, text, cv::Point(tx1, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);

	text = ": " + std::to_string(Count.Enter[COUNT_RIGHT]);
	cv::putText(img, text, cv::Point(tx2, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
	ty += tdy;

	text = "TOTAL";
	cv::putText(img, text, cv::Point(tx1, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);

	text = ": " + std::to_string(Count.TotalEnter);
	cv::putText(img, text, cv::Point(tx2, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
	ty += tdy;
	ty += tdy;

	text = "EXIT COUNTER";
	cv::putText(img, text, cv::Point(tx1, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
	ty += tdy;

	text = "UP";
	cv::putText(img, text, cv::Point(tx1, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);

	text = ": " + std::to_string(Count.Exit[COUNT_UP]);
	cv::putText(img, text, cv::Point(tx2, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
	ty += tdy;

	text = "DOWN";
	cv::putText(img, text, cv::Point(tx1, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);

	text = ": " + std::to_string(Count.Exit[COUNT_DOWN]);
	cv::putText(img, text, cv::Point(tx2, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
	ty += tdy;

	text = "LEFT";
	cv::putText(img, text, cv::Point(tx1, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);

	text = ": " + std::to_string(Count.Exit[COUNT_LEFT]);
	cv::putText(img, text, cv::Point(tx2, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
	ty += tdy;

	text = "RIGHT";
	cv::putText(img, text, cv::Point(tx1, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);

	text = ": " + std::to_string(Count.Exit[COUNT_RIGHT]);
	cv::putText(img, text, cv::Point(tx2, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
	ty += tdy;

	text = "TOTAL";
	cv::putText(img, text, cv::Point(tx1, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);

	text = ": " + std::to_string(Count.TotalExit);
	cv::putText(img, text, cv::Point(tx2, ty), cv::FONT_HERSHEY_TRIPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
	ty += tdy;
	ty += tdy;
}

void DrawEnableArea(void)
{
	cv::Scalar color = cv::Scalar(0, 255, 255);

	if (!bEnableArea){
		color = cv::Scalar(128, 128, 128);
	}

	float x = EnableArea.left_x * zoom + dx;
	float y = EnableArea.top_y * zoom + dy;
	float lx = EnableArea.right_x * zoom + dx - x;
	float ly = EnableArea.bottom_y * zoom + dy - y;
	cv::rectangle(img, cv::Rect((int)x, (int)y, (int)lx, (int)ly), color, 2, CV_AA);
}

void DrawSection(Frame3d* pframe3d)
{
	cv::rectangle(img, cv::Point(SIDE_VIEW_X, SIDE_VIEW_Y),
		cv::Point(SIDE_VIEW_X + SIDE_VIEW_WIDTH, SIDE_VIEW_Y + SIDE_VIEW_HEIGHT), cv::Scalar(0, 0, 0), -1);

	cv::rectangle(img, cv::Point(FRONT_VIEW_X, FRONT_VIEW_Y),
		cv::Point(FRONT_VIEW_X + FRONT_VIEW_WIDTH, FRONT_VIEW_Y + FRONT_VIEW_HEIGHT), cv::Scalar(0, 0, 0), -1);

	cv::Vec3b v;
	v.val[0] = 255;
	v.val[1] = 255;
	v.val[2] = 255;

	for (int y = 0; y < pframe3d->height; y++){
		for (int x = 0; x < pframe3d->width; x++){

			TofPoint p = pframe3d->frame3d[y * pframe3d->width + x];
			int h = height - (int)p.z;
			if ((h >= SECTION_HEIGHT_MIN) && (h <= SECTION_HEIGHT_MAX)){
				//In range of height

				int px = (int)p.x;
				int py = (int)p.y;
				if (py < 0){
					py *= -1;
				}

				//Side view
				int sdx = (SIDE_VIEW_RANGE - py) * SIDE_VIEW_WIDTH / SIDE_VIEW_RANGE;
				int sdy = ((SECTION_HEIGHT_MAX - SECTION_HEIGHT_MIN) - (h - SECTION_HEIGHT_MIN)) * SIDE_VIEW_HEIGHT / (SECTION_HEIGHT_MAX - SECTION_HEIGHT_MIN);
				if ((sdx >= 0) && (sdx < SIDE_VIEW_WIDTH) &&
					(sdy >= 0) && (sdy < SIDE_VIEW_HEIGHT)){
					img.at<cv::Vec3b>(sdy + SIDE_VIEW_Y, sdx + SIDE_VIEW_X) = v;
				}

				//Front view
				int fdx = (FRONT_VIEW_RANGE / 2 + px) * FRONT_VIEW_WIDTH / FRONT_VIEW_RANGE;
				int fdy = ((SECTION_HEIGHT_MAX - SECTION_HEIGHT_MIN) - (h - SECTION_HEIGHT_MIN)) * FRONT_VIEW_HEIGHT / (SECTION_HEIGHT_MAX - SECTION_HEIGHT_MIN);
				if ((fdx >= 0) && (fdx < SIDE_VIEW_WIDTH) &&
					(fdy >= 0) && (fdy < SIDE_VIEW_HEIGHT)){
					img.at<cv::Vec3b>(fdy + FRONT_VIEW_Y, fdx + FRONT_VIEW_X) = v;
				}
			}
		}
	}

	cv::Point p0;
	cv::Point p1;

	//Ruled line
	for (int i = 1; i < 4; i++){
		p0.x = SIDE_VIEW_X;
		p0.y = SIDE_VIEW_HEIGHT * i / 4 + SIDE_VIEW_Y;
		p1.x = SIDE_VIEW_X + SIDE_VIEW_WIDTH;
		p1.y = p0.y;
		cv::line(img, p0, p1, cv::Scalar(128, 128, 128), 1, CV_AA, 0);

		p0.x = SIDE_VIEW_WIDTH * i / 4 + SIDE_VIEW_X;
		p0.y = SIDE_VIEW_Y;
		p1.x = p0.x;
		p1.y = SIDE_VIEW_Y + SIDE_VIEW_HEIGHT;
		cv::line(img, p0, p1, cv::Scalar(128, 128, 128), 1, CV_AA, 0);

		p0.x = FRONT_VIEW_X;
		p0.y = FRONT_VIEW_HEIGHT * i / 4 + FRONT_VIEW_Y;
		p1.x = FRONT_VIEW_X + FRONT_VIEW_WIDTH;
		p1.y = p0.y;
		cv::line(img, p0, p1, cv::Scalar(128, 128, 128), 1, CV_AA, 0);

		p0.x = FRONT_VIEW_WIDTH * i / 4 + FRONT_VIEW_X;
		p0.y = FRONT_VIEW_Y;
		p1.x = p0.x;
		p1.y = FRONT_VIEW_Y + FRONT_VIEW_HEIGHT;
		cv::line(img, p0, p1, cv::Scalar(128, 128, 128), 1, CV_AA, 0);
	}

	//Ground line
	p0.x = SIDE_VIEW_X;
	p0.y = SIDE_VIEW_HEIGHT - SIDE_VIEW_HEIGHT * (0 - SECTION_HEIGHT_MIN) / (SECTION_HEIGHT_MAX - SECTION_HEIGHT_MIN) + SIDE_VIEW_Y;
	p1.x = SIDE_VIEW_X + SIDE_VIEW_WIDTH;
	p1.y = p0.y;
	cv::line(img, p0, p1, cv::Scalar(0, 255, 255), 1, CV_AA, 0);

	cv::putText(img, "Height 0[mm]", cv::Point(SIDE_VIEW_X + 5, p1.y + 18),
		cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 255, 255), 1, CV_AA);

	p0.x = FRONT_VIEW_X;
	p0.y = FRONT_VIEW_HEIGHT - FRONT_VIEW_HEIGHT * (0 - SECTION_HEIGHT_MIN) / (SECTION_HEIGHT_MAX - SECTION_HEIGHT_MIN) + FRONT_VIEW_Y;
	p1.x = FRONT_VIEW_X + FRONT_VIEW_WIDTH;
	p1.y = p0.y;
	cv::line(img, p0, p1, cv::Scalar(0, 255, 255), 1, CV_AA, 0);

	cv::putText(img, "Height 0[mm]", cv::Point(FRONT_VIEW_X + 5, p1.y + 18),
		cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 255, 255), 1, CV_AA);

	cv::putText(img, "Side View", cv::Point(SIDE_VIEW_X + 5, SIDE_VIEW_Y + SIDE_VIEW_HEIGHT - 5),
		cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(255, 0, 0), 1, CV_AA);

	cv::putText(img, "Front View", cv::Point(FRONT_VIEW_X + 5, FRONT_VIEW_Y + FRONT_VIEW_HEIGHT - 5),
		cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(255, 0, 0), 1, CV_AA);

	cv::rectangle(img, cv::Point(SIDE_VIEW_X, SIDE_VIEW_Y),
		cv::Point(SIDE_VIEW_X + SIDE_VIEW_WIDTH, SIDE_VIEW_Y + SIDE_VIEW_HEIGHT), cv::Scalar(255, 0, 0), 2);

	cv::rectangle(img, cv::Point(FRONT_VIEW_X, FRONT_VIEW_Y),
		cv::Point(FRONT_VIEW_X + FRONT_VIEW_WIDTH, FRONT_VIEW_Y + FRONT_VIEW_HEIGHT), cv::Scalar(255, 0, 0), 2);
}

//Save screen when f key is pushed
bool SaveFile(void){

	//Make file name with current time
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

//Catch humans detected by Human Detect function in SDK
void CatchHumans(FrameHumans *pframehumans)
{
	//Reset relation between humans managed in application and humans detected by SDK
	for (unsigned int ahno = 0; ahno < apphumans.size(); ahno++){
		apphumans[ahno].bEnable = false;
	}

	//Assign humans managed in application and humans detected by SDK
	for (int hno = 0; hno < pframehumans->numofhuman; hno++){

		if (bEnableArea && !InEnableArea(pframehumans->humans[hno].x, pframehumans->humans[hno].y)){
			//Out of Enable Area
			continue;
		}

		bool bAssigned = false;
		for (unsigned int ahno = 0; ahno < apphumans.size(); ahno++){

			if (apphumans[ahno].id == pframehumans->humans[hno].id){
				//Found the current human --> assign

				apphumans[ahno].bEnable = true;
				apphumans[ahno].prex = apphumans[ahno].x;
				apphumans[ahno].prey = apphumans[ahno].y;
				apphumans[ahno].x = pframehumans->humans[hno].x;
				apphumans[ahno].y = pframehumans->humans[hno].y;
				apphumans[ahno].direction = pframehumans->humans[hno].direction;
				apphumans[ahno].headheight = pframehumans->humans[hno].headheight;
				apphumans[ahno].handheight = pframehumans->humans[hno].handheight;

				//Status
				HumanStatus status = pframehumans->humans[hno].status;

				//Update status
				apphumans[ahno].status = status;

				//Register to tracking data(ring queue)
				apphumans[ahno].track[apphumans[ahno].nexttrack].x = apphumans[ahno].prex;
				apphumans[ahno].track[apphumans[ahno].nexttrack].y = apphumans[ahno].prey;
				apphumans[ahno].nexttrack++;
				if (apphumans[ahno].nexttrack == MAX_TRACKS){
					apphumans[ahno].nexttrack = 0;
				}
				apphumans[ahno].trackcnt++;
				if (apphumans[ahno].trackcnt > MAX_TRACKS){
					apphumans[ahno].trackcnt = MAX_TRACKS;
				}

				bAssigned = true;
				break;
			}
		}

		if (!bAssigned){
			//No corresponded human managed in application (New human)

			//Make new human information managed in application
			AppHuman ah;
			memset(&ah, 0, sizeof(ah));
			ah.bEnable = true;
			ah.id = pframehumans->humans[hno].id;
			ah.status = HumanStatus::Walk;
			ah.x = pframehumans->humans[hno].x;
			ah.y = pframehumans->humans[hno].y;
			ah.prex = ah.x;
			ah.prey = ah.y;
			ah.direction = pframehumans->humans[hno].direction;
			ah.headheight = pframehumans->humans[hno].headheight;
			ah.handheight = pframehumans->humans[hno].handheight;
			ah.enterdir = COUNT_NO;
			ah.exitdir = COUNT_NO;
			apphumans.push_back(ah);
		}
	}

	//Delete human managed in application who was not assigned
	for (int ahno = apphumans.size() - 1; ahno >= 0; ahno--){
		if (apphumans[ahno].bEnable == false){
			//No assigend

			apphumans.erase(apphumans.begin() + ahno);
		}
	}
}

bool ChangeAttribute(Tof& tof, float x, float y, float z, float rx, float ry, float rz)
{
	if (rx < 0) rx += 360;
	if (ry < 0) ry += 360;
	if (rz < 0) rz += 360;

	if (tof.SetAttribute(x, y, z, rx, ry, rz) != Result::OK){
		return false;
	}
	return true;
}

void main(void)
{
	//Initialize human counter
	memset(&Count, 0, sizeof(Count));
	Count.Square.left_x = -500;
	Count.Square.top_y = -2000;
	Count.Square.right_x = 500;
	Count.Square.bottom_y = -1000;

	//Initialize Enable Area
	EnableArea.left_x = -700;
	EnableArea.top_y = -2200;
	EnableArea.right_x = 700;
	EnableArea.bottom_y = -800;

	bool bEtof = false;
	Result ret = Result::OK;

	//Load ini file
	LoadIniFile();

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
		bEtof = true;
	}

	//Create Tof instance for a TOF sensor
	Tof tof;

	//Open Tof instance (Set TOF information)
	if (bEtof == false){
		if (tof.Open(ptofinfo[0]) != Result::OK){
			std::cout << "TOF ID " << ptofinfo[0].tofid << " Open Error" << endl;
			system("pause");
			return;
		}
	}
	else{
		CaptureInfo eTof;
		eTof.path = "";
		eTof.filename = "TofCapture.bin";

		if (tof.Open(eTof) != Result::OK){
			std::cout << "eTOF Open Error" << endl;
			system("pause");
			return;
		}
	}

	//Once Tof instances are started, TofManager is not necessary and closed
	if (tofm.Close() != Result::OK){
		std::cout << "TofManager Close Error" << endl;
		system("pause");
		return;
	}

	//Set camera mode as Depth mode
	if (tof.SetCameraMode(CameraMode::CameraModeDepth) != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Mode Error" << endl;
		system("pause");
		return;
	}

	//Set camera pixel
	if (tof.SetCameraPixel(CameraPixel::w320h240) != Result::OK){
		//	if (tof.SetCameraPixel(CameraPixel::w160h120) != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Pixel Error" << endl;
		system("pause");
		return;
	}

	//Set TOF sensor angle and height
	if (tof.SetAttribute(0, 0, height * -1, angle_x, angle_y, angle_z) != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Position Error" << endl;
		system("pause");
		return;
	}

	//Noise reduction(Low signal cutoff)
	if (tof.SetLowSignalCutoff(10) != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Low Signal Cutoff Error" << endl;
		system("pause");
		return;
	}

	//Edge noise reduction
	if (tof.SetEdgeSignalCutoff(EdgeSignalCutoff::Enable) != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Edge Noise Reduction Error" << endl;
		system("pause");
		return;
	}

	//Start human detection
	ret = tof.Run(RunMode::HumanDetect);
	if (ret != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Run Error: " << (int)ret << endl;
		system("pause");
		return;
	}
	std::cout << "TOF ID " << tof.tofinfo.tofid << " Run OK" << endl;

	//Create instances for reading frames
	FrameDepth frame;

	//Create instances for 3D data after conversion
	Frame3d frame3d;

	//Create instances for reading human frames
	FrameHumans framehumans;

	//Create color table
	frame.CreateColorTable(0, 65530);

	//Create display window as changeable size
	cv::namedWindow("Human Counter", CV_WINDOW_NORMAL);

	//Sub display
	cv::Mat subdisplay(SUB_DISPLAY_WIDTH, SUB_DISPLAY_HEIGHT, CV_8UC3);

	//Initialize human information
	InitializeHumans();

	//Initialize background
	back = cv::Mat::zeros(480 * 2, 640 * 2, CV_8UC3);

	bool brun = true;
	while (brun){

		//Get the latest frame number
		long frameno;
		TimeStamp timestamp;
		tof.GetFrameStatus(&frameno, &timestamp);

		if (frameno != frame.framenumber){
			//Read a new frame only if frame number is changed(Old data is shown if it is not changed.)

			//Read a frame of humans data
			Result ret = Result::OK;
			ret = tof.ReadFrame(&framehumans);
			if (ret != Result::OK) {
				std::cout << "read frame error" << endl;
				break;
			}

			//Read a frame of depth data
			ret = Result::OK;
			ret = tof.ReadFrame(&frame);
			if (ret != Result::OK) {
				std::cout << "read frame error" << endl;
				break;
			}

			if (bEtof == true){
				if (frame.framenumber < 0){
					//Replay whole data
					std::cout << "replay finish" << endl;
					break;
				}
			}

			//3D conversion(with lens correction)
			frame3d.Convert(&frame);

			//3D rotation in Z,Y,X order(to top view)
			frame3d.RotateZYX(angle_x, angle_y, angle_z);

			//Initialize Z-buffer
			memset(z_buffer, 0, sizeof(z_buffer));

			if (bBack){
				img = back.clone();
			}
			else {
				img = cv::Mat::zeros(480 * 2, 640 * 2, CV_8UC3);
			}

			if (bSubDisplay){
				//Initialize sub display
				subdisplay = cv::Mat::zeros(SUB_DISPLAY_HEIGHT, SUB_DISPLAY_WIDTH, CV_8UC3);
			}

			for (int y = 0; y < frame3d.height; y++){
				for (int x = 0; x < frame3d.width; x++){

					if ((frame.CalculateLength(frame.databuf[y * frame.width + x]) >= framehumans.distance_min) &&
						(frame.CalculateLength(frame.databuf[y * frame.width + x]) <= framehumans.distance_max)){
						//Valid data(Only data in specific distance for sensor is valid)

						TofPoint point;		//Coordinate after rotation

						//Get coordinates after 3D conversion
						point.x = frame3d.frame3d[y * frame3d.width + x].x;
						point.y = frame3d.frame3d[y * frame3d.width + x].y;
						point.z = frame3d.frame3d[y * frame3d.width + x].z;

						//Zoom
						point.x *= zoom;
						point.y *= zoom;

						//Shift to X/Y direction on display
						point.x += dx;
						point.y += dy;

						if ((point.x >= 0) && (point.x < img.size().width) &&
							(point.y >= 0) && (point.y < img.size().height)){

							if ((point.z >= framehumans.z_min) && (point.z < framehumans.z_max)){
								//Within range of Z direction

								if ((z_buffer[(int)point.x][(int)point.y] == 0) ||
									(z_buffer[(int)point.x][(int)point.y] > point.z)){
									//Front than data already registered in Z-buffer

									//Register to Z-buffer
									z_buffer[(int)point.x][(int)point.y] = point.z;

									//Register color to display image based on distance of Z direction
									long color = (long)(65530 * (point.z - framehumans.z_min) / ((framehumans.z_max - framehumans.z_min)));

									cv::Vec3b v;
									v.val[0] = frame.ColorTable[0][color];
									v.val[1] = frame.ColorTable[1][color];
									v.val[2] = frame.ColorTable[2][color];

									if (bPoint){
										img.at<cv::Vec3b>((int)point.y, (int)point.x) = v;
									}
								}

								if (bSubDisplay){
									//Sub display
									cv::Vec3b v;
									v.val[0] = frame.ColorTable[0][frame.databuf[y * frame.width + x]];
									v.val[1] = frame.ColorTable[1][frame.databuf[y * frame.width + x]];
									v.val[2] = frame.ColorTable[2][frame.databuf[y * frame.width + x]];
									subdisplay.at<cv::Vec3b>(y * SUB_DISPLAY_HEIGHT / frame3d.height,
										x * SUB_DISPLAY_WIDTH / frame3d.width) = v;
								}
							}
						}
					}
					else {
						//Invalid point is (x,y,z) = (0,0,0)
						frame3d.frame3d[y * frame3d.width + x].x = 0;
						frame3d.frame3d[y * frame3d.width + x].y = 0;
						frame3d.frame3d[y * frame3d.width + x].z = 0;
					}
				}
			}

			//Catch detected humans
			CatchHumans(&framehumans);

			//Human count
			CountHumans();

			//Draw Enable Area
			if (mode == 'e'){
				DrawEnableArea();
			}

			//Draw humans
			DrawHumans();

			if ((mode == 'a') || (mode == 'h')){
				//Draw side/front view
				DrawSection(&frame3d);
			}
			else if (bCount){
				//Draw human counter
				DrawCount();
			}

			//Display information
			int tx = 10;
			int ty = 80;
			int tdy = 40;
			cv::Scalar white(255, 255, 255);
			cv::Scalar blue(255, 0, 0);
			cv::Scalar red(0, 0, 255);
			cv::Scalar gray(128, 128, 128);
			cv::Scalar color = white;
			cv::Scalar color2 = white;

			string text = VERSION;

			cv::putText(img, text, cv::Point(1000, 30), cv::FONT_HERSHEY_TRIPLEX, 1.0, blue, 2, CV_AA);

			text = "q key for Quit, m key for Menu";
			cv::putText(img, text, cv::Point(tx, 30), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
			switch (mode){
			case 'a':
				text = "Angle x=" + std::to_string((int)angle_x);
				if (180 < angle_x){
					text += "(" + std::to_string((int)angle_x - 360) + ")";
				}
				text += "[degree]";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Angle y=" + std::to_string((int)angle_y);
				if (180 < angle_y){
					text += "(" + std::to_string((int)angle_y - 360) + ")";
				}
				text += "[degree]";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Angle z=" + std::to_string((int)angle_z);
				if (180 < angle_z){
					text += "(" + std::to_string((int)angle_z - 360) + ")";
				}
				text += "[degree]";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				break;
			case 's':
				text = "Shift x=" + std::to_string((int)dx) + "[mm]";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Shift y=" + std::to_string((int)dy) + "[mm]";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				break;
			case 'z':
				text = "Zoom=" + std::to_string((int)(zoom * 100)) + "%";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				break;
			case 'h':
				text = "Height from Floor=" + std::to_string((int)height) + "[mm]";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				break;
			case 'b':
				text = "Box ("
					+ std::to_string((int)Count.Square.left_x) + "[mm],"
					+ std::to_string((int)Count.Square.top_y) + "[mm]) : ("
					+ std::to_string((int)Count.Square.right_x) + "[mm],"
					+ std::to_string((int)Count.Square.bottom_y) + "[mm])";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				if (bBoxShift){
					color = blue;
				}
				else {
					color2 = blue;
				}
				ty += tdy;
				text = "Position (Move Left/Top)";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Size (Move Right/Bottom)";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color2, 2, CV_AA);
				break;
			case 'e':
				if (!bEnableArea){
					color = color2 = gray;
				}
				text = "Enable Area ("
					+ std::to_string((int)EnableArea.left_x) + "[mm],"
					+ std::to_string((int)EnableArea.top_y) + "[mm]) : ("
					+ std::to_string((int)EnableArea.right_x) + "[mm],"
					+ std::to_string((int)EnableArea.bottom_y) + "[mm])";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				if (!bEnableAreaShift){
					color = gray;
				}
				else {
					color2 = gray;
				}
				ty += tdy;
				text = "Position (Move Left/Top)";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Size (Move Right/Bottom)";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color2, 2, CV_AA);
				break;
			case 'p':
				text = "Display Key 1: Points ";
				if (bPoint){
					text += "ON";
				}
				else {
					text += "OFF";
				}
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Display Key 2: Footprints ";
				if (bBack){
					text += "ON";
				}
				else {
					text += "OFF";
				}
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Display Key 3: Counter ";
				if (bCount){
					text += "ON";
				}
				else {
					text += "OFF";
				}
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Display Key 4: Sub Display ";
				if (bSubDisplay){
					text += "ON";
				}
				else {
					text += "OFF";
				}
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				if (bBack){
					text = "Display Key 9: Reset Footprints";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
				}
				if (bCount){
					text = "Display Key 0: Reset Counter";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				}
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
			case '0':
				text = "Reset Counter ? y: yes";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				break;
			case 'm':
				text = "Key q: Quit";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key p: Display Key";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key a: Angle";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key s: Shift";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key z: Zoom";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key h: Height from Floor";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key e: Enable Area";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				if (bCount){
					text = "Key b: Box (Count Area)";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
				}
				text = "Key f: File Save";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key m: Menu";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
			}

			if (bSubDisplay){
				//Sub display

				if ((angle_y == 0) && ((angle_z < 45) || (angle_z > 270))){
					//Horizontal
					cv::Mat roi = img(cv::Rect(SUB_DISPLAY_X, SUB_DISPLAY_Y, SUB_DISPLAY_WIDTH, SUB_DISPLAY_HEIGHT));
					cv::resize(subdisplay, roi, roi.size(), cv::INTER_LINEAR);
				}
				else {
					//Vertical
					for (int y = 0; y < SUB_DISPLAY_HEIGHT; y++){
						for (int x = 0; x < SUB_DISPLAY_WIDTH; x++){
							//Transpose
							img.at<cv::Vec3b>(SUB_DISPLAY_Y - (SUB_DISPLAY_WIDTH - SUB_DISPLAY_HEIGHT) + x,
								SUB_DISPLAY_X + SUB_DISPLAY_HEIGHT - y) =
								subdisplay.at<cv::Vec3b>(y, x);
						}
					}
				}
			}
			if (NULL == cvGetWindowHandle("Human Counter")){
				brun = false;
			}
			else{
				cv::imshow("Human Counter", img);
			}
		}

		auto key = cv::waitKey(10);
		switch (key){
		case 'a':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 's':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 'z':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 'h':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 'b':
			if (bCount){
				if (mode == key){
					bBoxShift = !bBoxShift;
				}
				mode = key;
			}
			break;
		case 'p':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 'e':
			if (mode == key){
				if (!bEnableArea){
					bEnableArea = true;
					bEnableAreaShift = true;
				}
				else if (!bEnableAreaShift){
					bEnableArea = false;
				}
				else {
					bEnableAreaShift = false;
				}
			}
			mode = key;
			break;
		case 'f':
			if (mode == key){
				mode = 0;
			}
			else {
				//Save file
				if (!SaveFile()){
					//Failed
					savefile = "";
				}
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
		case 'l':
			break;

		case '1':
			if (mode == 'p'){
				bPoint = !bPoint;
			}
			break;
		case '2':
			if (mode == 'p'){
				bBack = !bBack;
			}
			break;
		case '3':
			if (mode == 'p'){
				bCount = !bCount;
			}
			break;
		case '4':
			if (mode == 'p'){
				bSubDisplay = !bSubDisplay;
			}
			break;
		case '9':
			if ((mode == 'p') && (bBack)){
				back = cv::Mat::zeros(480 * 2, 640 * 2, CV_8UC3);
			}
			break;
		case '0':
			if (mode == '0'){
				mode = 0;
			}
			else if ((mode == 'p') && (bCount)){
				mode = '0';
			}
			break;
		case 2490368:	//Up key
			switch (mode){
			case 'a':
				angle_x += ANGLE_ADJUSTMENT_DEGREE;
				if (angle_x >= 360){
					angle_x -= 360;
				}
				if (ChangeAttribute(tof, 0, 0, height * -1, angle_x, angle_y, angle_z) == false){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Attributee Error" << endl;
				}
				break;
			case 's':
				dy -= 100 * zoom;
				break;
			case 'z':
				zoom += 0.01f;
				break;
			case 'h':
				height += 100;
				if (ChangeAttribute(tof, 0, 0, height * -1, angle_x, angle_y, angle_z) == false){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Attributee Error" << endl;
				}
				break;
			case 'b':
				if (bBoxShift){
					Count.Square.top_y -= 200 * zoom;
					Count.Square.bottom_y -= 200 * zoom;
				}
				else {
					Count.Square.bottom_y -= 200 * zoom;
					if (Count.Square.bottom_y <= Count.Square.top_y){
						Count.Square.bottom_y = Count.Square.top_y + 1;
					}
				}
				break;
			case 'e':
				if (bEnableArea){
					if (bEnableAreaShift){
						EnableArea.top_y -= 200 * zoom;
						EnableArea.bottom_y -= 200 * zoom;
					}
					else {
						EnableArea.bottom_y -= 200 * zoom;
						if (EnableArea.bottom_y <= EnableArea.top_y){
							EnableArea.bottom_y = EnableArea.top_y + 1;
						}
					}
				}
				break;
			}
			break;
		case 2621440:	//Down key
			switch (mode){
			case 'a':
				angle_x -= ANGLE_ADJUSTMENT_DEGREE;
				if (angle_x < 0){
					angle_x += 360;
				}
				if (ChangeAttribute(tof, 0, 0, height * -1, angle_x, angle_y, angle_z) == false){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Attributee Error" << endl;
				}
				break;
			case 's':
				dy += 100 * zoom;
				break;
			case 'z':
				zoom -= 0.01f;
				break;
			case 'h':
				height -= 100;
				if (ChangeAttribute(tof, 0, 0, height * -1, angle_x, angle_y, angle_z) == false){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Attributee Error" << endl;
				}
				break;
			case 'b':
				if (bBoxShift){
					Count.Square.top_y += 200 * zoom;
					Count.Square.bottom_y += 200 * zoom;
				}
				else {
					Count.Square.bottom_y += 200 * zoom;
				}
				break;
			case 'e':
				if (bEnableArea){
					if (bEnableAreaShift){
						EnableArea.top_y += 200 * zoom;
						EnableArea.bottom_y += 200 * zoom;
					}
					else {
						EnableArea.bottom_y += 200 * zoom;
					}
				}
				break;
			}
			break;
		case 2555904:	//Right key
			switch (mode){
			case 'a':
				angle_z += ANGLE_ADJUSTMENT_DEGREE;
				if (angle_z >= 360){
					angle_z -= 360;
				}
				if (ChangeAttribute(tof, 0, 0, height * -1, angle_x, angle_y, angle_z) == false){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Attributee Error" << endl;
				}
				break;
			case 's':
				dx += 100 * zoom;
				break;
			case 'b':
				if (bBoxShift){
					Count.Square.left_x += 200 * zoom;
					Count.Square.right_x += 200 * zoom;
				}
				else {
					Count.Square.right_x += 200 * zoom;
				}
				break;
			case 'e':
				if (bEnableArea){
					if (bEnableAreaShift){
						EnableArea.left_x += 200 * zoom;
						EnableArea.right_x += 200 * zoom;
					}
					else {
						EnableArea.right_x += 200 * zoom;
					}
				}
				break;
			}
			break;
		case 2424832:	//Left key
			switch (mode){
			case 'a':
				angle_z -= ANGLE_ADJUSTMENT_DEGREE;
				if (angle_z < 0){
					angle_z += 360;
				}
				if (ChangeAttribute(tof, 0, 0, height * -1, angle_x, angle_y, angle_z) == false){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Attributee Error" << endl;
				}
				break;
			case 's':
				dx -= 100 * zoom;
				break;
			case 'b':
				if (bBoxShift){
					Count.Square.left_x -= 200 * zoom;
					Count.Square.right_x -= 200 * zoom;
				}
				else {
					Count.Square.right_x -= 200 * zoom;
					if (Count.Square.right_x <= Count.Square.left_x){
						Count.Square.right_x = Count.Square.left_x + 1;
					}
				}
				break;
			case 'e':
				if (bEnableArea){
					if (bEnableAreaShift){
						EnableArea.left_x -= 200 * zoom;
						EnableArea.right_x -= 200 * zoom;
					}
					else {
						EnableArea.right_x -= 200 * zoom;
						if (EnableArea.right_x <= EnableArea.left_x){
							EnableArea.right_x = EnableArea.left_x + 1;
						}
					}
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
			if (mode == '0'){
				InitializeCount();
			}
			mode = 0;
			break;
		default:
			break;
		}
	}

	//Stop and closr TOF sensor
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

	//Save ini file
	if (SaveIniFile() == false){
		std::cout << "Ini File Write Error" << endl;
		system("pause");
	}
}
