// JJRobots Air Hockey Robot Evo project
// Computer Vision for PS3 EYE Camera (using OpenCV)
// Author: Jose Julio (JJROBOTS)
// Date: Dec 2013 - Jan 2014
// Updated: 04/10/2016
// Version: 2.34

// This code needs OpenCV libraries (2.4.13)

// This code detects the puck and the robot in the hockey game at 60Hz
// Detects two objects PUCK and ROBOT with two different colors
// And sends serial packets to arduino via UDP (or serial)
// Serial packet:
//     Sync start: 2 bytes: mm
//     TimeStamp: 2 bytes
//     puckCoordX:     2 bytes (UINT16) units milimeters
//     puckCoordY:     2 bytes (UINT16)
//     robotCoordX:     2 bytes (UINT16)
//     robotCoordY:     2 bytes (UINT16)

// Steps: 
// Capture a frame, convert to HSV, 
// For both PUCK and ROBOT:
//    threshold based on HSV range, Object extraction, roundness and size filter, Center calculation
// For the PUCK we make a trajectory prediction to draw the lines...
// We record a video in the file output.mpeg and a log file
// We send UDP packets to the robot with the puck position and robot position.

// External configuration file: config.txt

// H : 70-94 S: 60-150 V: 10-145   GREEN EVA FOAM
// H : 5-20  S: 110-200 V: 90-200  ORANGE EVA FOAM

// Default parameters are for a GREEN EVA FOAM PUCK and an BLUE EVA FOAM ROBOT MARK

// PS3 Camera windows driver from : http://codelaboratories.com/products/eye/driver/

#include <opencv2\opencv.hpp>

#include <time.h>
#include <Windows.h>
#include <stdio.h>
#include <math.h>
#include <WinSock.h> 

// Winsock library link
#pragma comment(lib,"WSock32.lib")

// Default AIR HOCKEY TABLE size
// All units in milimeters
#define ROBOT_TABLE_LENGTH 710  
#define ROBOT_TABLE_WIDTH 400

// PS3 Camera pixels
// NOTE: We are using the camera at 320x240 but we are outputing a 640x480 pixel position
#define CAM_PIX_WIDTH 640
#define CAM_PIX_HEIGHT 480
#define CAM_PIX_TO_MM 1.25  // Default camera distante (this is rewrited on the configuration)

// default configurations
#define VIDEO_OUTPUT TRUE
#define LOG_OUTPUT TRUE
#define PREVIEW 1

// Global variables
// Parameters (with default values)
char comPort[20] = "COM3";
int minH=66;
int maxH=95;
int minS=60;
int maxS=150;
int minV=10;
int maxV=145;
int RminH=5;
int RmaxH=20;
int RminS=110;
int RmaxS=200;
int RminV=90;
int RmaxV=200;
int fps=60;
int viewWindow=1;

int robot_table_length=ROBOT_TABLE_LENGTH;
int robot_table_width=ROBOT_TABLE_WIDTH;
int robot_table_center_x=robot_table_width/2;
int robot_table_center_y=robot_table_length/2;
int puckSize=robot_table_width/16;  // Puck size (radio) estimation. it depends on table size (width)

int robot_y_offset=15;   // Robot offset from robot mark (projected to table) to real robot puck center

// OpenCV variables
CvFont font;
CvVideoWriter* record;
IplImage* imgThresh;
IplImage* imgThresh2;
IplImage* imgHSV;
IplImage* frameGrabbed;
IplImage* frame;
int lastX = -1;
int lastY = -1;
int posX;
int posY;
int objectSize;
int RposX;
int RposY;
int RobjectSize;
int status;

// time variables
time_t start,end;
DWORD frameTimestamp=0;
DWORD firstTimestamp=0;
DWORD oldFrameTimestamp;

HANDLE serialPort;  // Serial port
BYTE message[20];   // BYTES buffer

// Socket
SOCKET sConnect;
int udp_port;
int udp = 1;  // mode

FILE* logFile;
char tempStr[80];
char logStr[4096];

// Trajectory prediction
// Puck variables
int puckCoordX;
int puckCoordY;
int puckOldCoordX;
int puckOldCoordY;
int puckSpeedX;
int puckSpeedY;
float puckSpeed;         // mm/sec
float puckDirection;     // radians

// robot
int robotCoordX;
int robotCoordY;

int defense_position;
int predict_x;    // X position at impact (mm)
int predict_y;
int predict_x_old;
int predict_y_old;
int predict_time;   // time to impact in ms
char tempStr2[80];

// Camera variables
int camera_cap = CV_CAP_ANY;
int cam_center_x;
int cam_center_y;
float cam_pix_to_mm=CAM_PIX_TO_MM;
float cam_rotation=0.0;  //Camera rotation in radians 

// variables to draw the table
int table_pix_minx;
int table_pix_miny;
int table_pix_maxx;
int table_pix_maxy;

// application parameters
bool video_output = VIDEO_OUTPUT;
bool log_output = LOG_OUTPUT;
int preview = PREVIEW;

// Variables initialization
void cameraProcessInit()
{
	  // Default values
  cam_center_x = CAM_PIX_WIDTH/2;
  cam_center_y = CAM_PIX_HEIGHT/2;
  cam_rotation = 0; //radians  1º = 0.01745 2º = 0.035 4º = 0.07 5º = 0.087
  predict_x_old = -1;

  robot_table_center_x=robot_table_width/2;
  robot_table_center_y=robot_table_length/2;
  puckSize=robot_table_width/20;  // puck size (radio) estimation
  defense_position = 60+puckSize;  // Pusher defense position (for predictions)

  table_pix_maxx = ((robot_table_center_y-0)/cam_pix_to_mm + cam_center_x)/2.0;
  table_pix_maxy = ((robot_table_center_x-0)/cam_pix_to_mm + cam_center_y)/2.0;
  table_pix_minx = ((robot_table_center_y-robot_table_length)/cam_pix_to_mm + cam_center_x)/2.0;
  table_pix_miny = ((robot_table_center_x-robot_table_width)/cam_pix_to_mm + cam_center_y)/2.0;

  printf("Table %d,%d %d,%d\n",table_pix_minx,table_pix_miny,table_pix_maxx,table_pix_maxy);
}


// Draw the table marks
void drawTable()
{
  // Draw marks on image
  // Draw marker on center
  cvLine(frameGrabbed, cvPoint(CAM_PIX_WIDTH/4-5, CAM_PIX_HEIGHT/4), cvPoint(CAM_PIX_WIDTH/4+5, CAM_PIX_HEIGHT/4), cvScalar(100,255,200), 1);
  cvLine(frameGrabbed, cvPoint(CAM_PIX_WIDTH/4, CAM_PIX_HEIGHT/4-5), cvPoint(CAM_PIX_WIDTH/4, CAM_PIX_HEIGHT/4+5), cvScalar(100,255,200), 1);
  //Draw table limits
  //cvLine(frameGrabbed, cvPoint(table_pixx1,table_pixy1), cvPoint(table_pixx2,table_pixy2), cvScalar(100,255,200), 1);
  //cvLine(frameGrabbed, cvPoint(table_pixx2,table_pixy2), cvPoint(table_pixx3,table_pixy3), cvScalar(100,255,200), 1);
  //cvLine(frameGrabbed, cvPoint(table_pixx3,table_pixy3), cvPoint(table_pixx4,table_pixy4), cvScalar(100,255,200), 1);
  //cvLine(frameGrabbed, cvPoint(table_pixx4,table_pixy4), cvPoint(table_pixx1,table_pixy1), cvScalar(100,255,200), 1);

  cvLine(frameGrabbed, cvPoint(table_pix_minx,table_pix_miny), cvPoint(table_pix_minx+20,table_pix_miny), cvScalar(100,255,200), 1);
  cvLine(frameGrabbed, cvPoint(table_pix_minx,table_pix_maxy), cvPoint(table_pix_minx+20,table_pix_maxy), cvScalar(100,255,200), 1);
  cvLine(frameGrabbed, cvPoint(table_pix_maxx-20,table_pix_miny), cvPoint(table_pix_maxx,table_pix_miny), cvScalar(100,255,200), 1);
  cvLine(frameGrabbed, cvPoint(table_pix_maxx-20,table_pix_maxy), cvPoint(table_pix_maxx,table_pix_maxy), cvScalar(100,255,200), 1);
  cvLine(frameGrabbed, cvPoint(table_pix_minx,table_pix_miny), cvPoint(table_pix_minx,table_pix_miny+20), cvScalar(100,255,200), 1);
  cvLine(frameGrabbed, cvPoint(table_pix_maxx,table_pix_miny), cvPoint(table_pix_maxx,table_pix_miny+20), cvScalar(100,255,200), 1);
  cvLine(frameGrabbed, cvPoint(table_pix_minx,table_pix_maxy-20), cvPoint(table_pix_minx,table_pix_maxy), cvScalar(100,255,200), 1);
  cvLine(frameGrabbed, cvPoint(table_pix_maxx,table_pix_maxy-20), cvPoint(table_pix_maxx,table_pix_maxy), cvScalar(100,255,200), 1);
}

// Camera process, convert puck position to coordinates and generate trajectory prediction and visualization
// Simple lens distortion correction (one parameter) NOT USED NOW
// Xu = (Xd)/(1+param*dist2)  dist2 = distancia al cuadrado del pixel al centro
void cameraProcess(int time)
{
  int coordX;
  int coordY;
  int vectorX;
  int vectorY;
  double slope;

  int bounce_x;
  int bounce_y;
  int predict_pixX;
  int predict_pixY;
  int bounce_pixX;
  int bounce_pixY;

  // Convert from Camera reference system to Robot reference system
  
  // Camera X axis correspond to robot Y axis
  coordY = (posX - cam_center_x);   // First we convert image coordinates to center of image
  coordX = (posY - cam_center_y);
  
  coordY = robot_table_center_y - coordY*cam_pix_to_mm;
  coordX = robot_table_center_x - coordX*cam_pix_to_mm;
  
  // Calculate speed and angle
  vectorX = (coordX-puckCoordX);
  vectorY = (coordY-puckCoordY);
 
  puckSpeedX = vectorX*100/time;  // speed in dm/ms (
  puckSpeedY = vectorY*100/time;
  //puckSpeed = sqrt(vectorX*vectorX + vectorY*vectorY)*1000.0/time;
  //puckDirection = atan2(vectorY,vectorX);
  puckOldCoordX = puckCoordX;
  puckOldCoordY = puckCoordY;
  puckCoordX = coordX;
  puckCoordY = coordY;

  // Noise detection, big vector...
  if ((vectorY<-250)||(vectorY>250)||(vectorX>250)||(vectorX<-250))
  {
	  sprintf(tempStr2,"N %d",vectorY);
	  return;
  }
  
  // Its time to predict...
  // Based on actual position, speed and angle we need to know the future...
  // Posible impact?
  if (puckSpeedY<-35)
    {
      // Puck is comming...
      // We need to predict the puck position when it reaches our goal Y=0
      // slope formula: m = (y2-y1)/(x2-x1)
	  if (vectorX == 0)  // To avoid division by 0
		slope = 9999999;
	  else
		slope = (float)vectorY/(float)vectorX;
      // x = (y2-y1)/m + x1
	  predict_y = defense_position;
	  predict_x = (predict_y-coordY)/slope + coordX;
	 
	  // puck has a bounce with side wall?
      if ((predict_x<puckSize)||(predict_x>(robot_table_width-puckSize)))
        {
		// We start a new prediction
		// Wich side?
		if (predict_x<puckSize)
			{
			//Left side. We calculare the impact point
			bounce_x = puckSize;
			}
		else
			{
			//Right side. We calculare the impact point
			bounce_x = (robot_table_width-puckSize);
			}
		bounce_y = (bounce_x - coordX)*slope + coordY;
		bounce_pixX = cam_center_x - (bounce_y-robot_table_center_y)/cam_pix_to_mm;
		bounce_pixY = cam_center_y - (bounce_x-robot_table_center_x)/cam_pix_to_mm;
		predict_time = (bounce_y-puckCoordY)*100/puckSpeedY;  // time until bouce
		// bounce prediction
		// Slope change
		slope = -slope;
		predict_y = defense_position;
		predict_x = (predict_y-bounce_y)/slope + bounce_x;

		if ((predict_x<puckSize)||(predict_x>(robot_table_width-puckSize)))
			{
			// New bounce?? 
			// We do nothing then...
			sprintf(tempStr2,"2B %d %d",bounce_x,bounce_y);
			predict_x_old = -1;
			}
		else
			{
			// draw line
			if (preview==1)
				cvLine(frameGrabbed, cvPoint(posX/2, posY/2), cvPoint(bounce_pixX/2, bounce_pixY/2), cvScalar(255,0,0), 2);
			// result average
			if (predict_x_old != -1)
				predict_x = (predict_x_old + predict_x)>>1;
			predict_x_old = predict_x;
			predict_time = predict_time + (predict_y-puckCoordY)*100/puckSpeedY;  // in ms
			sprintf(tempStr2,"%d t%d",predict_x,predict_time);
			predict_pixX = cam_center_x - (predict_y-robot_table_center_y)/cam_pix_to_mm;
			predict_pixY = cam_center_y - (predict_x-robot_table_center_x)/cam_pix_to_mm;
			// draw line
			if (preview==1)
				cvLine(frameGrabbed, cvPoint(bounce_pixX/2, bounce_pixY/2), cvPoint(predict_pixX/2, predict_pixY/2), cvScalar(0,255,0), 2);
			}
		 }
      else  // No bounce, direct impact
        {
		// result average
		if (predict_x_old != -1)
			predict_x = (predict_x_old + predict_x)>>1;
		predict_x_old = predict_x;
		
		predict_time = (predict_y-puckCoordY)*100/puckSpeedY;  // in ms
		sprintf(tempStr2,"%d t%d",predict_x,predict_time);
		// Convert impact prediction position to pixels (to show on image)
		predict_pixX = cam_center_x - (predict_y-robot_table_center_y)/cam_pix_to_mm;
		predict_pixY = cam_center_y - (predict_x-robot_table_center_x)/cam_pix_to_mm;
		// draw line
		if (preview==1)
			cvLine(frameGrabbed, cvPoint(posX/2, posY/2), cvPoint(predict_pixX/2, predict_pixY/2), cvScalar(0,255,0), 2);
        }
    }
  else // Puck is moving slowly or to the other side
	{	
	sprintf(tempStr2,"",coordX,coordY,puckSpeedY);
	predict_x_old = -1;
	}

}

// Robot process, convert robot position to coordinates
void robotProcess()
{
  int coordX;
  int coordY;
  
  if ((RposX==0)||(RposY==0)){
	  robotCoordX = 0; 
	  robotCoordY = 0;
  }
  else{
	  // Convert from Camera reference system to Robot reference system
	  // Camera X axis correspond to robot Y axis
	  coordY = (RposX - cam_center_x);   // First we convert image coordinates to center of image
	  coordX = (RposY - cam_center_y);
	  robotCoordY = robot_table_center_y - coordY*cam_pix_to_mm + robot_y_offset;
	  robotCoordX = robot_table_center_x - coordX*cam_pix_to_mm;
  }
}



//This function threshold the HSV image and create a binary image
IplImage* GetThresholdedImage(IplImage* imgHSV, int minH, int maxH, int minS, int maxS, int minV, int maxV){       
    IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
    cvInRangeS(imgHSV, cvScalar(minH,minS,minV), cvScalar(maxH,maxS,maxV), imgThresh); 
    return imgThresh;
}


// Image segmentation, object extraction based on size and form (roundness)
void trackObjectPuck(IplImage* imgThresh){
 double area;
 double perimeter;
 double roundness;
 int num;
  
 CvSeq* contours;  //hold the pointer to a contour in the memory block
 CvSeq* result;   //hold sequence of points of a contour
 CvMemStorage *storage = cvCreateMemStorage(0); //storage area for all contours

 // Position initialization
 posX = 0;
 posY = 0;
 status = 0;
 num = 0;
 //finding all contours in the image (segmentation)
 cvFindContours(imgThresh, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
 if (contours)
	status=1;
 //sprintf(logStr,"\n");
 while (contours)
 {
	num++;
	 area = cvContourArea(contours);
	 //sprintf(logStr,"%s %d %.2f %.2f", logStr,status, area, roundness);
	 //sprintf(logStr,"%s;%d %.2f", logStr,num, area);
	 if ((area>12)&&(area<250))  // Min and Max size of object
	 {
		status = 2;
        //Detecting roundness   roundness = perimeter2/(2*pi*area)
		perimeter = cvArcLength(contours);
		roundness = (perimeter*perimeter)/(6.28*area);
		//sprintf(logStr,"%s %.2f", logStr,roundness);
		if (roundness < 8)
		{
			status = 3;
			CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
			cvMoments(contours, moments, 1);
			double moment10 = cvGetSpatialMoment(moments, 1, 0);
			double moment01 = cvGetSpatialMoment(moments, 0, 1);
			area = cvGetCentralMoment(moments, 0, 0);
			// Calculate object center
			// We are using 320x240 pix but we are going to output the 640x480 equivalent (*2)
			posX = floor(moment10*2/(double)area+0.5); // round
			posY = floor(moment01*2/(double)area+0.5);
			objectSize = area;
			// limit the region of interest (table).
			//printf("%d (%d %d) \n",posX,table_pix_minx*2,table_pix_maxx*2);
			
			if ((posX > table_pix_maxx*2)||(posX < table_pix_minx*2)||(posY > table_pix_maxy*2)||(posY < table_pix_miny*2))
			{
				status=1;
				posX = 0;
				posY = 0;
				contours = contours->h_next; 
				continue;  // continue with other contour... (this is outside the table)
			}
			
			// Draw contour
			if (preview==1)
				cvDrawContours(frameGrabbed,contours,cvScalar(255,0,0),cvScalar(0,0,255),0,1,8,cvPoint(0,0));
			if(lastX>=0 && lastY>=0)
				{
				status = 4;
				// Draw a line from the previous point to the current point
				if (preview==1)
					cvLine(frameGrabbed, cvPoint(posX/2, posY/2), cvPoint(lastX/2, lastY/2), cvScalar(255,255,0), 4);
				}
			lastX = posX;
			lastY = posY;
			break; // puck finded!
		}
	 }
	 //obtain the next contour
     contours = contours->h_next; 
 }
 //printf("\n");
 cvReleaseMemStorage(&storage);
}

// Image segmentation, object extraction based on size and form (roundness)
void trackObjectRobot(IplImage* imgThresh){

 CvSeq* contours;  //hold the pointer to a contour in the memory block
 CvSeq* result;   //hold sequence of points of a contour
 CvMemStorage *storage = cvCreateMemStorage(0); //storage area for all contours

 // Position initialization
 RposX = 0;
 RposY = 0;
 RobjectSize = 0;
 
 //finding all contours in the image (segmentation)
 cvFindContours(imgThresh, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
 while (contours)
 {
	 double area = cvContourArea(contours);
	 
	 if ((area>15)&&(area<150))  // Min and Max size of object
	 {
        //Detecting roundness   roundness = perimeter2/(2*pi*area)
		double perimeter = cvArcLength(contours);
		double roundness = (perimeter*perimeter)/(6.28*area);
		//printf("%lf",roundness);
		if (roundness < 8)
		{
			//printf("area:%.2lf R:%.2lf\n",area,roundness);
			CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
			cvMoments(contours, moments, 1);
			double moment10 = cvGetSpatialMoment(moments, 1, 0);
			double moment01 = cvGetSpatialMoment(moments, 0, 1);
			area = cvGetCentralMoment(moments, 0, 0);
		
			// Calculate object center
			// We are using 320x240 pix but we are going to output the 640x480 equivalent (*2)
			RposX = floor(moment10*2/(double)area+0.5); // round
			RposY = floor(moment01*2/(double)area+0.5);
			RobjectSize = area;
			
			// limit the region of interest (table)
			if ((RposX > (table_pix_maxx*2-5))||(RposX < (table_pix_minx*2+5))||(RposY > (table_pix_maxy*2-5))||(RposY < (table_pix_miny*2+5)))
			{
				//printf("aqui %d (%d %d) \n",RposX,table_pix_minx*2,table_pix_maxx*2);
				RposX = 0;
				RposY = 0;
				contours = contours->h_next; 
				continue;  // continue with other contour... (this is outside the table)
			}
			if(RposX>=0 && RposY>=0)
			{
				if (preview==1)
				{
					cvLine(frameGrabbed, cvPoint(RposX/2, RposY/2), cvPoint(RposX/2, RposY/2), cvScalar(100,255,50), 4);
					cvDrawContours(frameGrabbed,contours,cvScalar(0,0,100),cvScalar(100,0,0),0,1,8,cvPoint(0,0));
				}
				break;
			}
		}
	 }

	 //obtain the next contour
     contours = contours->h_next; 
 }
 cvReleaseMemStorage(&storage);
}


bool openComPort(wchar_t* portSpecifier)
{
	DCB dcb;

	// Open Serial Port
	wprintf(L"Opening COM PORT: %s",portSpecifier);
	printf("\n");

	serialPort = CreateFile(portSpecifier,GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,0,NULL);

	if (serialPort == INVALID_HANDLE_VALUE)
	{
		wprintf(L"Error opening Serial port: %s\n",portSpecifier);
		return(false);
	}

	//if (!GetCommState(serialPort,&dcb))
	//	return(false);

	// Serial port configuration
	dcb.BaudRate = CBR_115200;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	if (!SetCommState(serialPort,&dcb))
	{
		printf("Error configuring Serial Port\n");
		return(false);
	}
	return true;
}

bool writeComPort(BYTE *message,int length)
{
	DWORD byteswritten;

	bool retVal = WriteFile(serialPort,message,length,&byteswritten,NULL);
	return retVal;
}

// Read from COM PORT and output to console
bool readComPort()
{
	DWORD dwRead;
	char m_pDataBuf[4096];
	BYTE unbyte;
	DWORD temp; 
    COMSTAT comstat;
	BOOL readResult;

	// Get Serial stats
	ClearCommError(serialPort,&temp,&comstat);

	// New bytes pending read?
	if (comstat.cbInQue>0)
		{
		ReadFile(serialPort,m_pDataBuf, comstat.cbInQue, &dwRead, NULL);
		m_pDataBuf[dwRead] = 0;
		sprintf(tempStr,"[%ld]",frameTimestamp-firstTimestamp);
		fwrite(tempStr,strlen(tempStr),1,logFile);   
		fwrite(m_pDataBuf,dwRead,1,logFile);
		printf("%s\n",m_pDataBuf);
		sprintf(logStr,"%s",m_pDataBuf);
		}
	return true;
}


// Send message to Serial port with the object information 12 bytes message
bool sendMessageSerial()
{
	DWORD timestamp;
	// Initial sync 0x6D 0x6D
	message[0] = 0x6D;
	message[1] = 0x6D;
	// TimeStamp
	timestamp = frameTimestamp - firstTimestamp;
	message[2] = (timestamp>>8)&0xFF;
	message[3] = timestamp&0xFF;
	// Pos_X (high byte, low byte)
	message[4] = (puckCoordX>>8)&0xFF;
	message[5] = puckCoordX&0xFF;
	// Pos_Y (high byte, low byte)
	message[6] = (puckCoordY>>8)&0xFF;
	message[7] = puckCoordY&0xFF;
	// Robot Pos_X (high byte, low byte)
	message[8] = (robotCoordX>>8)&0xFF;
	message[9] = robotCoordX&0xFF;
	// Robot Pos_Y (high byte, low byte)
	message[10] = (robotCoordY>>8)&0xFF;
	message[11] = robotCoordY&0xFF;

	return writeComPort(message,12); // Send message (12 bytes)
}

void openUDPConnection(){

	// Setup socket
	SOCKADDR_IN addr;
	int addrlen = sizeof(addr);

	printf ("Opening UDP connection to 192.168.4.1:2222\n");

	sConnect = socket(AF_INET,SOCK_DGRAM,0);  // UDP protocol
	if (sConnect == INVALID_SOCKET){
		printf("Invalid socket!\n");
		}
	else{
		printf("Socket created\n");
		}

	// IP of server (192.168.4.1 , port 2222) default config for ESP8266 module
	addr.sin_addr.s_addr = inet_addr("192.168.4.1");
	addr.sin_family = AF_INET;
	addr.sin_port = htons(2222);

	// Connect to server
	int iResult;
	iResult = connect(sConnect,(SOCKADDR*)&addr,sizeof(addr));
	printf("Connection response: %d\n",iResult);
	if (iResult == SOCKET_ERROR){
		printf("Connection error!\n");
		}
	else{
		printf("Connected to server!\n");
		}
}


// Send message to Serial port with the object information 12 bytes message
bool sendMessageUDP()
{
	DWORD timestamp;
	int result;
	// Initial sync 0x6D 0x6D
	message[0] = 0x6D;
	message[1] = 0x6D;
	// TimeStamp
	timestamp = frameTimestamp - firstTimestamp;
	message[2] = (timestamp>>8)&0xFF;
	message[3] = timestamp&0xFF;
	// Pos_X (high byte, low byte)
	message[4] = (puckCoordX>>8)&0xFF;
	message[5] = puckCoordX&0xFF;
	// Pos_Y (high byte, low byte)
	message[6] = (puckCoordY>>8)&0xFF;
	message[7] = puckCoordY&0xFF;
	// Robot Pos_X (high byte, low byte)
	message[8] = (robotCoordX>>8)&0xFF;
	message[9] = robotCoordX&0xFF;
	// Robot Pos_Y (high byte, low byte)
	message[10] = (robotCoordY>>8)&0xFF;
	message[11] = robotCoordY&0xFF;

	// Send to socket
	result = send(sConnect, (const char *)message,12,0);
	if (result <= 0)
		printf("Error UDP\n");
	return result;
}

int readConfigFile()
{
	FILE *f;
	char aux_str[255];
	int aux_int;
	double aux_double;

	if ((f=fopen("config.txt","rt"))==NULL)
	{
		printf("CONFIG.TXT FILE NOT FOUND!");
		return -1;
	}
	printf("Reading CONFIG.TXT configuration file...\n");
	while (!feof(f))
	{
		strcpy (aux_str,"empty");
		if (fscanf(f,"%s",aux_str))
		{
			if (strcmp(aux_str,"empty")!=0)
			{
				//printf("%s\n",aux_str);
				if (strcmp(aux_str,"#")==0)
				{
					// COMMENT
					//printf("#->COMMENT->SKIP\n");
				}
				else if (strcmp(aux_str,"CAMERA")==0)
				{
					fscanf(f,"%d",&aux_int);
					camera_cap = aux_int;
					printf("CAMERA:%d\n",camera_cap);
				}
				else if (strcmp(aux_str,"PROTOCOL")==0)
				{
					fscanf(f,"%s",aux_str);
					if (strcmp(aux_str,"SERIAL")==0)
						udp =0;
					else
						udp =1;
					printf("PROTOCOL:%s %d\n",aux_str,udp);
				}
				else if (strcmp(aux_str,"CAMPIXTOMM")==0)
				{
					fscanf(f,"%lf",&aux_double);
					cam_pix_to_mm = aux_double;
					printf("PARAMCAMPIXTOMM:%lf\n",aux_double);
				}
				else if (strcmp(aux_str,"TABLELENGTH")==0)
				{
					fscanf(f,"%d",&aux_int);
					robot_table_length = aux_int;
					printf("TABLELENGTH:%d\n",robot_table_length);
				}
				else if (strcmp(aux_str,"TABLEWIDTH")==0)
				{
					fscanf(f,"%d",&aux_int);
					robot_table_width = aux_int;
					printf("TABLEWIDTH:%d\n",robot_table_width);
				}
				else if (strcmp(aux_str,"PUCKMINH")==0)
				{
					fscanf(f,"%d",&aux_int);
					minH = aux_int;
					printf("PUCKMINH:%d\n",minH);
				}
				else if (strcmp(aux_str,"PUCKMAXH")==0)
				{
					fscanf(f,"%d",&aux_int);
					maxH = aux_int;
					printf("PUCKMAXH:%d\n",maxH);
				}
				else if (strcmp(aux_str,"PUCKMINS")==0)
				{
					fscanf(f,"%d",&aux_int);
					minS = aux_int;
					printf("PUCKMINS:%d\n",minS);
				}
				else if (strcmp(aux_str,"PUCKMAXS")==0)
				{
					fscanf(f,"%d",&aux_int);
					maxS = aux_int;
					printf("PUCKMAXS:%d\n",maxS);
				}
				else if (strcmp(aux_str,"PUCKMINV")==0)
				{
					fscanf(f,"%d",&aux_int);
					minV = aux_int;
					printf("PUCKMINV:%d\n",minV);
				}
				else if (strcmp(aux_str,"PUCKMAXV")==0)
				{
					fscanf(f,"%d",&aux_int);
					maxV = aux_int;
					printf("PUCKMAXV:%d\n",maxV);
				}
				else if (strcmp(aux_str,"ROBOTMINH")==0)
				{
					fscanf(f,"%d",&aux_int);
					RminH = aux_int;
					printf("ROBOTMINH:%d\n",RminH);
				}
				else if (strcmp(aux_str,"ROBOTMAXH")==0)
				{
					fscanf(f,"%d",&aux_int);
					RmaxH = aux_int;
					printf("ROBOTMAXH:%d\n",RmaxH);
				}
				else if (strcmp(aux_str,"ROBOTMINS")==0)
				{
					fscanf(f,"%d",&aux_int);
					RminS = aux_int;
					printf("ROBOTMINS:%d\n",RminS);
				}
				else if (strcmp(aux_str,"ROBOTMAXS")==0)
				{
					fscanf(f,"%d",&aux_int);
					RmaxS = aux_int;
					printf("ROBOTMAXS:%d\n",RmaxS);
				}
				else if (strcmp(aux_str,"ROBOTMINV")==0)
				{
					fscanf(f,"%d",&aux_int);
					RminV = aux_int;
					printf("ROBOTMINV:%d\n",RminV);
				}
				else if (strcmp(aux_str,"ROBOTMAXV")==0)
				{
					fscanf(f,"%d",&aux_int);
					RmaxV = aux_int;
					printf("ROBOTMAXV:%d\n",RmaxV);
				}
				else if (strcmp(aux_str,"ROBOTYOFFSET")==0)
				{
					fscanf(f,"%d",&aux_int);
					robot_y_offset = aux_int;
					printf("ROBOTYOFFSET:%d\n",robot_y_offset);
				}
				else if (strcmp(aux_str,"FPS")==0)
				{
					fscanf(f,"%d",&aux_int);
					fps = aux_int;
					printf("FPS:%d\n",fps);
				}
				else if (strcmp(aux_str,"VIDEOOUTPUT")==0)
				{
					fscanf(f,"%s",aux_str);
					if (strcmp(aux_str,"YES")==0)
						video_output = true;
					else
						video_output = false;
					printf("VIDEOOUTPUT:%s\n",aux_str);
				}
				else if (strcmp(aux_str,"LOG")==0)
				{
					fscanf(f,"%s",aux_str);
					if (strcmp(aux_str,"YES")==0)
						log_output = true;
					else
						log_output = false;
					printf("LOG:%s\n",aux_str);
				}
				else if (strcmp(aux_str,"PREVIEW")==0)
				{
					fscanf(f,"%s",aux_str);
					if (strcmp(aux_str,"YES")==0)
						preview = 1;
					else if (strcmp(aux_str,"RAW")==0)
						preview = 2;
					else
						preview = 0;
					printf("PREVIEW:%s\n",aux_str);
				}
				else
				{
					//printf("PARAM NOT RECOGNIZED!\n");
				}
			}
		}
	}
	return 0;
}


int main(int argc, char* argv[]){
	int counter;
	wchar_t auxstr[20];

	// Socket dll version
	WSAData wsaData;
	WORD DLLVersion;
	DLLVersion = MAKEWORD(2,1);
	long answer;
	answer = WSAStartup(DLLVersion,&wsaData);

	
	printf("\nJJRobots AIR HOCKEY ROBOT EVO Vision System v0.34\n\n");
	cvWaitKey(1000);
	// Read config file
	readConfigFile();

	printf("\n\nPress <Space> to save an image. Press <ESC> to exit.\n");

	// LogFile
	if (log_output)
	{
		logFile = fopen("log.txt","wt");
		if (!logFile)
			{
			printf("Error opening Log File!\n");
			log_output = false;
			}
		else
			{
			fprintf(logFile,"AHR v2 LOG OUTPUT\n\n");
			fprintf(logFile,"CAM PIX TO MM: %lf\n",cam_pix_to_mm);
			fprintf(logFile,"TABLE LENGHT: %d\n",robot_table_length);
			fprintf(logFile,"TABLE WIDTH: %d\n",robot_table_width);
			fprintf(logFile,"PUCK MINH: %d\n",minH);
			fprintf(logFile,"PUCK MAXH: %d\n",maxH);
			fprintf(logFile,"PUCK MINS: %d\n",minS);
			fprintf(logFile,"PUCK MAXS: %d\n",maxS);
			fprintf(logFile,"PUCK MINV: %d\n",minV);
			fprintf(logFile,"PUCK MAXV: %d\n",maxV);
			fprintf(logFile,"ROBOT MINH: %d\n",RminH);
			fprintf(logFile,"ROBOT MAXH: %d\n",RmaxH);
			fprintf(logFile,"ROBOT MINS: %d\n",RminS);
			fprintf(logFile,"ROBOT MAXS: %d\n",RmaxS);
			fprintf(logFile,"ROBOT MINV: %d\n",RminV);
			fprintf(logFile,"ROBOT MAXV: %d\n",RmaxV);
			fprintf(logFile,"ROBOT Y OFFSET: %d\n",robot_y_offset);
			fprintf(logFile,"FPS: %d\n",fps);
			fprintf(logFile,"VIDEOOUTPUT: %d\n",video_output);
			fprintf(logFile,"LOG: %d\n",video_output);
			fprintf(logFile,"PREVIEW: %d\n",video_output);
			fprintf(logFile,"CAMERA: %d\n\n",camera_cap);
			}
	}

	cameraProcessInit();

	CvCapture* capture = 0;       
    //capture = cvCaptureFromCAM(CV_CAP_ANY);
	if (camera_cap>=0)
		capture = cvCaptureFromCAM(camera_cap);
	else
	{
		// TAKE A VIDEO FROM FILE
		capture = cvCaptureFromFile("input.mpeg");
		cvSetCaptureProperty(capture,CV_CAP_PROP_POS_MSEC,0);
	}

    if(!capture){
       printf("OPENCV Capture failure!\n");
       return -1;
    }
    cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,320);
	cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT,240);
	cvSetCaptureProperty(capture,CV_CAP_PROP_FPS,fps);
	//cvSetCaptureProperty(capture, CV_CAP_PROP_EXPOSURE, 2);
	

    frameGrabbed = cvQueryFrame(capture);           
    if(!frameGrabbed){
		printf("Error grabing fist frame!\n");
		cvWaitKey(2000);
		}
	
	// Get some properties
	frameGrabbed = cvQueryFrame(capture);
	if(!frameGrabbed){
		printf("Error grabing frame!->EXIT\n");
		return -1;
		}
    firstTimestamp = GetTickCount();
	frameTimestamp = firstTimestamp;
    //create a blank image and assgned to 'imgTracking' which has the same size of original video
	frame=cvCreateImage(cvGetSize(frameGrabbed),IPL_DEPTH_8U, 3);
	
	cvNamedWindow("Video",CV_WINDOW_NORMAL);
	cvResizeWindow("Video",CAM_PIX_WIDTH,CAM_PIX_HEIGHT);

	cvWaitKey(1000);
	if (udp)
		openUDPConnection();
	else
		openComPort(auxstr);
	cvWaitKey(1000);

if (video_output)
{
	// Video writer MPG1
	record = cvCreateVideoWriter("output.mpeg", CV_FOURCC('P','I','M','1'), fps, cvGetSize(frameGrabbed), 1);
}

	// Init font in order to write text on images
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.4,0.4,0,1);

	// MAIN LOOP: Iterate through each frames of the video
    while(true){
			
		oldFrameTimestamp = frameTimestamp;
		frameGrabbed = cvQueryFrame(capture);  // Query a new frame       
		if(!frameGrabbed)
		{
			printf("No frames!\n");
			break;
		}

		frameTimestamp = GetTickCount(); // Get timestamp (not too much resolution)
		//printf("%ld\n",(frameTimestamp-oldFrameTimestamp));
		frameGrabbed=cvCloneImage(frameGrabbed); 
    
		cvSmooth(frameGrabbed, frame, CV_GAUSSIAN,3,3); //smooth the original image using Gaussian kernel
		imgHSV=cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3); 
		cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
		imgThresh = GetThresholdedImage(imgHSV,minH,maxH,minS,maxS,minV,maxV);
		imgThresh2 = GetThresholdedImage(imgHSV,RminH,RmaxH,RminS,RmaxS,RminV,RmaxV);

		/// Apply the erosion-dilatation operation for filtering
		//cvErode( imgThresh, imgThresh, NULL,1 );
		//cvDilate( imgThresh, imgThresh, NULL,1 );
            
		//track the possition of the puck and the robot
		trackObjectPuck(imgThresh);
		trackObjectRobot(imgThresh2);

		// CAMERA PROCESS (puck coordinates, trajectory...)
		cameraProcess(1000/fps);
		// ROBOT PROCESS (robot coordinates)
		robotProcess();

		// Send Message to Serial Port
		if (udp)
			sendMessageUDP();
		else
			sendMessageSerial();

		if (preview==1)
		{
			// Put text over image
			sprintf(tempStr,"%ld %d,%d %d",frameTimestamp-firstTimestamp,puckCoordX,puckCoordY,puckSpeedY);
			cvPutText (frameGrabbed, tempStr, cvPoint(10,20), &font, cvScalar(255,255,0));
			cvPutText (frameGrabbed, tempStr2, cvPoint(170,20), &font, cvScalar(255,255,0));
			// Draw Table for reference
			drawTable();
		}

		// LOG TEXT
		//cvPutText (frameGrabbed, logStr, cvPoint(20,220), &font, cvScalar(50,220,220));

		if (preview>0)
			cvShowImage("Video", frameGrabbed);

		if (video_output)
		cvWriteFrame(record,frameGrabbed); //Write image to output video
			
		if (log_output)
		{
			// Write to logFile
			sprintf(tempStr,"%ld %d,%d;\n",(frameTimestamp-oldFrameTimestamp),posX,posY);
			fwrite(tempStr,strlen(tempStr),1,logFile);
		}
           
		
		if (udp==0){
			// Sometnig to read on SerialPort?
			readComPort();
		}

		//Wait 1mS necesary???
		int c = cvWaitKey(1);

		if ((char)c==32){
			// Save image
			cvSaveImage("image1.png",frame);
			cvSaveImage("image2.png",frameGrabbed);
		}
		
		// Clean up used images
		cvReleaseImage(&imgHSV);
		cvReleaseImage(&imgThresh);  
		cvReleaseImage(&imgThresh2); 
		cvReleaseImage(&frameGrabbed);

		//If 'ESC' is pressed, break the loop
		if((char)c==27 ) break;

		counter++;
    }

    cvDestroyAllWindows() ;
    cvReleaseCapture(&capture);
	if (video_output)
		cvReleaseVideoWriter(&record);
	if (log_output)
		fclose(logFile);

    return 0;
}