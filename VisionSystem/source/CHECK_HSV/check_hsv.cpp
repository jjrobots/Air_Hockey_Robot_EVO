
// AIR HOCKEY ROBOT EVO UTILITY
// Check and adjust HSV values...
//
// JJRobots

#include <opencv2\opencv.hpp>
#include <time.h>

int lowerH=70;
int lowerS=80;
int lowerV=50;
int upperH=95;
int upperS=255;
int upperV=180;

int mode=0;   // 0 = video display, 1 = image display (png)

int camera_device = 0;
bool process;

 //This function threshold the HSV image and create a binary image
 IplImage* GetThresholdedImage(IplImage* imgHSV){
  
 IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
 // Convert 0-180 to 0-255 scale
 cvInRangeS(imgHSV, cvScalar((lowerH*180)/255,lowerS,lowerV), cvScalar((upperH*180)/255,upperS,upperV), imgThresh); 
 
 return imgThresh;
}

//This function create two windows and 6 trackbars for the "Ball" window
void setwindowSettings(){
 cvNamedWindow("Video");
 cvNamedWindow("Object");
 cvNamedWindow("Controls");

 cvCreateTrackbar("LowerH", "Controls", &lowerH, 255, NULL);
 cvCreateTrackbar("UpperH", "Controls", &upperH, 255, NULL);

 cvCreateTrackbar("LowerS", "Controls", &lowerS, 255, NULL);
 cvCreateTrackbar("UpperS", "Controls", &upperS, 255, NULL);

 cvCreateTrackbar("LowerV", "Controls", &lowerV, 255, NULL);
 cvCreateTrackbar("UpperV", "Controls", &upperV, 255, NULL); 
}

int main(){
 char filename[80] = "image1.png";  // default image name image1.png
 CvCapture* capture =0; 

 // SHOW SIMPLE HELP
 printf("\n\n\n JJROBOTS AIR HOCKEY ROBOT EVO HSV utility v0.11\n");
 printf("\nHELP:\n");
 printf(" Press 'c' to change camera device (increase camera number)\n");
 printf(" Press '1'... to '9' to open images (image name: image1.png,image2.png ... image9.png)\n\n\n");
 cvWaitKey(2500);

 
 if (mode == 0){
	capture = cvCaptureFromCAM(0);

	if(!capture){
		printf("Capture failure\n");
		printf("CHANGE TO IMAGE MODE!\n");
		mode = 1;
		cvWaitKey(2000);
	}
	else{
		cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,320);
		cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT,240);
		cvSetCaptureProperty(capture,CV_CAP_PROP_FPS,30);
	}
 }

 IplImage* frame=0;
 setwindowSettings();

  //iterate through each frames of the video
 while(true){
  process = true;
  if (mode==0){
	frame = cvQueryFrame(capture);
	if(!frame){
		printf("Error capturing frame from camera!\n");
		process = false;
	}
	else
		frame=cvCloneImage(frame);
	}
  else{
	// optional load an image
	frame = cvLoadImage(filename,CV_LOAD_IMAGE_COLOR );
	if (!frame){
	  printf("Image not exists!\n");
	  process = false;
	}
  }

  
  if (process){
	  IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3); 
	  cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
  
	  IplImage* imgThresh = GetThresholdedImage(imgHSV);
  
	  cvShowImage("Object", imgThresh);
	  cvShowImage("Video", frame);

	   //Clean up used images
	  cvReleaseImage(&imgHSV);
	  cvReleaseImage(&imgThresh);
	  cvReleaseImage(&frame);
  }

   //Wait 80mS
  int c = cvWaitKey(80);
  //If 'ESC' is pressed, break the loop
  if((char)c==27 ) break;

  // if user press a number, open image[number].png 
  if((c>='0')&&(c<='9'))
	mode = 1; // file, not video
  if(c=='0') strcpy(filename,"image0.png");
  if(c=='1') strcpy(filename,"image1.png");
  if(c=='2') strcpy(filename,"image2.png");
  if(c=='3') strcpy(filename,"image3.png");
  if(c=='4') strcpy(filename,"image4.png");
  if(c=='5') strcpy(filename,"image5.png");
  if(c=='6') strcpy(filename,"image6.png");
  if(c=='7') strcpy(filename,"image7.png");
  if(c=='8') strcpy(filename,"image8.png");
  if(c=='9') strcpy(filename,"image9.png");

  if (c=='c'){
	  // Change to another camera
	  mode = 0;
	  camera_device++;
	  if (camera_device>2)
		  camera_device=0;
	  cvReleaseCapture(&capture);
	  capture = cvCaptureFromCAM(camera_device);

	if(!capture){
		printf("Capture failure\n");
		printf("CHANGE TO IMAGE MODE!\n");
		mode = 1;
	}
	else{
		cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,320);
		cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT,240);
		cvSetCaptureProperty(capture,CV_CAP_PROP_FPS,30);
	}
  }
  
 }

 cvDestroyAllWindows();
 cvReleaseCapture(&capture);

 return 0;
}
