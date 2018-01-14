#include "opencv2/objdetect.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/utility.hpp"

#include "opencv2/videoio/videoio_c.h"
#include "opencv2/highgui/highgui_c.h"

#include <librealsense/rs.hpp>

#include <cctype>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <time.h>

// included for usb serial access to Arduino
#include <stdlib.h>
#include <unistd.h>
#include "rs232.h"
#define BUF_SIZE 128

using namespace std;
using namespace cv;

// queue vars
const int queLen = 13;  // queue length
int queX[queLen];	// holds x target values
long queX_t[queLen];	// holds x value timestamps 
int queY[queLen];	// holds y target values
long queY_t[queLen];	// holds y value timestampes
int queZ[queLen];	// holds depth values
long queZ_t[queLen];	// holds depth value timestamps
int evict_t = 5; 	// Degan sayz how long (seconds)till gotta go

//global setting vars
bool show_cam = true;	//show the RS video output
int curr_x, curr_y; 	//holds x,y values after filtering 
int curr_z;	//holds our depth z val
int prev_x, prev_y; 	//holds previous x,y values 
uint16_t prev_z;	//holds our previous depth z val
long mash;		//x+y+z holds value sum to reduce repeats 
int depthLine = 420;	// y axis line where we read depth
time_t lastLumChk =0;	// last time we checked the avg env brightness 
bool depth_anti_lock= true;	//clear depth vals when no face present "DAL" 
int dal_wait = 20; 	// time (sec.) after no faces to implement DAL
time_t last_face_t =0;	// time stamp last face evicted for DAL

//usb serial vars
int cport_nr=1; /* /dev/ttyUSB0  port 24 , /dev/ttyS1 1 */
int bdrate=115200; /* 115200 baud */
char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit
char str_send[BUF_SIZE]; //holds our serial str

// prototypes
void QueuePush(int inval,int inQueue[]);
int QueuePop(int inQueue[]);
int QueueLength(int inQueue[]);
void LQueuePush(long inval,long inQueue[]);
long LQueuePop(long inQueue[]);
int LQueueLength(long inQueue[]);
long LQueuePeek(long inQueue[]);
float median(int n, int x[]);
bool evictQueue(int inQueue[], long inLQueue[]);
void queuePrint(int inQueue[]);
void LQueuePrint(long inQueue[]);
void serialStrOut();
int avgLum(Mat& ir);
int avgLumColor(Mat& colorMat);

void detectAndDraw( Mat& img, CascadeClassifier& cascade,
                    CascadeClassifier& nestedCascade,
                    double scale, bool tryflip );

string cascadeName = "haarcascade_frontalface_alt2.xml";  
string nestedCascadeName = "haarcascade_lefteye_2splits.xml";//"haarcascade_eye_tree_eyeglasses.xml";

int main( int argc, const char** argv )
{
    CvCapture* capture = 0;
    Mat frame, frameCopy, image;

    bool tryflip = false;

  for( int i = 1; i < argc; i++ )
  {
	if(i == 1){ 
		std::string arg1(argv[i]);
		if(arg1.compare("show=false")== 0){
		show_cam = false;
		cout << "RS Video out disabled" << endl;
		}
	}
	if(i == 2){ 
		std::string arg2(argv[i]);
		if(arg2.compare("dal=false")== 0){
		depth_anti_lock = false;
		cout << "DAL deactivated" << endl;
		}
	}
  }

    CascadeClassifier cascade, nestedCascade;
    double scale = 1;

     // open up the port to the Arduino
    if(RS232_OpenComport(cport_nr, bdrate, mode))
    {
        printf("Can not open comport\n");
        return(0);
    }
     /*
    RS232_enableDTR(cport_nr);
    // send motor home command to arduino
    RS232_cputs(cport_nr, "h");
	sleep(10);
	*/
    // Create a context object. This object owns the handles to all connected realsense devices
    rs::context ctx;
	
    cout << "[ INFO ] connected " << ctx.get_device_count() 
        << "realsense deveces" << endl;
    if (ctx.get_device_count() == 0) {
        cout << "[ ERROR ] No devece is connected" << endl;
        return 0;
    }

    // Access the first available RealSense device
    rs::device * dev = ctx.get_device(0);

    // Configure Infrared stream to run at VGA resolution at 30 frames per second
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 30);
    dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 30); // FOR IR
    // depth stream 
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30  );
    
    // Start streaming
    dev->start();

	//dev->set_option(rs::option::r200_lr_gain, 400);
	int ir_value = dev->get_option(rs::option::r200_lr_gain);
	cout << "emitter gain " << ir_value << endl;

	//dev->set_option(rs::option::r200_lr_exposure, 127);
	int exp_value = dev->get_option(rs::option::r200_lr_exposure);
	cout << "exposure  " << exp_value << endl;



    // Determine depth value corresponding to one meter
    const uint16_t one_meter = static_cast<uint16_t>(1.0f / dev->get_depth_scale());

    if( !cascade.load( cascadeName ) ){
	cerr << "ERROR: Could not load classifier cascade" << endl;
        return -1;
    }

  //  cvNamedWindow( "result", 1 );

     if( 1 ){
        cout << "In capture ..." << endl;
        for(;;)
        {
		
		// Camera warmup - Dropped several first frames to let auto-exposure stabilize
    		for(int i = 0; i < 1; i++)
       			dev->wait_for_frames();

		const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);


    		// Creating OpenCV Matrix from a color image
    		Mat color(Size(640, 480), CV_8UC3, (void*)dev->get_frame_data(rs::stream::color), Mat::AUTO_STEP);
	 	//Mat ir(Size(640, 480), CV_8UC1, (void*)dev->get_frame_data(rs::stream::infrared), Mat::AUTO_STEP); 
		// retrieve camera parameters for mapping between depth and color
		rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
		


      
	detectAndDraw( color, cascade, nestedCascade, scale, tryflip );
	   
	    uint16_t depth_value =0;
	    time_t current_time;
    	    current_time = time(NULL);

	    if(current_time > lastLumChk +300){
		//dev->set_option(rs::option::r200_emitter_enabled, 0);
		printf("avg lum: %i \n", avgLumColor(color));
		//dev->set_option(rs::option::r200_emitter_enabled, 1);
	    }
	    if(depth_anti_lock){
	    	if((last_face_t + dal_wait) > current_time){
			memset(queZ, 0, sizeof(queZ)); 
			memset(queZ_t, 0, sizeof(queZ_t));
	    	}
	    }
		
	    int qlx = QueueLength(queX);
	    int qlz = QueueLength(queZ);

	    if((qlx < 4) && (qlz < 5)){
		if(current_time % 2 == 0){
			curr_x = 320; curr_y = 420; curr_z = 3500;
			long newmash = curr_x + curr_y + curr_z;
			if((newmash > (5 + mash)) || (newmash < (mash - 5))){
				serialStrOut(); 
				mash = newmash;
	    		}
	     	}
	     }

	    else if (qlx > 4 || qlz > 4){

				
		uint16_t low_val =3500;
		long low_indx =2;
		for(int i =2;i<638;i++){
			uint16_t pix = depth_image[depthLine * depth_intrin.width + i];
			uint16_t pixL = depth_image[depthLine * depth_intrin.width + (i-1)];
			uint16_t pixR = depth_image[depthLine * depth_intrin.width + (i+1)];
			if((pix < low_val) && (pix > 600 )){
				if((pixL < (pix+20)) && (pixL > (pix-20))){
					if((pixR < (pix+20)) && (pixR > (pix-20))){
						low_val = pix; low_indx =i;
					}
				}
				
			}
		}
		uint16_t lowest_val = depth_image[depthLine * depth_intrin.width + low_indx];
/*
		printf("%u   %u   %u \n",depth_image[depthLine * depth_intrin.width + (low_indx-1)], lowest_val, depth_image[depthLine * depth_intrin.width + (low_indx-1)]);
		int lowest_neighbor_count =1;
		for(int i =1;i<638;i++){
			if((depth_image[depthLine * depth_intrin.width + i] < lowest_val + 100) && 
		   	   (depth_image[depthLine * depth_intrin.width + i] > lowest_val -100 )){
				lowest_neighbor_count++;
			}
		}

*/
		depth_value = lowest_val;
		//printf("low value: %u \n", lowest_val);
		

		if(depth_value > 0){
			if(depth_value > 3500){depth_value = 3500;}
			else if(depth_value < 650){depth_value = 650;}
			QueuePush((int)depth_value, queZ); LQueuePush(current_time, queZ_t); // load Z value
	    	}
		
		if(QueueLength(queZ) > 6){curr_z = median(QueueLength(queZ),queZ);}
		else{curr_z = 3500;}
		//printf(" %i \n", curr_z);
	    	if(current_time % 2 == 0){
			long newmash = curr_x + curr_y + curr_z;
			if((newmash > (5 + mash)) || (newmash < (mash - 5))){
				serialStrOut(); 
				mash = newmash;
	    		}


	     	}
/* 
	     else if(
		if(curr_x < depth_intrin.width && curr_y < depth_intrin.height){
		depth_value = depth_image[curr_y * depth_intrin.width + curr_x];
	        }
*/
	    
	   
	    
	    

	    prev_x = curr_x; prev_y = curr_y; prev_z = prev_z;
	    }  // end of else


            if( waitKey( 10 ) >= 0 ){
                goto _cleanup_;
	     }
        }

        waitKey(0);

_cleanup_:
        cvReleaseCapture( &capture );
    }
    

  //  cvDestroyWindow("result");

    return 0;
}	// end main() 

void detectAndDraw( Mat& img, CascadeClassifier& cascade,
                    CascadeClassifier& nestedCascade,
                    double scale, bool tryflip )
{
    int i = 0;
    double t = 0;
    vector<Rect> faces, faces2;
    const static Scalar colors[] =  { CV_RGB(0,0,255),
        CV_RGB(0,128,255),
        CV_RGB(0,255,255),
        CV_RGB(0,255,0),
        CV_RGB(255,128,0),
        CV_RGB(255,255,0),
        CV_RGB(255,0,0),
        CV_RGB(255,0,255)} ;
    Mat gray, smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );

	// gray =img.clone();  // FOR IR

    cvtColor( img, gray, COLOR_BGR2GRAY );  // comment out FOR IR
    resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
    equalizeHist( smallImg, smallImg );

    t = (double)cvGetTickCount();
    cascade.detectMultiScale( smallImg, faces,
        1.3, 4, 0
        //|CASCADE_FIND_BIGGEST_OBJECT
        //|CASCADE_DO_ROUGH_SEARCH
        |CASCADE_SCALE_IMAGE
        ,
        Size(30, 30) );
    
    t = (double)cvGetTickCount() - t;
    //printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );
    for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
    {
        Mat smallImgROI;
        vector<Rect> nestedObjects;
        Point center;
        Scalar color = colors[i%8];
        int radius;

        double aspect_ratio = (double)r->width/r->height;
        if( 0.75 < aspect_ratio && aspect_ratio < 1.3 )
        {
            center.x = cvRound((r->x + r->width*0.5)*scale);
            center.y = cvRound((r->y + r->height*0.5)*scale);
            radius = cvRound((r->width + r->height)*0.25*scale);
            circle( img, center, radius, color, 3, 8, 0 );
	   // printf("circle x,y: %d , %d\n",center.x,center.y);
	    time_t current_time;
    	    current_time = time(NULL);
	    QueuePush(center.x, queX); LQueuePush(current_time, queX_t); last_face_t =0; // load x values
	    QueuePush(center.y, queY); LQueuePush(current_time, queY_t); // load y values
        }
        else
            rectangle( img, cvPoint(cvRound(r->x*scale), cvRound(r->y*scale)),
                       cvPoint(cvRound((r->x + r->width-1)*scale), cvRound((r->y + r->height-1)*scale)),
                       color, 3, 8, 0);
        if( nestedCascade.empty() ){
	//printf("inner cascade empty");
            continue;
}
        smallImgROI = smallImg(*r);
        nestedCascade.detectMultiScale( smallImgROI, nestedObjects,
            1.1, 2, 0
            //|CASCADE_FIND_BIGGEST_OBJECT
            //|CASCADE_DO_ROUGH_SEARCH
            //|CASCADE_DO_CANNY_PRUNING
            |CASCADE_SCALE_IMAGE
            ,
            Size(30, 30) );
        for( vector<Rect>::const_iterator nr = nestedObjects.begin(); nr != nestedObjects.end(); nr++ )
        {
            center.x = cvRound((r->x + nr->x + nr->width*0.5)*scale);
            center.y = cvRound((r->y + nr->y + nr->height*0.5)*scale);
            radius = cvRound((nr->width + nr->height)*0.25*scale);
            circle( img, center, radius, color, 3, 8, 0 );
		printf("inner cascade target");
        }
    }
    line(img, cvPoint(0,depthLine),cvPoint(640,depthLine),colors[6]);
    if(show_cam == true ){cv::imshow( "result", img );}
    bool booted = evictQueue(queX, queX_t);
    if(booted && (QueueLength(queX)==0)){last_face_t  = time(NULL); }
    evictQueue(queY, queY_t);
    evictQueue(queZ, queZ_t);
    queuePrint(queZ);

   // LQueuePrint(queX_t);
    if(QueueLength(queX) > 5){
	curr_x = (int)median(QueueLength(queX),queX); curr_y = (int)median(QueueLength(queY),queY);
	if(curr_x > 640){curr_x = 640;} if(curr_y > 480){curr_y = 480;}
	//printf("x: %i   y: %i \n", curr_x,curr_y); 
    }
}  // end detectAndDraw() 

// helper func *********************************************************************
int avgLumIr(Mat& ir){		// finds avg brightness from IR stream
	time_t current_time;
    	current_time = time(NULL);
	double sum=0;
	uint8_t *myData = ir.data;
	int cols = ir.cols, rows = ir.rows;
	int _stride = ir.step;
	int tot_points =0;
	for(int i=0;i < rows; i +=10) {
		for(int j=0; j < cols; j +=10){
			sum += myData[i * _stride + j];
			tot_points ++;
		}
	}
	int avg = sum/tot_points;
	lastLumChk = current_time;
	return avg;
}
int avgLumColor(Mat& colorMat){
	Mat gray;
	cvtColor( colorMat, gray, COLOR_BGR2GRAY );
	time_t current_time;
    	current_time = time(NULL);
	double sum=0;
	uint8_t *myData = gray.data;
	int cols = gray.cols, rows = gray.rows;
	int _stride = gray.step;
	int tot_points =0;
	for(int i=0;i < rows; i +=10) {
		for(int j=0; j < cols; j +=10){
			sum += myData[i * _stride + j];
			tot_points ++;
		}
	}
	int avg = sum/tot_points;
	lastLumChk = current_time;
	return avg;
}
void serialStrOut(){ 	//builds the string we will send to Arduino
	if( curr_z > 0){
	char strOut[32] = "";
	sprintf(strOut,"g%i/%u/%i",curr_x, curr_z, curr_y);
	
	RS232_cputs(cport_nr, strOut); // sends string on serial
	//RS232_flushTX(cport_nr);
	//sleep(1);
	printf("%s\n", strOut);
	}
	//strcpy(str_send, "h");
}

// mediam function ***********************************************

    float median(int n, int xIn[]) {
	int x[n];
	std::fill_n(x,n,0);

 	memcpy(x,xIn,sizeof(x));//memcpy(x,xIn,sizeof(xIn));
	
        float temp;
        int i, j;
        // the following two loops sort the array x in ascending order
        for(i=0; i<n-1; i++) {
            for(j=i+1; j<n; j++) {
                if(x[j] < x[i]) {
                    // swap elements
                    temp = x[i];
                    x[i] = x[j];
                    x[j] = temp;
                }
            }
        }
        
        if(n%2==0) {
            // if there is an even number of elements, return mean of the two elements in the middle
            return((x[n/2] + x[n/2 - 1]) / 2.0);
        } else {
            // else return the element in the middle
            return x[n/2];
        }
    }  // end median()



bool evictQueue(int inQueue[], long inLQueue[]){  //don't pay rent .. . . you gotago 
	if(queLen > 0){
	   time_t current_time;
    	   current_time = time(NULL);
	   if((LQueuePeek(inLQueue) + evict_t) < current_time){ 
		QueuePop(inQueue); 
		LQueuePop(inLQueue);
		return true;
	   }
	}
	return false; 
	//printf("%li < %li \n", LQueuePeek(inLQueue), (current_time + evict_t));
}

void QueuePush(int inval,int inQueue[]){
    for(int i=queLen-1;i>-1;i--){
        inQueue[i] = inQueue[i-1];
    }
    inQueue[0]= inval;
}

int QueuePop(int inQueue[]){ // removes first in queue
    int out = 0;
    if (QueueLength(inQueue) > 0){
        for(int i=queLen-1;i>-1;i--){
            if (inQueue[i] >0){
                out = inQueue[i];
                inQueue[i] = 0;
                return out;
            }
        }
    }
    return out;
} // end queuePop

int QueueLength(int inQueue[]){
    int val = 0;
    for(int i=0;i<queLen;i++){
        if (inQueue[i] >0){ val++;}
    }
    return val;
}
// long queue functions *******************************

void LQueuePush(long inval,long inQueue[]){
    for(int i=queLen-1;i>-1;i--){
        inQueue[i] = inQueue[i-1];
    }
    inQueue[0]= inval;
}

long LQueuePop(long inQueue[]){ // removes first in queue
    long out = 0;
    if (LQueueLength(inQueue) > 0){
        for(int i=queLen-1;i>-1;i--){
            if (inQueue[i] >0){
                out = inQueue[i];
                inQueue[i] = 0;
                return out;
            }
        }
    }
    return out;
}

int LQueueLength(long inQueue[]){
    int val = 0;
    for(int i=0;i<queLen;i++){
        if (inQueue[i] >0){ val++;}
    }
        return val;
}

long LQueuePeek(long inQueue[]){  // returns oldest value
    return inQueue[(LQueueLength(inQueue)-1)];
}

void queuePrint(int inQueue[]){
    printf("[");
    for(int i=0;i<queLen;i++){ printf("%i, ", inQueue[i]);}
    printf("]\n");
}

void LQueuePrint(long inQueue[]){
    printf("[");
    for(int i=0;i<queLen;i++){ printf("%li, ", inQueue[i]);}
    printf("[\n");
}
