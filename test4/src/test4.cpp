#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>

#include <iostream>

#define POINTS 7
#define PI 3.14159


using namespace std;
using namespace cv;
int red = 60;
int blue = 60;
int green = 50;

int red_t_max = 60;
int blue_t_max =68;
int green_t_max = 0;

int green_t_min=70;
int blue_t_min =0;
int red_t_min = 0;

Rect bRect;
int  bRect_height,bRect_width;
vector<vector<Point> > contours;
vector<vector<int> >hullI;
vector<vector<Point> >hullP;
vector<vector<Vec4i> > defects;
vector <Point> fingerTips;
bool isHand = false;
int nrOfDefects;
int cIdx;
void on_trackbar( int, void* )
{


}
float distanceP2P(Point a, Point b){
	float d= sqrt(fabs( pow((double)(a.x-b.x),2) + pow((double)(a.y-b.y),2) )) ;
	return d;
}

float getAngle(Point s, Point f, Point e){
	float l1 = distanceP2P(f,s);
	float l2 = distanceP2P(f,e);
	float dot=(s.x-f.x)*(e.x-f.x) + (s.y-f.y)*(e.y-f.y);
	float angle = acos(dot/(l1*l2));
	angle=angle*180/PI;
	return angle;
}


void removeRedundantEndPoints(vector<Vec4i> newDefects,Mat *m){
	Vec4i temp;
	float avgX, avgY;
	float tolerance=bRect_width/6;
	int startidx, endidx, faridx;
	int startidx2, endidx2;
	for(int i=0;i<newDefects.size();i++){
		for(int j=i;j<newDefects.size();j++){
	    	startidx=newDefects[i][0]; Point ptStart(contours[cIdx][startidx] );
	   		endidx=newDefects[i][1]; Point ptEnd(contours[cIdx][endidx] );
	    	startidx2=newDefects[j][0]; Point ptStart2(contours[cIdx][startidx2] );
	   		endidx2=newDefects[j][1]; Point ptEnd2(contours[cIdx][endidx2] );
			if(distanceP2P(ptStart,ptEnd2) < tolerance ){
				contours[cIdx][startidx]=ptEnd2;
				break;
			}if(distanceP2P(ptEnd,ptStart2) < tolerance ){
				contours[cIdx][startidx2]=ptEnd;
			}
		}
	}
}

void eleminateDefects(Mat *m){
	int tolerance =  bRect_height/5;
	float angleTol=80;
	vector<Vec4i> newDefects;
	int startidx, endidx, faridx;
	vector<Vec4i>::iterator d=defects[cIdx].begin();
	while( d!=defects[cIdx].end() ) {
   	    Vec4i& v=(*d);
	    startidx=v[0]; Point ptStart(contours[cIdx][startidx] );
   		endidx=v[1]; Point ptEnd(contours[cIdx][endidx] );
  	    faridx=v[2]; Point ptFar(contours[cIdx][faridx] );
		if(distanceP2P(ptStart, ptFar) > tolerance && distanceP2P(ptEnd, ptFar) > tolerance && getAngle(ptStart, ptFar, ptEnd  ) < angleTol ){
			if( ptEnd.y > (bRect.y + bRect.height -bRect.height/4 ) ){
			}else if( ptStart.y > (bRect.y + bRect.height -bRect.height/4 ) ){
			}else {
				newDefects.push_back(v);
			}
		}
		d++;
	}
	nrOfDefects=newDefects.size();
	defects[cIdx].swap(newDefects);
	removeRedundantEndPoints(defects[cIdx], m);
}
bool detectIfHand(){

	double h = bRect_height;
	double w = bRect_width;
	isHand=true;
	if(fingerTips.size() > 5 ){
		isHand=false;
	}else if(h==0 || w == 0){
		isHand=false;
	}else if(h/w > 4 || w/h >4){
		isHand=false;
	}else if(bRect.x<20){
		isHand=false;
	}
	return isHand;
}

void drawFingerTips(Mat m){
	Point p;
	int k=0;
	if(fingerTips.size() != 0) {
		putText(m,"Robot number: " + string(std::to_string((long long int) fingerTips.size())),Point(40,20),FONT_HERSHEY_PLAIN,1,Scalar(0,0,0));

	}
	for(int i=0;i<fingerTips.size();i++){
		p=fingerTips[i];
		putText(m,string(std::to_string((long long int)i)),p-Point(0,30),FONT_HERSHEY_PLAIN, 1.2f,Scalar(200,200,200),2);
   		circle( m,p,   5, Scalar(100,255,100), 4 );
   	 }
}
void checkForOneFinger(Mat m){
	int yTol=bRect.height/6;
	Point highestP;
	highestP.y=m.rows;
	vector<Point>::iterator d=contours[cIdx].begin();
	while( d!=contours[cIdx].end() ) {
   	    Point v=(*d);
		if(v.y<highestP.y){
			highestP=v;
			cout<<highestP.y<<endl;
		}
		d++;
	}int n=0;
	d=hullP[cIdx].begin();
	while( d!=hullP[cIdx].end() ) {
   	    Point v=(*d);
			cout<<"x " << v.x << " y "<<  v.y << " highestpY " << highestP.y<< "ytol "<<yTol<<endl;
		if(v.y<highestP.y+yTol && v.y!=highestP.y && v.x!=highestP.x){
			n++;
		}
		d++;
	}if(n==0){
		fingerTips.push_back(highestP);
	}
}

void getFingerTips(Mat m){
	fingerTips.clear();
	int i=0;
	vector<Vec4i>::iterator d=defects[cIdx].begin();
	while( d!=defects[cIdx].end() ) {
   	    Vec4i& v=(*d);
	    int startidx=v[0]; Point ptStart(contours[cIdx][startidx] );
   		int endidx=v[1]; Point ptEnd(contours[cIdx][endidx] );
  	    int faridx=v[2]; Point ptFar(contours[cIdx][faridx] );
		if(i==0){
			fingerTips.push_back(ptStart);
			i++;
		}
		fingerTips.push_back(ptEnd);
		d++;
		i++;
   	}
	if(fingerTips.size()==0){
		checkForOneFinger(m);
	}
}



int main(int argc, char* argv[])
{
	CvCapture* capture = 0;
	Mat frame, frameCopy, image;
	char c;
	vector <Rect> roi;
	VideoCapture cap(0); // open the default camera

	if (!cap.isOpened())  {
		cout << "Error:" << endl;
	}
		// check if we succeeded
	namedWindow("result");
	 char TrackbarName[50];
	sprintf( TrackbarName, "Red max %d", red_t_max );
	createTrackbar( TrackbarName, "result", &red_t_max, 70, on_trackbar );
	sprintf( TrackbarName, "Red min %d", red_t_min );
	createTrackbar( TrackbarName, "result", &red_t_min, 70, on_trackbar );

	sprintf( TrackbarName, "Blue max %d", blue_t_max );
	createTrackbar( TrackbarName, "result", &blue_t_max, 70, on_trackbar );
	sprintf( TrackbarName, "Blue min %d", blue_t_min );
		createTrackbar( TrackbarName, "result", &blue_t_min, 70, on_trackbar );

	sprintf( TrackbarName, "Green max %d", green_t_max );
	createTrackbar( TrackbarName, "result", &green_t_max, 70, on_trackbar );
	sprintf( TrackbarName, "Green min %d", green_t_min );
	createTrackbar( TrackbarName, "result", &green_t_min, 70, on_trackbar );

	int count = 0;
	bool filter = false;
	cv::Scalar total_mean[POINTS];
	 namedWindow("Frame");
	 namedWindow("Background");
	 BackgroundSubtractorMOG2 bg;
	 bg.set("nmixtures",3);
	 bg.set("detectShadows",false);
	// namedWindow("FG Mask MOG 2");


	//cap >> frame;

	//namedWindow("filtered");
	 int background = 500;
	 Mat fore;
	 Mat back;
	for (;;)
	{
		cap >> frame;
		if(background>0)
				{
			bg.operator ()(frame,fore);background--;
				}
				else
				{
					putText(frame,"Ready",Point(10,20),FONT_HERSHEY_PLAIN,1,Scalar(0,0,0));
					filter = true;

					bg.operator()(frame,fore,0);
				}
		bg.getBackgroundImage(back);
		erode(fore,fore,Mat());
				dilate(fore,fore,Mat());
		imshow("Frame",frame);
		imshow("Background",back);
		imshow("Forground",fore);
		//cout<< background << endl;
		/* pMOG->operator()(frame, fgMaskMOG);
			 pMOG2->operator()(frame, fgMaskMOG2);
		 imshow("Frame", frame);
		  imshow("FG Mask MOG", fgMaskMOG);
		  imshow("FG Mask MOG 2", fgMaskMOG2);*/
	//	flip(frame,frameCopy,1);
		//imshow("result", frame);
	//	imshow("result2", frameCopy);
		if (filter) {
			//Mat dst;


		/*	Scalar lowerBound=Scalar( total_mean[0][0] -red_t_min , total_mean[0][1] -green_t_min, total_mean[0][2] - blue_t_min );
			Scalar upperBound=Scalar( total_mean[0][0] + red_t_max , total_mean[0][1] + green_t_max, total_mean[0][2] + blue_t_max );
			Mat dst;
			inRange(frame, lowerBound, upperBound, dst);
			for(int i =1; i < POINTS; i ++) {
				Scalar lowerBound=Scalar( total_mean[i][0] - red_t_min , total_mean[i][1] - green_t_min, total_mean[i][2] - blue_t_min );
				Scalar upperBound=Scalar( total_mean[i][0] + red_t_max , total_mean[i][1] + green_t_max, total_mean[i][2] + blue_t_max );
				Mat tmp;
				inRange(frame, lowerBound, upperBound, tmp);
				dst+=tmp;
			}
			medianBlur(dst, dst,7);
			imshow("result2", dst);
			pyrUp(dst,dst);

*/
			findContours(fore,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
			hullI=vector<vector<int> >(contours.size());
			hullP=vector<vector<Point> >(contours.size());
			defects=vector<vector<Vec4i> > (contours.size());
			//cout << " We found: " << contours.size() << endl;
			int indexOfBiggestContour = -1;
			int sizeOfBiggestContour = 0;
			for (int i = 0; i < contours.size(); i++){
				if(contours[i].size() > sizeOfBiggestContour){
					sizeOfBiggestContour = contours[i].size();
					indexOfBiggestContour = i;
				}
			}
			if(sizeOfBiggestContour < 300)
				continue;
			cout << "Size: " << sizeOfBiggestContour << endl;
			cIdx = indexOfBiggestContour;
			 bRect = boundingRect(Mat(contours[indexOfBiggestContour]));
			 bRect_height=bRect.height;
			 	bRect_width=bRect.width;
			convexHull(Mat(contours[indexOfBiggestContour]),hullP[indexOfBiggestContour],false,true);
			convexHull(Mat(contours[indexOfBiggestContour]),hullI[indexOfBiggestContour],false,false);
			approxPolyDP( Mat(hullP[indexOfBiggestContour]), hullP[indexOfBiggestContour], 18, true );
			if(contours[indexOfBiggestContour].size()>3 ){

				convexityDefects(contours[indexOfBiggestContour],hullI[indexOfBiggestContour],defects[indexOfBiggestContour]);
			    eleminateDefects(&fore);
			}
			getFingerTips(frame);
			bool isHand=detectIfHand();
			rectangle(frame,bRect,cv::Scalar(200,0,0));
			drawContours(frame,hullP,cIdx,cv::Scalar(200,0,0),2, 8, vector<Vec4i>(), 0, Point());
					if(isHand){
					//	getFingerTips(frame);
						drawFingerTips(frame);
					//	myDrawContours(frame);
					}

		}
		/*Scalar color = Scalar(0, 255, 0);
		roi.push_back(Rect(Point(frame.cols / 3, frame.rows / 6), Point(frame.cols / 3 + 20, frame.rows / 6 + 20)));
		roi.push_back(Rect(Point(frame.cols / 4, frame.rows / 2), Point(frame.cols / 4 + 20, frame.rows / 2 + 20)));
		roi.push_back(Rect(Point(frame.cols / 3, frame.rows / 1.5), Point(frame.cols / 3 + 20, frame.rows / 1.5 + 20)));
		roi.push_back(Rect(Point(frame.cols / 2, frame.rows / 2), Point(frame.cols / 2 + 20, frame.rows / 2 + 20)));
		roi.push_back(Rect(Point(frame.cols / 2.5, frame.rows / 2.5), Point(frame.cols / 2.5 + 20, frame.rows / 2.5 + 20)));
		roi.push_back(Rect(Point(frame.cols / 2, frame.rows / 1.5), Point(frame.cols / 2 + 20, frame.rows / 1.5 + 20)));
		roi.push_back(Rect(Point(frame.cols / 2.5, frame.rows / 1.8), Point(frame.cols / 2.5 + 20, frame.rows / 1.8 + 20)));
		rectangle(frame, roi[0], color, 2);
		rectangle(frame,roi[1] , color, 2);
		rectangle(frame, roi[2], color, 2);
		rectangle(frame, roi[3], color, 2);
		rectangle(frame, roi[4], color, 2);
		rectangle(frame, roi[5], color, 2);
		rectangle(frame, roi[6], color, 2);*/

		std::string win = "majd";
		imshow(win, frame);
		//imshow("faaa",frame);

		/*roi.push_back(My_ROI(Point(m->src.cols / 3, m->src.rows / 6), Point(m->src.cols / 3 + square_len, m->src.rows / 6 + square_len), m->src));
		roi.push_back(My_ROI(Point(m->src.cols / 4, m->src.rows / 2), Point(m->src.cols / 4 + square_len, m->src.rows / 2 + square_len), m->src));
		roi.push_back(My_ROI(Point(m->src.cols / 3, m->src.rows / 1.5), Point(m->src.cols / 3 + square_len, m->src.rows / 1.5 + square_len), m->src));
		roi.push_back(My_ROI(Point(m->src.cols / 2, m->src.rows / 2), Point(m->src.cols / 2 + square_len, m->src.rows / 2 + square_len), m->src));
		roi.push_back(My_ROI(Point(m->src.cols / 2.5, m->src.rows / 2.5), Point(m->src.cols / 2.5 + square_len, m->src.rows / 2.5 + square_len), m->src));
		roi.push_back(My_ROI(Point(m->src.cols / 2, m->src.rows / 1.5), Point(m->src.cols / 2 + square_len, m->src.rows / 1.5 + square_len), m->src));
		roi.push_back(My_ROI(Point(m->src.cols / 2.5, m->src.rows / 1.8), Point(m->src.cols / 2.5 + square_len, m->src.rows / 1.8 + square_len), m->src));
		*/
		c = waitKey(10);
		if (c == 'q') {
			return 0;
		}

	//	c = 'a';
		if (c == 'a') {
			//Mat tst = frame(roi[0]);
			//total_mean = cv::mean(tst);;


		   filter = true;


		}
	}
	return 0;
}

