#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include <sstream>
#include <string>

using namespace std;
using namespace cv;

const int canny_low_max = 200;
int canny_slider_low = 10;
int canny_slider_high = 50;

const int canny_high_max = 200;
int canny_low = canny_slider_low;
int canny_high = canny_slider_high;


const int kernel_max = 20;
int kernel_slider = 5;
int kernel_value = kernel_slider;

const int threshold_max = 300;
int threshold_slider = 80;
int threshold_value = threshold_slider;

const int min_line_length_max = 300;
int min_line_length_slider = 83;
int min_line_length = min_line_length_slider;

const int max_line_gap_max = 200;
int max_line_gap_slider = 18;
int max_line_gap = max_line_gap_slider;


void on_canny_low_trackbar( int, void* )
{
	canny_low = canny_slider_low;
}

void on_canny_high_trackbar( int, void* )
{
	canny_high = canny_slider_high;
}

void on_kernel_trackbar( int, void* )
{
	kernel_value = kernel_slider;
}

void on_threshold_trackbar( int, void* )
{
	threshold_value = threshold_slider;
}

void on_min_line_length_trackbar( int, void* )
{
	min_line_length = min_line_length_slider;
}

void on_max_line_gap_trackbar( int, void* )
{
	max_line_gap = max_line_gap_slider;
}

class cam_pipe{
	public:

		cam_pipe(){

		}

		VideoCapture get_cap(){
			VideoCapture cap(0);//"real_pipe1.mp4");//"GOPR1142.avi");//0);//"rtsp://10.42.0.126/z3-2.mp4");

			if (!cap.isOpened())
			{
				cout << "Cannot open the video cam" << endl;
			}

			double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
			double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

			cout << "frame size : " << dWidth << " x "<< dHeight << endl;

			namedWindow("frame", CV_WINDOW_NORMAL);
			resizeWindow("frame", 1200, 1000);

			return cap;
		}
		void add_trackbar(){
			// Adding trackbar
			char CannyLow[50];
		 	sprintf( CannyLow, "Canny low x %d", canny_low_max );

			createTrackbar( CannyLow, "frame", &canny_slider_low, canny_low_max, on_canny_low_trackbar );

			char CannyHigh[50];
		 	sprintf( CannyHigh, "Canny high x %d", canny_high_max );

			createTrackbar( CannyHigh, "frame", &canny_slider_high, canny_high_max, on_canny_high_trackbar );

			char Kernel[50];
		 	sprintf( Kernel, "Kernel x %d", kernel_value );

			createTrackbar( Kernel, "frame", &kernel_slider, kernel_max, on_kernel_trackbar );

			char Threshold[50];
		 	sprintf( Threshold, "Threshold x %d", threshold_value );

			createTrackbar( Threshold, "frame", &threshold_slider, threshold_max, on_threshold_trackbar );

			char Min_line_length[50];
		 	sprintf( Min_line_length, "Min_line_length x %d", min_line_length );

			createTrackbar( Min_line_length, "frame", &min_line_length_slider, min_line_length_max, on_min_line_length_trackbar );

			char Max_line_gap[50];
		 	sprintf( Max_line_gap, "Max_line_gap x %d", max_line_gap );

			createTrackbar( Max_line_gap, "frame", &max_line_gap_slider, max_line_gap_max, on_max_line_gap_trackbar );
		}


		Mat read(VideoCapture cap){
			Mat frame;

			bool bSuccess = cap.read(frame);

			if (!bSuccess)
			{
				cout << "Cannot read a frame from video stream" << endl;
			}
			return frame;
		}

		void showFrame(Mat frame){
			imshow("frame", frame);
		}

		string get_string(int i){
			stringstream i_ss;
			i_ss << i;
			string i_string = i_ss.str();
			return i_string;
		}

		void saveFrame(Mat frame, Mat original, int i){
			imwrite("images/analyzed_"+get_string(i)+"_"
					+"canL_"+get_string(canny_low)+"_"
					+"canH_"+get_string(canny_high)+"_"
					+"thresh_"+get_string(threshold_value)+"_"
					+"minLine_"+get_string(min_line_length)+"_"
					+"maxGap_"+get_string(max_line_gap)+"_"
					+".jpg",frame);
			imwrite("images/original_"+get_string(i)+"_"
					+"canL_"+get_string(canny_low)+"_"
					+"canH_"+get_string(canny_high)+"_"
					+"thresh_"+get_string(threshold_value)+"_"
					+"minLine_"+get_string(min_line_length)+"_"
					+"maxGap_"+get_string(max_line_gap)+"_"
					+".jpg",original);
		}

		Mat blur(Mat frame){
			GaussianBlur(frame, frame, Size(9,9),0,0);
			return frame;
		}

		Mat detect_edges(Mat frame){
			Mat edgy;
			Canny(frame,edgy,canny_low,canny_high,3);
			return edgy;
		}

		Mat dilate_erode(Mat frame){
			int an = kernel_value;
			int element_shape = MORPH_RECT;
			Mat element = getStructuringElement(element_shape, Size(an*2+1, an*2+1), Point(an, an) );
			dilate(frame, frame, element );
			erode(frame, frame, element );
			return frame;
		}

		vector<Vec4i> find_lines(Mat frame){
			vector<Vec4i> lines;
			HoughLinesP(frame, lines, 1, CV_PI/180, threshold_value, min_line_length, max_line_gap );
			return lines;
		}

		vector<Vec4i> remove_border_lines(vector<Vec4i> lines, VideoCapture cap){
			vector<Vec4i> lines_passed;
			double width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
			double height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
			for( size_t i = 0; i < lines.size(); i++ )
			{
				Vec4i l = lines[i];
				if(l[0] < width - 120 && l[0] > 120){
					if(l[1] < height - 120 && l[1] > 120){
						lines_passed.push_back(l);
					}
				}

			}
			return lines_passed;
		}

		vector<Vec4i> sort_lines(vector<Vec4i> lines){
			vector<Vec4i> lines_passed;
			double avg_angle = 0;

			for( size_t i = 0; i < lines.size(); i++ )
			{
				Vec4i l = lines[i];
				double delta_u = l[2]-l[0];
				double delta_v = l[3]-l[1];
				avg_angle += atan2(delta_v, delta_u)*180/3.14159265;

				lines_passed.push_back(l);
			}
			if(lines.size() > 0){
				avg_angle = avg_angle / lines.size();
				cout << avg_angle << endl;
			}
			return lines_passed;
		}

		Mat drawLines(Mat frame, vector<Vec4i> lines){
			cvtColor(frame, frame, CV_GRAY2BGR);
			for( size_t i = 0; i < lines.size(); i++ )
			{
				Vec4i l = lines[i];
				line( frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
			}
			return frame;
		}

		~cam_pipe()
		{
			cvDestroyWindow("Camera_Output");
		}
};

int main(int argc, char **argv)
{
	int i = 1;
	cam_pipe cam_object;
	VideoCapture cap = cam_object.get_cap();
	cam_object.add_trackbar();
	while(1){
		Mat frame = cam_object.read(cap);
		Mat original;
		vector<Vec4i> lines;
		frame.copyTo(original);
		frame = cam_object.blur(frame);
		frame = cam_object.detect_edges(frame);
		frame = cam_object.dilate_erode(frame);

		lines = cam_object.find_lines(frame);
		lines = cam_object.remove_border_lines(lines, cap);
		lines = cam_object.sort_lines(lines);

		frame = cam_object.drawLines(frame, lines);

		cam_object.showFrame(frame);
		int c = waitKey(30);
		if (c == 115) //Press "s" to save
		{
			cam_object.saveFrame(frame, original, i);
			i += 1;
			cout << "Saved images" << endl;
		} else if (c == 27)
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}
	return 0;
}