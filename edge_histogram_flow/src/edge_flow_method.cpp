#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include "divergence.h"



using namespace cv;



int main(int, char**)
{
	//Capture the webcam device
	cv::VideoCapture cap(0);
	cv::Mat frame;
	Mat vec,vec2;
	Size size(IMAGE_WIDTH,IMAGE_HEIGHT);

	// Define image buffers
	unsigned char  image_buffer[192*128];
	unsigned char *  image_buffer_p=image_buffer;
	unsigned char  image_edge_buffer[192*128];
	unsigned char *  image_edge_buffer_p=image_edge_buffer;
	cv::Mat  frame_processed;
	unsigned char *  frame_processed_buffer_p;
	unsigned char * frame_buffer_p=frame.data;

	//Define arrays and pointers for edge histogram and displacements
	struct displacement_t displacement;
	displacement.horizontal[IMAGE_WIDTH];
	displacement.vertical[IMAGE_HEIGHT];

	//Initializing the dynamic parameters and the edge histogram structure
	int rear=1, front=0;

	//Intializing edge histogram structure
	struct edge_hist_t* edge_hist;
	edge_hist=(struct edge_hist_t*)calloc(MAX_HORIZON,sizeof(struct edge_hist_t));

	struct edge_flow_t* edge_flow_plot;
	edge_flow_plot=(struct edge_flow_t*)calloc(IMAGE_WIDTH,sizeof(struct edge_flow_t));
	int rear_plot=1,front_plot=0;



	// For the displacment calculation
	int window_size=10,max_distance=50,edge_thres=50;

	float coveriance_x=0;
	float coveriance_y=0;

	struct edge_flow_t prev_edge_flow;

	prev_edge_flow.horizontal[0]=0.0;
	prev_edge_flow.horizontal[1]=0.0;
	prev_edge_flow.vertical[0]=0.0;
	prev_edge_flow.vertical[1]=0.0;

	//Initializing for divergence and flow parameters
	struct edge_flow_t edge_flow;
	edge_flow.horizontal[0]=0.0;
	edge_flow.horizontal[1]=0.0;
	edge_flow.vertical[0]=0.0;
	edge_flow.vertical[1]=0.0;

	cv::namedWindow("video");
	cvMoveWindow("video", 0, 0);

	//While camera is rolling
	while ( cap.isOpened() )
	{

		//capture and resize frame
		cap >> frame;
		if(frame.empty()) break;
		cv::cvtColor(frame,frame,CV_BGR2GRAY);
		cv::resize(frame,frame,size);
		frame_buffer_p=frame.data;
		memcpy(image_buffer_p,frame_buffer_p,192*128);

		// Calculate the edge histogram displacemen
		int median_features;
		median_features=calculate_edge_flow(image_buffer_p,image_edge_buffer_p,&displacement,&edge_flow,edge_hist,front,rear,window_size,max_distance,edge_thres,IMAGE_WIDTH,IMAGE_HEIGHT);

		plot_edge_histogram(edge_hist[front].horizontal,edge_hist[rear].horizontal,displacement.horizontal,edge_flow.horizontal[0],edge_flow.horizontal[1],IMAGE_WIDTH);

		//Show the results and edge video
		frame.copyTo(frame_processed);
		frame_processed_buffer_p=frame_processed.data;
		memcpy(frame_processed_buffer_p,image_edge_buffer_p,IMAGE_WIDTH*IMAGE_HEIGHT);
		//plot_edge_histogram(edge_histogram_y_p,prev_edge_histogram_y_p,displacement_y_p,slope_y,trans_y,IMAGE_HEIGHT);

		cv::namedWindow("video",CV_WINDOW_NORMAL );
		cv::imshow("video", frame_processed);


		// Kalman Filtering
		float Q=0.01;
		float R=1;
		float new_est_x,new_est_y;
		new_est_x=simpleKalmanFilter(&coveriance_x,prev_edge_flow.horizontal[1],edge_flow.horizontal[1],Q,R);
		new_est_y=simpleKalmanFilter(&coveriance_y,prev_edge_flow.vertical[1],edge_flow.vertical[1],Q,R);

		edge_flow.horizontal[1]=new_est_x;
		edge_flow.vertical[1]=new_est_y;



		// Move the dynamic indices and make them circular
		front++;
		rear++;

		if(front>MAX_HORIZON-1)
			front=0;
		if(rear>MAX_HORIZON-1)
			rear=0;

		memcpy(edge_flow_plot[front_plot].horizontal,&edge_flow.horizontal,2*sizeof(float));
		memcpy(edge_flow_plot[front_plot].vertical,&edge_flow.vertical,2*sizeof(float));

		plot_flow(edge_flow_plot,front_plot,rear_plot);

		// For the plot: Move the dynamic indices and make them circular
		front_plot++;
		rear_plot++;

		if(front_plot>IMAGE_WIDTH-1)
			front_plot=0;
		if(rear_plot>IMAGE_WIDTH-1)
			rear_plot=0;

		memcpy(&prev_edge_flow,&edge_flow,4*sizeof(float));


		if(cv::waitKey(50) >= 0) break;
	}
	return 0;
}



