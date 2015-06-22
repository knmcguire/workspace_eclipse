/*
 * divergence.h
 *
 *  Created on: Apr 28, 2015
 *      Author: knmcguire
 */

#ifndef DIVERGENCE_H_
#define DIVERGENCE_H_


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#define IMAGE_WIDTH 192
#define IMAGE_HEIGHT 128
#define MAX_HORIZON 10
#define MAX_FLOW 1.0
#define RANSAC 1
#define ADAPTIVE_THRES_EDGE 1


struct edge_hist_t{
	int  horizontal[IMAGE_WIDTH];
	int  vertical[IMAGE_HEIGHT];
};

struct edge_flow_t{
	float horizontal[2];
	float vertical[2];
};

struct displacement_t{
	int  horizontal[IMAGE_WIDTH];
	int  vertical[IMAGE_HEIGHT];
};

//Main functions
int calculate_edge_flow(unsigned char* in,unsigned char* out, struct displacement_t* displacement,struct edge_flow_t* edge_flow, struct edge_hist_t* edge_hist,int front,int rear,int windowsize,int max_distance,int edge_threshold,int image_width,int image_height);
void calculate_edge_histogram(unsigned char * in,unsigned char * out,int * edge_histogram, int thres,int image_width,int image_height,char direction);
void calculate_displacement(int * edge_histogram,int * edge_histogram_prev,int * displacement,int prev_frame_number,int windowsize,int max_distance,int size);

//Sub functions
void line_fit_RANSAC( int* displacement, float* Slope, float* Yint,int size);
void line_fit(int* displacement, float* Slope, float* Yint,int size);
float simpleKalmanFilter(float* cov,float previous_est, float current_meas,float Q,float R);

//Staticstical Calculations
int getMinimum(int* flow_error, int max_ind);
int getMinimumMiddle(int * flow_error, int  max_ind);
int getMaximum(int a[], int n);
int GetMedian(int* daArray, int iSize);
int GetMean(int* daArray, int iSize);

//Vizualizations
void plot_flow(struct edge_flow_t * edge_flow_plot,int front,int rear);
void plot_edge_histogram(int* edge_histogram,int* prev_edge_histogram,int* displacement, double slope, double yint,int size);

#endif /* DIVERGENCE_H_ */
