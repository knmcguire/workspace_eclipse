/*
 * divergence.cpp
 *
 *  Created on: Apr 28, 2015
 *      Author: knmcguire
 */
#include "divergence.h"

using namespace cv;


void calculate_edge_flow(unsigned char * in,unsigned char * out, struct displacement_t* displacement,struct edge_flow_t* edge_flow,struct edge_hist_t* edge_hist,int front,int rear)
{

	//Define arrays and pointers for edge histogram and displacements
	int edge_histogram_x[IMAGE_WIDTH],prev_edge_histogram_x[IMAGE_WIDTH];
	int * edge_histogram_x_p=edge_histogram_x;
	int * prev_edge_histogram_x_p=prev_edge_histogram_x;

	int edge_histogram_y[IMAGE_HEIGHT],prev_edge_histogram_y[IMAGE_HEIGHT];
	int * edge_histogram_y_p=edge_histogram_y;
	int * prev_edge_histogram_y_p=prev_edge_histogram_y;

	float slope_x=0.0;
	float trans_x=0.0;
	float slope_y=0.0;
	float trans_y=0.0;


	int previous_frame_number;

			if(fabs(edge_flow->horizontal[0])<MAX_FLOW&&!isnan(edge_flow->horizontal[0]))
			{
				previous_frame_number=(int)((MAX_HORIZON-2)*((float)MAX_FLOW-fabs(edge_flow->horizontal[0]))/(float)MAX_FLOW)+1;
			}
			else
				previous_frame_number=1;


			// the previous frame number relative to dynamic parameters
			int previous_frame_number_rel=front-previous_frame_number;
			if(previous_frame_number_rel<0)
				previous_frame_number_rel=rear+(MAX_HORIZON-1-abs(previous_frame_number));

			// Copy previous edge gram to pointer
			printf("%d %d %d %d \n",front,previous_frame_number,previous_frame_number_rel,rear);
			memcpy(prev_edge_histogram_x_p,&edge_hist[previous_frame_number_rel].horizontal,sizeof(int)*IMAGE_WIDTH);
			memcpy(prev_edge_histogram_y_p,&edge_hist[previous_frame_number_rel].vertical,sizeof(int)*IMAGE_HEIGHT);

			//Calculculate current edge_histogram
			calculate_edge_histogram(in,out,edge_histogram_x_p,IMAGE_WIDTH,IMAGE_HEIGHT,'x');
			calculate_edge_histogram(in,out,edge_histogram_y_p,IMAGE_WIDTH,IMAGE_HEIGHT,'y');


			//calculate displacement based on histogram
			calculate_displacement(edge_histogram_x_p,prev_edge_histogram_x_p,displacement->horizontal,previous_frame_number,IMAGE_WIDTH);
			calculate_displacement(edge_histogram_y_p,prev_edge_histogram_y_p,displacement->vertical,previous_frame_number,IMAGE_HEIGHT);


			//Line fit of the displacement by least square estimation or Ransac
	#ifdef RANSAC
			line_fit_RANSAC(displacement->horizontal, &slope_x,&trans_x,IMAGE_WIDTH);
			line_fit_RANSAC(displacement->vertical, &slope_y,&trans_y,IMAGE_HEIGHT);

	#else
			line_fit(displacement->horizontal, &slope_x,&trans_x,IMAGE_WIDTH);
			line_fit(displacement->vertical, &slope_y,&trans_y,IMAGE_HEIGHT);
	#endif

			//Correct Divergence slope and translation by the amount of frames skipped
			slope_x=slope_x/(float)previous_frame_number;
			trans_x=trans_x/(float)previous_frame_number;

			slope_y=slope_y/(float)previous_frame_number;
			trans_y=trans_y/(float)previous_frame_number;

			edge_flow->horizontal[0]=slope_x;
			edge_flow->horizontal[1]=trans_x;
			edge_flow->vertical[0]=slope_y;
			edge_flow->vertical[1]=trans_y;

			printf("%f %f\n", slope_x, trans_x);
			//plot_edge_histogram(edge_histogram_x_p,prev_edge_histogram_x_p,displacement->horizontal,slope_x,trans_x,IMAGE_WIDTH);



			//Copy new edge histogram to the structure
			memcpy(edge_hist[front].horizontal,edge_histogram_x_p,sizeof(int)*IMAGE_WIDTH);
			memcpy(edge_hist[front].vertical,edge_histogram_y_p,sizeof(int)*IMAGE_HEIGHT);
}

void plot_flow(struct edge_flow_t *edge_flow_plot,int front, int rear){
	cv::Mat flow_plot=cv::Mat::zeros(IMAGE_HEIGHT,IMAGE_WIDTH, CV_8UC3);
	cv::Point line1,line2;
	cv::Point line1_vert,line2_vert;
	cv::Point line1_zero,line2_zero;
	cv::Point line1_disp,line2_disp;


	line1_zero.x=0;
	line1_zero.y=IMAGE_HEIGHT/2;
	line2_zero.x=IMAGE_WIDTH;
	line2_zero.y=IMAGE_HEIGHT/2;

	cv::line(flow_plot,line1_zero,line2_zero,Scalar(255,255,255));

	int idx_plot=front;
	line1.x=0;
	line1.y=0;
	line1_vert.x=0;
	line1_vert.y=0;

	for( int x = 1; x < IMAGE_WIDTH; x++)
	{


		line2.y=edge_flow_plot[idx_plot].horizontal[1]*10+IMAGE_HEIGHT/2;
		line2.x=x;

		line2_vert.y=edge_flow_plot[idx_plot].vertical[1]*10+IMAGE_HEIGHT/2;
		line2_vert.x=x;

		cv::line(flow_plot,line1,line2,Scalar(0,255,0));
		cv::line(flow_plot,line1_vert,line2_vert,Scalar(255,0,0));


        line1.x=line2.x;
        line1.y=line2.y;

        line1_vert.x=line2_vert.x;
        line1_vert.y=line2_vert.y;

        idx_plot++;
        if (idx_plot>IMAGE_WIDTH)
        idx_plot=0;

	}



	cv::namedWindow("flow_plot",CV_WINDOW_NORMAL );

		cv::imshow("flow_plot", flow_plot);


}


void plot_edge_histogram(int* edge_histogram,int* prev_edge_histogram,int* displacement, double slope, double yint,int size)
{

	cv::Mat edge_histogram_plot=cv::Mat::zeros(128,size, CV_8UC3);
	cv::Point line1,line2;
	cv::Point line1_prev,line2_prev;
	cv::Point line1_disp,line2_disp;

	line1.x=0;
	line1.y=0;

	for( int x = 1; x < size; x++)
	{

		line2.x=x;
		line2.y=edge_histogram[x]/50;
		line2_prev.x=x;
		line2_prev.y=prev_edge_histogram[x]/50;
		line2_disp.x=x;
		line2_disp.y=displacement[x]+69;


		cv::line(edge_histogram_plot,line1_disp,line2_disp,Scalar(0,255,0));

		cv::line(edge_histogram_plot,line1_prev,line2_prev,Scalar(0,0,255));
		cv::line(edge_histogram_plot,line1,line2,Scalar(255,0,0));


		line1.x=line2.x;
		line1.y=line2.y;

		line1_prev.x=line2_prev.x;
		line1_prev.y=line2_prev.y;

		line1_disp.x=line2_disp.x;
		line1_disp.y=line2_disp.y;


	}
	line1_disp.x=0;
	line1_disp.y=yint+69;
	line2_disp.x=192;
	line2_disp.y=192*slope+yint+69;

	cv::line(edge_histogram_plot,line1_disp,line2_disp,Scalar(255,255,255));


	flip(edge_histogram_plot,edge_histogram_plot,0);
	cv::namedWindow("edge_histogram",CV_WINDOW_NORMAL );

	cv::imshow("edge_histogram", edge_histogram_plot);


}
void calculate_edge_histogram(unsigned char * in,unsigned char * out,int * edge_histogram,int image_width,int image_height,char direction)
{

	int  sobel;
	int Sobel[3] = {-1, 0, 1};
	int y,x;
	int edge_histogram_temp;
	int  c,r;
	int idx_filter;

	if(direction=='x')
	for( x = 0; x < image_width; x++)
	{
		edge_histogram_temp=0;
		for( y = 0; y < image_height; y++)
		{

			int idx = image_width*y + (x);
			sobel=0;

			for(c = -1; c <=1; c++)
			{
					idx_filter= image_width*(y) + (x+c);

				sobel += Sobel[c+1] * (int)(in[idx_filter]);
			}

			sobel=abs(sobel);

			edge_histogram_temp += sobel;
			out[idx]=sobel;

		}

		edge_histogram[x]=(int)edge_histogram_temp;

	}
	else if(direction=='y')

	for( y = 0; y < image_height; y++)
	{
		edge_histogram_temp=0;
		for( x = 0;x < image_width; x++)
		{

			int idx = image_width*y + (x);
			sobel=0;

			for(c = -1; c <=1; c++)
			{

					idx_filter = image_width*(y+c) + (x);

				sobel += Sobel[c+1] * (int)(in[idx_filter]);
			}

			sobel=abs(sobel);

			edge_histogram_temp += sobel;
			out[idx]=sobel;

		}

		edge_histogram[y]=(int)edge_histogram_temp;

	}
	else
		printf("direction is wrong!!\n");

}

void calculate_displacement(int * edge_histogram,int * edge_histogram_prev,int * displacement,int prev_frame_number,int size)
{	int  c,r;
int y,x,flow_ind;

int W=10;
int D=10;
int d;
int SAD_temp[2*D];
int i;
int min_cost;
int  min_index;

int mean_displacement;

int  minimum=D*2;
int sum_dis, avg;
sum_dis=0;


for(x=0; x<size;x++)
{
	minimum=D*2;

	//Sad_temp to 0;

	if(x>D&&x<size-D)
	{
		for(c=-D;c<D;c++)
		{
			SAD_temp[c+D]=0;
			for(r=-W;r<W;r++)
				SAD_temp[c+D]+=abs(edge_histogram[x+r]-edge_histogram_prev[x+r+c]);
		}

		min_index=getMinimum(SAD_temp,2*D);

		displacement[x]=(int)((min_index-D));
	}else
		displacement[x]=0;


}

}

void line_fit(int* displacement, float* Slope, float* Yint,int size)
{
	int x;

	int Count=192-20;
	int SumY,SumX,SumX2,SumXY;
	int XMean,YMean;
	int k;
	int count_disp;

	for(k=0;k<size;k++){
		count_disp+=displacement[k];    }

	double Slope_temp,Yint_temp;

	//double Slope,Yint;

	for(x=10;x<192-10;x++){

		SumX+=x;
		SumY+=displacement[x];

		SumX2+=x *x;
		SumXY+=x *displacement[x];

	}
	XMean=SumX/Count;
	YMean=SumY/Count;

	Slope_temp=((double)(SumXY-SumX*YMean)/(double)(SumX2-SumX*XMean));
	Yint_temp= (YMean-Slope_temp* (double)XMean);

	*Slope=Slope_temp;
	*Yint=Yint_temp;
	//printf("%f %f\n", Slope, Yint);


}

void line_fit_RANSAC( int* displacement, float* Slope, float* Yint,int size)
{

	//Fit a linear line with RANSAC (from Guido's code)
	int ransac_iter=20;
	int it;
	int k;
	int ind1, ind2, tmp, entry, total_error, best_ind;
	int dx, dflow, predicted_flow;
	// flow = a * x + b
	float a[ransac_iter];
	float b[ransac_iter];
	float a_temp,b_temp;
	int errors[ransac_iter];

	int X[size];
	int count_disp=0;

	for(k=0;k<size;k++){
		X[k]=k;
		count_disp+=displacement[k];    }

	if(count_disp!=0)
	{
		for(it = 0; it < ransac_iter; it++)
		{

			errors[it] = 0;
			total_error=0;

			ind1 = rand() % size;
			ind2 = rand() % size;

			while(ind1 == ind2)
				ind2 = rand() % size;

			if(X[ind1] > X[ind2])
			{
				tmp = ind2;
				ind2 = ind1;
				ind1 = tmp;
			}
			while(displacement[ind1]==0)
				ind1 = rand() % size;
			while(displacement[ind2]==0)
				ind2 = rand() % size;




			dx=X[ind2]-X[ind1];
			dflow = displacement[ind2] - displacement[ind1];

			//Fit line with two points

			a[it] = (float)dflow/(float)dx;
			b[it] = (float)displacement[ind1]- (a[it] *(float)(X[ind1]));

			// evaluate fit:
			for (entry = 0; entry < size; entry++)
			{
				predicted_flow = (int32_t)(a[it] * (float)(X[entry] ) + b[it]);
				total_error += (uint32_t) abs(displacement[entry] - predicted_flow);
			}
			errors[it] = total_error;
		}
		// select best fit:
		best_ind = getMinimum(errors, 20);
		//printf("%d\n",best_ind);
		(*Slope) = (float)a[best_ind] ;
		(*Yint) = (float)b[best_ind];
	}
	else
	{
		(*Slope) = 0.0 ;
		(*Yint) = 0.0;
	}




}


int getMinimum(int * flow_error, int  max_ind)
{
	uint32_t i;
	uint32_t min_ind = 0;
	uint32_t min_err = flow_error[0];
	for(i = 1; i < max_ind; i++)
	{
		if(flow_error[i] < min_err)
		{
			min_ind = i;
			min_err = flow_error[i];
		}
	}
	return min_ind;
}






