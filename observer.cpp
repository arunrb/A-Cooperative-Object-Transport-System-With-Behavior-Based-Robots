//#####################################################################################################################
// 	observer.cpp
//
//	Created by ARUN PRASSANTH RAMASWAMY BALASUBRAMANIAN, MAY 2017
//  https://www.linkedin.com/in/arun-prassanth-rb
//#####################################################################################################################

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <sys/time.h>
#include <sstream>
#include <string>
#include <iostream>
#include <pthread.h>
#include <unistd.h>


/* For the logitech webcam,vales from experiments*/

//for green (Xbox one case)
int gH_MIN = 26;
int gH_MAX = 63;
int gS_MIN = 81;
int gS_MAX = 128;
int gV_MIN = 43;
int gV_MAX = 84;

//for blue
int bH_MIN = 128;
int bH_MAX = 246;
int bS_MIN = 69;
int bS_MAX = 238;
int bV_MIN = 0;
int bV_MAX = 90;

//for red
int rH_MIN = 0;
int rH_MAX = 144;
int rS_MIN = 0;
int rS_MAX = 19;
int rV_MIN = 31;
int rV_MAX = 105;

//PINK
//JULY25

int pH_MIN = 55;
int pH_MAX = 166;
int pS_MIN = 20;
int pS_MAX = 105;
int pV_MIN = 139;
int pV_MAX = 204;


//default capture width and height
const int FRAME_WIDTH =960;
const int FRAME_HEIGHT =720;

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=10;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 15*15;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

#define MIN_DIST_TO_REACH_GOAL 15//short expt 60//old 65

/*global variables for storing overhead cam hints*/
std::vector<cv::Point2f> bot_center(2), obj_center(2), goal_center(1);


bool reached_goal = false;


/*!
 * Compute time difference
 *
 * \param difference difference between the two times, in structure timeval type
 * \param end_time end time
 * \param start_time start time
 *
 * \return difference between the two times in [us]
 *
 */
long long
timeval_diff(struct timeval *difference,
		struct timeval *end_time,
		struct timeval *start_time
)
{
	struct timeval temp_diff;

	if(difference==NULL)
	{
		difference=&temp_diff;
	}

	difference->tv_sec =end_time->tv_sec -start_time->tv_sec ;
	difference->tv_usec=end_time->tv_usec-start_time->tv_usec;

	/* Using while instead of if below makes the code slightly more robust. */

	while(difference->tv_usec<0)
	{
		difference->tv_usec+=1000000;
		difference->tv_sec -=1;
	}

	return 1000000LL*difference->tv_sec+
			difference->tv_usec;

} /* timeval_diff() */


/*Dilate or erode the binary image*/
void morphOps(cv::Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	cv::Mat erodeElement = getStructuringElement( cv::MORPH_RECT,cv::Size(3,3));
    //dilate with larger element so make sure object is nicely visible
	cv::Mat dilateElement = getStructuringElement( cv::MORPH_RECT,cv::Size(8,8));
	imshow("actual",thresh);
	erode(thresh,thresh,erodeElement);
	imshow("ero 1",thresh);
	erode(thresh,thresh,erodeElement);
	imshow("ero 2",thresh);


	dilate(thresh,thresh,dilateElement);
	imshow("dia 1",thresh);
	dilate(thresh,thresh,dilateElement);
	//imshow("dia 2",thresh);

}

/*
 *
 * Blob detection fn
 * params:
 * Original binary image, original image (for drawing on), color
 *
 * */
void find_blobs( cv::Mat matThresholded, cv::Mat &matCameraFeed, char color_interest)
{

	cv::Mat matGray;
	cv::cvtColor(matCameraFeed, matGray,cv::COLOR_BGR2GRAY);

	std::vector<cv::KeyPoint> keypoints;
	cv::KeyPoint current_interest_0, current_interest_1; /*biggest two blobs of current obj*/
	current_interest_0.size = current_interest_1.size = 0;
	cv::Point2f image_centre;
	image_centre.x = matThresholded.cols/2;
	image_centre.y = matThresholded.rows/2;

	// Setup SimpleBlobDetector parameters.
	cv::SimpleBlobDetector::Params params;


	/*filter by color FUNCTIONALITY IS BROKEN in 3.3.
	 * This flag must be FALSE*/
	params.filterByColor = false;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 100;

	// Filter by Circularity
	params.filterByCircularity = false;
	// Filter by Convexity
	params.filterByConvexity = false;
	// Filter by Inertia
	params.filterByInertia = false;

	params.minDistBetweenBlobs = 10; /*if too close, make them one blob..
									but  looks like broken */


	/*dialate*/
	cv::Mat dilateElement = getStructuringElement( cv::MORPH_RECT,cv::Size(3,3));
	//imshow("Before dialation", matThresholded );
	dilate(matThresholded,matThresholded,dilateElement);


	/*set a blob detector with params*/
	cv::SimpleBlobDetector blob_detector(params);

	/**detect blobs */
	blob_detector.detect( matThresholded, keypoints);



	cv::Mat im_with_keypoints;
	cv::drawKeypoints( matCameraFeed, keypoints, im_with_keypoints, cv::Scalar(0,255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );


	//Go thru the list of blobs found and pick the biggest two
	for(unsigned int i=0; i<keypoints.size(); i++ )
	{

		/*biggest blob will be in current_interest_1 and the second
		 * biggest blob will be current_interest_2*/
		if(current_interest_0.size < keypoints[i].size)
		{
			current_interest_1 = current_interest_0;
			current_interest_0 = keypoints[i];
		}else if(current_interest_1.size < keypoints[i].size) {
			current_interest_1 = keypoints[i];
		}
	}



	//	To draw keypoints on the image
	//  cv::imshow("keypoints", im_with_keypoints );
	//	cv::waitKey(10);

	/* aramaswamybalasubram
	 * Blobs can be reported on any order..
	 * so left and ringht are detected based
	 * on the x valeu of the pixel
	 * Jul 25, 2018 11:23:08 PM
	if(color_interest=='b')
	{
		//Blobs can be reported in any order.. so
		 //decide which blob is on LHS and which is on RHS
		if(current_interest_0.pt.x < current_interest_1.pt.x)
		{
			bot_center[0] = current_interest_0.pt;
			bot_center[1] = current_interest_1.pt;
		} else if(current_interest_0.pt.x > current_interest_1.pt.x)
		{
			bot_center[0] = current_interest_1.pt;
			bot_center[1] = current_interest_0.pt;
		} else { //if x val of both blobs are equal, compare y
			if(current_interest_0.pt.y > current_interest_1.pt.y)
			{
				//higher the y, lower the blob in current forward facing frame
				bot_center[0] = current_interest_0.pt;
				bot_center[1] = current_interest_1.pt;
			} else {
				bot_center[0] = current_interest_1.pt;
				bot_center[1] = current_interest_0.pt;
			}
		}
	}
	 */

	//blue blob (robot)
	if(color_interest=='b')
	{
		bot_center[1] = current_interest_0.pt;
	}

	//pink blob (robot)
	if(color_interest=='p')
	{
		bot_center[0] = current_interest_0.pt;
	}
	//red blobs (object)
	if (color_interest=='r')
	{
		/*Blobs can be reported in any order.. so
		 * decide which blob is on LHS and which is on RHS*/
		if(current_interest_0.pt.x < current_interest_1.pt.x)
		{
			obj_center[0] = current_interest_0.pt;
			obj_center[1] = current_interest_1.pt;
		} else if(current_interest_0.pt.x > current_interest_1.pt.x)
		{
			obj_center[0] = current_interest_1.pt;
			obj_center[1] = current_interest_0.pt;
		} else { /*if x val of both blobs are equal, compare y*/
			if(current_interest_0.pt.y > current_interest_1.pt.y)
			{
				/*higher the y, lower the blob in current forward facing frame*/
				obj_center[0] = current_interest_0.pt;
				obj_center[1] = current_interest_1.pt;
			} else {
				obj_center[0] = current_interest_1.pt;
				obj_center[1] = current_interest_0.pt;
			}
		}
	} else if (color_interest=='g') // green blob is goal
	{
		if(current_interest_0.pt.x != 0 && current_interest_0.pt.y != 0)
		{
			static cv::Point2f fixed_goal_center = current_interest_0.pt;
			goal_center[0] = fixed_goal_center;
		} else {
			goal_center[0] = current_interest_0.pt;
		}
	}

}


/*
 * Create a thread and write the centroids to the shared memory.
 * Write to a text file and then copy to another robot that acts
 * as a file exchange.Robot can be changed by changing the IP.
 * */
void* thread_write_to_shared_memory(void *arg)
{
	std::ostringstream obj_goal_xy ;
	std::string obj_goal_xy_str;

	//create the string
	//red = object 1 and object 2; green = goal; blue: robot 1 and robot 2 (pink)
	obj_goal_xy << "r "<<obj_center[0].x<<" "<<obj_center[0].y<<" "<<obj_center[1].x<<" "<<obj_center[1].y
			<<" "<<"g "<<goal_center[0].x<<" "<<goal_center[0].y<<" "<<
			"b "<<bot_center[0].x<<" "<<bot_center[0].y<<" "<<bot_center[1].x<<" "<<bot_center[1].y<<"\n";
	obj_goal_xy_str = obj_goal_xy.str();

	FILE *writestream = popen("cat > /v1/aramaswamybalasubram/OBSERVER/OVERHEAD.txt", "w");

	if (writestream != 0) {
	fputs(obj_goal_xy_str.c_str(), (FILE*)writestream);
	} else {
		std::cout << "ERROR WRITING TO FILE"<< "\n";
	}
	pclose(writestream);

	/*copy to remote robot*/
	if (system(NULL))
		system("scp /v1/aramaswamybalasubram/OBSERVER/OVERHEAD.txt root@10.42.0.17:/home/root/SHARED_MEMORY/");
	else
		std::cout << "Command processor doesn't exists"<<"\n";

	pthread_exit(NULL);
}


/* aramaswamybalasubram
	 * OLD: write commands to robos
	 * Apr 24, 2018 12:17:34 PM

bool write_commands_to_shared_memory()
{
	bool return_status = false;
	double xdiff, ydiff;
	std::ostringstream stream_lbot, stream_rbot;
	std::vector<double> distance(2) ;



	//dist[0] -> dist between LHS blob and goal
	xdiff =  (obj_center[0].x - goal_center[0].x);
	ydiff = (obj_center[0].y - goal_center[0].y);
	xdiff = xdiff*xdiff;
	ydiff = ydiff*ydiff;
	distance[0] = sqrt(xdiff+ydiff);

	//dist[1] -> dist between RHS blob and goal
	xdiff =  (obj_center[1].x - goal_center[0].x);
	ydiff = (obj_center[1].y - goal_center[0].y);
	xdiff = xdiff*xdiff;
	ydiff = ydiff*ydiff;
	distance[1] = sqrt(xdiff+ydiff);

	std::cout <<"DISTANCES: "<<distance[0] <<" & "<<distance[1] <<"\n";

	int left_bot, right_bot; //time to push in seconds
	float dist_diff = 0; //difference in distance from the goal
	left_bot = right_bot = -1;  //avoid junk

//	The find_blobs fn makes sure that
//	 distance[0] => LHS distance
//	 distance[1] => RHS distance
//	 the LHS blob is stored in the leftmost index of the vector and so on
//	 the following decisions on the right and left bot times are thus
//	 made based on this assumption.
	dist_diff = distance[0]-distance[1];
	//std::cout<<"###dist_diff= "<<dist_diff<<"\n";

	if((distance[0] <= 65) && (distance[1]<=65))
	{
		left_bot = -5;
		right_bot = -5;
		std::cout<<"\n REACHED GOAL, TERMINATING MISSION.\n";
		return_status = true;
	}
	else if(abs(dist_diff)<=15) //push together
	{
		left_bot = 1;
		right_bot = 1;
		return_status = false;
	}
	else
	{
		if(dist_diff>0)	//LHS is far away, push leftbot
		{
			if(dist_diff>15)
				left_bot = 1;
			else if(dist_diff>30)
				left_bot = 2;
			else if(dist_diff>50)
				left_bot = 3;

			right_bot =0;

			std::cout<<"###if dist_diff>0 , dist_diff= "<<dist_diff<<"\n";

		} else if(dist_diff<0)	//RHS is far away, push rightbot
		{
			dist_diff = fabs(dist_diff);
			if(dist_diff>15)
				right_bot = 1;
			if(dist_diff>30)
				right_bot = 2;
			if(dist_diff>50)
				right_bot = 3;

			left_bot =0;

			std::cout<<"###if dist_diff<0 , dist_diff= "<<dist_diff<<"\n";
		} else if(dist_diff == 0) //super super rare
		{
			int rand_num = -1;
			rand_num = random_gen();
			std::cout<<"### rand: "<<rand_num<<"\n";

			//rand = even, push left. Else push right
			if(!rand_num%2)
			{
				left_bot = 1;
				right_bot = 0;

			} else {
				left_bot = 0;
				right_bot = 1;
			}

		}
		return_status = false;
	}
	std::cout<<"### right_bot: "<<right_bot<<"    left_bot: "<<left_bot<<"\n";

	stream_lbot << "l "<<left_bot<<" ";
	stream_lbot << "r "<<right_bot<<"\n";
	//bot_xy << "b "<<(int)bot_center.x<<" "<<(int)bot_center.y<<"\n";
	//obj_xy << "r "<<(int)obj_center.x<<" "<<(int)obj_center.y<<"\n";
	//goal_xy << "g "<<(int)goal_center.x<<" "<<(int)goal_center.y<<"\n";


	std::string lbot_str, rbot_str, sum;
	lbot_str = stream_lbot.str();
	rbot_str = stream_rbot.str();
	sum = lbot_str+rbot_str;

	std::cout <<"---CMD WRITING LOCAL FILE:---\n"<< sum<<"\n";

	FILE *writestream = popen("cat > /v1/aramaswamybalasubram/OBSERVER/OVERHEAD.txt", "w");

	if (writestream != 0) {
	fputs(sum.c_str(), (FILE*)writestream);
	} else {
		std::cout << "ERROR WRITING TO FILE"<< "\n";
	}
	pclose(writestream);

	//copy to remote bot
	if (system(NULL))
	{
		system("scp /v1/aramaswamybalasubram/OBSERVER/OVERHEAD.txt root@10.42.0.17:/home/root/SHARED_MEMORY/");
	} else {
		std::cout << "Command processor doesn't exists"<<"\n";
	}

	return return_status;
}
*/


/*
 * bool find_intersection:
 * 	takes two lines (4 points) and finds where they intersect
 * 	The lines are defined by (o1, p1) and (o2, p2)
 * 	ex:find_intersection(bot_center, obj_center, goalline_x_axis.start, goalline_x_axis.end ,x_intercept)
 * */

bool find_intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2,
		cv::Point2f &r)
{
	cv::Point2f x = o2 - o1;
	cv::Point2f d1 = p1 - o1;
	cv::Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;

    /*arun: if x or y is negative, point is not usable, mark it
     * as (-111,-111) so that it can be ignored it later*/
    if(r.x < 0 || r.y<0 ) {
    	//std::cout <<"\n @@@@@@ find_intersection: intersection_negatives r="<<r<<"\n";
    	r.x = r.y = -111;
    	//std::cout <<" => mark r(-111) i.e. r="<<r <<"\n";
    	 return false;
    }

    /*arun: if the intersecting point is out of the frame, is not usable,
     *  mark it as (-111,-111) so that it can be ignored it later*/
       if(r.x > 960 || r.y>720 ) {
       	//std::cout <<"\n @@@@@@ find_intersection: intersection_out_of_frame r="<<r;
       	r.x = r.y = -111;
        return false;
       	//std::cout <<" => mark r(-111) i.e. r="<<r <<"\n";
       }


    return true;
}


std::vector<cv::Point2f> vec_dummy;
std::vector<cv::Point2f> vec_dummy_2;
std::vector<cv::Point2f> vec_sfnew;
cv::Mat with_box;

/*
 * Find safe zone for plotting on image. Just for users reference,
 * not conveyed to the robots. Robots can push while they are within this region.
 * Old, not used now as find_safe_zone_2() replaced this fn.
//returns -1 when point is outside the polygon
//returns 0  when point is on boundary
//returns 1 when point is inside polygon
 * */
int find_safe_zone()
{
	std::vector<cv::Point2f> vec_safe_zone;

	cv::Point2f topleft, bottomleft, topright, bottomright, intersection, zerozero;
	bool result;

	//order of insertion matters!!!
	//THis is the first point
	vec_safe_zone.push_back(obj_center[0]);


	zerozero.x = zerozero.y = 0;

	topleft.x = 0;
	topleft.y = 0;
	bottomleft.x = 0;
	bottomleft.y = 720;

	topright.x = 960;
	topright.y = 0;
	bottomright.x = 960;
	bottomright.y = 720;

	//************** BOTTOM **********
	//BOTTOM intersection with goal-obj0
	result = find_intersection(bottomleft, bottomright, obj_center[0], goal_center[0], intersection);

	if(result)
	{
		//THis is the second point
		vec_safe_zone.push_back(intersection);
		std::cout<<"obj_0-goal-BOTTOM: "<<result<<" "<<intersection<<"\n";
	}


	//BOTTOM intersection with goal-obj1
	result = find_intersection(bottomleft, bottomright, obj_center[1], goal_center[0], intersection);

	if(result)
	{
		//THis is the third point
		vec_safe_zone.push_back(intersection);
		std::cout<<"obj_1-goal-BOTTOM: "<<result<<" "<<intersection<<"\n";
	}

	//************** LEFT **********
	//LEFT intersection with goal-obj0
	result = find_intersection(topleft, bottomleft, obj_center[0], goal_center[0], intersection);

	if(result)
	{
		//THis is the second point
		vec_safe_zone.push_back(intersection);
		std::cout<<"obj_0-goal-LEFT: "<<result<<" "<<intersection<<"\n";
	}

	//LEFT intersection with goal-obj1
	result = find_intersection(topleft, bottomleft, obj_center[1], goal_center[0], intersection);

	if(result)
	{
		//THis is the second point
		vec_safe_zone.push_back(intersection);
		std::cout<<"obj_1-goal-LEFT: "<<result<<" "<<intersection<<"\n";
	}

	//************** RIGHT **********
	//RIGHT intersection with goal-obj0
	result = find_intersection(topright, bottomright, obj_center[0], goal_center[0], intersection);

	if(result)
	{
		//THis is the second point
		vec_safe_zone.push_back(intersection);
		std::cout<<"obj_0-goal-RIGHT: "<<result<<" "<<intersection<<"\n";
	}

	//LEFT intersection with goal-obj1
	result = find_intersection(topright, bottomright, obj_center[1], goal_center[0], intersection);

	if(result)
	{
		//THis is the second point
		vec_safe_zone.push_back(intersection);
		std::cout<<"obj_1-goal-RIGHT: "<<result<<" "<<intersection<<"\n";
	}

//	//************** TOP **********
//		//BOTTOM intersection with goal-obj0
//		result = find_intersection(topleft, topright, obj_center[0], goal_center[0], intersection);
//
//		if(result)
//		{
//			//THis is the second point
//			vec_safe_zone.push_back(intersection);
//			std::cout<<"obj_0-goal-TOP: "<<result<<" "<<intersection<<"\n";
//		}
//
//
//		//BOTTOM intersection with goal-obj1
//		result = find_intersection(topleft, topright, obj_center[1], goal_center[0], intersection);
//
//		if(result)
//		{
//			//THis is the third point
//			vec_safe_zone.push_back(intersection);
//			std::cout<<"obj_1-goal-TOP: "<<result<<" "<<intersection<<"\n";
//		}

	//order of insertion matters!!!
	//THis is the last point
	vec_safe_zone.push_back(obj_center[1]);

	int i ;
	for(i=0 ; i<vec_safe_zone.size(); i++)
		std::cout<<vec_safe_zone[i]<<"\n";

	if(bot_center[0] == zerozero )
	{
		std::cout<<"BOT NOT SEEN\n";
		return 0;

	}

	//returns -1 when point is outside the polygon
	//returns 0  when point is on boundary
	//returns 1 when point is inside polygon
	int poly_return = pointPolygonTest(vec_safe_zone, bot_center[0], false);

	std::cout<<"##### PINK SAFE: "<<poly_return<<"\n";
	vec_dummy = vec_safe_zone;
	return 1;

}


/*
 * Find safe zone for plotting on image. Just for users reference,
 * not conveyed to the robots. Robots can push while they are within this region.
//returns -1 when point is outside the polygon
//returns 0  when point is on boundary
//returns 1 when point is inside polygon
 * */
int find_safe_zone_2()
{
	std::vector<cv::Point2f> vec_local;

	cv::Point2f topleft, bottomleft, topright, bottomright, intersection, zerozero;
	cv::Point2f interleft, interright, intertop, interbottom;
	bool result;
	bool left, right, top, bottom;
	left = right  = top = bottom = false;



	zerozero.x = zerozero.y = 0;

	topleft.x = 0;
	topleft.y = 0;
	bottomleft.x = 0;
	bottomleft.y = 720;

	topright.x = 960;
	topright.y = 0;
	bottomright.x = 960;
	bottomright.y = 720;


	//******** LHS intersection ********
	result = find_intersection(topleft, bottomleft, obj_center[0], obj_center[1], intersection);
	if(result)
	{
		left = true;
		interleft = intersection;
		vec_local.push_back(intersection);
		std::cout<<"LEFT intersection: "<<result<<" "<<intersection<<"\n";
	}


	//******** RHS intersection ********
	result = find_intersection(topright, bottomright, obj_center[0], obj_center[1], intersection);
	if(result)
	{
		right = true;
		interright = intersection;
		vec_local.push_back(intersection);
		std::cout<<"RIGHT intersection: "<<result<<" "<<intersection<<"\n";
	}

	//******** TOP intersection ********
	result = find_intersection(topleft, topright, obj_center[0], obj_center[1], intersection);
	if(result)
	{
		top = true;
		intertop = intersection;
		vec_local.push_back(intersection);
		std::cout<<"TOP intersection: "<<result<<" "<<intersection<<"\n";
	}

	//******** BOTTOM intersection ********
	result = find_intersection(bottomleft, bottomright, obj_center[0], obj_center[1], intersection);
	if(result)
	{
		bottom = true;
		interbottom = intersection;
		vec_local.push_back(intersection);
		std::cout<<"BOTTOM intersection: "<<result<<" "<<intersection<<"\n";
	}

	//****************************************************************************************************
	if(left&&right)
	{
		vec_local.push_back(bottomright);
		vec_local.push_back(bottomleft);
	} else if(bottom&&right)
	{
		vec_local.push_back(bottomright);
	} else if(bottom&&left)
	{
		vec_local.push_back(bottomleft);
	} else if(top&&bottom)
	{
		if(intertop.x>goal_center[0].x)
		{
			//bottom first as that would be the previous entry in vector
			vec_local.push_back(bottomright);
			vec_local.push_back(topright);

		} else {
			//bottom first as that would be the previous entry in vector
			vec_local.push_back(bottomleft);
			vec_local.push_back(topleft);
		}
	} else if(top&&right)
	{
		if(intertop.x<goal_center[0].x)
		{
			vec_local.push_back(topleft);
			vec_local.push_back(bottomleft);
			vec_local.push_back(bottomright);


		} else {
			//bottom first as that would be the previous entry in vector
			vec_local.push_back(topright);
		}
	} else if(top&&left)
	{
		if(intertop.x>goal_center[0].x)
		{
			vec_local.push_back(topright);
			vec_local.push_back(bottomright);
			vec_local.push_back(bottomleft);

		} else {
			vec_local.push_back(bottomleft);
		}
	}

	int i ;
	for(i=0 ; i<vec_local.size(); i++)
		std::cout<<"vec_new "<<i<<": "<<vec_local[i]<<"\n";

	if(bot_center[0] == zerozero )
	{
		std::cout<<"BOT NOT SEEN\n";
		return 0;

	}

	//returns -1 when point is outside the polygon
	//returns 0  when point is on boundary
	//returns 1 when point is inside polygon
	int poly_return = pointPolygonTest(vec_local, bot_center[0], false);

	std::cout<<"##### NEW  SAFE: "<<poly_return<<"\n";
	vec_sfnew.clear();
	vec_sfnew = vec_local;
	return 1;

}

/*
 * Find hazard zone for plotting on image. Just for users reference,
 * not conveyed to the robots. Robots cannot push while they are within this region.
//returns -1 when point is outside the polygon
//returns 0  when point is on boundary
//returns 1 when point is inside polygon
 * */
int find_hazard_zone()
{

	cv::Point2f topleft, bottomleft, topright, bottomright, intersection, zerozero;
	std::vector<cv::Point2f> vec_hazard_zone;
	bool result;


	vec_hazard_zone.push_back(goal_center[0]);

	zerozero.x = zerozero.y = 0;

	topleft.x = 0;
	topleft.y = 0;
	bottomleft.x = 0;
	bottomleft.y = 720;

	topright.x = 960;
	topright.y = 0;
	bottomright.x = 960;
	bottomright.y = 720;



	//LHS intersection
	result = find_intersection(topleft, bottomleft, obj_center[0], obj_center[1], intersection);
	if(result)
	{
		vec_hazard_zone.push_back(intersection);
		std::cout<<"LEFT intersection: "<<result<<" "<<intersection<<"\n";
	}


	//RHS intersection
	result = find_intersection(topright, bottomright, obj_center[0], obj_center[1], intersection);

	if(result)
		{
		vec_hazard_zone.push_back(intersection);
			std::cout<<"RIGHT intersection: "<<result<<" "<<intersection<<"\n";
		}

	//TOP intersection
	result = find_intersection(topleft, topright, obj_center[0], obj_center[1], intersection);

	if(result)
		{
		vec_hazard_zone.push_back(intersection);
			std::cout<<"TOP intersection: "<<result<<" "<<intersection<<"\n";
		}

	//BOTTOM intersection
	result = find_intersection(bottomleft, bottomright, obj_center[0], obj_center[1], intersection);

	if(result)
		{
		vec_hazard_zone.push_back(intersection);
			std::cout<<"BOTTOM intersection: "<<result<<" "<<intersection<<"\n";
		}


	int i ;
		for(i=0 ; i<vec_hazard_zone.size(); i++)
			std::cout<<vec_hazard_zone[i]<<"\n";

		if(bot_center[0] == zerozero )
		{
			std::cout<<"BOT NOT SEEN\n";
			return 0;

		}
		//returns -1 when point is outside the polygon
		//returns 0  when point is on boundary
		//returns 1 when point is inside polygon
	int poly_return = pointPolygonTest(vec_hazard_zone, bot_center[0], false);

	std::cout<<"PINK INSIDE HZ OR NOT: "<<poly_return<<"\n";

	vec_dummy_2 = vec_hazard_zone;
	return 1;
}



/*
 * Create a thread to deaw the regions on the frame for users'
 * reference.
 * */
void* thread_draw(void *arg)
{
	find_hazard_zone();
	//find_safe_zone();
	find_safe_zone_2();
	int i;


	//safe zone old
//	cv::vector<cv::Point> mypoint;
//	mypoint.push_back(cv::Point(vec_dummy[0].x,vec_dummy[0].y));  //point1
//	mypoint.push_back(cv::Point(vec_dummy[1].x,vec_dummy[1].y));  //point2
//	mypoint.push_back(cv::Point(vec_dummy[2].x,vec_dummy[2].y));  //point3
//	mypoint.push_back(cv::Point(vec_dummy[3].x,vec_dummy[3].y));  //point4

	cv::vector<cv::Point> mypoint_2;
	mypoint_2.push_back(cv::Point(vec_dummy_2[0].x,vec_dummy_2[0].y));  //point1
	mypoint_2.push_back(cv::Point(vec_dummy_2[1].x,vec_dummy_2[1].y));  //point2
	mypoint_2.push_back(cv::Point(vec_dummy_2[2].x,vec_dummy_2[2].y));  //point3
		//mypoint.push_back(cv::Point(vec_dummy_2[3].x,vec_dummy_2[3].y));  //point4

	cv::vector<cv::Point> sz2;
	for(i=0; i<vec_sfnew.size(); i++ )
		{
			sz2.push_back(cv::Point(vec_sfnew[i].x,vec_sfnew[i].y));
		}

	//safe old //cv::fillConvexPoly(with_box, mypoint,cv::Scalar(0,255,0),CV_AA,0 );
	cv::fillConvexPoly(with_box, mypoint_2,cv::Scalar(0,0,255),CV_AA,0 );
	cv::fillConvexPoly(with_box, sz2,cv::Scalar(255,0,0),CV_AA,0 );

	cv::imshow("safe-not safe", with_box );
	cv::waitKey(20);

	pthread_exit(NULL);
}

//calculate euclidean distance between two points
float find_euclidean_dis(cv::Point2f pt1, cv::Point2f pt2)
{
	float euc_dist, xdiff, ydiff;

	xdiff =  (pt1.x - pt2.x);
	ydiff = (pt1.y - pt2.y);
	xdiff = xdiff*xdiff;
	ydiff = ydiff*ydiff;
	euc_dist = sqrt(xdiff+ydiff);

	return euc_dist;
}

/*
 * MAIN
 * */
int main(int argc, char* argv[])
{
	//some boolean variables for enabling morphing operations
    bool useMorphOps = false; // for applying erosion and dialation

    //Matrix to store each frame of the webcam feed
    cv::Mat matCameraFeed;
    //matrix storage for HSV image
    cv::Mat matHSV;
    //matrix storage for binary thresholded images of each color
    cv::Mat /*matThresholded,*/rmatThresholded,gmatThresholded ,bmatThresholded, pmatThresholded;

    int frame_count = 0;
    int dummy_count = 0;

    //Centroid of object
    cv::Point2f centroid;
    std::string obj_goal_xy_str;
    double xdiff, ydiff;
    std::vector<double> distance(3) ;


	time_t current_time = time(0);
	std::cout <<"\nDATE: "<<ctime(&current_time)<< "\n";



	//video capture object to acquire webcam feed
	cv::VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open(0);
	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	//capture.set(CV_CAP_PROP_FPS, 1);
	struct timeval loop_starttime, loop_endtime, full_starttime, full_endtime/*arun*/;


	/*loop
	all of our operations will be performed within this loop*/
	gettimeofday(&full_starttime,0x0);
	char filename[200];
	//int ABORT_VAR = 9;
	while(!reached_goal){


		usleep(450000); //too many threads choke ssh,
						//so limit no. of. frames


		capture.read(matCameraFeed);	//capture frame

		cv::Mat matDummy;
		matDummy = matCameraFeed;
		//write current time on frame
		current_time = time(0);
		std::ostringstream time_now;
		time_now <<ctime(&current_time);
		std::string time_string;
		time_string = time_now.str();
		cv::putText(matDummy, time_string, cvPoint(50,30),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255,255,255), 1, CV_AA);

		//save frames to local folder so that videos can be compiled later
		dummy_count++;
		if(dummy_count<2)
			continue;
		else {
			dummy_count = 0;
			sprintf(filename, "/media/aramaswamybalasubram/940291E20291C99E/ARUN_CONTENTS/SAVED_OVERHEAD/f_%06d.jpg", frame_count);
			cv::imwrite(filename, matDummy);
			frame_count++;
		}

		//Threshold each color red, green, blue and pink
		//NOTE: can use threads to make things faster
		cv::inRange(matCameraFeed,cv::Scalar(rH_MIN,rS_MIN,rV_MIN),cv::Scalar(rH_MAX,rS_MAX,rV_MAX),rmatThresholded);
		cv::inRange(matCameraFeed,cv::Scalar(gH_MIN,gS_MIN,gV_MIN),cv::Scalar(gH_MAX,gS_MAX,gV_MAX),gmatThresholded);
		cv::inRange(matCameraFeed,cv::Scalar(bH_MIN,bS_MIN,bV_MIN),cv::Scalar(bH_MAX,bS_MAX,bV_MAX),bmatThresholded);
		cv::inRange(matCameraFeed,cv::Scalar(pH_MIN,pS_MIN,pV_MIN),cv::Scalar(pH_MAX,pS_MAX,pV_MAX),pmatThresholded);


		//pass in thresholded frame to the object find_blobs function
		//this function will return the x and y coordinates of the
		//filtered object

		find_blobs(bmatThresholded,matCameraFeed,'b');
		find_blobs(rmatThresholded,matCameraFeed,'r');
		find_blobs(gmatThresholded,matCameraFeed,'g');
		find_blobs(pmatThresholded,matCameraFeed,'p');


		//create a thread and write the coordiantes to the remote file
		pthread_t file_writer;
		int return_var;
		return_var =  pthread_create(&file_writer, NULL, thread_write_to_shared_memory, NULL);
		if(return_var != 0) {
			std::cout<<"Error: pthread_create() failed\n";
			exit(EXIT_FAILURE);
		}

		//create a thread to draw the regions on the image
		with_box = matCameraFeed;
		//thread_draw
		pthread_t imshowthread;
		int return_var_imshow;
		return_var_imshow =  pthread_create(&imshowthread, NULL, thread_draw, NULL);
		if(return_var_imshow != 0) {
			std::cout<<"Error: pthread_create() failed\n";
			exit(EXIT_FAILURE);
		}

		//calcilate distance between teh robots and the object
		cv::Point2f obj_mid;
		obj_mid.x = (obj_center[0].x + obj_center[1].x)/2;
		obj_mid.y = (obj_center[0].y + obj_center[1].y)/2;
		std::cout<<"\n####DISTANCES bot-left: " <<find_euclidean_dis(bot_center[0],obj_center[0])<<" bot-cent: "<<
				find_euclidean_dis(bot_center[0],obj_mid)<<" bot-righ: "<<find_euclidean_dis(bot_center[0],obj_center[1])<<"\n";


		/*dist[0] -> dist between LHS blob and goal*/
		distance[0] = find_euclidean_dis(obj_center[0], goal_center[0]);

		/*dist[1] -> dist between RHS blob and goal*/
		distance[0] = find_euclidean_dis(obj_center[1], goal_center[0]);

		//JUN19 CENTROID
		centroid.x = (obj_center[0].x + obj_center[1].x)/2;
		centroid.y = (obj_center[0].y + obj_center[1].y)/2;

		/*dist[2] -> dist between centroid and and goal*/
		xdiff =  (centroid.x - goal_center[0].x);
		ydiff = (centroid.y - goal_center[0].y);
		xdiff = xdiff*xdiff;
		ydiff = ydiff*ydiff;
		distance[2]  = find_euclidean_dis(centroid, goal_center[0]);

		std::cout <<"GOAL DISTANCES: "<<distance[0] <<" & "<<distance[1] <<" CENTROID_D: "<<distance[2]<<"\n";

		//print coordinates
		std::cout<<obj_goal_xy_str;

		// if close enough, set flag to exit loop
		if((distance[0] <= MIN_DIST_TO_REACH_GOAL) || (distance[1]<=MIN_DIST_TO_REACH_GOAL ||
				distance[2]<=MIN_DIST_TO_REACH_GOAL	))
		{
			reached_goal = true;
			std::cout<<obj_goal_xy_str;
			std::cout <<"TIME TO STOP...reached_goal: "<<reached_goal<<"\n";
		}

	}
	//print total time taken
	gettimeofday(&full_endtime,0x0);
	long long full_time_delta = timeval_diff(NULL,&full_endtime,&full_starttime);
	std::cout << "FULL TIME: " << full_time_delta/1000000<<"."<<full_time_delta%1000000 << "\n";

	current_time = time(0);
	std::cout <<"\nDATE: "<<ctime(&current_time)<< "\n";

	exit(0);
}


