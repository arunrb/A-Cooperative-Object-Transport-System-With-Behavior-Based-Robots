//#####################################################################################################################
// 	robot.cpp
//
//	Created by ARUN PRASSANTH RAMASWAMY BALASUBRAMANIAN, APRIL 2017
//#####################################################################################################################

#include <khepera/khepera.h>
#include <signal.h>

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include<math.h>


using namespace cv;
using namespace std;

static knet_dev_t * dsPic; // robot pic microcontroller access
int maxsp,accinc,accdiv,minspacc, minspdec; // for speed profile
bool reached_goal = 0; // quit variable for loop


//default capture width and height
unsigned int FRAME_WIDTH =  192;
unsigned int FRAME_HEIGHT = 144;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 100;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 15*15;
unsigned int EXPECTED_OBJECT_AREA = (FRAME_HEIGHT*FRAME_WIDTH)/4;


bool in_contact = false;

/*Timer variables for the experiment*/
struct timeval experiment_starttime, experiment_endtime/*arun*/;

/*MOTOR SPEEDS */
#define MOTOR_SPEED_SPIN 400 /*old 360 | oldold330*/
#define MOTOR_SPEED_MOVE 450 /*old 450 oldold 400*/
#define MOTOR_SPEED_FAST_MOVE 450//AUG1 550 /*old 500*/
#define MOTOR_SPEED_BIAS_STEP 15
#define MOTOR_SPEED_PUSH_BIAS_STEP 20
#define IR_MAX_PROXIMITY 35 	/*how close the robot can get to the object*/

#define ERROR_CODE -11 /*error code for functions*/
#define TERMINATION_CODE -5 /*code for termination, overhead wirtes
							-55 when the bots are */


//goal is < this, then consider reached goal
#define MIN_DIST_TO_REACH_GOAL 30//prev 50  //10//to make contact//15 for the big expt
//push durations
#define MIN_PUSH_MULTIPLE 3.5//4 //3.5
#define MAX_PUSH_MULTIPLE 6//7 //6

/*absolute distance difference between LHS and RHS
should be less than this macro for COOP push..*/
#define D1D2_DIFF 40

/*Local status variables*/
const string STATUS_ACTIVE = "ACTIVE\n";
const string STATUS_PAUSED = "PAUSED\n";
const string STATUS_WAITING = "WAITING\n";
const string STATUS_DND = "DND\n";

string HOSTNAME="";
string PEERNAME="";

Mat matBRGFrame;
int global_stuck_timer = 0;

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


static int quitReq = 0; // quit variable for loop

/*--------------------------------------------------------------------*/
/*!
 * Make sure the program terminate properly on a ctrl-c
 */
static void ctrlc_handler( int sig )
{
	quitReq = 1;

	/*Total run time of the expeiment*/
	gettimeofday(&experiment_endtime,0x0);
	long long experiment_time_delta = timeval_diff(NULL,&experiment_endtime,&experiment_starttime);
	cout << "CTRL_C TOTAL EXPT TIME: " << experiment_time_delta/1000000<<"."<<experiment_time_delta%1000000 << " seconds"<<"\n";

	/*set motors to idle and release camera*/
	kh4_set_speed(0 ,0 ,dsPic); // stop robot
	kh4_SetMode( kh4RegIdle,dsPic );
	kb_camera_release();

	kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds

	kb_change_term_mode(0); // revert to original terminal if called

	time_t current_time = time(0);
	std::cout <<"\nDATE: "<<ctime(&current_time)<< "\n";

	exit(0);
}

//generate random number
int random_gen()
{
	//old int max = 5, min = 3, range;
	int max = 7, min = 5, range;
	range = max-min + 1;
	return rand()%range + min;
}




//Infrared variables
short ir_proximity_sensors[12];
char Buffer[100];
short ir_front, ir_front_left, ir_front_right;
short ir_left, ir_back_left;
int ir_error_front_right = 0;
int ir_error_front = 0;

/*IR error correction.. read error values from a local file
 * how we get the error values: right now manual.. if need to implement,
 *assuming that no obstacle around, make robot do some basic actuation..
 * move front back etc. take a bunch of
 * readings from IR sensors, calculate an average and estimate the error */
void ir_prox_calib()
{
	FILE *stream = popen("cat /home/root/PROXIMITY_CALIBRATION.txt", "r");
	char file_contents[6]="";
	int temp;

	if (stream != 0) {
		char which_ir[3]="";
		fgets(file_contents, 6,(FILE*)stream);
		//getline((FILE*)stream, line_1);
		sscanf(file_contents, "%s %d",which_ir , &temp );
		pclose(stream);

		if(strcmp(which_ir,"fr")==0)
		{
			ir_error_front_right = temp;
		} else if(strcmp(which_ir,"f")==0)
		{
			ir_error_front = temp;
		}
		cout << "########## IR ERROR CALIB: ir_error_front="<< ir_error_front<<" ir_error_front_right=" <<ir_error_front_right<<"\n";
	} else
	{
		pclose(stream);
		cout <<"ir_prox_calib: ERROR..."<<"\n";
	}
}

/*Fn to READ IR PROX SENSOR*/
void get_proximity_IR(string caller_fn)
{

	kh4_proximity_ir(Buffer, dsPic);

	ir_front = (Buffer[3*2] | Buffer[3*2+1]<<8);
	ir_front_left = (Buffer[2*2] | Buffer[2*2+1]<<8);
	ir_front_right = (Buffer[4*2] | Buffer[4*2+1]<<8);
	//JULY15
	ir_left = (Buffer[1*2] | Buffer[1*2+1]<<8);
	ir_back_left = (Buffer[0*2] | Buffer[0*2+1]<<8);

	//cout <<"ir_front with error: " << ir_front << "ir_front_error: " <<ir_error_front<< "\n";


	/*IR ERROR CORRECTION
	 * if IR value is greater than a threshold, dont do ant error correction
	 * since that would fool the bot thinking that there is nothing in fornt.
	 * also, if ir value after correction is less than 0,set a default value of 10*/
	if(ir_front_right<50) {
		ir_front_right = ir_front_right - ir_error_front_right;
		if(ir_front_right<10) {
			ir_front_right = 10;
		}
	}

	if(ir_front<50) {
		ir_front = ir_front - ir_error_front;

		if(ir_front<10) {
			ir_front = 10;
		}
	}
	//		 	std::cout << "IR:  F_left: " << ir_front_left << "  F: "
	//			<< ir_front << "  F_right: " << ir_front_right<<" | Caller: "<<caller_fn <<"\n";
	//		//get all 12
	//		/*int i =0;
	//		for(i=0;i<12;i++)
	//		{
	//			ir_proximity_sensors[i]=(Buffer[i*2] | Buffer[i*2+1]<<8);
	//
	//		}
	//		printf("\n                    near               far\
	//				\nback left      : %4u  \nleft           : %4u  \
	//				\nfront left     : %4u  \nfront          : %4u  \
	//				\nfront right    : %4u  \nright          : %4u  \
	//				\nback right     : %4u  \nback           : %4u  \n",
	//				ir_proximity_sensors[0],  ir_proximity_sensors[1],
	//				ir_proximity_sensors[2],  ir_proximity_sensors[3],
	//				ir_proximity_sensors[4],  ir_proximity_sensors[5],
	//				ir_proximity_sensors[6],  ir_proximity_sensors[7]
	//		);*/
}


/*left jerk*/
void left_jerk (int time_ms)
{
	if(time_ms<150 || time_ms> 500) 	/*current range: 150 t0 500 ms*/
		time_ms = 200000; /*default 200 ms*/
	else time_ms = time_ms*1000;

	//cout<<"<<<<<<< LEFT JERK <<<<<<"<<"\n";
	kh4_set_speed( (-320), (300), dsPic);
	usleep(time_ms);
	kh4_set_speed( 0, 0, dsPic);
	usleep(100000);
}

/*right jerk*/
void right_jerk (int time_ms)
{
	if(time_ms<150 || time_ms> 500) 	/*current range: 150 t0 500 ms*/
		time_ms = 200000; /*default 200 ms*/
	else time_ms = time_ms*1000;

	//cout<<"<<<<<<< RIGHT JERK <<<<<<"<<"\n";
	kh4_set_speed( (300), (-320), dsPic);
	usleep(time_ms);
	kh4_set_speed( 0, 0, dsPic);
}
/************** not used in current code: read commands from file  ********/
/*struct to store push duration - */
//struct push_duration_struct {
//	int self_s;
//	int	peer_s;
//};

//struct push_duration_struct read_remote_file_cmd()
//{
//	struct push_duration_struct push_time_tmp;
//	char bot_1_name [2] = "";
//	char bot_2_name [2] = "";
//	int bot_1_duration = -1;
//	int bot_2_duration = -1;
//
//	char file_contents[8] = "";
//
//	FILE *stream = popen("ssh root@10.42.0.17 cat /home/root/SHARED_MEMORY/OVERHEAD.txt", "r");
//
//	if (stream != 0) { /*file stream readable*/
//		fgets(file_contents, 10,(FILE*)stream);
//		sscanf(file_contents, "%c %d %c %d",bot_1_name , &bot_1_duration, bot_2_name,&bot_2_duration );
//		pclose(stream);
//
//		std::cout << "\t\t=== Read FILE (bot,time): (" <<bot_1_name << ","<<
//				bot_1_duration<<") & (" <<bot_2_name << ","<< bot_2_duration<<")" <<"====="<<"\n";
//
//		/*current assumption is that k6 is LHS and k9 is RHS*/
//		if(HOSTNAME  == "khepera4_1006")
//		{
//			std::cout <<"READ REMOTE CMD.. K6 - left"<<"\n";
//			if(bot_1_duration == TERMINATION_CODE)
//			{
//				reached_goal = true;
//				std::cout <<"TIME TO STOP...reached_goal: "<<reached_goal<<"\n";
//			}
//			push_time_tmp.self_s = bot_1_duration;
//			push_time_tmp.peer_s = bot_2_duration;
//			return push_time_tmp;
//		} else if(HOSTNAME  == "khepera4_1009")
//		{
//			std::cout <<"READ REMOTE CMD.. K9 - right"<<"\n";
//			if(bot_2_duration == TERMINATION_CODE)
//			{
//				reached_goal = true;
//				std::cout <<"TIME TO STOP...reached_goal: "<<reached_goal<<"\n";
//			}
//			push_time_tmp.self_s = bot_2_duration;
//			push_time_tmp.peer_s = bot_1_duration;
//			return push_time_tmp;
//		} else {
//			std::cout << "*** Different bot? ****"<<"\n";
//			push_time_tmp.self_s = push_time_tmp.peer_s = ERROR_CODE;
//			return push_time_tmp;
//		}
//	} else /*unable to read file*/
//	{
//		pclose(stream);
//		std::cout <<"read_file_remote: unable to read"<<"\n";
//		push_time_tmp.self_s = push_time_tmp.peer_s = ERROR_CODE;
//		return push_time_tmp;
//	}
//}

//read the peer's current status
string read_peer_status()
{
	char dummy_char[14] = "";

	ostringstream read_peer_cmd;
	read_peer_cmd<< "ssh root@"<<PEERNAME<<" cat /home/root/SHARED_MEMORY/LOCAL_STATUS.txt" ;
	string read_peer_cmd_str = read_peer_cmd.str();
	FILE *peerstream = popen (read_peer_cmd_str.c_str(), "r");
	if (peerstream != 0) {
		fgets(dummy_char, 14,(FILE*)peerstream);
		pclose(peerstream);
		//LLL 	cout << "%%%%%%%%%%%%% read_peer_status READINGGG: " << dummy_char<<"\n";
	} else {
		//LLL cout <<"======= READ ERROR: read_peer_status =====" << "\n";
		pclose(peerstream);
	}

	return dummy_char;
}

/*	Get the hostname
 * 	peer's hostname is hardcoded as of now
 * */
void get_hostname_peername()
{
	FILE *hoststream = popen("hostname", "r");
	if (hoststream != 0) {
		char temp[14] = "";
		fgets(temp, 14,(FILE*)hoststream);
		pclose(hoststream);
		HOSTNAME = temp;

		cout << "%%%%%%%%%%%%% HOSTNAME: " << HOSTNAME<<"\n";
	} else {
		cout <<"======= READ ERROR: get_hostname_peername =====" << "\n";
		pclose(hoststream);
	}

	string k6_hostname ="khepera4_1006";
	string k9_hostname ="khepera4_1009";

	if(HOSTNAME  == k9_hostname)
	{
		PEERNAME = k6_hostname;
		cout << "======== HOST: "<<HOSTNAME<< "  PE: "<<PEERNAME<<"======== "<<"\n";
	} else if (HOSTNAME == k6_hostname)
	{
		PEERNAME = k9_hostname;
		cout << "======== HOST: "<<HOSTNAME<< "  PE: "<<PEERNAME<<"======== "<<"\n";
	} else
	{
		cout << "============= UNKNOWN BOT(S).. ======== "<<"\n";
		cout << "======== HOST: "<<HOSTNAME<< "  PE: "<<PEERNAME<<"======== "<<"\n";
	}
}

//update own status
void write_status_local(string local_status)
{


	FILE *writestream = popen("cat > /home/root/SHARED_MEMORY/LOCAL_STATUS.txt", "w");
	if (writestream != 0) {
		fputs(local_status.c_str(), (FILE*)writestream);
		pclose(writestream);
		//LLL 	cout <<"success!! write_status_local: "<< local_status<<" "<<"\n";
	} else
	{
		pclose(writestream);
		//LLL 	cout <<"Failed!! write_status_local: "<< local_status<<" "<<"\n";
	}
}




/*global variables for storing overhead cam hints*/
std::vector<cv::Point2f> bot_center(2), obj_center(2), goal_center(1);
cv::Point2f color_bot; //JULY25
cv::Point2f obj_mid; //JULY26

/*
 * Read the shared file and extract the coordinates
 * */
void read_file_remote_points()
{
	std::vector<cv::Point2f> read_obj_center(2), read_goal_center(1), read_bot_center(2);
	cv::Point2f temp;
	char file_contents[100] = "";

	temp.x = temp.y = 0;
	read_obj_center[0] = read_obj_center[1] = temp;
	goal_center[0] = temp;

	FILE *stream = popen("ssh root@10.42.0.17 cat /home/root/SHARED_MEMORY/OVERHEAD.txt", "r");
	if (stream != 0) {
		char color_code[]="";

		fgets(file_contents, 100,(FILE*)stream);
		sscanf(file_contents, "%s %f %f %f %f %s %f %f %s %f %f %f %f",color_code , &read_obj_center[0].x,
				&read_obj_center[0].y, &read_obj_center[1].x, &read_obj_center[1].y,
				color_code , &read_goal_center[0].x, &read_goal_center[0].y, color_code,
				&read_bot_center[0].x, &read_bot_center[0].y, &read_bot_center[1].x, &read_bot_center[1].y);
		pclose(stream);

		static cv::Point2f fixed_goal_center = read_goal_center[0];
		obj_center[0] = read_obj_center[0];
		obj_center[1] = read_obj_center[1];
		goal_center[0] = fixed_goal_center;
		bot_center[0] = read_bot_center[0];
		bot_center[1] = read_bot_center[1];


		if(HOSTNAME  == "khepera4_1006")
		{
			color_bot = bot_center[0];
		} else if(HOSTNAME  == "khepera4_1009")
		{
			color_bot = bot_center[1];
		} else {
			std::cout<<"########## UNKNOWN BOT ###############\n";
		}

		obj_mid.x = (obj_center[0].x +obj_center[1].x)/2;
		obj_mid.y = (obj_center[0].y +obj_center[1].y)/2;

		//		std::cout << "=========== Read ONE line: "<<"  obj_center=" <<obj_center[0]<<obj_center[1]<<"\n";
		//    	std::cout << "=========== "<<"  goal_center=" <<goal_center[0]<<"\n";

	} else
	{
		pclose(stream);
		//LLL 	std::cout <<"read_file_remote: unable to read"<<"\n";
	}
}


//struct to store distance D1 and D2
struct s_distances {
	std::vector<double> distance;
};

//fn to calculate distances
s_distances distances_from_coordinates()
{
	double xdiff, ydiff;
	std::vector<double> distance(2) ;

	struct s_distances dist_instance;


	/*dist[0] -> dist between LHS blob and goal*/
	xdiff =  (obj_center[0].x - goal_center[0].x);
	ydiff = (obj_center[0].y - goal_center[0].y);
	xdiff = xdiff*xdiff;
	ydiff = ydiff*ydiff;
	distance[0] = sqrt(xdiff+ydiff);

	/*dist[1] -> dist between RHS blob and goal*/
	xdiff =  (obj_center[1].x - goal_center[0].x);
	ydiff = (obj_center[1].y - goal_center[0].y);
	xdiff = xdiff*xdiff;
	ydiff = ydiff*ydiff;
	distance[1] = sqrt(xdiff+ydiff);


	dist_instance.distance = distance;
	//LLL 	std::cout <<"FN_DISTANCES: "<<dist_instance.distance[0] <<" & "<<dist_instance.distance[1] <<"\n";
	return dist_instance;
}

//records and plays sound for 2s. used when stuck to let users know.
void* play_sound(void* arg)
{

#define SAMPLING_FREQ 22050 // sampling frequency [Hz]
#define SAMPLE_SIZE 16 // bits of the sample
#define NB_CHANNELS 2 // 2 for stereo
#define LENGTH 2 // [s] duration of the sound
#define ENDIAN 0 //endianness: 0 little, 1 BIG
#define SIGNED 1 // sign : 0 unsigned, 1 signed
#define NB_SAMPLES (SAMPLING_FREQ*NB_CHANNELS*LENGTH*SAMPLE_SIZE/8)

	char sound[NB_SAMPLES];
	int err;

	// initialise sound
	if (kb_sound_init()<0)
	{
		fprintf(stderr,"Error: Unable to initialize the sound!\r\n");
	}

	// mute Ultrasounds
	kh4_activate_us(0,dsPic);

	// set the volume of the speakers	to 0 for record: needs to be off during recording otherwise
	// loop noise appears!
	set_speakers_volume(0,0);

	// set the volume of the microphones (in %)
	set_microphones_volume(100,100);


	// configure the sound: (sampling_frequency,sample_size = nb of bits,sign: 0 = unsigned,endian : 0 = LSB,channels)
	if ((err=kb_sound_configure(SAMPLING_FREQ,SAMPLE_SIZE,SIGNED,ENDIAN,NB_CHANNELS))<0)
	{
		fprintf(stderr,"Error: Unable to configure sound: error number %d!\r\n",err);
		kb_sound_release();
	}

	// set "hardware" speaker switch off
	mute_speaker(1);
	switch_speakers_ON_OFF(0);

	printf(" Start recording:\n");
	record_buffer(sound,NB_SAMPLES);

	// set the volume of the microphones (in %)
	set_microphones_volume(0,0);
	// set "hardware" speaker switch on
	switch_speakers_ON_OFF(1);
	mute_speaker(0);

	// set the volume of the speakers	(in %)
	set_speakers_volume(100,100);


	usleep(1000000); // waits 1s
	printf("\n Play recorded sound\n");
	play_buffer(sound,NB_SAMPLES);

	wait_end_of_play();


	switch_speakers_ON_OFF(0);
	mute_speaker(1);

	kb_sound_release();

	// unmute Ultrasounds
	kh4_activate_us(31,dsPic);
	pthread_exit(NULL);
}



/* JULY26 2018
 * bool find_intersection:
 * 	takes two lines (4 points) and finds where they intersect
 * 	RETURNS 1 if intercept withing the domain, else return false
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

//cooperative push
void coop_push(struct s_distances dist_inst_coop)
{

	global_stuck_timer = 0;

	write_status_local(STATUS_WAITING);

	/*read peer status*/
	string peer_status = read_peer_status();

	float dist_diff = 0; /*difference in distance from the goal*/

	//LLL 	cout << "push_two_bots: WORK TOGETHER" << "\n";

	int coop_fuse = 0;
	//struct s_distances dist_inst_coop;
	while(peer_status != STATUS_WAITING)
	{
		usleep(150000); // old 150000
		peer_status = read_peer_status();
		if(peer_status == STATUS_WAITING) //peer ready for COOP, break loop
			break;

		/*make sure if COOP status is still the same*/
		read_file_remote_points();

		dist_inst_coop = distances_from_coordinates();
		dist_diff = dist_inst_coop.distance[0]-dist_inst_coop.distance[1];

		if(abs(dist_diff)>D1D2_DIFF) //status has changer, go out
		{
			//LLL 	std::cout<<"coop_push: distance diff more than threshold"<<"\n";
			break;
		}

		if(coop_fuse>18) //timeout, go out of the loop
		{
			//LLL 	cout << "coop_push: COOP FUSE" << "\n";
			break;
		}
		coop_fuse++;
		//LLL cout << "coop_push: WAITING IN WHILE" << "\n";
	}

	/*push_pulse is the time for pushing which would be multiplied with
	 *the basic unit of time which is 1.2s as of June 12. push_pulse is
	 *calculated based on the distance btw the center od the object and
	 *the goal*/
	float push_pulse = (dist_inst_coop.distance[0]+dist_inst_coop.distance[1])/2;
	push_pulse = (push_pulse/100);

	if(push_pulse < MIN_PUSH_MULTIPLE) {
		push_pulse = MIN_PUSH_MULTIPLE;
	} else if(push_pulse > MAX_PUSH_MULTIPLE) {
		push_pulse = MAX_PUSH_MULTIPLE;
	}

	if(peer_status==STATUS_WAITING && (abs(dist_diff)<=D1D2_DIFF))
	{
		//LLL cout << "coop_push: COOP PUSH" << "\n";
		kh4_set_speed(500 , 500, dsPic);
		usleep(150000);
		kh4_set_speed(700 , 700, dsPic);
		usleep(1200000*push_pulse); //cooperative push duration
		kh4_set_speed( 0, 0, dsPic);

		std::cout<<"COOP_PUSH push_pulse: "<<push_pulse*1.2<<"\n";

		/*update local status*/
		write_status_local(STATUS_PAUSED);
		int rand_num = rand()%9+1;
		//LLL cout << "coop_push: COOP SLEEPIN for rand_num: "<< rand_num<< "\n";
		usleep(100000*rand_num);
		return;
	} else
	{
		/*update local status*/
		write_status_local(STATUS_PAUSED);
		//LLL cout << "push_two_bots: COOP cancelled" << "\n";
	}
}


/*
 * Function to align the box*/
int log_id_counter = 0;
void push_to_align_for_coop()
{

	log_id_counter++;
	//LLL 	std::cout<<"angle_push_to_align\n";

	/*read peer status*/
	string peer_status = read_peer_status();

	if(peer_status==STATUS_ACTIVE || peer_status == STATUS_DND)
	{
		/*update local status*/
		write_status_local(STATUS_PAUSED);
		return;
	} else if(peer_status==STATUS_PAUSED)
	{
		/*set local status active*/
		write_status_local(STATUS_ACTIVE);

		cv::Point2f zero_point;
		zero_point.x = zero_point.y = 0;

		/*No goal seen !*/
		if(goal_center[0] == zero_point)
		{
			std::cout<< "%%%%%%%%%%%%%%% DUNNO WHERE TO GO%%%%%%%%%%%%%%%" <<"\n";
			/*update local status*/
			write_status_local(STATUS_PAUSED);
			return;
		}
		else if((obj_center[0] == zero_point ) || (obj_center[1] == zero_point))
		{
			/*Requirement is that object should always be seen.
			 * if both object and bot are known, then common centre is the average
			 * else common centre is just the centre of the obj.
			 * Assumpption: the bot may or may not be seen cos of the small size.
			 *  If bot is seen, it is like additional info*/

			std::cout <<"%%%%%%%%%%%%%%% Object missin %%%%%%%%%%%%%%%%%%%%"<<"\n";
			/*update local status*/
			write_status_local(STATUS_PAUSED);
			return;
		}

		/*read coordinates before pushing*/
		read_file_remote_points();


		static struct s_distances prev_dist;
		prev_dist = distances_from_coordinates();
		std::cout<<"\nB4 GOAL: "<<goal_center[0]<<" OBJ : "<<obj_center[0]<<obj_center[1]<<
				"\nB4    d1: "<<prev_dist.distance[0]<<" d2: "<<prev_dist.distance[1]<<"\n";

		int pos,other;
		pos = other =-1;
		pos = (prev_dist.distance[0]>prev_dist.distance[1]) ? 0 : 1;
		other = (pos == 0 )? 1 : 0;

		//push
		kh4_set_speed(500 , 500, dsPic);
		usleep(150000);
		kh4_set_speed(700 , 700, dsPic);
		usleep(1100000); //for 3 point box
		kh4_set_speed(0 , 0, dsPic);

		/*read coordinates after pushing*/
		//JULY06 sleep(2);
		usleep(850000);  //JULY06
		read_file_remote_points();

		//calculate status before and after pushing
		struct s_distances curr_dist;
		curr_dist = distances_from_coordinates();
		std::cout<<"AF GOAL: "<<goal_center[0]<<" OBJ : "<<obj_center[0]<<obj_center[1]<<
				"\nAF  d1: "<<curr_dist.distance[0]<<" d2: "<<curr_dist.distance[1]<<"\n";

		double diff_pos, diff_other;
		diff_pos = prev_dist.distance[pos] - curr_dist.distance[pos];
		diff_other = prev_dist.distance[other] - curr_dist.distance[other];

		cout <<"NEW_LOGIC-- diff_pos: "<<diff_pos<<" diff_other: "<<diff_other<<"\n";
		if ((diff_pos <=0) ||
				((diff_pos > 0) && (diff_other>0) && (diff_other >= diff_pos)))
		{
			/*set local status paused*/
			write_status_local(STATUS_PAUSED);
			cout <<"\n"<<log_id_counter<<"-------WRONG_BOT paused"<<"\n";
			if(abs(curr_dist.distance[0]-curr_dist.distance[1])>D1D2_DIFF)
			{
				cout <<"\n"<<log_id_counter<<"-------WRONG_BOT SLEEP"<<"\n";
				//JULY06 sleep(7);
				sleep(4);  //JULY06
			}
		}

	}
}



/*
 * Read coodrinates and distaces adn decide
 * whether to align on cooperatively push
 * */
void push_or_cooperative_push()
{
	/*PUSH!!
	 * NOTE: 100 is just a trial value*/
	in_contact = true;

	//LLL 	cout << "######## push_overhead_coordinates_COOP_angle" << "\n";

	struct s_distances s_inst;

	read_file_remote_points();
	s_inst = distances_from_coordinates();

	float dist_diff = 0; /*difference in distance from the goal*/

	dist_diff = s_inst.distance[0]-s_inst.distance[1];

	if((s_inst.distance[0] <= MIN_DIST_TO_REACH_GOAL) || (s_inst.distance[1]<=MIN_DIST_TO_REACH_GOAL))
	{
		reached_goal = true;

	}
	else if(abs(dist_diff)<=D1D2_DIFF/*15*/) /*push together*/
	{
		coop_push(s_inst);
	}
	else	//align
	{
		push_to_align_for_coop();
	}

}

//givena an angle, turn right
void turn_right(int angle)
{
	// 500 ms =  ~45 degrees @ 350-370 speed
	// 200 ms =   ~22 degrees @ 350-370 speed
	int time_ms;
	time_ms = angle*11; // old 11.5
	int local_speed = 0;


	if(HOSTNAME  == "khepera4_1006")
	{
		time_ms = angle*11.5;
		local_speed = 370;
	} else if (HOSTNAME  == "khepera4_1009")
	{
		time_ms = angle*11;
		local_speed = 425;
	} else {
		time_ms = angle*11;
		local_speed = 425;
	}


	if(time_ms < 200)
	{
		time_ms = 200;
	}

	std::cout <<"TURN: angle = "<<angle<<" time_ms: "<<time_ms<<"\n";

	kh4_set_speed( (local_speed), (-local_speed), dsPic); //old 350, -370
	usleep(time_ms*1000);
	kh4_set_speed( 0, 0, dsPic);
}


//box follow behavior
void box_follow()
{
	//write_status_local(STATUS_ACTIVE);	//JULY26 STATUS
	write_status_local(STATUS_DND); //AUG2

	kh4_set_speed( (-400), (-400), dsPic);
	usleep(500*1000);
	kh4_set_speed( 0, 0, dsPic);

	turn_right(90);

	//speed bias for robots as the wheels have issues.
	int add = 0;
	if(HOSTNAME  == "khepera4_1006")
	{
		add = 0;
	} else if (HOSTNAME  == "khepera4_1009")
	{
		add = 20;
	} else {
		add = 0;
	}

	get_proximity_IR("bx folo");
	kh4_set_speed( (400+add), (400+add), dsPic);
	int box_follow_fuse = 0;


	while(1)
	{
		if(box_follow_fuse  > 200)
		{
			pthread_t mysound3;
			int return_var;
			return_var =  pthread_create(&mysound3, NULL, play_sound, NULL);
			if(return_var != 0) {
				std::cout<<"Error: pthread_create() failed fo play sound\n";
				exit(EXIT_FAILURE);

				break;
			}
		}

		box_follow_fuse++;

		get_proximity_IR("bx folo");

		std::cout<<"IR: L = "<<ir_left<<" BK_L = "<<ir_back_left<<"\n";
		if(ir_left < 100) //moving away, come back
		{
			kh4_set_speed( (400+add), (450+add), dsPic);//orig: 400, 430
		}

		if(ir_left > 200) // too close, move away
		{
			if(ir_left>800) //too too close, go fast
				kh4_set_speed( (550+add), (400+add), dsPic);
			else
				kh4_set_speed( (450+add), (400+add), dsPic); //orig 430, 400
		}

		if(ir_left > 100 && ir_left < 200) //continue
		{
			kh4_set_speed( (400+add), (400+add), dsPic);
		}

		if(ir_left < 25 && ir_back_left > 30 ) //turn left
		{

			kh4_set_speed( (400+add), (700), dsPic);
			std::cout<<"LEFT TURN\n";
		}

		if(ir_front>40)
		{
			std::cout<<"STOP..\n";
			kh4_set_speed( (0), (0), dsPic);
			break;
		}

		usleep(300*1000);
	}
	write_status_local(STATUS_PAUSED);
}


//Find safe zone - old. replaced by find_safe_zone_2()
//Robots can push while they are within this region.
//returns -1 when point is outside the polygon
//returns 0  when point is on boundary
//returns 1 when point is inside polygon
int find_safe_zone()
{
	std::vector<cv::Point2f> vec_safe_zone;
	cv::Point2f topleft, bottomleft, topright, bottomright, intersection, zerozero;
	bool result;
	int poly_return_safe;
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

	//order of insertion matters!!!
	//THis is the last point
	vec_safe_zone.push_back(obj_center[1]);

	int i ;
	for(i=0 ; i<vec_safe_zone.size(); i++)
		std::cout<<vec_safe_zone[i]<<"\n";

	if(color_bot == zerozero )
	{
		std::cout<<"BOT NOT SEEN\n";
		return -11;

	}

	//returns -1 when point is outside the polygon
	//returns 0  when point is on boundary
	//returns 1 when point is inside polygon
	poly_return_safe = pointPolygonTest(vec_safe_zone, color_bot, false);

	std::cout<<"##### IS SAFE: "<<poly_return_safe<<"\n";
	return poly_return_safe;
}


//Find safe zone
//Robots can push while they are within this region.
//returns -1 when point is outside the polygon
//returns 0  when point is on boundary
//returns 1 when point is inside polygon
int find_safe_zone_2()
{
	std::vector<cv::Point2f> vec_local;

	cv::Point2f topleft, bottomleft, topright, bottomright, intersection, zerozero;
	//std::vector<cv::Point2f> vec_safe_zone;
	cv::Point2f interleft, interright, intertop, interbottom;
	bool result;
	bool left, right, top, bottom;
	left = right  = top = bottom = false;

	int poly_return_safe;

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

	//************************************************************************************************************
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

	if(color_bot == zerozero )
	{
		std::cout<<"BOT NOT SEEN\n";
		return -11;

	}

	//returns -1 when point is outside the polygon
	//returns 0  when point is on boundary
	//returns 1 when point is inside polygon
	poly_return_safe = pointPolygonTest(vec_local, color_bot, false);

	std::cout<<"##### IS NEW SAFE: "<<poly_return_safe<<"\n";
	return poly_return_safe;
}


//Find hazard zone
//robot cannot push when within this zone
//returns -1 when point is outside the polygon
//returns 0  when point is on boundary
//returns 1 when point is inside polygon
int find_hazard_zone()
{
	cv::Point2f topleft, bottomleft, topright, bottomright, intersection, null;
	std::vector<cv::Point2f> vec_hazard_zone;
	bool result;
	int poly_return_hz;

	null.x = null.y = 0;

	read_file_remote_points();


	vec_hazard_zone.push_back(goal_center[0]);

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
	{
		std::cout<<vec_hazard_zone[i]<<"\n";
	}

	if(color_bot == null )
	{
		std::cout<<"BOT NOT SEEN\n";
		return -11;

	}

	//pointPolygonTest: https://stackoverflow.com/questions/11210027/pointpolygontest-in-opencv
	//https://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=pointpolygontest#pointpolygontest
	//returns -1 when point is outside the polygon
	//returns 0  when point is on boundary
	//returns 1 when point is inside polygon
	poly_return_hz = pointPolygonTest(vec_hazard_zone, color_bot, false);

	std::cout<<"##### HAZARD:  "<<poly_return_hz<<"\n";

	return poly_return_hz;
}

//robot has found something that looks red.
//decide what to do based on the region it is in
int coop_or_go_around()
{
	cv::Point2f  null;
	int in_hazard, is_safe;


	null.x = null.y = 0;

	read_file_remote_points();

	//some vale is missing!!
	if(color_bot == null || goal_center[0] == null ||
			obj_center[0] == null || obj_center[1] == null)
	{
		std::cout<<"\n##### SOMETHING NOT SEEN  !!! #####\n coop_or_go_around: bot="
				<<color_bot<<" goal="<<goal_center[0]<<" obj="<< obj_center[0]<<obj_center[1]<<"\n";
		if(color_bot == null)
			return 0;
	}

	//find the regions
	in_hazard = find_hazard_zone();
	//is_safe = find_safe_zone();
	is_safe = find_safe_zone_2();

	if(in_hazard == -11 || is_safe == -11) //error, do nothing
		return 0;

	if(in_hazard < 1 && is_safe < 0)	//not near the object, just backoff
	{
		//backoff for 1.2 seconds
		std::cout<<"\nBACK coop_or_go_around\n";
		kh4_set_speed(-450 , -450, dsPic); /*old 400*/
		usleep(200000);
		kh4_set_speed(-500 , -500, dsPic);
		usleep(400000);
		kh4_set_speed(-550 , -550, dsPic);
		sleep(1);
		kh4_set_speed( 0, 0, dsPic);
		//turn_right(180);

	}else if(in_hazard > -1)	//wrong side of the object, do box-follow
	{
		box_follow();
	} else if(is_safe > -1)		//on right side, push
	{
		std::cout<<"start_coop\n";
		push_or_cooperative_push();
	}
	return 1;
}


//go gorwadd for given time
void go_fwd(int time_ms)
{
	if(time_ms < 400 )
	{
		time_ms = 700;
	}

	if(time_ms > 3000 )
	{
		time_ms = 3000;
	}

	std::cout <<"FWD :"<<time_ms<<"\n";

	kh4_set_speed( (500), (550), dsPic); //old 450
	usleep(time_ms*1000);
	kh4_set_speed( 0, 0, dsPic);
}

//while searching if there is any obstacle,
//back off and turn away
void avoid_obs_in_search()
{
	if(ir_front > 60)
	{
		std::cout<<"search TURN 180 IR: F = "<<ir_front<<"\n";
		kh4_set_speed( (-400), (-400), dsPic);
		usleep(500*1000);
		kh4_set_speed( 0, 0, dsPic);

		turn_right(180);
	}

}


//function to calculate euclidean distance between two points
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


//recovery behavior for search
int search_stuck_timer = 0;
void stuck_recover()
{
	std::cout<<"stuck_recover\n";

	//make nouse
	pthread_t mysound;
	int return_var;
	return_var =  pthread_create(&mysound, NULL, play_sound, NULL);
	if(return_var != 0) {
		std::cout<<"Error: pthread_create() failed fo play sound\n";
		exit(EXIT_FAILURE);
	}

	//jerk and try to recover
	left_jerk (170);
	right_jerk (180);
	left_jerk (200);
	right_jerk (180);
	left_jerk (200);
	right_jerk (180);

	kh4_set_speed(-450 , -450, dsPic); /*old 400*/
	usleep(200000);
	kh4_set_speed(-500 , -500, dsPic); //backoff
	right_jerk (180);
	usleep(200000);
	right_jerk (180);					//jerk again
	kh4_set_speed(-600 , -600, dsPic);	//backoff
	sleep(1);
	kh4_set_speed(-550 , 550, dsPic);	//turn away
	sleep(1);
	kh4_set_speed( 0, 0, dsPic);


}

//turn function.. if fornt IR picks signigicant reading,
//turn away and continue
int read_ir_turn_180(int when)
{
	get_proximity_IR("from search");
	if(ir_front > 60)
	{
		std::cout<<"search step:"<< when<<" TURN 180 IR: F = "<<ir_front<<"\n";
		kh4_set_speed( (-500), (-500), dsPic);
		usleep(600*1000);
		kh4_set_speed( 0, 0, dsPic);

		turn_right(60);
		return 1;
	} else {
		return 0;
	}
}

//search using overhead info
#define PI 3.1415
int new_search()
{
	std::cout<<"Overhead search \n";
	write_status_local(STATUS_ACTIVE); //JULY26 STATUS


	cv::Point2f  obot, bot,obj, null,lastbot;
	null.x = null.y = 0;

	if(read_ir_turn_180(0)) //if some obstacle, turn
	{
		return 0;
	}

	read_file_remote_points();	//if some obstacle, turn
	obot = color_bot;

	go_fwd(1800);
	usleep(900000);

	if(read_ir_turn_180(1))	//if some obstacle, turn
	{
		return 0;
	}

	read_file_remote_points();

	bot = color_bot;
	obj = obj_mid;//obj_center[1] ;

	std::cout<<"POINTS: old_bot:"<< obot <<" curr_bot:"<<bot<<" Obj:"<<obj<<"\n";

	if(obot == null || bot == null || obj == null)
	{
		std::cout<<"MISSING VALUE:......\n";
		return 0;
	}


	double obot_bot, bot_obj, obot_obj, new_bot_obj_dist, lastbot_bot;

	//obot_bot
	obot_bot = find_euclidean_dis(obot, bot);

	//bot_obj
	bot_obj = find_euclidean_dis(bot, obj);

	//obot_obj
	obot_obj = find_euclidean_dis(obot, obj);

	double temp = ((obot_bot*obot_bot)+(bot_obj*bot_obj)-(obot_obj*obot_obj));
	temp = temp/(2*obot_bot*bot_obj);

	double angle = acos(temp) * 180/PI;
	std::cout<<"ANGLE: "<<angle<<"\n";
	angle = 180 - angle;

	std::cout<<"ANGLE: "<<angle<<"\n";

	if(std::isnan(angle)) {
		std::cout<<"NAN: "<<angle<<"\n";
	}
	turn_right(angle);

	if(read_ir_turn_180(2)) //if some obstacle, turn
	{
		return 0;
	}

	go_fwd(1800);

	usleep(900000);
	read_file_remote_points();

	bot = color_bot;
	obj = obj_mid;//obj_center[1] ;


	//new bot_obj
	new_bot_obj_dist = find_euclidean_dis(bot, obj);

	if(new_bot_obj_dist > bot_obj)
	{
		int finish_angle;
		finish_angle = 180-angle;
		finish_angle = finish_angle*2;
		std::cout <<"corrected angle..\n";
		turn_right(finish_angle);
		go_fwd(3000);
	} else {
		std::cout<<"right direction...\n";
		go_fwd(3000);
	}

	read_file_remote_points();
	lastbot = color_bot;

	lastbot_bot = find_euclidean_dis(lastbot, bot);

	if(abs(lastbot_bot + obot_bot)/2 < 5)
	{
		search_stuck_timer++;
		std::cout<<"STUCK? lastbot_bot = "<<lastbot_bot<<" obot_bot = "<<obot_bot<<"\nstuck timer: " <<search_stuck_timer<<"\n";
		if(search_stuck_timer > 1)
		{
			stuck_recover();
			search_stuck_timer = 0;
		}
	} else {
		search_stuck_timer = 0;
		std::cout<<"STUCK? lastbot_bot = "<<lastbot_bot<<" obot_bot = "<<obot_bot<<"\nstuck timer: " <<search_stuck_timer<<"\n";
		std::cout<<"Not stuck\n";
	}

	write_status_local(STATUS_PAUSED);
	return 1;
}


//decide on which search to perform
//if in safe region or close to the object, spin in place and try to find red
//if far away or spin-serch timed out, use overhead info for search
int stuck_timer =0;
void full_search()
{
	read_file_remote_points();
	int is_safe;
	is_safe = find_safe_zone();
	float dl,dm,dr;
	dl = find_euclidean_dis(color_bot,obj_center[0]);
	dm = find_euclidean_dis(color_bot,obj_mid);
	dr = find_euclidean_dis(color_bot,obj_center[1]);

	if((is_safe > -1) || (dl < 200 || dm < 200 || dr < 200))
	{
		{
			//rotate, and continue.. the loop will take care of the rest
			//in every iteration, a frame will be captured and if red is
			//seen, the regular behaviors will kick in and take care of things
			kh4_set_speed(-MOTOR_SPEED_SPIN , MOTOR_SPEED_SPIN, dsPic);
			stuck_timer++;
			usleep(600000);
			kh4_set_speed(0 , 0, dsPic);

			if(stuck_timer > 10) // timeout, do ovearhead search
			{
				std::cout <<"STUCK TIMER: "<< stuck_timer <<"\n";
				new_search();	//overhead info search
				stuck_timer = 0;
			}
		}
	} else {
		new_search();	//overhead info search
	}
}


//keep track of position of the robot from the overhead info
int corner_backoff_fuse = 0;
struct memory
{
	cv::Point2f mbot;
	double distance;
};

int memorycount = 0;
struct memory roll_mem;

//calculate the average distance travelled by the bot
void avg_dist_travelled ()
{
	std::cout<<"#####roll_mem dist: "<<roll_mem.distance<<" mbot: "<<roll_mem.mbot<<" f: "<<memorycount<<"\n";
	double temp = find_euclidean_dis(roll_mem.mbot, color_bot);
	roll_mem.distance = (roll_mem.distance + temp) / 2 ;
	std::cout<<"##### AVG DIST: "<<roll_mem.distance<<" temp_dist= "<<temp<<" f: "<<memorycount<<"\n";
	roll_mem.mbot = color_bot;
}

//this functions executes the approach object or reposition based on the
//red pixels seen and the infrared sensor readings
int roll_the_bot(KeyPoint current_interest, float total_area_of_blobs, Mat &matThresholded)

{
	std::cout <<"roll_fn\n";

	int speed_bias_left = 0, speed_bias_right = 0;
	int object_found = 0;
	int turn_factor_int; 				/*value from 1 to 10: factor by which we multiply the
										MOTOR_SPEED_BIAS_STEP which is like the base unit*/

	float distance_from_centre = 0;		/*frame_centre's x value - x value of interest's center
	 	 	 	 	 	 	 	 	 	 +ve distance => object on LHS ; -ve obj on RHS*/
	char turn_direction = 'X';

	Point2f image_centre;
	image_centre.x = matThresholded.cols/2;
	image_centre.y = matThresholded.rows/2;


	/*if no obj of interest - avoidinga junk val
	 * major change after backup_GOLD_IR*/
	if(current_interest.pt.x == 0 &&
			current_interest.pt.y == 0)
	{
		distance_from_centre =0;
		turn_factor_int =0;
	}

	//calculate speed left and right wheel speed bias based on the
	//position of the red blob
	if(current_interest.size>0 && (current_interest.pt.x != 0)) {
		/*Turn L or R?
		 * (centre.x - current_interest.pt.x) > 0 means the object is
		 * located in the left half of the image i.e. need to turn left
		 * and vice versa
		 * */
		distance_from_centre = image_centre.x - current_interest.pt.x;
		if( distance_from_centre > 0) 			/*object in left half*/
		{
			turn_direction = 'L';
			turn_factor_int = distance_from_centre/10;
			if (turn_factor_int>10)
				turn_factor_int = 10;/*try this part first*/

			speed_bias_right = MOTOR_SPEED_BIAS_STEP*turn_factor_int;
		} else if( distance_from_centre < 0) 	{ 					/*object in the right half*/
			turn_direction = 'R';
			turn_factor_int = (distance_from_centre/10)*(-1);
			if (turn_factor_int>10)
				turn_factor_int = 10;/*try this part first*/

			speed_bias_left = MOTOR_SPEED_BIAS_STEP*turn_factor_int;
		}

		std::cout <<"roll_fn: speed bias l and r = "<<speed_bias_left<<","<<speed_bias_right<<"\n";
	}

	get_proximity_IR("roll_the_bot");

	unsigned int red_pixel_count = countNonZero(matThresholded);


	if(ir_front < IR_MAX_PROXIMITY) /*FRONT ir < 30 means obj is around 15cm away or less,
						moving much closer would affect visibility, so switching to
						step by step actions */
	{
		corner_backoff_fuse = 0; //reset corner case backoff fuse
		if((current_interest.size > 59) || (red_pixel_count > EXPECTED_OBJECT_AREA )
				||(total_area_of_blobs > 59)/*not working as expected, use IR*/)
		{
			/* aramaswamybalasubram
			 *  not needed (to stop)
			 * Apr 15, 2018 2:48:19 PM
				cout <<" \tARUN:   \t ************  STOP  ************ " <<
						"\n"<<"*** red_pixel_count: " <<red_pixel_count<<"/"
						<<FRAME_HEIGHT*FRAME_WIDTH <<"*** "<< "\n";
				kh4_set_speed( 0, 0, dsPic);
				//reset stuck_timer
				stuck_timer =0;
				//sleep(3);
				object_found = 1;
			 */
		}

		else if((current_interest.size >= 4) && (current_interest.size < 59) && //mroe red, approach faster
				(red_pixel_count < EXPECTED_OBJECT_AREA))
		{
			kh4_set_speed( (MOTOR_SPEED_FAST_MOVE + speed_bias_left), (MOTOR_SPEED_FAST_MOVE + speed_bias_right), dsPic);
			std::cout <<"roll_fn: ROLL\n";
			stuck_timer =0;

		}
		//go slow (greater than 20 pixels)
		else if(((current_interest.size >= 1) && (current_interest.size < 4)))
		{
			kh4_set_speed((MOTOR_SPEED_MOVE + speed_bias_left), (MOTOR_SPEED_MOVE + speed_bias_right), dsPic);
			std::cout <<"roll_fn: SMALL MOVE\n";
			stuck_timer =0;

		}
		//size less than 10 pixels, search
		else if(current_interest.size < 1)
		{
			full_search(); //cal search fn.The fn will take care of what type of search to use.
		}
	} else if (ir_front > IR_MAX_PROXIMITY && current_interest.size > 0)
	{ 		/*IF FRONT IR > 30, object is 15 cm away or closer..
		 	 	 	 	 switching to step-step actions*/
		std::cout <<"ELSE: step-step actions | turn_factor_int: "<<turn_factor_int<<"\n";
		float ir_difference_front_LR = 0; // difference btw front left and right ir

		kh4_set_speed( 0, 0, dsPic);

		corner_backoff_fuse = 0; //reset corner case backoff fuse

		/*back off*/
		if(current_interest.size < 1 && !in_contact)
		{
			kh4_set_speed(-400 , -400, dsPic);
			sleep(1);
			cout <<".. Backoff...."<<"\n";
			kh4_set_speed( 0, 0, dsPic);
		}

		//jerk and turn in position to dock
		if(turn_direction == 'L' && turn_factor_int > 1)
		{
			left_jerk(200);		/*200 ms*/
		} else if(turn_direction == 'R' && turn_factor_int > 1)
		{
			right_jerk(200);	/*200 ms*/
		}

		//align with IRs
		get_proximity_IR("step-step actions");   /*this fetch should be here as we use the variable
									in the immediate condition chk*/


		ir_difference_front_LR = ir_front_left - ir_front_right;

		if((ir_front >= IR_MAX_PROXIMITY)) //red is seen and object is closer, now dock
		{
			int fuse =0;
			while(ir_front < 1021 && (ir_front >= IR_MAX_PROXIMITY))

			{
				/*Approach until max ir reading
				 * dont remove!! these two lines are for the while loop*/
				get_proximity_IR("~~slowly_approaching");
				ir_difference_front_LR = ir_front_left - ir_front_right;

				if( ir_difference_front_LR > 10)
				{
					kh4_set_speed( (MOTOR_SPEED_MOVE), (MOTOR_SPEED_MOVE+(MOTOR_SPEED_BIAS_STEP*5)), dsPic);
					usleep(150000);
				}
				else if(ir_difference_front_LR > 10)
				{
					kh4_set_speed( (MOTOR_SPEED_MOVE+(MOTOR_SPEED_BIAS_STEP*5)), (MOTOR_SPEED_MOVE), dsPic);
					usleep(150000);
				}
				else {
					kh4_set_speed( (MOTOR_SPEED_MOVE), (MOTOR_SPEED_MOVE), dsPic);
					usleep(150000);
				}

				/*timeout breaking condn*/
				fuse ++;
				if(fuse>20)
				{
					kh4_set_speed(-400 , -400, dsPic);
					usleep(600000);
					kh4_set_speed(0 , 0, dsPic);
					//LLL 	cout <<" FUSE Backoff...."<<"\n";
					break;

				}
			}
			//jerk and reposition until sufficiently docked
			if(ir_front > 1021 && (abs(ir_difference_front_LR) > 600/*500*//*300*//*200*/))
			{
				kh4_set_speed( (0), (0), dsPic);
				in_contact = true;
				int fuse_2 = 0;

				while(ir_front > 1021 && (abs(ir_difference_front_LR) > 600/*500*//*200*//*old 100*/)  )
				{
					if(ir_difference_front_LR > 0)
					{
						left_jerk (160);	/*150 ms*/
						get_proximity_IR("TOUCHY jerk L");
						ir_difference_front_LR = ir_front_left - ir_front_right;

					} else if (ir_difference_front_LR < 0)
					{
						right_jerk (165);	/*150 ms*/
						get_proximity_IR("TOUCHY jerk R");
						ir_difference_front_LR = ir_front_left - ir_front_right;
					}
					usleep(400000);
					fuse_2++;
					if(fuse_2>10) //timeout
					{
						kh4_set_speed(-400 , -400, dsPic);
						usleep(600000);
						kh4_set_speed(0 , 0, dsPic);
						break;
					}
				}
			} else {
				in_contact = false; //made contacct with object
			}
		}
		if (ir_front > 1021 && (abs(ir_difference_front_LR) < 600)) //docked, now ready to push
		{
			kh4_set_speed( (0), (0), dsPic);
			std::cout <<"calling coop_or_go_around\n";
			coop_or_go_around();
		}

	} else  {

		if(corner_backoff_fuse >= 4) //something happened and the bot is stuck
		{
			/*RECOVER FROM STUCK POSITION by jerking and backing-off*/
			left_jerk (170);
			right_jerk (180);

			kh4_set_speed(-450 , -450, dsPic); /*old 400*/
			usleep(200000);
			kh4_set_speed(-500 , -500, dsPic);
			usleep(200000);
			kh4_set_speed(-550 , -550, dsPic);
			sleep(1);
			kh4_set_speed( 0, 0, dsPic);
			corner_backoff_fuse = 0;
			cout <<".. BIGG corner Backoff to RECOVER...."<<"\n";
			/*update local status*/
			write_status_local(STATUS_PAUSED);
		} else {
			kh4_set_speed(-450 , -450, dsPic); /*old 400*/
			usleep(600000);
			kh4_set_speed( 0, 0, dsPic);
		}
		corner_backoff_fuse ++;
	}
	return object_found;
}

/*
 *
 * Blob detection fn, detects onlu red as of now
 *
 * */
int find_blobs( Mat &matThresholded, Mat &matCameraFeed)
{

	cout <<"\nFUNC_FIND_BLOBS" << "\n";

	vector<KeyPoint> keypoints;
	KeyPoint current_interest;
	current_interest.size = 0;
	float total_area_of_blobs = 0;
	Point2f image_centre;
	image_centre.x = matThresholded.cols/2;
	image_centre.y = matThresholded.rows/2;

	// Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;

	/*filter by color FUNCTIONALITY IS BROKEN.
	 * This flag must be FALSE*/
	params.filterByColor = false;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 5;

	// Filter by Circularity
	params.filterByCircularity = false;
	// Filter by Convexity
	params.filterByConvexity = false;
	// Filter by Inertia
	params.filterByInertia = false;


	/*dialate*/
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(3,3));
	dilate(matThresholded,matThresholded,dilateElement);


	/*set a blob detector with params*/
	Ptr<SimpleBlobDetector> blob_detector = SimpleBlobDetector::create(params);

	/**detect blobs */
	blob_detector->detect( matThresholded, keypoints);

	for(unsigned int i=0; i<keypoints.size(); i++ ) //find the biggest blob
	{
		if(current_interest.size < keypoints[i].size)
		{
			current_interest = keypoints[i];
		}
		/*get the total area*/
		total_area_of_blobs = total_area_of_blobs + keypoints[i].size ;
	}

	int return_from_roll = roll_the_bot(current_interest, total_area_of_blobs,matThresholded);
	return return_from_roll;
}


//create a thread to write the images captured
Mat matThresholded;
int frame_count = 0;
void* thread_save_images(void *arg)
{

	char filename[50];

	sprintf(filename, "/home/root/frames/f_%06d.jpg", frame_count);
	cv::imwrite(filename, matBRGFrame);
	frame_count++;
	sprintf(filename, "/home/root/frames/f_%06d.jpg", frame_count);
	cv::imwrite(filename, matThresholded);
	frame_count++;

	pthread_exit(NULL);
}


/*
 * MAIN
 * */
int main( int argc, char** argv )
{
	unsigned char* img_buffer=NULL; // pointer for image buffer
	int flag;
	int found_object = 0;

	//khepera params
	int kp,ki,kd;
	int pmarg;

	char  revision,version;

	/*red khepera new apr 2018*/
	int rH_MIN = 61;
	int rH_MAX = 100;
	int rS_MIN = 107;
	int rS_MAX = 123;
	int rV_MIN = 154;
	int rV_MAX = 191;

	/*NEW:KHEPERA for blue
	int bH_MIN = 121;
	int bH_MAX = 180;
	int bS_MIN = 156;
	int bS_MAX = 189;
	int bV_MIN = 133;
	int bV_MAX = 154;*/


	//Mat dummy_frame, matBRGFrame;
	Mat matCameraFeed;


	time_t current_time = time(0);
	std::cout <<"\nDATE: "<<ctime(&current_time)<< "\n";

	// initiate libkhepera and robot access
	if ( kh4_init(argc ,argv)!=0)
	{
		std::cout <<"\nERROR: could not initiate the libkhepera!\n\n";
		return -1;
	}

	/* open robot socket and store the handle in their respective pointers */
	dsPic  = knet_open( "Khepera4:dsPic" , KNET_BUS_I2C , 0 , NULL );

	if ( dsPic==NULL)
	{
		std::cout <<"\nERROR: could not initiate communication with Kh4 dsPic\n\n";
		return -2;
	}

	/* initialize the motors controlers*/
	/* tuned parameters */
	pmarg=20;
	kh4_SetPositionMargin(pmarg,dsPic );              // position control margin
	kp=10;
	ki=5;
	kd=1;
	kh4_ConfigurePID( kp , ki , kd,dsPic  );          // configure P,I,D

	accinc=3;//3;
	accdiv=0;
	minspacc=20;
	minspdec=1;
	maxsp=400;
	// configure acceleration slope
	kh4_SetSpeedProfile(accinc,accdiv,minspacc, minspdec,maxsp,dsPic ); // Acceleration increment ,  Acceleration divider, Minimum speed acc, Minimum speed dec, maximum speed

	kh4_SetMode( kh4RegIdle,dsPic );                                // Put in idle mode (no control)

	// get revision
	if(kh4_revision(Buffer, dsPic)==0){
		version=(Buffer[0]>>4) +'A';
		revision=Buffer[0] & 0x0F;
		std::cout <<"\r\nVersion = " <<version<<", Revision = \n"<<revision;
	}

	/*camera initialization*/
	flag = kb_camera_init( &FRAME_WIDTH, &FRAME_HEIGHT);
	cout << "\ncamra_int_returned: " << flag << "\n";

	switch(flag) {
	case 0:
		std::cout <<"case 0 SUCCESS\r\n";
		break;
	case 1:
		std::cout <<"case 1 width adjusted to "<< FRAME_WIDTH <<"\n";
		break;
	case 2:
		std::cout <<"case 2 height adjusted to "<< FRAME_HEIGHT<<"\n";
		break;
	case 3:
		std::cout <<"case 3 width adjusted to "<<FRAME_WIDTH<<" and height adjusted to %d !"<<FRAME_HEIGHT<<"\n";
		break;
	default:
		std::cout << "SOME INIT ERRRR code is:" << flag << "\n";
		break;
	}


	// allocating memory for image
	img_buffer= (unsigned char*)malloc(FRAME_WIDTH*FRAME_HEIGHT*3*sizeof(char));

	if (img_buffer==NULL)
	{
		fprintf(stderr,"could alloc image buffer!\r\n");
		free(img_buffer);
		return -2;
	}


	/*set mode - openloop*/
	kh4_SetMode(kh4RegSOpenLoop,dsPic );

	//one time actions
	/*ger IR calibration data from file*/
	ir_prox_calib();

	/*get the host name and set peer name(peer is hardcoded now)*/
	get_hostname_peername();

	/*clear temp files & Initialize the local status in the shared memory
	 * Paused: means the peer can push if he is at the object first*/
	if (system(NULL)) {
		system("cd /home/root/SHARED_MEMORY/ ; rm *.*~ ; .*.*~");
		system("cd /home/root/frames/ ; rm *");
	}
	std::cout<<"\nPARAMS:\n\tD1D2_DIFF: "<<D1D2_DIFF
			<<"\n\tMIN_DIST_TO_REACH_GOAL: "
			<<MIN_DIST_TO_REACH_GOAL<<"\n";

	write_status_local(STATUS_PAUSED);

	/*
	 * OPENING FOR MULTIPLE FRAME READ
	 * */
	if(kb_captureStart()<0)
	{
		free(img_buffer);
		kb_camera_release();
		fprintf(stderr,"ERROR: capture start error in mutli frames!\r\n");
		return -3;
	}
	usleep(100000); // wait for 100ms initialisation

	kb_change_term_mode(1); // change terminal mode for kbhit and getchar to return immediately

	gettimeofday(&experiment_starttime,0x0);

	signal( SIGINT , ctrlc_handler ); // set signal for catching ctrl-c

	int return_val;
	cv::Point2f prev_bot;

	//	time_t main_time;
	struct s_distances s_inst;
	bool backoff_once = false;

	//Loop: All actions happen here
	//A camera frame is captured, blob detection is done.. actions are taken based camera,
	// IR readings and the observer data
	while(!reached_goal)
	{
		/*read peer status*/
		string local_peer_status = read_peer_status();
		if(local_peer_status == STATUS_DND)
		{
			if(!backoff_once)
			{
				backoff_once = true;
				kh4_set_speed(-500 , -500, dsPic);
				sleep(1);
				kh4_set_speed(0 , 0, dsPic);
			}
			kh4_set_speed(0 , 0, dsPic);
			sleep(1);

		} else{

			/*star capturing multiple frames*/
			if ((return_val=kb_frameRead(img_buffer))<0)
			{
				fprintf(stderr,"ERROR: frame capture error %d!\r\n",return_val);
			} else {
				//read the img_buffer as a Mat object
				matBRGFrame = Mat( int(FRAME_HEIGHT), int(FRAME_WIDTH), CV_8UC3, img_buffer);

				cvtColor(matBRGFrame, matBRGFrame, CV_RGB2BGR);;

				if(matBRGFrame.empty())
				{
					cout << "\n\t!!!!!! Empty Frame !!!!"<< "\n";
					return 0;
				}

				/*threshold red */
				inRange(matBRGFrame,Scalar(rH_MIN,rS_MIN,rV_MIN),Scalar(rH_MAX,rS_MAX,rV_MAX),matThresholded);

				//create a thread and save the image locally
				pthread_t file_writer;
				int return_var;
				return_var =  pthread_create(&file_writer, NULL, thread_save_images, NULL);
				if(return_var != 0) {
					std::cout<<"Error: imwrite pthread_create() failed\n";
					exit(EXIT_FAILURE);
				}


				/*Find Blob fn*/
				found_object = find_blobs( matThresholded, matBRGFrame);

				//read coordinates and calculate distance
				read_file_remote_points();
				s_inst = distances_from_coordinates();

				if((s_inst.distance[0] <= MIN_DIST_TO_REACH_GOAL) || (s_inst.distance[1]<=MIN_DIST_TO_REACH_GOAL))
				{
					reached_goal = true;
					std::cout <<"TIME TO STOP...reached_goal: "<<reached_goal<<"\n";
					//return_status = true;
				}

				//if the robot's distance has not changed in a while,
				//robot is stuck. Try to recover.
				if(	memorycount == 0)
				{
					roll_mem.mbot = color_bot;
					roll_mem.distance = 0;
					memorycount++;
				} else {
					avg_dist_travelled();
					memorycount++;
					if(memorycount%5 == 0)
					{
						if(roll_mem.distance < 5)
						{
							//make noise
							pthread_t mysound2;
							int return_var;
							return_var =  pthread_create(&mysound2, NULL, play_sound, NULL);
							if(return_var != 0) {
								std::cout<<"Error: pthread_create() failed fo play sound\n";
								exit(EXIT_FAILURE);

								kh4_set_speed(-500 , -500, dsPic); //backoff
								sleep(1);
								kh4_set_speed(0 , 0, dsPic);
							}
						}
						roll_mem.distance = 0;
					}
				}

				if(found_object)
				{
					cout << "\n" <<"Found the object, halting the bot for a few s... " << "\n";
				}
			}
		}
	}

	/*Total run time of the expeiment*/
	gettimeofday(&experiment_endtime,0x0);
	long long experiment_time_delta = timeval_diff(NULL,&experiment_endtime,&experiment_starttime);
	cout << "TOTAL EXPERIMNENT TIME: " << experiment_time_delta/1000000<<"."<<experiment_time_delta%1000000 << " seconds"<<"\n";

	/*set motors to idle and release camera*/
	kh4_set_speed(0 ,0 ,dsPic); // stop robot
	kh4_SetMode( kh4RegIdle,dsPic );
	kb_camera_release();

	kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy

	kb_change_term_mode(0); // revert to original terminal if called

	current_time = time(0);
	std::cout <<"\nDATE: "<<ctime(&current_time)<< "\n";

	exit(0);

	//return 0;
}
