#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h> // Unbuntu 18 on x86_64 has this at /usr/include/x86_64-linux-gnu/sys/socket.h
#include <signal.h>
#include <pthread.h>
#include <fcntl.h>     // for open
#include <unistd.h>    // for close
#include <arpa/inet.h> // for inet_addr
#include <sys/time.h>
#include <unistd.h>

#define HPVM
#define HPVM_CV_ROOT
#define HPVM_PROCESS_LIDAR
#define HPVM_PROCESS_LIDAR_INTERNAL
//#define HPVM_RECV_PIPELINE
//#define RECV_CALLER
#define COLLAPSE_NODES

#include "globals.h"
#include "debug.h"
#include "getopt.h"
#include "cv_toolset.h"
#include "occgrid.h"   // Occupancy Grid Map Create/Fuse
#include "lz4.h"       // LZ4 Compression/Decompression
#include "xmit_pipe.h" // IEEE 802.11p WiFi SDR Transmit Pipeline
#include "recv_pipe.h" // IEEE 802.11p WiFi SDR Receive Pipeline

#include "globalsRecv.h"
#include "globalsXmitPipe.h"
#include "globalsOccgrid.h"
#include "globalsSDRViterbi.h"
#include "crc.h"

#include <stdint.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image/stb_image.h"

#if defined(HPVM)
#include "hpvm.h"
#include "hetero.h"
#endif

// #define USE_OLD_MODEL

#define PARALLEL_PTHREADS false

#define ERA1


#ifdef ERA1
char * IMAGE_FN = "gridimage_era1_";
#define BAG_PORT 5556
#define XMIT_PORT 5558
#define RECV_PORT 5559
#define CAR_PORT 5562
#endif

#ifdef ERA2
char * IMAGE_FN = "gridimage_era2_";
#define BAG_PORT 5557
#define XMIT_PORT 5560
#define RECV_PORT 5561
#define CAR_PORT 5563
#endif

#define d_fft_len 64
#define d_fft_logn 6
#define d_rolloff_len 2
#define d_cp_size 16
#define d_num_sync_words 4
#define d_size_sync_words 128
#define d_num_pilot_carriers 1
#define d_size_pilot_carriers_val 4
#define d_num_pilot_symbols 127
#define d_size_pilot_symbols 4
static const int d_size_pilot_carriers[d_num_pilot_carriers] = {
	d_size_pilot_carriers_val
};

uint8_t d_psdu_org[MAX_PSDU_SIZE];
uint8_t d_map_out_copy_org[32768];

uint16_t d_seq_nr_org = 0;
uint8_t d_scrambler_org = 0;
char d_symbols_org[24528];
int d_symbols_offset_org = 0;
int d_symbols_len_org = 0;

ofdm_param d_ofdm_org;
frame_param d_frame_org;

int d_pilot_carriers_org[d_num_pilot_carriers][d_size_pilot_carriers_val] = {
        {
                -21, -7, 7, 21
        }
};

int d_occupied_carriers_org[d_num_occupied_carriers][d_size_occupied_carriers] = {
        {
                -26, -25, -24, -23, -22, -20, -19, -18, //  8
                -17, -16, -15, -14, -13, -12, -11, -10, // 16
                -9, -8, -6, -5, -4, -3, -2, -1, // 24
                1, 2, 3, 4, 5, 6, 8, 9, // 32
                10, 11, 12, 13, 14, 15, 16, 17, // 40
                18, 19, 20, 22, 23, 24, 25, 26
        }
}; // 48


size_t d_psdu_org_size = MAX_PSDU_SIZE * sizeof(uint8_t);
size_t d_map_out_org_size = 32768;

size_t d_seq_nr_org_sz = sizeof(uint16_t);
size_t d_scrambler_org_sz = sizeof(uint8_t);
size_t d_symbols_org_sz = 24528;
size_t d_symbols_offset_org_sz = sizeof(int);
size_t d_symbols_len_org_sz = sizeof(int);

size_t d_ofdm_org_sz = sizeof(ofdm_param);
size_t d_frame_org_sz = sizeof(frame_param);

size_t d_pilot_carriers_org_sz = d_num_pilot_carriers*d_size_pilot_carriers_val*sizeof(int);
size_t d_occupied_carriers_org_sz = d_num_occupied_carriers*d_size_occupied_carriers*sizeof(int);

crc  crcTable[256];
size_t crcTable_sz = 256*sizeof(crc);

// The PORTS are defined in the compilation process, and comforms to the
// definition in the read_bag_x.py files and wifi_comm_x.py files.

char bag_inet_addr_str[20];
char wifi_inet_addr_str[20];
char car_inet_addr_str[20];
unsigned max_time_steps = ~1;

int bag_sock = 0;
int xmit_sock = 0;
int recv_sock = 0;
int car_sock = 0;

char * ack = "OK";

float odometry[] = {
	0.0,
	0.0,
	0.0 };

// We will define 2 observations; one "current" and one that is to be constructed to be the new current.
int curr_obs = 1;
int next_obs = 0;
Observation observationsArr[2];

// These variables capture "time" spent in various parts ofthe workload
struct timeval stop_prog, start_prog;

struct timeval stop_proc_odo, start_proc_odo;
uint64_t proc_odo_sec = 0LL;
uint64_t proc_odo_usec = 0LL;

struct timeval stop_proc_rdbag, start_proc_rdbag;
uint64_t proc_rdbag_sec = 0LL;
uint64_t proc_rdbag_usec = 0LL;

struct timeval stop_proc_lidar, start_proc_lidar;
uint64_t proc_lidar_sec = 0LL;
uint64_t proc_lidar_usec = 0LL;

struct timeval stop_proc_data, start_proc_data;
uint64_t proc_data_sec = 0LL;
uint64_t proc_data_usec = 0LL;

struct timeval stop_proc_cv, start_proc_cv;
uint64_t proc_cv_sec = 0LL;
uint64_t proc_cv_usec = 0LL;

#ifdef INT_TIME
struct timeval stop_pd_cloud2grid, start_pd_cloud2grid;
uint64_t pd_cloud2grid_sec = 0LL;
uint64_t pd_cloud2grid_usec = 0LL;

struct timeval stop_pd_lz4_cmp, start_pd_lz4_cmp;
uint64_t pd_lz4_cmp_sec = 0LL;
uint64_t pd_lz4_cmp_usec = 0LL;

struct timeval stop_pd_wifi_pipe, start_pd_wifi_pipe;
uint64_t pd_wifi_pipe_sec = 0LL;
uint64_t pd_wifi_pipe_usec = 0LL;

struct timeval stop_pd_wifi_send, start_pd_wifi_send;
uint64_t pd_wifi_send_sec = 0LL;
uint64_t pd_wifi_send_usec = 0LL;

struct timeval stop_pd_wifi_send_rl, start_pd_wifi_send_rl;
uint64_t pd_wifi_send_rl_sec = 0LL;
uint64_t pd_wifi_send_rl_usec = 0LL;

struct timeval stop_pd_wifi_send_im, start_pd_wifi_send_im;
uint64_t pd_wifi_send_im_sec = 0LL;
uint64_t pd_wifi_send_im_usec = 0LL;

struct timeval stop_pd_wifi_recv_th, start_pd_wifi_recv_th;
uint64_t pd_wifi_recv_th_sec = 0LL;
uint64_t pd_wifi_recv_th_usec = 0LL;

struct timeval stop_pd_wifi_lmap_wait, start_pd_wifi_lmap_wait;
uint64_t pd_wifi_lmap_wait_sec = 0LL;
uint64_t pd_wifi_lmap_wait_usec = 0LL;

struct timeval stop_pd_wifi_recv_wait, start_pd_wifi_recv_wait;
uint64_t pd_wifi_recv_wait_sec = 0LL;
uint64_t pd_wifi_recv_wait_usec = 0LL;

struct timeval stop_pd_wifi_recv_all, start_pd_wifi_recv_all;
uint64_t pd_wifi_recv_all_sec = 0LL;
uint64_t pd_wifi_recv_all_usec = 0LL;

struct timeval stop_pd_wifi_recv_rl, start_pd_wifi_recv_rl;
uint64_t pd_wifi_recv_rl_sec = 0LL;
uint64_t pd_wifi_recv_rl_usec = 0LL;

struct timeval stop_pd_wifi_recv_im, start_pd_wifi_recv_im;
uint64_t pd_wifi_recv_im_sec = 0LL;
uint64_t pd_wifi_recv_im_usec = 0LL;

struct timeval stop_pd_recv_pipe, start_pd_recv_pipe;
uint64_t pd_recv_pipe_sec = 0LL;
uint64_t pd_recv_pipe_usec = 0LL;

struct timeval stop_pd_lz4_uncmp, start_pd_lz4_uncmp;
uint64_t pd_lz4_uncmp_sec = 0LL;
uint64_t pd_lz4_uncmp_usec = 0LL;

struct timeval stop_pd_combGrids, start_pd_combGrids;
uint64_t pd_combGrids_sec = 0LL;
uint64_t pd_combGrids_usec = 0LL;

struct timeval stop_pd_wifi_car, start_pd_wifi_car;
uint64_t pd_wifi_car_sec = 0LL;
uint64_t pd_wifi_car_usec = 0LL;
#endif

int arr_counter = 0;
int ascii_counter = 0;

unsigned odo_count = 0;
unsigned lidar_count = 0;
unsigned lmap_count = 0;
unsigned xmit_count = 0;
unsigned recv_count = 0;
unsigned car_send_count = 0;
unsigned cv_count = 0;

// Forward Declarations
void print_usage(char * pname);
void dump_final_run_statistics();
void INThandler(int dummy);
// in globals.h void closeout_and_exit(char* last_msg, int rval);

// Functions, code, etc.
void print_usage(char * pname) {
	printf("Usage: %s <OPTIONS>\n", pname);
	printf(" OPTIONS:\n");
	printf("    -h         : print this helpful usage info\n");
	printf("    -B <str>   : set the internet-address for the bagfile server to <str>\n");
	printf("    -W <str>   : set the internet-address for the WiFi server to <str>\n");
	printf("    -C <str>   : set the internet-address for the Car Map Output server to <str>\n");
	printf("    -s <Num>   : exit run after <Num> Lidar time-steps (msgs)\n");
}

void INThandler(int dummy) {
	printf("In SIGINT INThandler -- Closing the connection and exiting\n");
	closeout_and_exit("Received a SIGINT...", -1);
}

void SIGPIPE_handler(int dummy) {
	printf("In SIGPIPE_handler -- Closing the connection and exiting\n");
	closeout_and_exit("Received a SIGPIPE...", -1);
}

#ifdef XMIT_HW_FFT
extern void free_XMIT_FFT_HW_RESOURCES();
#endif
#ifdef RECV_HW_FFT
extern void free_RECV_FFT_HW_RESOURCES();
#endif

// This cleans up the state before exit
void closeout_and_exit(char * last_msg, int rval) {
	if (lidar_count > 0) {
		dump_final_run_statistics();
	}
	printf("closeout_and_exit -- Closing the connection and exiting %d\n", rval);
	if (bag_sock != 0) {
		close(bag_sock);
	}
	if (xmit_sock != 0) {
		close(xmit_sock);
	}
	if (recv_sock != 0) {
		close(recv_sock);
	}
	if (car_sock != 0) {
		close(car_sock);
	}

#ifdef XMIT_HW_FFT
	free_XMIT_FFT_HW_RESOURCES();
#endif
#ifdef RECV_HW_FFT
	free_RECV_FFT_HW_RESOURCES();
#endif
	printf("%s\n", last_msg);
	exit(rval);
}

/*
	 float bytes_to_float(unsigned char * bytes)
	 {
	 unsigned char b[] = {bytes[0], bytes[1], bytes[2], bytes[3]};
	 float f;
	 memcpy(&f, &b, sizeof(f));

	 return f;
	 }
	 */

void write_array_to_file(unsigned char * data, long size) {
	const int dimx = 50, dimy = 50;
	int i, j;

	char file_name[32];

	snprintf(file_name, sizeof(char) * 32, "%s%04d.ppm", IMAGE_FN, arr_counter);

	FILE * fp = fopen(file_name, "w");
	fprintf(fp, "P3 %d %d 255\n", dimx, dimy);

	for (j = 0; j < dimy * dimx; ++j) {
		fprintf(fp, " %d %d %d ", data[j], data[j], data[j]);
	}

	fclose(fp);
	arr_counter++;
}

#define MAX_UNCOMPRESSED_DATA_SIZE sizeof(Costmap2D)        // MAX_GRID_SIZE
#define MAX_COMPRESSED_DATA_SIZE MAX_UNCOMPRESSED_DATA_SIZE //(In case of no compression)?

#define MAX_XMIT_OUTPUTS 41800 // Really something like 41782 I think

int read_all(int sock, char * buffer, int xfer_in_bytes) {
	char * ptr;
	int message_size = xfer_in_bytes;
	char * message_ptr = buffer;
	int total_recvd = 0;
	while (total_recvd < message_size) {
		unsigned rem_len = (message_size - total_recvd);
		int valread = read(sock, message_ptr, rem_len);
		message_ptr = message_ptr + valread;
		total_recvd += valread;
		DBGOUT2(printf("        read %d bytes for %d total bytes of %d\n", valread, total_recvd,
			message_size));
		if (valread == 0) {
			DBGOUT(printf("  read_all got ZERO bytes -- END of TRANSFER?\n"));
			return total_recvd;
		}
	}
	return total_recvd;
}

__attribute__ ((noinline)) void decompress(unsigned char * uncmp_data, size_t uncmp_data_sz,
	int * recvd_msg_len, size_t recvd_msg_len_sz,
	unsigned char * recvd_msg, size_t recvd_msg_sz,
	int * dec_bytes, size_t dec_bytes_sz,
	Observation * observations, size_t observation_sz
) {
#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && true 
	void * Section_Inner = __hetero_section_begin();
	void * T2 = __hetero_task_begin(5, uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_msg, recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observations, observation_sz,
		2, uncmp_data, uncmp_data_sz, dec_bytes, dec_bytes_sz, "decompress_task");

#endif
#if !defined(HPVM)
	// Now we decompress the grid received via transmission...
	DBGOUT(printf("Calling LZ4_decompress_default...\n"));
#endif
#if defined(INT_TIME) && !(defined(HPVM) && defined(HPVM_RECV_PIPELINE)) 
	gettimeofday(&start_pd_lz4_uncmp, NULL);
#endif
	DEBUG(printf("Calling LZ4_decompress_safe with %d input bytes...\n", recvd_msg_len));
	*dec_bytes = LZ4_decompress_safe((char *) recvd_msg, (char *) uncmp_data, *recvd_msg_len,
		MAX_UNCOMPRESSED_DATA_SIZE);
	if (*dec_bytes < 0) {
//		printf("LZ4_decompress_safe ERROR : %d\n", *dec_bytes);
	}
	DEBUG(
	else {
		printf("LZ4_decompress_safe returned %d bytes\n", *dec_bytes);
	});
#if defined(INT_TIME) && !(defined(HPVM) && defined(HPVM_RECV_PIPELINE)) 
	gettimeofday(&stop_pd_lz4_uncmp, NULL);
	pd_lz4_uncmp_sec += stop_pd_lz4_uncmp.tv_sec - start_pd_lz4_uncmp.tv_sec;
	pd_lz4_uncmp_usec += stop_pd_lz4_uncmp.tv_usec - start_pd_lz4_uncmp.tv_usec;
#endif


	DEBUG(printf("Recevied %d decoded bytes from the wifi...\n", *dec_bytes));
	Costmap2D * remote_map = (Costmap2D *) &(uncmp_data); // Convert "type" to Costmap2D

	DBGOUT(printf("  Back from LZ4_decompress_safe with %u decompressed bytes\n", *dec_bytes);
	printf("  Remote CostMAP: AV x %lf y %lf z %lf\n", remote_map -> av_x, remote_map -> av_y,
		remote_map -> av_z);
	printf("                : Cell_Size %lf X-Dim %u Y-Dim %u\n", remote_map -> cell_size,
		remote_map -> x_dim, remote_map -> y_dim); print_ascii_costmap(stdout, remote_map));

	// Get the current local-map
//	printf("Receive step %u : Processing fusion for curr_obs = %d\n", recv_count, curr_obs);
	Costmap2D * local_map = &(observations[curr_obs].master_costmap);

#ifdef WRITE_ASCII_MAP
	char ascii_file_name[32];
	snprintf(ascii_file_name, sizeof(char) * 32, "%s%04d.txt", ASCII_FN, ascii_counter);
	FILE * ascii_fp = fopen(ascii_file_name, "w");

	// printf("Input CostMAP: AV x %lf y %lf z %lf\n", local_map->av_x, local_map->av_y, local_map->av_z);
	fprintf(ascii_fp, "Input CostMAP: AV x %lf y %lf z %lf\n", local_map -> av_x, local_map -> av_y,
		local_map -> av_z);
	fprintf(ascii_fp, "             : Cell_Size %lf X-Dim %u Y-Dim %u\n", local_map -> cell_size,
		local_map -> x_dim, local_map -> y_dim);
	print_ascii_costmap(ascii_fp, local_map);

	fprintf(ascii_fp, "\n\nRemote CostMAP: AV x %lf y %lf z %lf\n", remote_map -> av_x, remote_map -> av_y,
		remote_map -> av_z);
	// printf("\n\nRemote CostMAP: AV x %lf y %lf z %lf\n", remote_map->av_x, remote_map->av_y, remote_map->av_z);
	fprintf(ascii_fp, "              : Cell_Size %lf X-Dim %u Y-Dim %u\n", remote_map -> cell_size,
		remote_map -> x_dim, remote_map -> y_dim);
	print_ascii_costmap(ascii_fp, remote_map);
	fclose(ascii_fp);
#endif

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && true 
	__hetero_task_end(T2);
	__hetero_section_end(Section_Inner);
#endif
}


#if false // Hite all the wrapper functions for decompress

void decompress_Wrapper2(unsigned char * uncmp_data, size_t uncmp_data_sz,
	int * recvd_msg_len, size_t recvd_msg_len_sz,
	unsigned char * recvd_msg, size_t recvd_msg_sz,
	int * dec_bytes, size_t dec_bytes_sz,
	Observation * observation, size_t observation_sz
) {
#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER)  && true 
	void * Section_Caller = __hetero_section_begin();
	void * T2_Wrapper = __hetero_task_begin(5, uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_msg, recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observation, observation_sz,
		2, uncmp_data, uncmp_data_sz, dec_bytes, dec_bytes_sz, "decompress_task_Wrapper2");
#endif

	decompress(uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz, recvd_msg,
		recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observation, observation_sz);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER) && true 
	__hetero_task_end(T2_Wrapper);
	__hetero_section_end(Section_Caller);
#endif

}

void decompress_Wrapper3(unsigned char * uncmp_data, size_t uncmp_data_sz,
	int * recvd_msg_len, size_t recvd_msg_len_sz,
	unsigned char * recvd_msg, size_t recvd_msg_sz,
	int * dec_bytes, size_t dec_bytes_sz,
	Observation * observation, size_t observation_sz
) {
#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER)  && true 
	void * Section_Caller = __hetero_section_begin();
	void * T2_Wrapper = __hetero_task_begin(5, uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_msg, recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observation, observation_sz,
		2, uncmp_data, uncmp_data_sz, dec_bytes, dec_bytes_sz, "decompress_task_Wrapper3");
#endif

	decompress_Wrapper2(uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz, recvd_msg,
		recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observation, observation_sz);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER) && true 
	__hetero_task_end(T2_Wrapper);
	__hetero_section_end(Section_Caller);
#endif

}

void decompress_Wrapper4(unsigned char * uncmp_data, size_t uncmp_data_sz,
	int * recvd_msg_len, size_t recvd_msg_len_sz,
	unsigned char * recvd_msg, size_t recvd_msg_sz,
	int * dec_bytes, size_t dec_bytes_sz,
	Observation * observation, size_t observation_sz
) {
#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER)  && true 
	void * Section_Caller = __hetero_section_begin();
	void * T2_Wrapper = __hetero_task_begin(5, uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_msg, recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observation, observation_sz,
		2, uncmp_data, uncmp_data_sz, dec_bytes, dec_bytes_sz, "decompress_task_Wrapper4");
#endif

	decompress_Wrapper3(uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz, recvd_msg,
		recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observation, observation_sz);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER) && true 
	__hetero_task_end(T2_Wrapper);
	__hetero_section_end(Section_Caller);
#endif
}

#endif

void grid_fusion(Observation * observations, size_t observations_sz,
	unsigned char * uncmp_data, size_t uncmp_data_sz) {

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && false 
	void * Section_Inner = __hetero_section_begin();
	void * T4 = __hetero_task_begin(2, observations, observations_sz, uncmp_data, uncmp_data_sz,
		1, observations, observations_sz, "gridFusion_task");
#endif

	// Then we should "Fuse" the received GridMap with our local one
	//  We need to "peel out" the remote odometry data from somewhere (in the message?)
	// unsigned char* combineGrids(unsigned char* grid1, unsigned char* grid2, double robot_x1, double robot_y1,
	DBGOUT(printf("\nCalling combineGrids...\n"));
	// Note: The direction in which this is called is slightly significant:
	//  The first map is copied into the second map, in this case remote into local,
	//  and the x_dim, et.c MUST correspond to that second map (here local)
#ifdef INT_TIME
	gettimeofday(&start_pd_combGrids, NULL);
#endif
	// Copied the following two variable above task; this wasn't being modified in the above task so
	// just copying it down should be safe
	Costmap2D * local_map_cp = &(observations[curr_obs].master_costmap);
	Costmap2D * remote_map_cp = (Costmap2D *) (uncmp_data); // Convert "type" to Costmap2D
	fuseIntoLocal(local_map_cp, remote_map_cp);
	/*combineGrids(remote_map->costmap, local_map->costmap,
		remote_map->av_x, remote_map->av_y,
		local_map->av_x, local_map->av_y,
		local_map->x_dim, local_map->y_dim, local_map->cell_size);
		*/
#ifdef INT_TIME
	gettimeofday(&stop_pd_combGrids, NULL);
	pd_combGrids_sec += stop_pd_combGrids.tv_sec - start_pd_combGrids.tv_sec;
	pd_combGrids_usec += stop_pd_combGrids.tv_usec - start_pd_combGrids.tv_usec;
#endif
#ifdef WRITE_ASCII_MAP
	char ascii_file_name[32];
	snprintf(ascii_file_name, sizeof(char) * 32, "%s%04d.txt", ASCII_FN, ascii_counter);
	FILE * ascii_fp = fopen(ascii_file_name, "w");
	DEBUG2(printf(ascii_fp, "  Fused CostMAP : AV x %lf y %lf z %lf\n", local_map_cp -> av_x,
		local_map_cp -> av_y, local_map_cp -> av_z); printf(ascii_fp, "                : Cell_Size %lf X-Dim %u Y-Dim %u\n", local_map_cp -> cell_size,
			local_map_cp -> x_dim, local_map_cp -> y_dim); print_ascii_costmap(stdout, local_map_cp));
#endif

#ifdef WRITE_ASCII_MAP
	fprintf(ascii_fp, "\n\nFused CostMAP : AV x %lf y %lf z %lf\n", local_map_cp -> av_x, local_map_cp -> av_y,
		local_map_cp -> av_z);
	// printf("\n\nFused CostMAP : AV x %lf y %lf z %lf\n", local_map_cp->av_x, local_map_cp->av_y, local_map_cp->av_z);
	fprintf(ascii_fp, "              : Cell_Size %lf X-Dim %u Y-Dim %u\n", local_map_cp -> cell_size,
		local_map_cp -> x_dim, local_map_cp -> y_dim);
	print_ascii_costmap(ascii_fp, local_map_cp);
	fclose(ascii_fp);
	ascii_counter++;
#endif
	// Write the combined map to a file
#ifdef WRITE_FUSED_MAPS
	write_array_to_file(local_map_cp -> costmap, COST_MAP_ENTRIES);
#endif

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && false 
	__hetero_task_end(T4);
	__hetero_section_end(Section_Inner);
#endif
}

#if false // hide all the wrapper functions for grid_fusion

void grid_fusion_Wrapper2(Observation * observations, size_t observations_sz,
	unsigned char * uncmp_data, size_t uncmp_data_sz) {

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER)  && false 
	void * Section_Caller = __hetero_section_begin();
	void * T4_Caller = __hetero_task_begin(2, observations, observations_sz, uncmp_data, uncmp_data_sz,
		1, observations, observations_sz, "gridFusion_task_wrapper2");
#endif

	grid_fusion(observations, observations_sz, uncmp_data, uncmp_data_sz);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER)  && false 
	__hetero_task_end(T4_Caller);
	__hetero_section_end(Section_Caller);
#endif
}

void grid_fusion_Wrapper3(Observation * observations, size_t observations_sz,
	unsigned char * uncmp_data, size_t uncmp_data_sz) {

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER)  && false 
	void * Section_Caller = __hetero_section_begin();
	void * T4_Caller = __hetero_task_begin(2, observations, observations_sz, uncmp_data, uncmp_data_sz,
		1, observations, observations_sz, "gridFusion_task_wrapper3");
#endif

	grid_fusion_Wrapper2(observations, observations_sz, uncmp_data, uncmp_data_sz);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER)  && false 
	__hetero_task_end(T4_Caller);
	__hetero_section_end(Section_Caller);
#endif
}

void grid_fusion_Wrapper4(Observation * observations, size_t observations_sz,
	unsigned char * uncmp_data, size_t uncmp_data_sz) {

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER)  && false 
	void * Section_Caller = __hetero_section_begin();
	void * T4_Caller = __hetero_task_begin(2, observations, observations_sz, uncmp_data, uncmp_data_sz,
		1, observations, observations_sz, "gridFusion_task_wrapper4");
#endif

	grid_fusion_Wrapper3(observations, observations_sz, uncmp_data, uncmp_data_sz);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER)  && false 
	__hetero_task_end(T4_Caller);
	__hetero_section_end(Section_Caller);
#endif
}

#endif // if false


void fuse_maps(int n_recvd_in,
	float * recvd_in_real, size_t recvd_in_real_sz,
	float * recvd_in_imag, size_t recvd_in_imag_sz,
	int * recvd_msg_len, size_t recvd_msg_len_sz,
	unsigned char * recvd_msg, size_t recvd_msg_sz,
	Observation * observations, size_t observations_sz,
	unsigned char * uncmp_data, size_t uncmp_data_sz,
	int * dec_bytes, size_t dec_bytes_sz,
	// Start variables used by do_recv_pipeline
	//              Local variables used by do_recv_pipeline
	uint8_t * scrambled_msg, size_t scrambled_msg_sz,
	float * ss_freq_offset, size_t ss_freq_offset_sz,
	unsigned * num_sync_short_vals, size_t num_sync_short_vals_sz,
	float * sl_freq_offset, size_t sl_freq_offset_sz,
	unsigned * num_sync_long_vals, size_t num_sync_long_vals_sz,
	fx_pt1 * fft_ar_r, size_t fft_ar_r_sz,
	fx_pt1 * fft_ar_i, size_t fft_ar_i_sz,
	unsigned * num_fft_outs, size_t num_fft_outs_sz,
	fx_pt * toBeEqualized, size_t toBeEqualized_sz,
	fx_pt * equalized, size_t equalized_sz,
	unsigned * num_eq_out_bits, size_t num_eq_out_bits_sz,
	unsigned * psdu, size_t psdu_sz,
	//              Global variables used by do_recv_pipeline
	fx_pt * delay16_out_arg /*= delay16_out -> global*/, size_t delay16_out_arg_sz /*= DELAY_16_MAX_OUT_SIZE*/,
	fx_pt * input_data_arg /*= input_data -> global*/, size_t input_data_sz /*=DELAY_16_MAX_OUT_SIZE - 16*/,
	fx_pt * cmpx_conj_out_arg /*= cmpx_conj_out -> global*/, size_t cmpx_conj_out_arg_sz /*= CMP_CONJ_MAX_SIZE*/,
	fx_pt * cmpx_mult_out_arg /*= cmpx_mult_out -> global*/, size_t cmpx_mult_out_arg_sz /*= CMP_MULT_MAX_SIZE*/,
	fx_pt * correlation_complex_arg /*= correlation_complex -> global*/, size_t correlation_complex_arg_sz /*= FIRC_MAVG48_MAX_SIZE*/,
	fx_pt1 * correlation_arg /*= correlation -> global*/, size_t correlation_arg_sz /*= CMP2MAG_MAX_SIZE*/,
	fx_pt1 * signal_power_arg /*= signal_power -> global*/, size_t signal_power_arg_sz /*= CMP2MAGSQ_MAX_SIZE*/,
	fx_pt1 * avg_signal_power_arg /*= avg_signal_power -> global*/, size_t avg_signal_power_arg_sz /*= FIR_MAVG64_MAX_SIZE*/,
	fx_pt1 * the_correlation_arg /*= the_correlation -> global*/, size_t the_correlation_arg_sz /*= DIVIDE_MAX_SIZE*/,
	fx_pt * sync_short_out_frames_arg /*= sync_short_out_frames -> global*/, size_t sync_short_out_frames_arg_sz /*=320*/,
	fx_pt * d_sync_long_out_frames_arg /*= d_sync_long_out_frames -> global*/, size_t d_sync_long_out_frames_arg_sz /*= SYNC_L_OUT_MAX_SIZE*/,
	fx_pt* frame_d_arg, size_t frame_d_arg_sz,
	// Local variable used by do_rcv_ff_work (task in do_recv_pipeline)
        unsigned* num_fft_outs_rcv_fft, size_t num_fft_outs_rcv_fft_sz,
	// 		Local variablse used by decode_signal (task in do_recv_pipeline)
	unsigned* num_dec_bits, size_t num_dec_bits_sz /*= sizeof(unsigned)*/,
	uint8_t* bit_r, size_t bit_r_sz /*= DECODE_IN_SIZE_MAX*/,
	uint8_t* bit, size_t bit_sz /*= DECODE_IN_SIZE_MAX + OFDM_PAD_ENTRIES*/,
	ofdm_param* ofdm, size_t ofdm_sz /*= sizeof(ofdm_param)*/,
	frame_param* frame, size_t frame_sz /*= sizeof(frame_param)*/,
	int* n_res_char, size_t n_res_char_sz /*= sizeof(int)*/,
	// 		Local variables for sdr_decode_ofdm (called by decode_signal, a task in do_recv_pipeline
	uint8_t* inMemory, size_t inMemory_sz /*= 24852*/,
	uint8_t* outMemory, size_t outMemory_sz /*= 18585*/,
	int* d_ntraceback_arg, size_t d_ntraceback_arg_sz /*= sizeof(int)*/
	// End variables used by do_recv_pipeline
) {
#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && true
	void * SectionLoop = __hetero_section_begin();
#endif

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && true
	// 38 inputs, 3 outputs
	void * T1 = __hetero_task_begin(39, n_recvd_in, recvd_in_real, recvd_in_real_sz,
		recvd_msg, recvd_msg_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_in_imag, recvd_in_imag_sz,
		// Start variables used by do_recv_pipeline
		//              Local variables used by do_recv_pipeline
		scrambled_msg, scrambled_msg_sz, ss_freq_offset, ss_freq_offset_sz,
		num_sync_short_vals, num_sync_short_vals_sz,
		sl_freq_offset, sl_freq_offset_sz,
		num_sync_long_vals, num_sync_long_vals_sz, fft_ar_r, fft_ar_r_sz,
		fft_ar_i, fft_ar_i_sz, num_fft_outs, num_fft_outs_sz,
		toBeEqualized, toBeEqualized_sz, equalized, equalized_sz,
		num_eq_out_bits, num_eq_out_bits_sz, psdu, psdu_sz,
		//              Global variables used by do_recv_pipeline
		delay16_out_arg, delay16_out_arg_sz,
		input_data_arg, input_data_sz,
		cmpx_conj_out_arg, cmpx_conj_out_arg_sz,
		cmpx_mult_out_arg, cmpx_mult_out_arg_sz,
		correlation_complex_arg, correlation_complex_arg_sz,
		correlation_arg, correlation_arg_sz,
		signal_power_arg, signal_power_arg_sz,
		avg_signal_power_arg, avg_signal_power_arg_sz,
		the_correlation_arg, the_correlation_arg_sz,
		sync_short_out_frames_arg, sync_short_out_frames_arg_sz,
		d_sync_long_out_frames_arg, d_sync_long_out_frames_arg_sz,
		frame_d_arg, frame_d_arg_sz,
		// Local variable used by do_rcv_ff_work (task in do_recv_pipeline)
        	num_fft_outs_rcv_fft, num_fft_outs_rcv_fft_sz,
		// Local variables for decode_signal, a task in do_recv_pipeline
                num_dec_bits, num_dec_bits_sz,
                bit_r, bit_r_sz,
                bit, bit_sz,
                ofdm, ofdm_sz,
                frame, frame_sz,
                n_res_char, n_res_char_sz,
                // Local variables for sdr_decode_ofdm (called by decode_signal, a task in do_recv_pipeline)
                inMemory, inMemory_sz,
                outMemory, outMemory_sz,
                d_ntraceback_arg, d_ntraceback_arg_sz,
		// End variables used by do_recv_pipeline
		15, recvd_msg_len, recvd_msg_len_sz,
		recvd_in_real, recvd_in_real_sz,
		recvd_in_imag, recvd_in_imag_sz,
		delay16_out_arg, delay16_out_arg_sz,
                input_data_arg, input_data_sz,
                cmpx_conj_out_arg, cmpx_conj_out_arg_sz,
                cmpx_mult_out_arg, cmpx_mult_out_arg_sz,
                correlation_complex_arg, correlation_complex_arg_sz,
                correlation_arg, correlation_arg_sz,
                signal_power_arg, signal_power_arg_sz,
                avg_signal_power_arg, avg_signal_power_arg_sz,
                the_correlation_arg, the_correlation_arg_sz,
                sync_short_out_frames_arg, sync_short_out_frames_arg_sz,
                d_sync_long_out_frames_arg, d_sync_long_out_frames_arg_sz,
                frame_d_arg, frame_d_arg_sz,
		"recieve_pipeline_task");
#endif

	// Now we have the tranmission input data to be decoded...
#if !(defined(HPVM) && defined(HPVM_RECV_PIPELINE))
	DBGOUT(printf("Calling do_recv_pipeline...\n"));
#endif
	// Fake this with a "loopback" of the xmit message...

#if defined(INT_TIME) && !(defined(HPVM) ||  defined(HPVM_RECV_PIPELINE))
	gettimeofday(&start_pd_recv_pipe, NULL);
#endif

	do_recv_pipeline(n_recvd_in, recvd_in_real, recvd_in_real_sz, recvd_in_imag, recvd_in_imag_sz,
		recvd_msg_len, recvd_msg_len_sz, (char *) recvd_msg, recvd_msg_sz,
		//              Local variables used by do_recv_pipeline
		scrambled_msg, scrambled_msg_sz, ss_freq_offset, ss_freq_offset_sz,
		num_sync_short_vals, num_sync_short_vals_sz, sl_freq_offset, sl_freq_offset_sz,
		num_sync_long_vals, num_sync_long_vals_sz, fft_ar_r, fft_ar_r_sz,
		fft_ar_i, fft_ar_i_sz, num_fft_outs, num_fft_outs_sz,
		toBeEqualized, toBeEqualized_sz, equalized, equalized_sz,
		num_eq_out_bits, num_eq_out_bits_sz, psdu, psdu_sz,
		//              Global variables used by do_recv_pipeline
		delay16_out_arg, delay16_out_arg_sz,
		input_data_arg, input_data_sz,
		cmpx_conj_out_arg, cmpx_conj_out_arg_sz,
		cmpx_mult_out_arg, cmpx_mult_out_arg_sz,
		correlation_complex_arg, correlation_complex_arg_sz,
		correlation_arg, correlation_arg_sz,
		signal_power_arg, signal_power_arg_sz,
		avg_signal_power_arg, avg_signal_power_arg_sz,
		the_correlation_arg, the_correlation_arg_sz,
		sync_short_out_frames_arg, sync_short_out_frames_arg_sz,
		d_sync_long_out_frames_arg, d_sync_long_out_frames_arg_sz,
		frame_d_arg, frame_d_arg_sz,
		// 		Local variable used by do_rcv_ff_work (task in do_recv_pipeline)
        	num_fft_outs_rcv_fft, num_fft_outs_rcv_fft_sz,
		// 		Local variables for decode_signal, a task in do_recv_pipeline
                num_dec_bits, num_dec_bits_sz,
                bit_r, bit_r_sz,
                bit, bit_sz,
                ofdm, ofdm_sz,
                frame, frame_sz,
                n_res_char, n_res_char_sz,
                // 		Local variables for sdr_decode_ofdm (called by decode_signal, a task in do_recv_pipeline)
                inMemory, inMemory_sz,
                outMemory, outMemory_sz,
                d_ntraceback_arg, d_ntraceback_arg_sz);

#if defined(INT_TIME) && !(defined(HPVM) ||  defined(HPVM_RECV_PIPELINE))
	gettimeofday(&stop_pd_recv_pipe, NULL);
	pd_recv_pipe_sec += stop_pd_recv_pipe.tv_sec - start_pd_recv_pipe.tv_sec;
	pd_recv_pipe_usec += stop_pd_recv_pipe.tv_usec - start_pd_recv_pipe.tv_usec;
#endif
	// do_recv_pipeline(n_xmit_out, xmit_out_real, xmit_out_imag, &recvd_msg_len, recvd_msg);
#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && true
	__hetero_task_end(T1);
#endif

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && true
	void * T2_Wrapper1 = __hetero_task_begin(5, uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_msg, recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observations, observations_sz,
		2, uncmp_data, uncmp_data_sz, dec_bytes, dec_bytes_sz, "decompress_task_Wrapper1");
	// Don't run this task on fpga as they do a lot of pointer arithemtic in the LZ4 library they use for decompress
	// Given this, it doesn't seem to useful to run this on fpga; plus, it has some weird compilation issues
#endif

	decompress(uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_msg, recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observations, observations_sz);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && true
	__hetero_task_end(T2_Wrapper1);
#endif

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && true
	void * T4_Wrapper1 = __hetero_task_begin(2, observations, observations_sz, uncmp_data, uncmp_data_sz,
		1, observations, observations_sz, "gridFusion_task_Wrapper1");
#endif

	grid_fusion(observations, observations_sz, uncmp_data, uncmp_data_sz);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && true
	__hetero_task_end(T4_Wrapper1);
#endif

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && true
	// End graph here as we are doing IO in the following section so it should probably not be run in
	// parallel with other sections of the code as it can create race conditions.
	__hetero_section_end(SectionLoop);
#endif
}

/* GLOBALS NEEDED FOR RECIEVE PIPELINE */
fx_pt  delay16_out[DELAY_16_MAX_OUT_SIZE];
fx_pt  cmpx_conj_out[CMP_CONJ_MAX_SIZE];
fx_pt  firc_input[CMP_MULT_MAX_SIZE + COMPLEX_COEFF_LENGTH]; // holds cmpx_mult_out but pre-pads with zeros
fx_pt * cmpx_mult_out = &(firc_input[COMPLEX_COEFF_LENGTH]);
fx_pt  correlation_complex[FIRC_MAVG48_MAX_SIZE]; // (firc mov_avg48 output
//fx_pt correlation_complex_m48[MOV_AVG48_MAX_SIZE]; // (mov_avg48 output

fx_pt1  correlation[CMP2MAG_MAX_SIZE]; // complex_to_mangitude outpu
fx_pt1  fir_input[CMP2MAGSQ_MAX_SIZE + COEFF_LENGTH]; // holds signal_power but pre-pads with zeros
fx_pt1 * signal_power = &(fir_input[COEFF_LENGTH]);
fx_pt1  avg_signal_power[FIR_MAVG64_MAX_SIZE]; // fir moving-average-64
//fx_pt1 avg_signal_power_m64[MOV_AVG64_MAX_SIZE]; // moving-average64

fx_pt1 the_correlation[DIVIDE_MAX_SIZE];

fx_pt frame_d[DELAY_320_MAX_OUT_SIZE]; // delay320 output
//fx_pt sync_short_out_frames[SYNC_S_OUT_MAX_SIZE]; // sync_short output
fx_pt * sync_short_out_frames = &(frame_d[320]); // auto-prepends the delay-320 behavior
fx_pt d_sync_long_out_frames[SYNC_L_OUT_MAX_SIZE]; // sync_long_output

//uint8_t  decoded_message[MAX_PAYLOAD_SIZE];   // Holds the resulting decodede message.

// The input data goes through a delay16 that simply re-indexes the data (prepending 16 0+0i values)...
fx_pt * input_data = &delay16_out[16]; // [2*(RAW_DATA_IN_MAX_SIZE + 16)];  // Holds the input data (plus a "front-pad" of 16 0's for delay16


/*
 * This function is called by receive_and_fuse_maps after the state of lmap_count changes to 0.
 * This function excuetes all the tasks that are dependent on this change of state,
 */
void * receive_and_fuse_maps_impl(Observation * observations /*=observations -> global*/, size_t observations_sz /*=2*/) {
	// Now we take in a received transmission with the other AV's map
	// If we receive a transmission, the process to turn it back into the gridMap is:
	int n_recvd_in = 0;
	float recvd_in_real[MAX_XMIT_OUTPUTS];
	size_t recvd_in_real_sz = MAX_XMIT_OUTPUTS;
	float recvd_in_imag[MAX_XMIT_OUTPUTS];
	size_t recvd_in_imag_sz = MAX_XMIT_OUTPUTS;

#if PARALLEL_PTHREADS
	while (1) {
#endif
		DBGOUT(printf("\nTrying to Receive data on RECV port %u socket\n", RECV_PORT));
#ifdef INT_TIME
		gettimeofday(&start_pd_wifi_recv_wait, NULL);
#endif
		// The first 8-bytes of the message is a unsigned long (plus some metadata) telling the size of
		// the actual message sent. So, the size is first read into r_buffer

		char r_buffer[10];
		int valread = read_all(recv_sock, r_buffer, 8);
		DBGOUT(printf("  RECV got %d bytes :'%s'\n", valread, r_buffer));
		DBGOUT2(printf("  RECV msg psn %s\n", "01234567890"));

		if (valread == 8) { // Read in 8-bytes; so we read a unsigned long (plus metadata)

			// In the following esction we are sending message through recv_sock. Since this is an IO it most
			// likely should not go in parallel with other tasks
#ifdef INT_TIME
			gettimeofday(&start_pd_wifi_recv_all, NULL);
#endif

			if (!(r_buffer[0] == 'X' && r_buffer[7] == 'X')) {
				printf("ERROR: Unexpected message from WiFi...\n");
				closeout_and_exit("Unexpected WiFi message...", -3);
			}
			send(recv_sock, ack, 2, 0);

			// Convert r_butter to unsigned long to find out the actual size of the message
			char * ptr;
			unsigned xfer_in_bytes = strtol(r_buffer + 1, &ptr, 10);
			n_recvd_in = xfer_in_bytes / sizeof(float);

			DBGOUT(printf("     Recv %u REAL values %u bytes from RECV port %u socket\n", n_recvd_in,
				xfer_in_bytes, RECV_PORT));
#ifdef INT_TIME
			gettimeofday(&start_pd_wifi_recv_rl, NULL);
#endif

			// Read the actual message (part 1)
			valread = read_all(recv_sock, (char *) recvd_in_real, xfer_in_bytes);
			if (valread < xfer_in_bytes) {
				if (valread == 0) {
					printf("  RECV REAL got ZERO bytes -- END of TRANSFER?\n");
					closeout_and_exit("RECV REAL got zero bytes..", -1);
				}
				else {
					printf("  RECV REAL got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread,
						xfer_in_bytes);
					closeout_and_exit("RECV REAL got too few bytes..", -1);
				}
			}
			DBGOUT2(printf("XFER %4u : Dumping RECV-PIPE REAL raw bytes\n", recv_count);
			for (int i = 0; i < n_recvd_in; i++) {
				printf("XFER %4u REAL-byte %6u : %f\n", odo_count, i, recvd_in_real[i]);
			} printf("\n"));

			DBGOUT(printf("     Recv %u IMAG values %u bytes from RECV port %u socket\n", n_recvd_in,
				xfer_in_bytes, RECV_PORT));
#ifdef INT_TIME
			gettimeofday(&stop_pd_wifi_recv_rl, NULL);
#endif

			// Read the actual message (part 2)
			valread = read_all(recv_sock, (char *) recvd_in_imag, xfer_in_bytes);
			if (valread < xfer_in_bytes) {
				if (valread == 0) {
					printf("  RECV IMAG got ZERO bytes -- END of TRANSFER?\n");
					closeout_and_exit("RECV IMAG got zero bytes..", -1);
				}
				else {
					printf("  RECV IMAG got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, xfer_in_bytes);
					closeout_and_exit("RECV IMAG got too few bytes..", -1);
				}
			}
			DBGOUT2(printf("XFER %4u : Dumping RECV-PIPE IMAG raw bytes\n", xmit_recv_count);
			for (int i = 0; i < n_recvd_in; i++) {
				printf("XFER %4u IMAG-byte %6u : %f\n", odo_count, i, recvd_in_imag[i]);
			} printf("\n"));
#ifdef INT_TIME
			gettimeofday(&stop_pd_wifi_recv_all, NULL);
			pd_wifi_recv_wait_sec += start_pd_wifi_recv_all.tv_sec - start_pd_wifi_recv_wait.tv_sec;
			pd_wifi_recv_wait_usec += start_pd_wifi_recv_all.tv_usec - start_pd_wifi_recv_wait.tv_usec;
			pd_wifi_recv_all_sec += stop_pd_wifi_recv_all.tv_sec - start_pd_wifi_recv_all.tv_sec;
			pd_wifi_recv_all_usec += stop_pd_wifi_recv_all.tv_usec - start_pd_wifi_recv_all.tv_usec;
			pd_wifi_recv_rl_sec += stop_pd_wifi_recv_rl.tv_sec - start_pd_wifi_recv_rl.tv_sec;
			pd_wifi_recv_rl_usec += stop_pd_wifi_recv_rl.tv_usec - start_pd_wifi_recv_rl.tv_usec;
			pd_wifi_recv_im_sec += stop_pd_wifi_recv_all.tv_sec - stop_pd_wifi_recv_rl.tv_sec;
			pd_wifi_recv_im_usec += stop_pd_wifi_recv_all.tv_usec - stop_pd_wifi_recv_rl.tv_usec;
#endif

			// Now we start processing all the messages we read in the previous two sections
			// Helper variables
			int recvd_msg_len = 0;
			size_t recvd_msg_len_sz = sizeof(int);
			unsigned char recvd_msg[1500]; // MAX size of original message in bytes
			size_t recvd_msg_sz = 1500;
			unsigned char uncmp_data[MAX_UNCOMPRESSED_DATA_SIZE];
			size_t uncmp_data_sz = MAX_UNCOMPRESSED_DATA_SIZE;
			int dec_bytes = 0;
			size_t dec_bytes_sz = sizeof(int);

			// Variables to be passed into do_recv_pipeline
			uint8_t scrambled_msg[MAX_ENCODED_BITS * 3 / 4];
			size_t scrambled_msg_sz = MAX_ENCODED_BITS * 3 / 4;
			float ss_freq_offset = 0;
			size_t ss_freq_offset_sz = sizeof(float);
			unsigned num_sync_short_vals = 0;
			size_t num_sync_short_vals_sz = sizeof(unsigned);
			float sl_freq_offset = 0;
			size_t sl_freq_offset_sz = sizeof(float);
			unsigned num_sync_long_vals = 0;
			size_t num_sync_long_vals_sz = sizeof(unsigned);
			fx_pt1 fft_ar_r[FRAME_EQ_IN_MAX_SIZE];
			size_t fft_ar_r_sz = FRAME_EQ_IN_MAX_SIZE * sizeof(fx_pt1);
			fx_pt1 fft_ar_i[FRAME_EQ_IN_MAX_SIZE];
			size_t fft_ar_i_sz = FRAME_EQ_IN_MAX_SIZE * sizeof(fx_pt1);
			unsigned num_fft_outs = 0;
			size_t num_fft_outs_sz = sizeof(unsigned);
			fx_pt toBeEqualized[FRAME_EQ_IN_MAX_SIZE];
			size_t toBeEqualized_sz = FRAME_EQ_IN_MAX_SIZE * sizeof(fx_pt);
			fx_pt equalized[FRAME_EQ_OUT_MAX_SIZE];
			size_t equalized_sz = FRAME_EQ_OUT_MAX_SIZE * sizeof(fx_pt);
			unsigned num_eq_out_bits = 0;
			size_t num_eq_out_bits_sz = sizeof(unsigned);
			unsigned psdu = 0;
			size_t psdu_sz = sizeof(unsigned);

			// Local variablse used by decode_signal
			unsigned num_dec_bits = 0; size_t num_dec_bits_sz = sizeof(unsigned);
			size_t bit_r_sz = DECODE_IN_SIZE_MAX;
			uint8_t bit_r[DECODE_IN_SIZE_MAX];
			size_t bit_sz = DECODE_IN_SIZE_MAX + OFDM_PAD_ENTRIES;
			uint8_t bit[DECODE_IN_SIZE_MAX + OFDM_PAD_ENTRIES];
			ofdm_param ofdm; size_t ofdm_sz = sizeof(ofdm_param);
			frame_param frame; size_t frame_sz = sizeof(frame_param);
			int n_res_char = 0; size_t n_res_char_sz = sizeof(int);

			// Local variables for sdr_decode_ofdm
			uint8_t inMemory[24852]; size_t inMemory_sz = 24852;
			uint8_t outMemory[18585]; size_t outMemory_sz = 18585;
			int d_ntraceback = 0;
			size_t d_ntraceback_arg_sz = sizeof(int);

        		unsigned num_fft_outs_rcv_fft = 0; size_t num_fft_outs_rcv_fft_sz = sizeof(unsigned);

			printf("%s %d Calling fuse_maps", __FILE__, __LINE__);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && true
			// 41 inputs, 7 outputs
			void * LaunchInner = __hetero_launch((void *) fuse_maps, 42,
				n_recvd_in,
				recvd_in_real, recvd_in_real_sz,
				recvd_in_imag, recvd_in_imag_sz,
				&recvd_msg_len, recvd_msg_len_sz,
				recvd_msg, recvd_msg_sz,
				observations, observations_sz,
				uncmp_data, uncmp_data_sz,
				&dec_bytes, dec_bytes_sz,
				// Start variables used by do_recv_pipeline
				// 	Local variables used by do_recv_pipeline
				scrambled_msg, scrambled_msg_sz,
				&ss_freq_offset, ss_freq_offset_sz,
				&num_sync_short_vals, num_sync_short_vals_sz,
				&sl_freq_offset, sl_freq_offset_sz,
				&num_sync_long_vals, num_sync_long_vals_sz,
				fft_ar_r, fft_ar_r_sz,
				fft_ar_i, fft_ar_i_sz,
				&num_fft_outs, num_fft_outs_sz,
				toBeEqualized, toBeEqualized_sz,
				equalized, equalized_sz,
				&num_eq_out_bits, num_eq_out_bits_sz,
				&psdu, psdu_sz,
				//      Global variables used by do_recv_pipeline
				delay16_out, DELAY_16_MAX_OUT_SIZE * sizeof(fx_pt),
				input_data, (DELAY_16_MAX_OUT_SIZE - 16) * sizeof(fx_pt),
				cmpx_conj_out, CMP_CONJ_MAX_SIZE * sizeof(fx_pt),
				cmpx_mult_out, CMP_MULT_MAX_SIZE * sizeof(fx_pt),
				correlation_complex, FIRC_MAVG48_MAX_SIZE * sizeof(fx_pt),
				correlation, CMP2MAG_MAX_SIZE * sizeof(fx_pt1),
				signal_power, CMP2MAGSQ_MAX_SIZE * sizeof(fx_pt1),
				avg_signal_power, FIR_MAVG64_MAX_SIZE * sizeof(fx_pt1),
				the_correlation, DIVIDE_MAX_SIZE * sizeof(fx_pt1),
				sync_short_out_frames, 320 * sizeof(fx_pt),
				d_sync_long_out_frames, SYNC_L_OUT_MAX_SIZE * sizeof(fx_pt),
				frame_d, DELAY_320_MAX_OUT_SIZE * sizeof(fx_pt),
				// 	Local variable used by do_rcv_ff_work (task in do_recv_pipeline)
        			&num_fft_outs_rcv_fft, num_fft_outs_rcv_fft_sz,
				//      Local variablse used by decode_signal (task in do_recv_pipeline)
				&num_dec_bits, num_dec_bits_sz,
				bit_r, bit_r_sz,
				bit, bit_sz,
				&ofdm, ofdm_sz,
				&frame, frame_sz,
				&n_res_char, n_res_char_sz,
				//       Local variables for sdr_decode_ofdm (called by decode_signal, a task in do_recv_pipeline
				inMemory, inMemory_sz,
				outMemory, outMemory_sz,
				&d_ntraceback, d_ntraceback_arg_sz,
				// End variables used by do_recv_pipeline
				18,
				recvd_in_real, recvd_in_real_sz,
				recvd_in_imag, recvd_in_imag_sz,
				&recvd_msg_len, recvd_msg_len_sz,
				uncmp_data, uncmp_data_sz,
				observations, observations_sz,
				&dec_bytes, dec_bytes_sz,
				delay16_out, DELAY_16_MAX_OUT_SIZE * sizeof(fx_pt),
                                input_data, (DELAY_16_MAX_OUT_SIZE - 16) * sizeof(fx_pt),
                                cmpx_conj_out, CMP_CONJ_MAX_SIZE * sizeof(fx_pt),
                                cmpx_mult_out, CMP_MULT_MAX_SIZE * sizeof(fx_pt),
                                correlation_complex, FIRC_MAVG48_MAX_SIZE * sizeof(fx_pt),
                                correlation, CMP2MAG_MAX_SIZE * sizeof(fx_pt1),
                                signal_power, CMP2MAGSQ_MAX_SIZE * sizeof(fx_pt1),
                                avg_signal_power, FIR_MAVG64_MAX_SIZE * sizeof(fx_pt1),
                                the_correlation, DIVIDE_MAX_SIZE * sizeof(fx_pt1),
                                sync_short_out_frames, 320 * sizeof(fx_pt),
                                d_sync_long_out_frames, SYNC_L_OUT_MAX_SIZE * sizeof(fx_pt),
                                frame_d, DELAY_320_MAX_OUT_SIZE * sizeof(fx_pt)
				);
			__hetero_wait(LaunchInner);
#else
			fuse_maps(
				n_recvd_in,
				recvd_in_real, recvd_in_real_sz,
				recvd_in_imag, recvd_in_imag_sz,
				&recvd_msg_len, recvd_msg_len_sz,
				recvd_msg, recvd_msg_sz,
				observations, observations_sz,
				uncmp_data, uncmp_data_sz,
				&dec_bytes, dec_bytes_sz,
				// Start variables used by do_recv_pipeline
				// 	Local variables used by do_recv_pipeline
				scrambled_msg, scrambled_msg_sz,
				&ss_freq_offset, ss_freq_offset_sz,
				&num_sync_short_vals, num_sync_short_vals_sz,
				&sl_freq_offset, sl_freq_offset_sz,
				&num_sync_long_vals, num_sync_long_vals_sz,
				fft_ar_r, fft_ar_r_sz,
				fft_ar_i, fft_ar_i_sz,
				&num_fft_outs, num_fft_outs_sz,
				toBeEqualized, toBeEqualized_sz,
				equalized, equalized_sz,
				&num_eq_out_bits, num_eq_out_bits_sz,
				&psdu, psdu_sz,
				//      Global variables used by do_recv_pipeline
				delay16_out, DELAY_16_MAX_OUT_SIZE * sizeof(fx_pt),
				input_data, (DELAY_16_MAX_OUT_SIZE - 16) * sizeof(fx_pt),
				cmpx_conj_out, CMP_CONJ_MAX_SIZE * sizeof(fx_pt),
				cmpx_mult_out, CMP_MULT_MAX_SIZE * sizeof(fx_pt),
				correlation_complex, FIRC_MAVG48_MAX_SIZE * sizeof(fx_pt),
				correlation, CMP2MAG_MAX_SIZE * sizeof(fx_pt1),
				signal_power, CMP2MAGSQ_MAX_SIZE * sizeof(fx_pt1),
				avg_signal_power, FIR_MAVG64_MAX_SIZE * sizeof(fx_pt1),
				the_correlation, DIVIDE_MAX_SIZE * sizeof(fx_pt1),
				sync_short_out_frames, 320 * sizeof(fx_pt),
				d_sync_long_out_frames, SYNC_L_OUT_MAX_SIZE * sizeof(fx_pt),
				frame_d, DELAY_320_MAX_OUT_SIZE * sizeof(fx_pt),
				// 	Local variable used by do_rcv_ff_work (task in do_recv_pipeline)
        			&num_fft_outs_rcv_fft, num_fft_outs_rcv_fft_sz,
				//      Local variablse used by decode_signal (task in do_recv_pipeline)
				&num_dec_bits, num_dec_bits_sz,
				bit_r, bit_r_sz,
				bit, bit_sz,
				&ofdm, ofdm_sz,
				&frame, frame_sz,
				&n_res_char, n_res_char_sz,
				//       Local variables for sdr_decode_ofdm (called by decode_signal, a task in do_recv_pipeline
				inMemory, inMemory_sz,
				outMemory, outMemory_sz,
				&d_ntraceback, d_ntraceback_arg_sz
				// End variables used by do_recv_pipeline
				);
#endif
			printf("%s %d Out of fuse_maps", __FILE__, __LINE__);

			// This is now the fused map that should be sent to the AV(Car)
			//  The n values of the (fused) local_map Costmap
			// Connect to the Car-Socket and send the data...

			Costmap2D * local_map = &(observations[curr_obs].master_costmap); // Copied from fuse_maps::T3
			//APORVA TODO: Use this fused map to determine what message to send back to CARLA
#ifdef INT_TIME
			gettimeofday(&start_pd_wifi_send, NULL);
#endif
			unsigned car_bytes = sizeof(Costmap2D);
			snprintf(r_buffer, 9, "X%-6uX", car_bytes);
			DBGOUT(printf("\nCAR-OUT Sending %s on CAR port %u socket\n", r_buffer, CAR_PORT));
			send(car_sock, r_buffer, 8, 0);
			DBGOUT(printf("     Send %u bytes on CAR port %u socket\n", car_bytes, CAR_PORT));
			char * car_out_chars = (char *) &(local_map);
			DBGOUT2(printf("CAR-OUT %4u : Dumping XMIT-PIPE REAL raw bytes\n", car_send_count);
			for (int i = 0; i < car_bytes; i++) {
				unsigned char c = car_out_chars[i];
				printf("CAR-OUT %4u REAL-byte %6u : %u\n", car_sendcount, i, c);
			} printf("\n"));
#ifdef INT_TIME
			gettimeofday(&start_pd_wifi_car, NULL);
#endif
			send(car_sock, car_out_chars, car_bytes, 0);
			DBGOUT(printf("     Send %u bytes on CAR port %u socket\n", car_bytes, CAR_PORT));
#ifdef INT_TIME
			gettimeofday(&stop_pd_wifi_car, NULL);
			pd_wifi_car_sec += stop_pd_wifi_car.tv_sec - start_pd_wifi_car.tv_sec;
			pd_wifi_car_usec += stop_pd_wifi_car.tv_usec - start_pd_wifi_car.tv_usec;
#endif
			car_send_count++;
			recv_count++;
		}
		else { // (valread != 8)
			// I don't see the need to put the following code in a graph as there is no parallelism here.
#ifdef INT_TIME
			gettimeofday(&stop_pd_wifi_recv_wait, NULL);
			pd_wifi_recv_wait_sec += stop_pd_wifi_recv_wait.tv_sec - start_pd_wifi_recv_wait.tv_sec;
			pd_wifi_recv_wait_usec += stop_pd_wifi_recv_wait.tv_usec - start_pd_wifi_recv_wait.tv_usec;
#endif
#if PARALLEL_PTHREADS
			if (valread == 0) {
				printf("  RECV header got ZERO bytes -- END of TRANSFER?\n");
				closeout_and_exit("RECV header got zero bytes...", -1);
			}
			else {
				printf("  RECV header got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, 8);
				closeout_and_exit("RECV header got too few bytes...", -1);
			}
#endif
		}
#ifdef INT_TIME
		gettimeofday(&stop_pd_wifi_recv_th, NULL);
		pd_wifi_recv_th_sec = stop_pd_wifi_recv_th.tv_sec - start_pd_wifi_recv_th.tv_sec;
		pd_wifi_recv_th_usec = stop_pd_wifi_recv_th.tv_usec - start_pd_wifi_recv_th.tv_usec;
#endif

#if PARALLEL_PTHREADS
	} // while (1)
#endif
	return NULL;
}

void * receive_and_fuse_maps(void * parm_ptr, size_t parm_ptr_sz) {
#if PARALLEL_PTHREADS
	printf("The receive_and_fuse_maps routine is started: lmap_count = %u\n", lmap_count);
#endif
#ifdef INT_TIME
	gettimeofday(&start_pd_wifi_recv_th, NULL);
#endif
	// We are waiting on lmap_count to change state and as soon as it is done we simply
	// record the time it took.
	// All future tasks in this function can only run after the state of lmap_count changes
	while (lmap_count == 0) {
		DEBUG(printf("  lmap_count = %u\n", lmap_count));
		usleep(1); // wait for first observation (local map) to exist
	}
#ifdef INT_TIME
	gettimeofday(&stop_pd_wifi_lmap_wait, NULL);
	pd_wifi_lmap_wait_sec += stop_pd_wifi_lmap_wait.tv_sec - start_pd_wifi_recv_th.tv_sec;
	pd_wifi_lmap_wait_usec += stop_pd_wifi_lmap_wait.tv_usec - start_pd_wifi_recv_th.tv_usec;
#endif
	receive_and_fuse_maps_impl(observationsArr, sizeof(Observation) * 2);

	return NULL;
}

/* Moved to occgrid.h
	 typedef struct lidar_inputs_struct {
	 float odometry[3];
	 int data_size;
	 char data[200002];
	 }
	 lidar_inputs_t;
	 */

	 // Note: Kindof a major change; previously the entire observation's array was being passed into the function
	 // but only of the value from the array was being used (the index for that value was given by the global
	 // next_obs).
	 // Now, that particular index (and only that index) was being used in the function. So a change was
	 // made to directly pass in the observation value that the index pointed to by next_obs.

#if !defined(COLLAPSE_NODES)
__attribute__ ((noinline)) 
	void process_lidar_to_occgrid(lidar_inputs_t * lidar_inputs, size_t lidarin_sz /*=sizeof( * lidar_inputs)*/,
	Observation * observationVal /* observations[*next_obs_cp] -> from global array*/, size_t observations_sz /*=sizeof(Observation)*/,
	int * n_cmp_bytes /*return by arg*/, size_t n_cmp_bytes_sz /*=1*/,
	unsigned char * cmp_data /*return by arg*/, size_t cmp_data_sz /*=MAX_COMPRESSED_DATA_SIZE*/,
	// Start of global variables used internally by function
	int * curr_obs_cp /*=curr_obs -> global*/, size_t curr_obs_cp_sz /*=sizeof(int)*/,
	int * next_obs_cp /*=next_obs -> global*/, size_t next_obs_cp_sz /*=sizeof(int)*/,
	int * lmap_count_cp /*=lmap_count -> global*/, size_t lmap_count_cp_sz /*=sizeof(unsigned)*/,
	// End of global variables used internally by function
	// Start of arguments to cloudToOccgrid (called by process_lidar_to_occgrid)
	double * AVxyzw, size_t AVxyzw_sz /*=sizeof(double)*/,
	bool * rolling_window, size_t rolling_window_sz /*=sizeof(bool)*/,
	double * min_obstacle_height, size_t min_obstacle_height_sz /*=sizeof(double)*/,
	double * max_obstacle_height, size_t max_obstacle_height_sz /*=sizeof(double)*/,
	double * raytrace_range, size_t raytrace_range_sz /*=sizeof(double)*/,
	unsigned int * size_x, size_t size_x_sz /*=sizeof(unsigned int)*/,
	unsigned int * size_y, size_t size_y_sz /*=sizeof(unsigned int)*/,
	unsigned int * resolution, size_t resolution_sz /*=sizeof(unsigned int)*/,
	int * timer_sequentialize, size_t timer_sequentialize_sz /*=sizeof(int) */
	// End of arguments to cloudToOccgrid (called by process_lidar_to_occgrid)
) {


#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	void * Section = __hetero_section_begin();
#endif

	// CloudToOccgrid
	{

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		void * T1_cloudToOccgrid_Task = __hetero_task_begin(10, observationVal, observations_sz, lidar_inputs, lidarin_sz,
			rolling_window, rolling_window_sz, min_obstacle_height, min_obstacle_height_sz,
			max_obstacle_height, max_obstacle_height_sz, raytrace_range, raytrace_range_sz,
			size_x, size_x_sz, size_y, size_y_sz, resolution, resolution_sz,
			timer_sequentialize, timer_sequentialize_sz,
			2, observationVal, observations_sz, timer_sequentialize, timer_sequentialize_sz,
			"initCostmap_task");
	__hpvm__hint(DEVICE); 
#endif
		{
			*timer_sequentialize = 1;
#ifdef INT_TIME
			// gettimeofday(&ocgr_c2g_total_start, NULL); // See note above this function for why this call was commented out
			gettimeofday(&ocgr_c2g_initCM_start, NULL);
#endif
			double robot_x = lidar_inputs->odometry[0];
			double robot_y = lidar_inputs->odometry[1];
			double robot_z = lidar_inputs->odometry[2];

			initCostmap(observationVal, *rolling_window, *min_obstacle_height, *max_obstacle_height, *raytrace_range,
				*size_x, *size_y, *resolution, robot_x, robot_y, robot_z);
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_initCM_stop, NULL);
			ocgr_c2g_initCM_sec += ocgr_c2g_initCM_stop.tv_sec - ocgr_c2g_initCM_start.tv_sec;
			ocgr_c2g_initCM_usec += ocgr_c2g_initCM_stop.tv_usec - ocgr_c2g_initCM_start.tv_usec;
#endif

		}
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		__hetero_task_end(T1_cloudToOccgrid_Task);
#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		void * T2_cloudToOccgrid_Task = __hetero_task_begin(2, observationVal, observations_sz, lidar_inputs, lidarin_sz,
			1, observationVal, observations_sz, "updateOrigin_task");
	__hpvm__hint(DEVICE);
#endif

		{
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_updOrig_start, NULL);
#endif

			double robot_x = lidar_inputs->odometry[0];
			double robot_y = lidar_inputs->odometry[1];
			double robot_z = lidar_inputs->odometry[2];
			//printf("(1) Number of elements : %d ... ", data_size);
			//printf("First Coordinate = <%f, %f>\n", *data, *(data+1));
			//MOVED to physically inlined here... updateMap(obs_ptr, data, data_size, robot_x, robot_y, robot_z, robot_yaw);
			if (observationVal->rolling_window) {
				//printf("\nUpdating Map .... \n");
				//printf("   robot_x = %f, robot_y = %f, robot_yaw = %f \n", robot_x, robot_y, AVxyzw);
				//printf("   Master Origin = (%f, %f)\n", obs_ptr->master_origin.x, obs_ptr->master_origin.y);
				double new_origin_x = robot_x - observationVal->master_costmap.x_dim / 2;
				double new_origin_y = robot_y - observationVal->master_costmap.y_dim / 2;
				updateOrigin(observationVal, new_origin_x, new_origin_y);
			}
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_updOrig_stop, NULL);
			ocgr_c2g_updOrig_sec += ocgr_c2g_updOrig_stop.tv_sec - ocgr_c2g_updOrig_start.tv_sec;
			ocgr_c2g_updOrig_usec += ocgr_c2g_updOrig_stop.tv_usec - ocgr_c2g_updOrig_start.tv_usec;
#endif
		}
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		__hetero_task_end(T2_cloudToOccgrid_Task);
#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		void * T3_cloudToOccgrid_Task = __hetero_task_begin(3, observationVal, observations_sz, lidar_inputs, lidarin_sz,
			AVxyzw, AVxyzw_sz, 1, observationVal, observations_sz, "updateBounds_task");
	__hpvm__hint(DEVICE);
#endif
		{
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_updBnds_start, NULL);
#endif
			double robot_x = lidar_inputs->odometry[0];
			double robot_y = lidar_inputs->odometry[1];
			double robot_z = lidar_inputs->odometry[2];

			float * data = (float *) (lidar_inputs->data);
			unsigned int data_size = lidar_inputs->data_size / sizeof(float);

			double min_x = 1e30;
			double min_y = 1e30;
			double max_x = -1e30;
			double max_y = -1e30;

			//printf("(1) Number of elements : %d ... ", data_size);
			//printf("First Coordinate = <%f, %f>\n", *data, *(data+1));
			//rotating_window = true; //Comment out if not rolling window

			updateBounds(observationVal, data, data_size, robot_x, robot_y, robot_z,
				*AVxyzw, &min_x, &min_y, &max_x, &max_y);

			//printMap();
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_updBnds_stop, NULL);
			ocgr_c2g_updBnds_sec += ocgr_c2g_updBnds_stop.tv_sec - ocgr_c2g_updBnds_start.tv_sec;
			ocgr_c2g_updBnds_usec += ocgr_c2g_updBnds_stop.tv_usec - ocgr_c2g_updBnds_start.tv_usec;

			// Note (located above this function) explains why the following lines were commented out
			// ocgr_c2g_total_sec  += ocgr_c2g_updBnds_stop.tv_sec  - ocgr_c2g_total_start.tv_sec;
			// ocgr_c2g_total_usec += ocgr_c2g_updBnds_stop.tv_usec - ocgr_c2g_total_start.tv_usec;
#endif
		}
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		__hetero_task_end(T3_cloudToOccgrid_Task);
#endif
#if defined(INT_TIME) && !(defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL))
		gettimeofday(&ocgr_c2g_total_stop, NULL);
		ocgr_c2g_total_sec += ocgr_c2g_total_stop.tv_sec - ocgr_c2g_total_start.tv_sec;
		ocgr_c2g_total_usec += ocgr_c2g_total_stop.tv_usec - ocgr_c2g_total_start.tv_usec;
#endif


	}


	// Write the read-in image to a file
	// write_array_to_file(grid, COST_MAP_ENTRIES);
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	void * T2 = __hetero_task_begin(6, observationVal, observations_sz, n_cmp_bytes, n_cmp_bytes_sz,
		cmp_data, cmp_data_sz, next_obs_cp, next_obs_cp_sz, curr_obs_cp, curr_obs_cp_sz,
		lmap_count_cp, lmap_count_cp_sz,
		3, observationVal, observations_sz, n_cmp_bytes, n_cmp_bytes_sz, cmp_data, cmp_data_sz,
		"compressMap_Task");

	__hpvm__hint(CPU_TARGET);
	// This task is not being placed on the fpga as it does weird pointer arithemetic which causes issues with hpvm
#endif

	printf("%s %d In T2", __FILE__, __LINE__);

	// Now we compress the grid for transmission...
	Costmap2D * local_map = &(observationVal->master_costmap);
	// Now we update the current observation index and the next observation index
	//      Switch curr_obs (global referred to by curr_obs_cp) and next_obs (global referred to by next_obs_cp)
	//      between 0 and 1
	* curr_obs_cp = 1 - *curr_obs_cp;
	*next_obs_cp = 1 - *next_obs_cp;
	(*lmap_count_cp)++;
	// And now we compress to encode for Wifi transmission, etc.
#ifdef INT_TIME
	gettimeofday(&start_pd_lz4_cmp, NULL);
#endif
	* n_cmp_bytes = LZ4_compress_default((char *) local_map, (char *) cmp_data,
		MAX_UNCOMPRESSED_DATA_SIZE, MAX_COMPRESSED_DATA_SIZE);

#ifdef INT_TIME
	gettimeofday(&stop_pd_lz4_cmp, NULL);
	pd_lz4_cmp_sec += stop_pd_lz4_cmp.tv_sec - start_pd_lz4_cmp.tv_sec;
	pd_lz4_cmp_usec += stop_pd_lz4_cmp.tv_usec - start_pd_lz4_cmp.tv_usec;
#endif


#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	__hetero_task_end(T2);
#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	__hetero_section_end(Section);
#endif
}
#endif

// Local variables for do_xmit_pipeline
#define MAX_SIZE 24600          // from xmit_pipe.c
#define ofdm_max_out_size 33280 // from xmit_pipe.c

void transmit_occgrid(
	int n_xmit_out, float * xmit_out_real, size_t xmit_out_real_sz,
	float * xmit_out_imag, size_t xmit_out_imag_sz
	) {
	// This section has no tasks as we are doing IO; introducing tasks can lead to race conditions
	// Now we transmit the grid...

	printf("  Back from do_xmit_pipeline with ..\n");

	// This is now the content that should be sent out via IEEE 802.11p WiFi
	// The n_xmit_out values of xmit_out_real and xmit_out_imag
	// Connect to the Wifi-Socket and send the n_xmit_out
	char w_buffer[10];
#ifdef INT_TIME
	gettimeofday(&start_pd_wifi_send, NULL);
#endif
	unsigned xfer_bytes = (n_xmit_out) * sizeof(float);
	snprintf(w_buffer, 9, "X%-6uX", xfer_bytes);
//	DBGOUT(printf("\nXMIT Sending %s on XMIT port %u socket\n", w_buffer, XMIT_PORT));
	send(xmit_sock, w_buffer, 8, 0);
//	DBGOUT(printf("     Send %u REAL values %u bytes on XMIT port %u socket\n", n_xmit_out, xfer_bytes,
//		XMIT_PORT));
//	DBGOUT2(printf("XFER %4u : Dumping XMIT-PIPE REAL raw bytes\n", xmit_count);
//	for (int i = 0; i < n_xmit_out; i++) {
//		printf("XFER %4u REAL-byte %6u : %f\n", xmit_count, i, xmit_out_real[i]);
//	} printf("\n"));
#ifdef INT_TIME
	gettimeofday(&start_pd_wifi_send_rl, NULL);
#endif
	printf("Sending data over socket ..\n");
	printf("Xmit_out_real: %s\n", (char *) (xmit_out_real));
	printf("n_xmit_out: %d\n", n_xmit_out);
	send(xmit_sock, (char *) (xmit_out_real), (n_xmit_out) * sizeof(float), 0);
//	DBGOUT(printf("     Send %u IMAG values %u bytes on XMIT port %u socket\n", n_xmit_out, xfer_bytes,
//		XMIT_PORT));
//	DBGOUT2(printf("XFER %4u : Dumping XMIT-PIPE IMAG raw bytes\n", xmit_count);
//	for (int i = 0; i < (n_xmit_out); i++) {
//		printf("XFER %4u IMAG-byte %6u : %f\n", xmit_count, i, xmit_out_imag[i]);
//	} printf("\n"));
#ifdef INT_TIME
	gettimeofday(&stop_pd_wifi_send_rl, NULL);
#endif
	printf("Sending xmit_out_imag\n");
	send(xmit_sock, (char *) (xmit_out_imag), (n_xmit_out) * sizeof(float), 0);
#ifdef INT_TIME
	gettimeofday(&stop_pd_wifi_send, NULL);
	pd_wifi_send_sec += stop_pd_wifi_send.tv_sec - start_pd_wifi_send.tv_sec;
	pd_wifi_send_usec += stop_pd_wifi_send.tv_usec - start_pd_wifi_send.tv_usec;
	pd_wifi_send_rl_sec += stop_pd_wifi_send_rl.tv_sec - start_pd_wifi_send_rl.tv_sec;
	pd_wifi_send_rl_usec += stop_pd_wifi_send_rl.tv_usec - start_pd_wifi_send_rl.tv_usec;
	pd_wifi_send_im_sec += stop_pd_wifi_send.tv_sec - stop_pd_wifi_send_rl.tv_sec;
	pd_wifi_send_im_usec += stop_pd_wifi_send.tv_usec - stop_pd_wifi_send_rl.tv_usec;
#endif

	printf("Incrementing xmit_count\n");
	xmit_count++;
	printf("Leaving transmit_occgrid\n");
}


// Functions pasted from xmit_pipe.c for collapsing of do_xmit_pipeline
static int do_mapper_work(int psdu_length, uint8_t * d_psdu, size_t d_psdu_size, uint8_t * d_map_out, size_t d_map_out_sz,
		uint8_t * d_scrambler, size_t d_scrambler_sz,
		char* d_symbols, size_t d_symbols_sz,
		int* d_symbols_offset, size_t d_symbols_offset_sz,
		int* d_symbols_len, size_t d_symbols_len_sz,
		ofdm_param * d_ofdm, size_t d_ofdm_sz,
		frame_param * d_frame, size_t d_frame_sz
		) 
	// int noutput, gr_vector_int& ninput_items, gr_vector_const_void_star& input_items, gr_vector_void_star& output_items )
{
	char data_bits[12264]; // = (char*)calloc(frame.n_data_bits, sizeof(char));
	char interleaved_data[24528]; // = (char*)calloc(frame.n_encoded_bits, sizeof(char));
	char symbols[24528]; // = (char*)calloc((frame.n_encoded_bits / d_ofdm->n_bpsc), sizeof(char)); ## n_bpsc == 1 ##
	char scrambled_data[12264]; // = (char*)calloc(frame.n_data_bits, sizeof(char));
	char encoded_data[12264 * 2]; // = (char*)calloc(frame.n_data_bits * 2, sizeof(char));
	char * punctured_data = encoded_data; // TRUE for BPSK_1_2, QPSK_1_2, and QAM16_1_2
#ifdef INT_TIME
	gettimeofday(&xdmw_total_start, NULL);
#endif
	/* frame_param frame(d_ofdm, psdu_length); */
	d_frame->psdu_size = psdu_length; // psdu_length <= 2572
	// number of symbols (17-11)
	d_frame->n_sym = (int) ceil((16 + 8 * d_frame->psdu_size + 6) / (double) d_ofdm->n_dbps); // n_sym <= (16 + (8*2572)+6)/24 = 860
	d_frame->n_data_bits = d_frame->n_sym * d_ofdm->n_dbps;
	// number of padding bits (17-13)
	d_frame->n_pad = d_frame->n_data_bits - (16 + 8 * d_frame->psdu_size + 6);
	d_frame->n_encoded_bits = d_frame->n_sym * d_ofdm->n_cbps;


	//generate the WIFI data field, adding service field and pad bits
	//     generate_bits(psdu, data_bits, frame);
	//     void generate_bits(const char *psdu, char *data_bits, frame_param &frame)
#ifdef INT_TIME
	gettimeofday(&xdmw_genDF_start, NULL);
#endif 
	{
		// first 16 bits are zero (SERVICE/DATA field)
		//memset(data_bits, 0, 16);
		//data_bits += 16;
		for (int i = 0; i < 16; i++) {
			data_bits[i] = 0;
		}
		for (int i = 0; i < d_frame->psdu_size; i++) {
			for (int b = 0; b < 8; b++) {
				data_bits[16 + i * 8 + b] = !!(d_psdu[i] & (1 << b));
			}
		}
	}
	DEBUG({
			int di_row = 0;
			int symbols_len = d_frame->n_data_bits;
			printf("\ndata_bits out:\n%6u : ", di_row);
			for (int di = 0; di < symbols_len; di++) {
			printf("%1x", data_bits[di]);
			if ((di % 128) == 127) {
			di_row++;
			printf("\n%6u : ", di_row);
			}
			else if ((di % 8) == 7) {
			printf(" ");
			}
			}
			printf("\n\n");
			});
#ifdef INT_TIME
	gettimeofday(&xdmw_genDF_stop, NULL);
	xdmw_genDF_sec += xdmw_genDF_stop.tv_sec - xdmw_genDF_start.tv_sec;
	xdmw_genDF_usec += xdmw_genDF_stop.tv_usec - xdmw_genDF_start.tv_usec;
#endif

	// scrambling
	//     scramble(     data_bits, scrambled_data,              frame,      d_scrambler++);
	//     void scramble(const char *in, char *out,      frame_param &frame, char initial_state)
	{
		int state = (*d_scrambler)++; // initial_state;
		int feedback;

		for (int i = 0; i < d_frame->n_data_bits; i++) {
			feedback = (!!(state & 64)) ^ (!!(state & 8));
			scrambled_data[i] = feedback ^ data_bits[i];
			DEBUG(printf("  %u : state %u   feedback %u   data %u   scrambled %u\n", i, state, feedback, data_bits[i], scrambled_data[i]));
			state = ((state << 1) & 0x7e) | feedback;
		}
	}

	if (*d_scrambler > 127) {
		*d_scrambler = 1;
	}

	// reset tail bits
	//     reset_tail_bits(scrambled_data, frame);
	//     void reset_tail_bits(char *scrambled_data, frame_param &frame)
	{
		//memset(scrambled_data + d_frame->n_data_bits - d_frame->n_pad - 6, 0, 6 * sizeof(char));
		for (int i = 0; i < 6; i++) {
			scrambled_data[d_frame->n_data_bits - d_frame->n_pad - 6 + i] = 0;
		}
	}
	DEBUG({
			int di_row = 0;
			int symbols_len = d_frame->n_data_bits;
			printf("\nscrambled out:\n%6u : ", di_row);
			for (int di = 0; di < symbols_len; di++) {
			printf("%1x", scrambled_data[di]);
			if ((di % 128) == 127) {
			di_row++;
			printf("\n%6u : ", di_row);
			}
			else if ((di % 8) == 7) {
			printf(" ");
			}
			}
			printf("\n\n");
			});
#ifdef INT_TIME
	gettimeofday(&xdmw_scrmbl_stop, NULL);
	xdmw_scrmbl_sec += xdmw_scrmbl_stop.tv_sec - xdmw_genDF_stop.tv_sec;
	xdmw_scrmbl_usec += xdmw_scrmbl_stop.tv_usec - xdmw_genDF_stop.tv_usec;
#endif

	// encoding
	//     convolutional_encoding(scrambled_data, encoded_data, frame);
	//     void convolutional_encoding(const char *in, char *out, frame_param &frame)
	convolutional_encoding(scrambled_data, encoded_data, d_frame->n_data_bits);
	/* { */
	/*      int state = 0; */
	/*      for(int i = 0; i < d_frame->n_data_bits; i++) { */
	/*        assert(scrambled_data[i] == 0 || scrambled_data[i] == 1); */
	/*        state = ((state << 1) & 0x7e) | scrambled_data[i]; */
	/*        encoded_data[i * 2]     = ones_count(state & 0155) % 2; */
	/*        encoded_data[i * 2 + 1] = ones_count(state & 0117) % 2; */
	/*      } */
	/* } */
	DEBUG({
			int di_row = 0;
			int symbols_len = d_frame->n_data_bits;
			printf("\nencoded out:\n%6u : ", di_row);
			for (int di = 0; di < symbols_len; di++) {
			printf("%1x", encoded_data[di]);
			if ((di % 128) == 127) {
			di_row++;
			printf("\n%6u : ", di_row);
			}
			else if ((di % 8) == 7) {
			printf(" ");
			}
			}
			printf("\n\n");
			});
#ifdef INT_TIME
	gettimeofday(&xdmw_cnvEnc_stop, NULL);
	xdmw_cnvEnc_sec += xdmw_cnvEnc_stop.tv_sec - xdmw_scrmbl_stop.tv_sec;
	xdmw_cnvEnc_usec += xdmw_cnvEnc_stop.tv_usec - xdmw_scrmbl_stop.tv_usec;
#endif

	// puncturing
	//puncturing(encoded_data, punctured_data, frame, d_ofdm);
	/**## TODO: I think we are only using BPSK_1_2 for now -- can hard-code that to avoid the SWITCH ##**/
	/**   NOTE: If we use BPSK_1_2 then this just COPIES the data UNCHANGED -- so it is really a NOP **
	//void puncturing(const char *in, char *out , frame_param *frame, ofdm_param *ofdm)
	{
	int mod;
	int oidx = 0;
	for (int i = 0; i < d_frame->n_data_bits * 2; i++) {
	switch(d_ofdm->encoding) {
	case BPSK_1_2:
	case QPSK_1_2:
	case QAM16_1_2:
	punctured_data[oidx] = encoded_data[i];
	oidx++;
	break;

	case QAM64_2_3:
	if (i % 4 != 3) {
	punctured_data[oidx] = encoded_data[i];
	oidx++;
	}
	break;

	case BPSK_3_4:
	case QPSK_3_4:
	case QAM16_3_4:
	case QAM64_3_4:
	mod = i % 6;
	if (!(mod == 3 || mod == 4)) {
	punctured_data[oidx] = encoded_data[i];
	oidx++;
	}
	break;
defaut:
assert(false);
break;
}
}
}**/
	// EFFECTIVELY: punctured_data = encoded_data
	DEBUG({
			int di_row = 0;
			int symbols_len = d_frame->n_data_bits;
			printf("\npunctured out:\n%6u : ", di_row);
			for (int di = 0; di < symbols_len; di++) {
			printf("%1x", punctured_data[di]);
			if ((di % 128) == 127) {
			di_row++;
			printf("\n%6u : ", di_row);
			}
			else if ((di % 8) == 7) {
			printf(" ");
			}
			}
			printf("\n\n");
			});
#ifdef INT_TIME
gettimeofday(&xdmw_punct_stop, NULL);
xdmw_punct_sec += xdmw_punct_stop.tv_sec - xdmw_cnvEnc_stop.tv_sec;
xdmw_punct_usec += xdmw_punct_stop.tv_usec - xdmw_cnvEnc_stop.tv_usec;
#endif

	//printf("Calling interleave\n");
//std::cout << "punctured" << std::endl;
// interleaving
//     interleave(punctured_data, interleaved_data, frame, d_ofdm);
// printf("ERA: psdu_size: %d\n", d_frame->psdu_size);
interleave(punctured_data, interleaved_data, d_frame->n_sym, d_ofdm->n_cbps, d_ofdm->n_bpsc, false);
/* //std::cout << "interleaved" << std::endl; */
DEBUG({
		int di_row = 0;
		int symbols_len = d_frame->n_sym * 48; // 24528
		printf("\ninterleaved out:\n%6u : ", di_row);
		for (int di = 0; di < symbols_len; di++) {
		printf("%1x", interleaved_data[di]);
		if ((di % 128) == 127) {
		di_row++;
		printf("\n%6u : ", di_row);
		}
		else if ((di % 8) == 7) {
		printf(" ");
		}
		}
		printf("\n\n");
		});
#ifdef INT_TIME
gettimeofday(&xdmw_intlv_stop, NULL);
xdmw_intlv_sec += xdmw_intlv_stop.tv_sec - xdmw_punct_stop.tv_sec;
xdmw_intlv_usec += xdmw_intlv_stop.tv_usec - xdmw_punct_stop.tv_usec;
#endif

// one byte per symbol
//     split_symbols(interleaved_data, symbols, frame, d_ofdm);
//     void split_symbols(const char *in, char *out, frame_param &frame, ofdm_param &ofdm)
// printf("On line 704 in xmit_pipe.c\n");
{
	int n_symbols = d_frame->n_sym * 48;
	int idx = 0;
	for (int i = 0; i < n_symbols; i++) {
		symbols[i] = 0;
		for (int k = 0; k < d_ofdm->n_bpsc; k++) {
			// assert(interleaved_data[idx] == 1 || interleaved_data[idx] == 0); // HPVM: can't have asserts on fpga
			symbols[i] |= (interleaved_data[idx] << k);
			idx++;
		}
	}
}

//printf("Assigning d_symbols_len\n");
*d_symbols_len = d_frame->n_sym * 48; // 24528
//printf("Done assigning d_symbols_len\n");
//printf("d_symbols_len = %u * 48 = %u\n", d_frame->n_sym, *d_symbols_len);

//printf("Assigning d_symbols\n");
int termination = *d_symbols_len;
for (int di = 0; di < termination; di++) {
	//printf("symbols[%d] = %d\n", di, symbols[di]);
	//printf("d_symbols[%d] = %d\n", di, d_symbols[di]);
	d_symbols[di] = symbols[di];
}
//printf("Done assigning d_symbols\n");
//printf("d_symbols_len = %u * 48 = %u\n", d_frame->n_sym, d_symbols_len); fflush(stdout);
DEBUG({
		int di_row = 0;
		printf("\nd_symbols out:\n%6u : ", di_row);
		for (int di = 0; di < *d_symbols_len; di++) {
		printf("%1x", d_symbols[di]);
		if ((di % 128) == 127) {
		di_row++;
		printf("\n%6u : ", di_row);
		}
		else if ((di % 8) == 7) {
		printf(" ");
		}
		}
		printf("\n\n");
		});
#ifdef INT_TIME
gettimeofday(&xdmw_symbls_stop, NULL);
xdmw_symbls_sec += xdmw_symbls_stop.tv_sec - xdmw_intlv_stop.tv_sec;
xdmw_symbls_usec += xdmw_symbls_stop.tv_usec - xdmw_intlv_stop.tv_usec;
#endif

int i = *d_symbols_len - *d_symbols_offset;
//printf("output i = %u :  d_sym = %p and d_sym_off = %u\n", i, (void *) d_symbols, *d_symbols_offset);
for (int di = 0; di < i; di++) {
	d_map_out[di] = d_symbols[*d_symbols_offset + di];
	DEBUG(
			if (di < 16) {
			printf("%2u : d_map_out[%2u] = %1x : d_symbols[%2u] = %1x\n", i, di, d_map_out[di], (*d_symbols_offset + di), d_symbols[*d_symbols_offset + di]);
			});
}
//printf("Updating the value of d_symbols_offset\n");
//printf(" Done with do_mapper_work %d\n", 5*(*d_symbols_len));
*d_symbols_offset += i;

if (*d_symbols_offset == *d_symbols_len) {
	*d_symbols_offset = 0;
	DEBUG(printf("reset d_symbols_offset to 0\n"));
}
//printf("finished updating the value of d_symbols_offset\n");

DEBUG({
		int di_row = 0;
		printf("\nMapper out:\n%6u : ", di_row);
		for (int di = 0; di < d_frame->n_encoded_bits /*noutput*/; di++) {
		printf("%1x", d_map_out[di]);
		if ((di % 128) == 127) {
		di_row++;
		printf("\n%6u : ", di_row);
		}
		else if ((di % 8) == 7) {
		printf(" ");
		}
		}
		printf("\n");
		fflush(stdout);
		});
#ifdef INT_TIME
gettimeofday(&xdmw_mapout_stop, NULL);
//gettimeofday(&xdmw_stop, NULL);
xdmw_mapout_sec += xdmw_mapout_stop.tv_sec - xdmw_symbls_stop.tv_sec;
xdmw_mapout_usec += xdmw_mapout_stop.tv_usec - xdmw_symbls_stop.tv_usec;
xdmw_total_sec += xdmw_mapout_stop.tv_sec - xdmw_total_start.tv_sec;
xdmw_total_usec += xdmw_mapout_stop.tv_usec - xdmw_total_start.tv_usec;
#endif
return i;
}

static int get_bit(int b, int i) { // Is this really an efficient way to do this?
	return (b & (1 << i) ? 1 : 0); // This is one shift, one and, one compare, and a branch.
	//return ((b >> i) & 0x1); // Isn't this a better way?  One shift, one and
}

static void generate_signal_field(char * out,
		ofdm_param * d_ofdm, size_t d_ofdm_sz,
		frame_param * d_frame, size_t d_frame_sz
		) {
	//data bits of the signal header
	//char *signal_header = (char *) malloc(sizeof(char) * 24);
	char signal_header[24];
	//signal header after...
	//convolutional encoding
	//char *encoded_signal_header = (char *) malloc(sizeof(char) * 48);
	char encoded_signal_header[48];
	//interleaving
	//char *interleaved_signal_header = (char *) malloc(sizeof(char) * 48);
	//char interleaved_signal_header[48]; -- this writes into "out"

	int length = d_frame->psdu_size;

	// first 4 bits represent the modulation and coding scheme
	signal_header[0] = get_bit(d_ofdm->rate_field, 3);
	signal_header[1] = get_bit(d_ofdm->rate_field, 2);
	signal_header[2] = get_bit(d_ofdm->rate_field, 1);
	signal_header[3] = get_bit(d_ofdm->rate_field, 0);
	// 5th bit is reserved and must be set to 0
	signal_header[4] = 0;
	// then 12 bits represent the length
	signal_header[5] = get_bit(length, 0);
	signal_header[6] = get_bit(length, 1);
	signal_header[7] = get_bit(length, 2);
	signal_header[8] = get_bit(length, 3);
	signal_header[9] = get_bit(length, 4);
	signal_header[10] = get_bit(length, 5);
	signal_header[11] = get_bit(length, 6);
	signal_header[12] = get_bit(length, 7);
	signal_header[13] = get_bit(length, 8);
	signal_header[14] = get_bit(length, 9);
	signal_header[15] = get_bit(length, 10);
	signal_header[16] = get_bit(length, 11);
	//18-th bit is the parity bit for the first 17 bits
	int sum = 0;
	for (int i = 0; i < 17; i++) {
		if (signal_header[i]) {
			sum++;
		}
	}
	signal_header[17] = sum % 2;

	DEBUG(unsigned hdr_psdu = 0x0; printf("ENC LENGTH PSDU      ");
			for (int i = 16; i >= 5; i--) {
			printf("%01x", signal_header[i]);
			hdr_psdu = (hdr_psdu << 1) | signal_header[i];
			}
			printf("  = %03x vs 0x%03x = %u vs %u\n", hdr_psdu, length, hdr_psdu, length));

	// last 6 bits must be set to 0
	for (int i = 0; i < 6; i++) {
		signal_header[18 + i] = 0;
	}

	ofdm_param signal_ofdm; //(BPSK_1_2);
	signal_ofdm.encoding = BPSK_1_2;
	signal_ofdm.n_bpsc = 1;
	signal_ofdm.n_cbps = 48;
	signal_ofdm.n_dbps = 24;
	signal_ofdm.rate_field = 0x0D; // 0b00001101

	frame_param signal_param; // (signal_ofdm, 0);
	signal_param.psdu_size = 0;
	signal_param.n_sym = (int) ceil((16 + 8 * signal_param.psdu_size + 6) / (double) signal_ofdm.n_dbps);
	signal_param.n_data_bits = signal_param.n_sym * signal_ofdm.n_dbps;
	signal_param.n_pad = signal_param.n_data_bits - (16 + 8 * signal_param.psdu_size + 6);
	signal_param.n_encoded_bits = signal_param.n_sym * signal_ofdm.n_cbps;

	// convolutional encoding (scrambling is not needed)
	convolutional_encoding(signal_header, encoded_signal_header, signal_param.n_data_bits);
	// interleaving
	interleave(encoded_signal_header, out, signal_param.n_sym, signal_ofdm.n_cbps, signal_ofdm.n_bpsc, false);

	/* free(signal_header); */
	/* free(encoded_signal_header); */
	/* free(interleaved_signal_header); */
}

static int do_packet_header_gen(unsigned int packet_len, uint8_t * out,
		ofdm_param * d_ofdm, size_t d_ofdm_sz,
		frame_param * d_frame, size_t d_frame_sz
		) // int noutput_items, int ninput_items, uint8_t* input_items, uint8_t* output_items)
{
	//## From     int packet_headergenerator_bb_impl::work (int noutput_items,
	//  NOTE: Need to know what type of header -- default or OFDM (?) as the
	//    OFDM header does a scramble pass after the default header setup.
	//    I'm pretty sure we are using OFDM here... BUT MAYBE NOT OFDM HEADERS?
	/* I'm inlining this call, as it just calles gneerate_signal_field
	   signal_field_header_formatter(packet_len, out); // , const std::vector<tag_t> &tags)
	   */
	generate_signal_field((char *) out, d_ofdm, d_ofdm_sz, d_frame, d_frame_sz); //, d_frame, d_ofdm);
	return 48; // this is the length of the output header -- the convolutional encoded and interleaved header...
}

static float bpsk_chunks2sym(int index) {
	if (index == 0) {
		return -1;
	}
	return 1;
}

static int do_ofdm_carrier_allocator_cvc_impl_work(int noutput_items,
		int ninput_items,
		float * in_real, float * in_imag, // complex numbers
		float * out_real, float * out_imag, // complex numbers
		int* d_pilot_carriers, size_t d_pilot_carriers_sz,
		int* d_occupied_carriers, size_t d_occupied_carriers_sz
		) {
const float d_sync_words_imag[d_num_sync_words][d_size_sync_words] = {
	{
		0.0,
		0.0,
		0.0,
		0.0, //   4
		0.0,
		0.0,
		0.0,
		0.0,
		+1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0, // 32
		+1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		+1.4719601443879746,
		0.0,
		0.0,
		0.0, // 64
		0.0,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		+1.4719601443879746,
		0.0,
		0.0,
		0.0, // 96
		+1.4719601443879746,
		0.0,
		0.0,
		0.0,
		+1.4719601443879746,
		0.0,
		0.0,
		0.0,
		+1.4719601443879746,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0
	}, // 128
	{
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		+1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		+1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		+1.4719601443879746,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		+1.4719601443879746,
		0.0,
		0.0,
		0.0,
		+1.4719601443879746,
		0.0,
		0.0,
		0.0,
		+1.4719601443879746,
		0.0,
		0.0,
		0.0,
		+1.4719601443879746,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0
	},
	{
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		1,
		0,
		1,
		0,
		1,
		0,
		-1,
		0,
		1,
		0,
		-1,
		0,
		1,
		0,
		1,
		0,
		1,
		0,
		1,
		0,
		-1,
		0,
		-1,
		0,
		1,
		0,
		-1,
		0,
		-1,
		0,
		-1,
		0,
		1,
		0,
		-1,
		0,
		-1,
		0,
		1,
		0,
		1,
		0,
		1,
		0,
		1,
		0,
		-1,
		0,
		1,
		0,
		-1,
		0,
		0,
		0,
		0,
		0,
		0
	},
	{
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0
	}
};

const float d_sync_words_real[d_num_sync_words][d_size_sync_words] = {
	{
		0.0,
		0.0,
		0.0,
		0.0, //   4
		0.0,
		0.0,
		0.0,
		0.0,
		1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0, // 32
		1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		1.4719601443879746,
		0.0,
		0.0,
		0.0, // 64
		0.0,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		1.4719601443879746,
		0.0,
		0.0,
		0.0, // 96
		1.4719601443879746,
		0.0,
		0.0,
		0.0,
		1.4719601443879746,
		0.0,
		0.0,
		0.0,
		1.4719601443879746,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0
	}, // 128
	{
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		1.4719601443879746,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		-1.4719601443879746,
		0.0,
		0.0,
		0.0,
		1.4719601443879746,
		0.0,
		0.0,
		0.0,
		1.4719601443879746,
		0.0,
		0.0,
		0.0,
		1.4719601443879746,
		0.0,
		0.0,
		0.0,
		1.4719601443879746,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0
	},
	{
		0,
		0,
		0,
		0,
		0,
		0,
		-1,
		0,
		-1,
		0,
		-1,
		0,
		-1,
		0,
		1,
		0,
		1,
		0,
		-1,
		0,
		1,
		0,
		1,
		0,
		1,
		0,
		-1,
		0,
		1,
		0,
		-1,
		0,
		0,
		0,
		1,
		0,
		1,
		0,
		1,
		0,
		-1,
		0,
		1,
		0,
		-1,
		0,
		1,
		0,
		1,
		0,
		1,
		0,
		-1,
		0,
		1,
		0,
		1,
		0,
		-1,
		0,
		0,
		0,
		0,
		0
	},
	{
		0,
		0,
		0,
		0,
		0,
		0,
		1,
		1,
		-1,
		-1,
		1,
		1,
		-1,
		1,
		-1,
		1,
		1,
		1,
		1,
		1,
		1,
		-1,
		-1,
		1,
		1,
		-1,
		1,
		-1,
		1,
		1,
		1,
		1,
		0,
		1,
		-1,
		-1,
		1,
		1,
		-1,
		1,
		-1,
		1,
		-1,
		-1,
		-1,
		-1,
		-1,
		1,
		1,
		-1,
		-1,
		1,
		-1,
		1,
		-1,
		1,
		1,
		1,
		1,
		0,
		0,
		0,
		0,
		0
	}
};


	/* const gr_complex *in = (const gr_complex *) input_items[0]; */
	/* gr_complex *out = (gr_complex *) output_items[0]; */
	/* std::vector<tag_t> tags; */

	DEBUG(printf("In ofdm_carrier_allocator_cvc_impl work with nout = %u and d_num_sync_words = %u\n", noutput_items, d_num_sync_words); fflush(stdout));
	// Reset the contents of the output_items to 0x00 (so any not over-written remain 0x00?)
	//memset((void *) out_real, 0x00, sizeof(float) * d_fft_len * noutput_items);
	//memset((void *) out_imag, 0x00, sizeof(float) * d_fft_len * noutput_items);
	for (int ti = 0; ti < d_fft_len * (noutput_items + d_num_sync_words); ti++) {
		out_real[ti] = 0.0;
		out_imag[ti] = 0.0;
	}

	// Copy Sync word
	//printf("\nCopy sync words...\n");
	int o_offset = 0;
	for (unsigned i = 0; i < d_num_sync_words; i++) {
		//memcpy((void *) out, (void *) &d_sync_words[i][0], sizeof(gr_complex) * d_fft_len);
		int oidx = o_offset;
		for (int ti = 0; ti < d_fft_len; ti++) {
			out_real[oidx] = d_sync_words_real[i][ti];
			out_imag[oidx] = d_sync_words_imag[i][ti];
			DEBUG(printf(" out[%4u] = %19.16f + %19.16f i = d_sync_words[%u][%2u] = %19.16f + %19.16f i\n", oidx, out_real[oidx], out_imag[oidx], i, ti, d_sync_words_real[i][ti], d_sync_words_imag[i][ti]));
			oidx++;
		}
		DEBUG(printf("\n"));
		//out += d_fft_len;
		o_offset += d_fft_len;
	}
	//printf("Done initializing out using d_sync_words_real & d_sync_words_imag\n");

	// Copy data symbols
	long n_ofdm_symbols = 0; // Number of output items
	int curr_set = 0;
	int symbols_to_allocate = d_size_occupied_carriers;
	int symbols_allocated = 0;
	//printf("\nCopy data symbols -- symbols_to_allocate = %u\n", symbols_to_allocate);
	for (int i = 0; i < ninput_items; i++) {
		if (symbols_allocated == 0) {
			/*   // Copy all tags associated with these input symbols onto this OFDM symbol */
			/*   get_tags_in_range(tags, 0, */
			/*                      nitems_read(0)+i, */
			/*                      nitems_read(0)+std::min(i+symbols_to_allocate, (int) ninput_items[0]) */
			/*                      ); */
			/*   for (unsigned t = 0; t < tags.size(); t++) { */
			/*      add_item_tag( */
			/*                   0, */
			/*                   nitems_written(0) + n_ofdm_symbols + (n_ofdm_symbols == 0 ? 0 : d_sync_words.size()), */
			/*                   tags[t].key, */
			/*                   tags[t].value */
			/*                   ); */
			/*   } */
			n_ofdm_symbols++;
		}
		//printf("Accessing d_occupied_carriers[%d][%d]\n", curr_set, symbols_allocated);
		//printf("Accessing d_occupied_carriers[%d] = %d\n", curr_set, d_occupied_carriers[curr_set]);
		//printf("d_occupied_carriers[%d][%d] = %d\n", curr_set, symbols_allocated, d_occupied_carriers[(curr_set * d_num_occupied_carriers) + symbols_allocated]);
		int o_idx = o_offset + (n_ofdm_symbols - 1) * d_fft_len + 
										d_occupied_carriers[(curr_set * d_num_occupied_carriers) + symbols_allocated];
		//printf("o_idx = %d\n", o_idx);
		out_real[o_idx] = in_real[i];
		out_imag[o_idx] = in_imag[i];
		//printf("Initialized out_real at index %d\n", i);
		symbols_allocated++;
		if (symbols_allocated == symbols_to_allocate) {
			curr_set = (curr_set + 1) % d_num_occupied_carriers; //.size();
			symbols_to_allocate = d_size_occupied_carriers; //[curr_set]; //.size();
			symbols_allocated = 0;
		}
		//printf("Updated symbols_allocated\n");
	}

	//printf("\nCopy pilot symbols: n_ofdm_symbols = %lu\n", n_ofdm_symbols);
	// Copy pilot symbols
	for (int i = 0; i < n_ofdm_symbols; i++) {
		for (unsigned k = 0; k < d_size_pilot_carriers[i % d_num_pilot_carriers]; k++) {
			int pcidx = i % d_num_pilot_carriers;
			int psidx = i % d_num_pilot_symbols;
			//int oidx = o_offset + i * d_fft_len + d_pilot_carriers[i % d_num_pilot_carriers][k];
			int oidx = o_offset + i * d_fft_len + d_pilot_carriers[(pcidx * d_num_pilot_carriers) + k];
			out_real[oidx] = 0; //d_pilot_symbols_real[psidx][k];
			out_imag[oidx] = 0; //d_pilot_symbols_imag[psidx][k];
		}
	}
	//printf("\nReturning %lu (symbols + sync_words)\n", n_ofdm_symbols + d_num_sync_words);
	return n_ofdm_symbols + d_num_sync_words;
}

static void
do_ofdm_cyclic_prefixer_impl_work(int n_symbols, const float * in_real, const float * in_imag, float * out_real, float * out_imag) {
	int d_input_size = 64;
	int d_output_size = d_input_size + d_cp_size;
	float d_up_flank[d_rolloff_len - 1];
	float d_down_flank[d_rolloff_len - 1];

	float d_delay_line_real[d_rolloff_len - 1];
	float d_delay_line_imag[d_rolloff_len - 1];

	int symbols_to_read = n_symbols; // std::min(noutput_items / (int) d_output_size, ninput_items[0]);

	// The actual flanks are one sample shorter than d_rolloff_len, because the
	// first sample of the up- and down flank is always zero and one, respectively
	for (int i = 1; i < d_rolloff_len; i++) {
		d_up_flank[i - 1] = 0.5 * (1 + cos(M_PI * i / d_rolloff_len - M_PI));
		d_down_flank[i - 1] = 0.5 * (1 + cos(M_PI * (d_rolloff_len - i) / d_rolloff_len - M_PI));
		d_delay_line_real[i - 1] = 0; // Initialize delay line to zero
		d_delay_line_imag[i - 1] = 0;
	}

	// 2) Do the cyclic prefixing and, optionally, the pulse shaping
	int out_offset = 0;
	int in_offset = 0;
	for (int sym_idx = 0; sym_idx < symbols_to_read; sym_idx++) {
		//memcpy((void *)(out + d_cp_size), (void *) in, d_fft_len * sizeof(gr_complex));
		for (int i = 0; i < d_fft_len; i++) {
			out_real[out_offset + i + d_cp_size] = in_real[in_offset + i];
			out_imag[out_offset + i + d_cp_size] = in_imag[in_offset + i];
		}
		//memcpy((void *) out, (void *) (in + d_fft_len - d_cp_size), d_cp_size * sizeof(gr_complex));
		for (int i = 0; i < d_cp_size; i++) {
			out_real[out_offset + i] = in_real[in_offset + i + d_fft_len - d_cp_size];
			out_imag[out_offset + i] = in_imag[in_offset + i + d_fft_len - d_cp_size];
		}
		for (int i = 0; i < d_rolloff_len - 1; i++) {
			out_real[out_offset + i] = out_real[out_offset + i] * d_up_flank[i] + d_delay_line_real[i];
			out_imag[out_offset + i] = out_imag[out_offset + i] * d_up_flank[i] + d_delay_line_imag[i];
			d_delay_line_real[i] = in_real[in_offset + i] * d_down_flank[i];
			d_delay_line_imag[i] = in_imag[in_offset + i] * d_down_flank[i];
		}
		in_offset += d_fft_len;
		out_offset += d_output_size;
	}

	// 3) If we're in packet mode: (we are)
	//    - flush the delay line, if applicable
	for (unsigned i = 0; i < (d_rolloff_len - 1); i++) {
		out_real[out_offset + i] = d_delay_line_real[i];
		out_imag[out_offset + i] = d_delay_line_imag[i];
	}
}


/***********************************************************************************
	void encode_transmit_occgrid(int *n_cmp_bytes, size_t n_cmp_bytes_sz,
	unsigned char *cmp_data, size_t cmp_data_sz,
	int n_xmit_out, float *xmit_out_real, size_t xmit_out_real_sz,
	float *xmit_out_imag, size_t xmit_out_imag_sz,
	int psdu_len, size_t psdu_len_sz,
	uint8_t *pckt_hdr_out, size_t pckt_hdr_out_sz,
	int pckt_hdr_len, size_t pckt_hdr_len_sz,
	float *msg_stream_real, size_t msg_stream_real_sz,
	float *msg_stream_imag, size_t msg_stream_imag_sz,
	float *ofdm_car_str_real, size_t ofdm_car_str_real_sz,
	float *ofdm_car_str_imag, size_t ofdm_car_str_imag_sz,
	int ofc_res, size_t ofc_res_sz,
	float *fft_out_real, size_t fft_out_real_sz,
	float *fft_out_imag, size_t fft_out_imag_sz,
	float *cycpref_out_real, size_t cycpref_out_real_sz,
	float *cycpref_out_imag, size_t cycpref_out_imag_sz)
	{
// This section has no tasks as we are doing IO; introducing tasks can lead to race conditions
// Now we encode and transmit the grid...
DBGOUT(printf("Calling do_xmit_pipeline for %u compressed grid elements\n", n_cmp_bytes));

#ifdef INT_TIME
gettimeofday(&start_pd_wifi_pipe, NULL);
#endif

#if defined(HPVM) && true
void *LaunchInner = __hetero_launch((void *)do_xmit_pipeline, 17,
n_cmp_bytes, n_cmp_bytes_sz, (char *)cmp_data, cmp_data_sz,
&n_xmit_out, sizeof(int), xmit_out_real, xmit_out_real_sz,
xmit_out_imag, xmit_out_imag_sz,
// Start of local variables for do_xmit_pipeline
&psdu_len, psdu_len_sz, pckt_hdr_out, pckt_hdr_out_sz,
&pckt_hdr_len, pckt_hdr_len_sz,
msg_stream_real, msg_stream_real_sz,
msg_stream_imag, msg_stream_imag_sz,
ofdm_car_str_real, ofdm_car_str_real_sz,
ofdm_car_str_imag, ofdm_car_str_imag_sz, &ofc_res, ofc_res_sz,
fft_out_real, fft_out_real_sz, fft_out_imag, fft_out_imag_sz,
cycpref_out_real, cycpref_out_real_sz,
cycpref_out_imag, cycpref_out_imag_sz,
// End of local variables for do_xmit_pipeline
3,
&n_xmit_out, sizeof(int), xmit_out_real, xmit_out_real_sz,
xmit_out_imag, xmit_out_imag_sz);
__hetero_wait(LaunchInner);
#else
do_xmit_pipeline(n_cmp_bytes, n_cmp_bytes_sz, (char *)cmp_data, cmp_data_sz,
&n_xmit_out, sizeof(int), xmit_out_real, xmit_out_real_sz,
xmit_out_imag, xmit_out_imag_sz,
// Start of local variables for do_xmit_pipeline
&psdu_len, psdu_len_sz, pckt_hdr_out, pckt_hdr_out_sz,
&pckt_hdr_len, pckt_hdr_len_sz,
msg_stream_real, msg_stream_real_sz,
msg_stream_imag, msg_stream_imag_sz,
ofdm_car_str_real, ofdm_car_str_real_sz,
ofdm_car_str_imag, ofdm_car_str_imag_sz, &ofc_res, ofc_res_sz,
fft_out_real, fft_out_real_sz, fft_out_imag, fft_out_imag_sz,
cycpref_out_real, cycpref_out_real_sz,
cycpref_out_imag, cycpref_out_imag_sz);
#endif

transmit_occgrid(n_cmp_bytes, n_cmp_bytes_sz,
cmp_data, cmp_data_sz,
 *n_xmit_out,
 xmit_out_real, xmit_out_real_sz,
 xmit_out_imag, xmit_out_imag_sz,
 psdu_len, psdu_len_sz,
 pckt_hdr_out, pckt_hdr_out_sz,
 pckt_hdr_len, pckt_hdr_len_sz,
 msg_stream_real, msg_stream_real_sz,
msg_stream_imag, msg_stream_imag_sz,
	ofdm_car_str_real, ofdm_car_str_real_sz,
	ofdm_car_str_imag, ofdm_car_str_imag_sz,
	ofc_res, ofc_res_sz,
	fft_out_real, fft_out_real_sz,
	fft_out_imag, fft_out_imag_sz,
	cycpref_out_real, cycpref_out_real_sz,
	cycpref_out_imag, cycpref_out_imag_sz);

// #ifdef INT_TIME
//                                                              gettimeofday( & stop_pd_wifi_pipe, NULL);
//                                                              pd_wifi_pipe_sec += stop_pd_wifi_pipe.tv_sec - start_pd_wifi_pipe.tv_sec;
//                                                              pd_wifi_pipe_usec += stop_pd_wifi_pipe.tv_usec - start_pd_wifi_pipe.tv_usec;
// #endif
//                                                              DBGOUT(printf("  Back from do_xmit_pipeline with %u xmit outputs...\n", *n_xmit_out));

//                                                              // This is now the content that should be sent out via IEEE 802.11p WiFi
//                                                              // The n_xmit_out values of xmit_out_real and xmit_out_imag
//                                                              // Connect to the Wifi-Socket and send the n_xmit_out
//                                                              char w_buffer[10];
// #ifdef INT_TIME
//                                                              gettimeofday( & start_pd_wifi_send, NULL);
// #endif
//                                                              unsigned xfer_bytes = (n_xmit_out) * sizeof(float);
//                                                              snprintf(w_buffer, 9, "X%-6uX", xfer_bytes);
//                                                              DBGOUT(printf("\nXMIT Sending %s on XMIT port %u socket\n", w_buffer, XMIT_PORT));
//                                                              send(xmit_sock, w_buffer, 8, 0);
//                                                              DBGOUT(printf("     Send %u REAL values %u bytes on XMIT port %u socket\n", n_xmit_out, xfer_bytes,
//                                                                                                                              XMIT_PORT));
//                                                              DBGOUT2(printf("XFER %4u : Dumping XMIT-PIPE REAL raw bytes\n", xmit_count);
//                                                                                                                              for (int i = 0; i < n_xmit_out; i++) {
//                                                                                                                              printf("XFER %4u REAL-byte %6u : %f\n", xmit_count, i, xmit_out_real[i]);
//                                                                                                                              }
//                                                                                                                              printf("\n"));
// #ifdef INT_TIME
//                                                              gettimeofday( & start_pd_wifi_send_rl, NULL);
// #endif
//                                                              send(xmit_sock, (char * )(xmit_out_real), (n_xmit_out) * sizeof(float), 0);
//                                                              DBGOUT(printf("     Send %u IMAG values %u bytes on XMIT port %u socket\n", n_xmit_out, xfer_bytes,
//                                                                                                                              XMIT_PORT));
//                                                              DBGOUT2(printf("XFER %4u : Dumping XMIT-PIPE IMAG raw bytes\n", xmit_count);
//                                                                                                                              for (int i = 0; i < (n_xmit_out); i++) {
//                                                                                                                              printf("XFER %4u IMAG-byte %6u : %f\n", xmit_count, i, xmit_out_imag[i]);
//                                                                                                                              }
//                                                                                                                              printf("\n"));
// #ifdef INT_TIME
//                                                              gettimeofday( & stop_pd_wifi_send_rl, NULL);
// #endif
//                                                              send(xmit_sock, (char * )(xmit_out_imag), (n_xmit_out) * sizeof(float), 0);
// #ifdef INT_TIME
//                                                              gettimeofday( & stop_pd_wifi_send, NULL);
//                                                              pd_wifi_send_sec += stop_pd_wifi_send.tv_sec - start_pd_wifi_send.tv_sec;
//                                                              pd_wifi_send_usec += stop_pd_wifi_send.tv_usec - start_pd_wifi_send.tv_usec;
//                                                              pd_wifi_send_rl_sec += stop_pd_wifi_send_rl.tv_sec - start_pd_wifi_send_rl.tv_sec;
//                                                              pd_wifi_send_rl_usec += stop_pd_wifi_send_rl.tv_usec - start_pd_wifi_send_rl.tv_usec;
//                                                              pd_wifi_send_im_sec += stop_pd_wifi_send.tv_sec - stop_pd_wifi_send_rl.tv_sec;
//                                                              pd_wifi_send_im_usec += stop_pd_wifi_send.tv_usec - stop_pd_wifi_send_rl.tv_usec;
// #endif

//                                                              xmit_count++;
}
***********************************************************************************/

void lidar_root(lidar_inputs_t * lidar_inputs, size_t lidarin_sz /*=sizeof( * lidar_inputs)*/,
	Observation * observationVal /* observations[*next_obs_cp] -> from global array*/, size_t observations_sz /*=sizeof(Observation)*2*/,
	int * n_cmp_bytes /*return by arg*/, size_t n_cmp_bytes_sz /*=sizeof(int)*1*/,
	unsigned char * cmp_data /*return by arg*/, size_t cmp_data_sz /*=MAX_COMPRESSED_DATA_SIZE*/,
	// Start of global variables used internally by function
	int * curr_obs_cp /*=curr_obs -> global*/, size_t curr_obs_cp_sz /*=sizeof(int)*/,
	int * next_obs_cp /*=next_obs -> global*/, size_t next_obs_cp_sz /*=sizeof(int)*/,
	int * lmap_count_cp /*=lmap_count -> global*/, size_t lmap_count_cp_sz /*=sizeof(unsigned)*/,
	// End of global variables used internally by function
	// Start of arguments to cloudToOccgrid (called indirectly by lidar_root)
	double * AVxyzw, size_t AVxyzw_sz /*=sizeof(double)*/,
	bool * rolling_window, size_t rolling_window_sz /*=sizeof(bool)*/,
	double * min_obstacle_height, size_t min_obstacle_height_sz /*=sizeof(double)*/,
	double * max_obstacle_height, size_t max_obstacle_height_sz /*=sizeof(double)*/,
	double * raytrace_range, size_t raytrace_range_sz /*=sizeof(double)*/,
	unsigned int * size_x, size_t size_x_sz /*=sizeof(unsigned int)*/,
	unsigned int * size_y, size_t size_y_sz /*=sizeof(unsigned int)*/,
	unsigned int * resolution, size_t resolution_sz /*=sizeof(unsigned int)*/,
	int * timer_sequentialize, size_t timer_sequentialize_sz /*=sizeof(int) */,
	// End of arguments to cloudToOccgrid (called indirectly by lidar_root)
	// Start of arguments to encode_occgrid (called indirectly by lidar_root)
	int * n_xmit_out, size_t n_xmit_out_sz,
	float * xmit_out_real, size_t xmit_out_real_sz,
	float * xmit_out_imag, size_t xmit_out_imag_sz,
	int * psdu_len, size_t psdu_len_sz,
	uint8_t * pckt_hdr_out, size_t pckt_hdr_out_sz,
	int * pckt_hdr_len, size_t pckt_hdr_len_sz,
	float * msg_stream_real, size_t msg_stream_real_sz,
	float * msg_stream_imag, size_t msg_stream_imag_sz,
	float * ofdm_car_str_real, size_t ofdm_car_str_real_sz,
	float * ofdm_car_str_imag, size_t ofdm_car_str_imag_sz,
	int * ofc_res, size_t ofc_res_sz,
	float * fft_out_real, size_t fft_out_real_sz,
	float * fft_out_imag, size_t fft_out_imag_sz,
	float * cycpref_out_real, size_t cycpref_out_real_sz,
	float * cycpref_out_imag, size_t cycpref_out_imag_sz,
	int* d_occupied_carriers, size_t d_occupied_carriers_sz,
	// the following are inputs and outputs because they are used to maintain state in the program
	uint8_t * d_psdu_arg, size_t d_psdu_arg_sz,
	uint8_t * d_map_out_copy_arg, size_t d_map_out_copy_arg_sz,
	uint16_t * d_seq_nr, size_t d_seq_nr_sz/*=sizeof(uint16_t)*/,
        uint8_t * d_scrambler, size_t d_scrambler_sz,
        char* d_symbols, size_t d_symbols_sz,
        int* d_symbols_offset, size_t d_symbols_offset_sz,
        int* d_symbols_len, size_t d_symbols_len_sz,
	ofdm_param * d_ofdm, size_t d_ofdm_sz,
        frame_param * d_frame, size_t d_frame_sz,
	int* d_pilot_carriers, size_t d_pilot_carriers_sz,
	crc* crcTable, size_t crcTable_sz
	// End of arguments to encode_occgrid (called indirectly by lidar_root)
) {
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR)) && true
	void * Section = __hetero_section_begin();
#endif

#if !defined(COLLAPSE_NODES)
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR)) && true
	void * T1 = __hetero_task_begin(16, lidar_inputs, lidarin_sz, n_cmp_bytes, n_cmp_bytes_sz,
		cmp_data, cmp_data_sz, observationVal, observations_sz, timer_sequentialize, timer_sequentialize_sz,
		// Args for cloudToOccgrid
		AVxyzw, AVxyzw_sz,
		rolling_window, rolling_window_sz, min_obstacle_height, min_obstacle_height_sz,
		max_obstacle_height, max_obstacle_height_sz, raytrace_range, raytrace_range_sz,
		size_x, size_x_sz, size_y, size_y_sz, resolution, resolution_sz,
		// Global vars used by process_lidar_to_occgrid
		curr_obs_cp, curr_obs_cp_sz, next_obs_cp, next_obs_cp_sz, 
		lmap_count_cp, lmap_count_cp_sz,
		// Output
		3, observationVal, observations_sz, n_cmp_bytes, n_cmp_bytes_sz, cmp_data, cmp_data_sz,
		"proccess_lidar_to_occgrid_caller_task");
//	__hpvm__hint(DEVICE);
#endif

	process_lidar_to_occgrid(lidar_inputs, lidarin_sz, observationVal, observations_sz,
		n_cmp_bytes, n_cmp_bytes_sz, cmp_data, cmp_data_sz,
		// Global vars used by process_lidar_to_occgrid
		curr_obs_cp, curr_obs_cp_sz, next_obs_cp, next_obs_cp_sz, 
		lmap_count_cp, lmap_count_cp_sz,
		// Args for cloudToOccgrid
		AVxyzw, AVxyzw_sz, rolling_window, rolling_window_sz, min_obstacle_height, min_obstacle_height_sz,
		max_obstacle_height, max_obstacle_height_sz, raytrace_range, raytrace_range_sz,
		size_x, size_x_sz, size_y, size_y_sz, resolution, resolution_sz,
		timer_sequentialize, timer_sequentialize_sz); // buffer, total_bytes_read);
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR)) && true
	__hetero_task_end(T1);
#endif
#else
	void *T_CPU_Wrapper = __hetero_task_begin(
		43,
		lidar_inputs, lidarin_sz,
		n_cmp_bytes, n_cmp_bytes_sz,
		cmp_data, cmp_data_sz,
		observationVal, observations_sz, 
		timer_sequentialize, timer_sequentialize_sz,
		AVxyzw, AVxyzw_sz,
		rolling_window, rolling_window_sz, 
		min_obstacle_height, min_obstacle_height_sz,
		max_obstacle_height, max_obstacle_height_sz, 
		raytrace_range, raytrace_range_sz,
		size_x, size_x_sz, 
		size_y, size_y_sz, 
		resolution, resolution_sz,
		curr_obs_cp, curr_obs_cp_sz, 
		next_obs_cp, next_obs_cp_sz, 
		lmap_count_cp, lmap_count_cp_sz,
		n_xmit_out, n_xmit_out_sz,
		xmit_out_real, xmit_out_real_sz,
		xmit_out_imag, xmit_out_imag_sz,
		psdu_len, psdu_len_sz,
		pckt_hdr_out, pckt_hdr_out_sz,
		pckt_hdr_len, pckt_hdr_len_sz,
		msg_stream_real, msg_stream_real_sz,
		msg_stream_imag, msg_stream_imag_sz,
		ofdm_car_str_real, ofdm_car_str_real_sz,
		ofdm_car_str_imag, ofdm_car_str_imag_sz,
		ofc_res, ofc_res_sz,
		fft_out_real, fft_out_real_sz,
		fft_out_imag, fft_out_imag_sz,
		cycpref_out_real, cycpref_out_real_sz,
		cycpref_out_imag, cycpref_out_imag_sz,
		d_occupied_carriers, d_occupied_carriers_sz,
		d_psdu_arg, d_psdu_arg_sz,
		d_map_out_copy_arg, d_map_out_copy_arg_sz,
		d_seq_nr, d_seq_nr_sz, 
		d_scrambler, d_scrambler_sz, 
		d_symbols, d_symbols_sz, 
		d_symbols_offset, d_symbols_offset_sz, 
		d_symbols_len, d_symbols_len_sz,
		d_ofdm, d_ofdm_sz,
		d_frame, d_frame_sz,
		d_pilot_carriers, d_pilot_carriers_sz,
		crcTable, crcTable_sz,
		17, 
		observationVal, observations_sz,
		n_cmp_bytes, n_cmp_bytes_sz, 
		cmp_data, cmp_data_sz,
		n_xmit_out, n_xmit_out_sz,
		xmit_out_real, xmit_out_real_sz,
		xmit_out_imag, xmit_out_imag_sz,
		d_psdu_arg, d_psdu_arg_sz,
		d_map_out_copy_arg, d_map_out_copy_arg_sz,
		d_seq_nr, d_seq_nr_sz, 
		d_scrambler, d_scrambler_sz, 
		d_symbols, d_symbols_sz, 
		d_symbols_offset, d_symbols_offset_sz, 
		d_symbols_len, d_symbols_len_sz,
		d_ofdm, d_ofdm_sz,
		d_frame, d_frame_sz,
		d_pilot_carriers, d_pilot_carriers_sz,
		crcTable, crcTable_sz, 
		"wrapper_cpu_task");
	void * Section_Main = __hetero_section_begin();

	// CloudToOccgrid
	{

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		void * T1_cloudToOccgrid_Task = __hetero_task_begin(10, observationVal, observations_sz, lidar_inputs, lidarin_sz,
			rolling_window, rolling_window_sz, min_obstacle_height, min_obstacle_height_sz,
			max_obstacle_height, max_obstacle_height_sz, raytrace_range, raytrace_range_sz,
			size_x, size_x_sz, size_y, size_y_sz, resolution, resolution_sz,
			timer_sequentialize, timer_sequentialize_sz,
			2, observationVal, observations_sz, timer_sequentialize, timer_sequentialize_sz,
			"initCostmap_task");
	__hpvm__hint(DEVICE); 
#endif
		{
			*timer_sequentialize = 1;
#ifdef INT_TIME
			// gettimeofday(&ocgr_c2g_total_start, NULL); // See note above this function for why this call was commented out
			gettimeofday(&ocgr_c2g_initCM_start, NULL);
#endif
			double robot_x = lidar_inputs->odometry[0];
			double robot_y = lidar_inputs->odometry[1];
			double robot_z = lidar_inputs->odometry[2];

			initCostmap(observationVal, *rolling_window, *min_obstacle_height, *max_obstacle_height, *raytrace_range,
				*size_x, *size_y, *resolution, robot_x, robot_y, robot_z);
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_initCM_stop, NULL);
			ocgr_c2g_initCM_sec += ocgr_c2g_initCM_stop.tv_sec - ocgr_c2g_initCM_start.tv_sec;
			ocgr_c2g_initCM_usec += ocgr_c2g_initCM_stop.tv_usec - ocgr_c2g_initCM_start.tv_usec;
#endif

		}
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		__hetero_task_end(T1_cloudToOccgrid_Task);
#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		void * T2_cloudToOccgrid_Task = __hetero_task_begin(2, observationVal, observations_sz, lidar_inputs, lidarin_sz,
			1, observationVal, observations_sz, "updateOrigin_task");
	__hpvm__hint(DEVICE);
#endif

		{
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_updOrig_start, NULL);
#endif

			double robot_x = lidar_inputs->odometry[0];
			double robot_y = lidar_inputs->odometry[1];
			double robot_z = lidar_inputs->odometry[2];
			//printf("(1) Number of elements : %d ... ", data_size);
			//printf("First Coordinate = <%f, %f>\n", *data, *(data+1));
			//MOVED to physically inlined here... updateMap(obs_ptr, data, data_size, robot_x, robot_y, robot_z, robot_yaw);
			if (observationVal->rolling_window) {
				//printf("\nUpdating Map .... \n");
				//printf("   robot_x = %f, robot_y = %f, robot_yaw = %f \n", robot_x, robot_y, AVxyzw);
				//printf("   Master Origin = (%f, %f)\n", obs_ptr->master_origin.x, obs_ptr->master_origin.y);
				double new_origin_x = robot_x - observationVal->master_costmap.x_dim / 2;
				double new_origin_y = robot_y - observationVal->master_costmap.y_dim / 2;
				updateOrigin(observationVal, new_origin_x, new_origin_y);
			}
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_updOrig_stop, NULL);
			ocgr_c2g_updOrig_sec += ocgr_c2g_updOrig_stop.tv_sec - ocgr_c2g_updOrig_start.tv_sec;
			ocgr_c2g_updOrig_usec += ocgr_c2g_updOrig_stop.tv_usec - ocgr_c2g_updOrig_start.tv_usec;
#endif
		}
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		__hetero_task_end(T2_cloudToOccgrid_Task);
#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		void * T3_cloudToOccgrid_Task = __hetero_task_begin(3, observationVal, observations_sz, lidar_inputs, lidarin_sz,
			AVxyzw, AVxyzw_sz, 1, observationVal, observations_sz, "updateBounds_task");
	__hpvm__hint(DEVICE);
#endif
		{
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_updBnds_start, NULL);
#endif
			double robot_x = lidar_inputs->odometry[0];
			double robot_y = lidar_inputs->odometry[1];
			double robot_z = lidar_inputs->odometry[2];

			float * data = (float *) (lidar_inputs->data);
			unsigned int data_size = lidar_inputs->data_size / sizeof(float);

			double min_x = 1e30;
			double min_y = 1e30;
			double max_x = -1e30;
			double max_y = -1e30;

			//printf("(1) Number of elements : %d ... ", data_size);
			//printf("First Coordinate = <%f, %f>\n", *data, *(data+1));
			//rotating_window = true; //Comment out if not rolling window

			updateBounds(observationVal, data, data_size, robot_x, robot_y, robot_z,
				*AVxyzw, &min_x, &min_y, &max_x, &max_y);

			//printMap();
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_updBnds_stop, NULL);
			ocgr_c2g_updBnds_sec += ocgr_c2g_updBnds_stop.tv_sec - ocgr_c2g_updBnds_start.tv_sec;
			ocgr_c2g_updBnds_usec += ocgr_c2g_updBnds_stop.tv_usec - ocgr_c2g_updBnds_start.tv_usec;

			// Note (located above this function) explains why the following lines were commented out
			// ocgr_c2g_total_sec  += ocgr_c2g_updBnds_stop.tv_sec  - ocgr_c2g_total_start.tv_sec;
			// ocgr_c2g_total_usec += ocgr_c2g_updBnds_stop.tv_usec - ocgr_c2g_total_start.tv_usec;
#endif
		}
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		__hetero_task_end(T3_cloudToOccgrid_Task);
#endif
#if defined(INT_TIME) && !(defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL))
		gettimeofday(&ocgr_c2g_total_stop, NULL);
		ocgr_c2g_total_sec += ocgr_c2g_total_stop.tv_sec - ocgr_c2g_total_start.tv_sec;
		ocgr_c2g_total_usec += ocgr_c2g_total_stop.tv_usec - ocgr_c2g_total_start.tv_usec;
#endif


	}


	// Write the read-in image to a file
	// write_array_to_file(grid, COST_MAP_ENTRIES);
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	void * T2_process_lidar = __hetero_task_begin(6, observationVal, observations_sz, n_cmp_bytes, n_cmp_bytes_sz,
		cmp_data, cmp_data_sz, next_obs_cp, next_obs_cp_sz, curr_obs_cp, curr_obs_cp_sz,
		lmap_count_cp, lmap_count_cp_sz,
		3, observationVal, observations_sz, n_cmp_bytes, n_cmp_bytes_sz, cmp_data, cmp_data_sz,
		"compressMap_Task");

	__hpvm__hint(CPU_TARGET);
	// This task is not being placed on the fpga as it does weird pointer arithemetic which causes issues with hpvm
#endif

	printf("%s %d In T2", __FILE__, __LINE__);

	// Now we compress the grid for transmission...
	Costmap2D * local_map = &(observationVal->master_costmap);
	// Now we update the current observation index and the next observation index
	//      Switch curr_obs (global referred to by curr_obs_cp) and next_obs (global referred to by next_obs_cp)
	//      between 0 and 1
	* curr_obs_cp = 1 - *curr_obs_cp;
	*next_obs_cp = 1 - *next_obs_cp;
	(*lmap_count_cp)++;
	// And now we compress to encode for Wifi transmission, etc.
#ifdef INT_TIME
	gettimeofday(&start_pd_lz4_cmp, NULL);
#endif
	* n_cmp_bytes = LZ4_compress_default((char *) local_map, (char *) cmp_data,
		MAX_UNCOMPRESSED_DATA_SIZE, MAX_COMPRESSED_DATA_SIZE);

#ifdef INT_TIME
	gettimeofday(&stop_pd_lz4_cmp, NULL);
	pd_lz4_cmp_sec += stop_pd_lz4_cmp.tv_sec - start_pd_lz4_cmp.tv_sec;
	pd_lz4_cmp_usec += stop_pd_lz4_cmp.tv_usec - start_pd_lz4_cmp.tv_usec;
#endif


#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	__hetero_task_end(T2_process_lidar);
#endif
#endif


#if !defined(COLLAPSE_NODES)
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR)) && true
	void * T2 = __hetero_task_begin(29,
		// Args for encode_occgrid
		n_cmp_bytes, n_cmp_bytes_sz,
		cmp_data, cmp_data_sz,
		n_xmit_out, n_xmit_out_sz,
		xmit_out_real, xmit_out_real_sz,
		xmit_out_imag, xmit_out_imag_sz,
		// Start of local variables for do_xmit_pipeline
		psdu_len, psdu_len_sz,
		pckt_hdr_out, pckt_hdr_out_sz,
		pckt_hdr_len, pckt_hdr_len_sz,
		msg_stream_real, msg_stream_real_sz,
		msg_stream_imag, msg_stream_imag_sz,
		ofdm_car_str_real, ofdm_car_str_real_sz,
		ofdm_car_str_imag, ofdm_car_str_imag_sz,
		ofc_res, ofc_res_sz,
		fft_out_real, fft_out_real_sz,
		fft_out_imag, fft_out_imag_sz,
		cycpref_out_real, cycpref_out_real_sz,
		cycpref_out_imag, cycpref_out_imag_sz,
		d_occupied_carriers, d_occupied_carriers_sz,
		d_psdu_arg, d_psdu_arg_sz,
		d_map_out_copy_arg, d_map_out_copy_arg_sz,
		d_seq_nr, d_seq_nr_sz, 
		d_scrambler, d_scrambler_sz, 
		d_symbols, d_symbols_sz, 
		d_symbols_offset, d_symbols_offset_sz, 
		d_symbols_len, d_symbols_len_sz,
		d_ofdm, d_ofdm_sz,
		d_frame, d_frame_sz,
		d_pilot_carriers, d_pilot_carriers_sz,
		crcTable, crcTable_sz,
		// End of local variables for do_xmit_pipeline
		14, n_xmit_out, n_xmit_out_sz,
		xmit_out_real, xmit_out_real_sz,
		xmit_out_imag, xmit_out_imag_sz,
		d_psdu_arg, d_psdu_arg_sz,
		d_map_out_copy_arg, d_map_out_copy_arg_sz,
		d_seq_nr, d_seq_nr_sz, 
		d_scrambler, d_scrambler_sz, 
		d_symbols, d_symbols_sz, 
		d_symbols_offset, d_symbols_offset_sz, 
		d_symbols_len, d_symbols_len_sz,
		d_ofdm, d_ofdm_sz,
		d_frame, d_frame_sz,
		d_pilot_carriers, d_pilot_carriers_sz,
		crcTable, crcTable_sz,
		"TX_task");
//	__hpvm__hint(DEVICE);
#endif

	do_xmit_pipeline(n_cmp_bytes, n_cmp_bytes_sz, (char*)cmp_data, cmp_data_sz,
		n_xmit_out, n_xmit_out_sz, xmit_out_real, xmit_out_real_sz,
		xmit_out_imag, xmit_out_imag_sz,
		// Start of local variables for do_xmit_pipeline
		psdu_len, psdu_len_sz, pckt_hdr_out, pckt_hdr_out_sz,
		pckt_hdr_len, pckt_hdr_len_sz,
		msg_stream_real, msg_stream_real_sz,
		msg_stream_imag, msg_stream_imag_sz,
		ofdm_car_str_real, ofdm_car_str_real_sz,
		ofdm_car_str_imag, ofdm_car_str_imag_sz, ofc_res, ofc_res_sz,
		fft_out_real, fft_out_real_sz, fft_out_imag, fft_out_imag_sz,
		cycpref_out_real, cycpref_out_real_sz,
		cycpref_out_imag, cycpref_out_imag_sz,
		d_occupied_carriers, d_occupied_carriers_sz,
		d_psdu_arg, d_psdu_arg_sz,
		d_map_out_copy_arg, d_map_out_copy_arg_sz,
		d_seq_nr, d_seq_nr_sz, 
		d_scrambler, d_scrambler_sz, 
		d_symbols, d_symbols_sz, 
		d_symbols_offset, d_symbols_offset_sz, 
		d_symbols_len, d_symbols_len_sz,
		d_ofdm, d_ofdm_sz,
		d_frame, d_frame_sz,
		d_pilot_carriers, d_pilot_carriers_sz,
		crcTable, crcTable_sz
	);

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR)) && true
	__hetero_task_end(T2);
#endif
#else
// do_xmit_pipeline inling for flattening
// rename arguments for inlined code
	int *in_msg_len = n_cmp_bytes;
	size_t in_msg_len_sz = n_cmp_bytes_sz;
	char *in_msg = (char*) cmp_data;
	size_t in_msg_sz = cmp_data_sz;
	int *num_final_outs = n_xmit_out;
        size_t num_final_outs_sz = n_xmit_out_sz;
	float *final_out_real = xmit_out_real;
	size_t final_out_real_sz = xmit_out_real_sz;
	float *final_out_imag = xmit_out_imag;
	size_t final_out_imag_sz = xmit_out_imag_sz;
	uint8_t *d_psdu = d_psdu_arg;
	size_t d_psdu_size = d_psdu_arg_sz;
	uint8_t *d_map_out = d_map_out_copy_arg;
	size_t d_map_out_sz = d_map_out_copy_arg_sz;

// do_xmit_pipeline definition
#if defined(HPVM)
									void * T1 = __hetero_task_begin(6, in_msg_len, in_msg_len_sz, in_msg, in_msg_sz, psdu_len, psdu_len_sz,
											d_psdu, d_psdu_size, 
											d_seq_nr, d_seq_nr_sz,
											crcTable, crcTable_sz,
											4, psdu_len, psdu_len_sz,  d_psdu, d_psdu_size, 
											d_seq_nr, d_seq_nr_sz,
											crcTable, crcTable_sz,
											"mac_data_task_wrapper");
						__hpvm__hint(DEVICE);
#endif
									            * psdu_len = 0;
            generate_mac_data_frame(in_msg, *in_msg_len, psdu_len, d_psdu, d_psdu_size, d_seq_nr, d_seq_nr_sz,
                            crcTable, crcTable_sz);
#ifdef INT_TIME
            gettimeofday(&x_genmacfr_stop, NULL);
            x_genmacfr_sec += x_genmacfr_stop.tv_sec - x_genmacfr_start.tv_sec;
            x_genmacfr_usec += x_genmacfr_stop.tv_usec - x_genmacfr_start.tv_usec;
#endif
            //printf("Done with generate_mac_data_frame\n");

#if defined(HPVM)
									__hetero_task_end(T1);
#endif

#if defined(HPVM)
									void * T2 = __hetero_task_begin(11, pckt_hdr_out, pckt_hdr_out_sz, psdu_len, psdu_len_sz, pckt_hdr_len, pckt_hdr_len_sz,
											d_psdu, d_psdu_size, d_map_out, d_map_out_sz,	
											d_scrambler, d_scrambler_sz,
											d_symbols, d_symbols_sz,
											d_symbols_offset, d_symbols_offset_sz,
											d_symbols_len, d_symbols_len_sz,
											d_ofdm, d_ofdm_sz,
											d_frame, d_frame_sz,
											11, pckt_hdr_out, pckt_hdr_out_sz, psdu_len, psdu_len_sz, pckt_hdr_len, pckt_hdr_len_sz,
											d_scrambler, d_scrambler_sz,
											d_symbols, d_symbols_sz,
											d_symbols_offset, d_symbols_offset_sz,
											d_symbols_len, d_symbols_len_sz,
											d_ofdm, d_ofdm_sz,
											d_frame, d_frame_sz,
											d_psdu, d_psdu_size, d_map_out, d_map_out_sz, "mapper_task_wrapper");
#endif

#if defined(HPVM)
						__hpvm__hint(DEVICE);
#endif
#ifdef INT_TIME
            gettimeofday(&x_domapwk_start, NULL);
#endif

            // do_mapper_work(32768, psdu_len); // noutput always seems to be 32768 ? Actualy data size is 24528 ?
            do_mapper_work(*psdu_len, d_psdu, d_psdu_size, d_map_out, d_map_out_sz,
                d_scrambler, d_scrambler_sz,
                d_symbols, d_symbols_sz,
                d_symbols_offset, d_symbols_offset_sz,
                d_symbols_len, d_symbols_len_sz,
                d_ofdm, d_ofdm_sz,
                d_frame, d_frame_sz
                    ); // noutput always seems to be 32768 ? Actualy data size is 24528 ?
            // The mapper results in 24528 output bytes for a 1500 character input payload
#ifdef INT_TIME
            gettimeofday(&x_domapwk_stop, NULL);
            x_domapwk_sec += x_domapwk_stop.tv_sec - x_domapwk_start.tv_sec;
            x_domapwk_usec += x_domapwk_stop.tv_usec - x_domapwk_start.tv_usec;
#endif
            //printf("Done with do_mapper_work\n");

#if defined(HPVM)
									__hetero_task_end(T2);
#endif

#if defined(HPVM)
									void * T3 = __hetero_task_begin(4, pckt_hdr_out, pckt_hdr_out_sz, pckt_hdr_len, pckt_hdr_len_sz,
											d_ofdm, d_ofdm_sz,
											d_frame, d_frame_sz,
											4, pckt_hdr_out, pckt_hdr_out_sz, pckt_hdr_len, pckt_hdr_len_sz, 
											d_ofdm, d_ofdm_sz,
											d_frame, d_frame_sz,
											"packer_hdr_task_wrapper");
#endif
#if defined(HPVM)
						__hpvm__hint(CPU_TARGET); // TODO: Put on FPGA
#endif
#if defined(INT_TIME) && !defined(HPVM)
            gettimeofday(&x_phdrgen_start, NULL);
#endif

            int mapper_payload_size = d_frame->n_encoded_bits;
            // uint8_t pckt_hdr_out[64]; // I think this only needs to be 48 bytes...
            *pckt_hdr_len = do_packet_header_gen(mapper_payload_size, pckt_hdr_out, d_ofdm, d_ofdm_sz, d_frame, d_frame_sz);

#if defined(INT_TIME) && !defined(HPVM)
            gettimeofday(&x_phdrgen_stop, NULL);
            x_phdrgen_sec += x_phdrgen_stop.tv_sec - x_phdrgen_start.tv_sec;
            x_phdrgen_usec += x_phdrgen_stop.tv_usec - x_phdrgen_start.tv_usec;
#endif
	
            DO_NUM_IOS_ANALYSIS(printf("Called do_packet_header_gen: IN payload_size %u OUT packet_hdr_len %u\n",
                  mapper_payload_size, *pckt_hdr_len));
            DEBUG(printf("packet_header = ");
                for (int i = 0; i < *pckt_hdr_len; i++) {
                printf("%1x ", pckt_hdr_out[i]);
                }
                printf("\n"));
            //printf("Done with do_packet_header_gen\n");

#if defined(HPVM)
									__hetero_task_end(T3);
#endif

#if defined(HPVM)
									void * T4 = __hetero_task_begin(7, pckt_hdr_out, pckt_hdr_out_sz, pckt_hdr_len, pckt_hdr_len_sz,
											msg_stream_real, msg_stream_real_sz, msg_stream_imag, msg_stream_imag_sz,
											d_map_out, d_map_out_sz,
											d_ofdm, d_ofdm_sz,
											d_frame, d_frame_sz,
											5, msg_stream_real, msg_stream_real_sz, msg_stream_imag, msg_stream_imag_sz,
											d_map_out, d_map_out_sz,
											d_ofdm, d_ofdm_sz,
											d_frame, d_frame_sz,
											"chuck_strm_task_wrapper");
#endif

#if defined(HPVM)
							 __hpvm__hint(DEVICE);

#endif

						// Convert the header chunks to symbols (uses simple BPSK_1_2 map: 0 -> -1+0i and 1 -> +1+0i)
						// Convert the payload chunks to symbols (for now also using simple BPSK_1_2 map: 0 -> -1+0i and 1 -> +1+0i)
						// We will also do the Tagged Stream Mux functionality (concatenate the Payload after the Header)
						DEBUG(printf("\nConverting to chunks, and doing the tagged stream mux stuff...\n"));
#ifdef INT_TIME 
						gettimeofday(&x_ck2sym_start, NULL);
#endif

						// float msg_stream_real[MAX_SIZE];
						// float msg_stream_imag[MAX_SIZE];
						int msg_idx = 0;
						/*float bpsk_chunks2sym[2] = {
						  -1.0,
						  1.0
						  };*/
						// This is to clear any left-over storage locations...
						for (int i = 0; i < MAX_SIZE; i++) {
							msg_stream_real[i] = 0.0;
							msg_stream_imag[i] = 0.0;
						}
						int terminationCondition = (*pckt_hdr_len);
						for (int i = 0; i < terminationCondition; i++) {
							msg_stream_real[msg_idx] = bpsk_chunks2sym(pckt_hdr_out[i]);
							msg_stream_imag[msg_idx] = 0.0;
							msg_idx++;
						}
						//  printf("\n");
						int mapper_payload_size_cp = d_frame->n_encoded_bits; // HPVM: Copied from T3 (as d_frame was not modified by T3)
						for (int i = 0; i < mapper_payload_size_cp; i++) {
							msg_stream_real[msg_idx] = bpsk_chunks2sym(d_map_out[i]);
							msg_stream_imag[msg_idx] = 0.0;
							msg_idx++;
						}
						//  printf("\n");
						//printf("Done with chuck_strm\n");

#if defined(INT_TIME) && !defined(HPVM)
						gettimeofday(&x_ck2sym_stop, NULL);
						x_ck2sym_sec += x_ck2sym_stop.tv_sec - x_ck2sym_start.tv_sec;
						x_ck2sym_usec += x_ck2sym_stop.tv_usec - x_ck2sym_start.tv_usec;
#endif
						DEBUG(printf("\nTagged Stream Mux output:\n");
								for (int i = 0; i < ((*pckt_hdr_len) + mapper_payload_size_cp); i++) {
								printf(" TSM_OUT %5u : %4.1f %4.1f\n", i, msg_stream_real[i], msg_stream_imag[i]);
								});

#if false

									// Convert the header chunks to symbols (uses simple BPSK_1_2 map: 0 -> -1+0i and 1 -> +1+0i)
									// Convert the payload chunks to symbols (for now also using simple BPSK_1_2 map: 0 -> -1+0i and 1 -> +1+0i)
									// We will also do the Tagged Stream Mux functionality (concatenate the Payload after the Header)
									DEBUG(printf("\nConverting to chunks, and doing the tagged stream mux stuff...\n"));
#ifdef INT_TIME
									gettimeofday(&x_ck2sym_start, NULL);
#endif

									// float msg_stream_real[MAX_SIZE];
									// float msg_stream_imag[MAX_SIZE];
									//
									int msg_idx = 0;
									//	float bpsk_chunks2sym[2] = { -1.0, 1.0 };
									for (int i = 0; i < (*pckt_hdr_len); i++) {
										msg_stream_real[msg_idx] = bpsk_chunks2sym(pckt_hdr_out[i]);
										msg_stream_imag[msg_idx] = 0.0;
										//    DEBUG(printf("HDR: msg_stream[%2u] = %4.1f + %4.1f\n", msg_idx, msg_stream_real[msg_idx], msg_stream_imag[msg_idx]));
										msg_idx++;
									}
									//  printf("\n");
									int mapper_payload_size_cp = d_frame->n_encoded_bits; // HPVM: Copied from T3 (as d_frame was not modified by T3)
									for (int i = 0; i < mapper_payload_size_cp; i++) {
										msg_stream_real[msg_idx] = bpsk_chunks2sym(d_map_out[i]);
										msg_stream_imag[msg_idx] = 0.0;
										//    DEBUG(printf("PYLD: msg_stream[%4u] = %4.1f + %4.1f\n", msg_idx, msg_stream_real[msg_idx], msg_stream_imag[msg_idx]));
										msg_idx++;
									}
									//  printf("\n");
									// This is to clear any left-over storage locations...
									for (int i = msg_idx; i < MAX_SIZE; i++) {
										msg_stream_real[i] = 0.0;
										msg_stream_imag[i] = 0.0;
										//    DEBUG(printf("LAST: msg_stream[%4u] = %4.1f + %4.1f\n", i, msg_stream_real[i], msg_stream_imag[i]));
									}

#ifdef INT_TIME
									gettimeofday(&x_ck2sym_stop, NULL);
									x_ck2sym_sec += x_ck2sym_stop.tv_sec - x_ck2sym_start.tv_sec;
									x_ck2sym_usec += x_ck2sym_stop.tv_usec - x_ck2sym_start.tv_usec;
#endif
									DEBUG(printf("\nTagged Stream Mux output:\n");
											for (int i = 0; i < ((*pckt_hdr_len) + mapper_payload_size_cp); i++) {
											printf(" TSM_OUT %5u : %4.1f %4.1f\n", i, msg_stream_real[i], msg_stream_imag[i]);
											});

#endif // if false

#if defined(HPVM)
									__hetero_task_end(T4);
#endif

#if defined(HPVM)
									void * T5 = __hetero_task_begin(9, msg_stream_real, msg_stream_real_sz, msg_stream_imag, msg_stream_imag_sz,
											ofdm_car_str_real, ofdm_car_str_real_sz, ofdm_car_str_imag, ofdm_car_str_imag_sz,
											ofc_res, ofc_res_sz,
											d_ofdm, d_ofdm_sz, 
											d_frame, d_frame_sz,
											d_pilot_carriers, d_pilot_carriers_sz,
											d_occupied_carriers, d_occupied_carriers_sz,
											6, ofdm_car_str_real, ofdm_car_str_real_sz, ofdm_car_str_imag, ofdm_car_str_imag_sz,
											d_ofdm, d_ofdm_sz, 
											d_frame, d_frame_sz,
											d_pilot_carriers, d_pilot_carriers_sz,
											ofc_res, ofc_res_sz, "carrier_alloc_task_wrapper");
#endif

#if defined(HPVM)
						__hpvm__hint(DEVICE); 
#endif

						// DEBUG(printf("\nCalling do_ofdm_carrier_allocator_cvc_impl_work( %u, %u, msg_stream)\n", 520, 24576));
						//DEBUG(printf("\nCalling do_ofdm_carrier_allocator_cvc_impl_work( %u, %u, msg_stream)\n",
						//			d_frame->n_sym, d_frame->n_encoded_bits));

						// float ofdm_car_str_real[ofdm_max_out_size];
						// float ofdm_car_str_imag[ofdm_max_out_size];

						//DO_NUM_IOS_ANALYSIS(printf("Calling do_ofdm_carrier_alloc: IN n_sym %u n_enc_bits %u\n", d_frame->n_sym,
						//			d_frame->n_encoded_bits));
						// int ofc_res = do_ofdm_carrier_allocator_cvc_impl_work(520, 24576, msg_stream_real, msg_stream_imag, ofdm_car_str_real, ofdm_car_str_imag);
#ifdef INT_TIME
	//					gettimeofday(&x_ocaralloc_start, NULL);
#endif
						*ofc_res = do_ofdm_carrier_allocator_cvc_impl_work(d_frame->n_sym, d_frame->n_encoded_bits, 
											 msg_stream_real, msg_stream_imag, 	ofdm_car_str_real, ofdm_car_str_imag, 
											 d_pilot_carriers, d_pilot_carriers_sz, d_occupied_carriers, d_occupied_carriers_sz); 

						/*
						DO_NUM_IOS_ANALYSIS(printf("Back from do_ofdm_carrier_alloc: OUT ofc_res %u : %u max outputs (of %u)\n", 
														ofc_res, ofc_res * d_fft_len, ofdm_max_out_size));
						DEBUG(printf(" return value was %u so max %u outputs\n", *ofc_res, (*ofc_res) * d_fft_len); 
											 printf(" do_ofdm_carrier_allocator_cvc_impl_work output:\n");
								for (int ti = 0; ti < ((*ofc_res) * 64); ti++) {
								printf("  ofdm_car %6u : %9.6f + %9.6f i\n", ti, ofdm_car_str_real[ti], ofdm_car_str_imag[ti]);
								});
						printf("Done with carrier_alloc\n");
						*/

#ifdef INT_TIME
						gettimeofday(&x_ocaralloc_stop, NULL);
						x_ocaralloc_sec += x_ocaralloc_stop.tv_sec - x_ocaralloc_start.tv_sec;
						x_ocaralloc_usec += x_ocaralloc_stop.tv_usec - x_ocaralloc_start.tv_usec;
#endif

#if false

									// DEBUG(printf("\nCalling do_ofdm_carrier_allocator_cvc_impl_work( %u, %u, msg_stream)\n", 520, 24576));
									DEBUG(printf("\nCalling do_ofdm_carrier_allocator_cvc_impl_work( %u, %u, msg_stream)\n",
												d_frame->n_sym, d_frame->n_encoded_bits));

									// float ofdm_car_str_real[ofdm_max_out_size];
									// float ofdm_car_str_imag[ofdm_max_out_size];

									DO_NUM_IOS_ANALYSIS(printf("Calling do_ofdm_carrier_alloc: IN n_sym %u n_enc_bits %u\n", d_frame->n_sym,
												d_frame->n_encoded_bits));
									// int ofc_res = do_ofdm_carrier_allocator_cvc_impl_work(520, 24576, msg_stream_real, msg_stream_imag, ofdm_car_str_real, ofdm_car_str_imag);
#ifdef INT_TIME
									gettimeofday(&x_ocaralloc_start, NULL);
#endif
									* ofc_res = do_ofdm_carrier_allocator_cvc_impl_work(d_frame->n_sym, d_frame->n_encoded_bits,
											msg_stream_real, msg_stream_imag, ofdm_car_str_real, ofdm_car_str_imag);
									DO_NUM_IOS_ANALYSIS(printf("Back from do_ofdm_carrier_alloc: OUT ofc_res %u : %u max outputs (of %u)\n", ofc_res,
												ofc_res * d_fft_len, ofdm_max_out_size));
									DEBUG(printf(" return value was %u so max %u outputs\n", *ofc_res, (*ofc_res) * d_fft_len); printf(" do_ofdm_carrier_allocator_cvc_impl_work output:\n");
											for (int ti = 0; ti < ((*ofc_res) * 64); ti++) {
											printf("  ofdm_car %6u : %9.6f + %9.6f i\n", ti, ofdm_car_str_real[ti], ofdm_car_str_imag[ti]);
											});

#ifdef INT_TIME
									gettimeofday(&x_ocaralloc_stop, NULL);
									x_ocaralloc_sec += x_ocaralloc_stop.tv_sec - x_ocaralloc_start.tv_sec;
									x_ocaralloc_usec += x_ocaralloc_stop.tv_usec - x_ocaralloc_start.tv_usec;
#endif

#endif // if false

#if defined(HPVM)
									__hetero_task_end(T5);
#endif

#if defined(HPVM)
									void * T6 = __hetero_task_begin(5, ofc_res, ofc_res_sz, ofdm_car_str_real, ofdm_car_str_real_sz,
											ofdm_car_str_imag, ofdm_car_str_imag_sz, fft_out_real, fft_out_real_sz,
											fft_out_imag, fft_out_imag_sz,
											2, fft_out_real, fft_out_real_sz, fft_out_imag, fft_out_imag_sz, "xmit_fft_task");
#endif

#if !defined(HPVM)
#ifdef INT_TIME
									gettimeofday(&x_fft_start, NULL);
#endif

									// The FFT operation...  This is where we are currently "broken"
									//   The outputs match for the first one or two 64-entry windows, and then diverge a lot...
									DEBUG(printf("\nCalling do_xmit_fft_work for %u data values\n", ofdm_max_out_size));
									int n_ins = (*ofc_res) * d_fft_len; // max is ofdm_max_out_size
									// float fft_out_real[ofdm_max_out_size];
									// float fft_out_imag[ofdm_max_out_size];

									float scale = 1 / sqrt(52.0);

									DO_NUM_IOS_ANALYSIS(printf("Calling do_xmit_fft_work: IN n_ins %u\n", n_ins));
#endif

#if defined(HPVM)
	__hpvm__hint(DEVICE);
#endif
	{
		int n_inputs = (*ofc_res) * d_fft_len; // max is ofdm_max_out_size
		float scale = 1 / sqrt(52.0); // HPVM: Copied from do_xmit_pipeline in T6

		// Do the FFT in 64-entry windows, and add the "shift" operation to each
		//   Also add the weighting/scaling for the window
		bool inverse = true;
		bool shift = true;
		bool swap_odd_signs = false; // We shift the inputs instead?
		int size = d_fft_len;
		int log_size = d_fft_logn;
		float recluster[2] = {1.0, 1.0}; // used to alter sign of "odd" fft results
		if (swap_odd_signs) {
			recluster[1] = -1.0;
		}

		// Show the function-time input_{real/imag}
		FFT_DEBUG(
				for (int k = 0; k < (n_inputs + (size - 1)); k += size) {
				for (int i = 0; i < size; i++) {
				if (k == 0) { //  && (i < 2)
					printf("  Call_INPUT %u : REAL %f IMAG %f\n", k + i, ofdm_car_str_real[k + i], ofdm_car_str_imag[k + i]);
				}
				}
				});
				}

#ifdef XMIT_HW_FFT

		float scale = 1 / sqrt(52.0); // HPVM: Copied from do_xmit_pipeline in T6
		int n_inputs = (*ofc_res) * d_fft_len; // max is ofdm_max_out_size

		// Do the FFT in 64-entry windows, and add the "shift" operation to each
		//   Also add the weighting/scaling for the window
		bool inverse = true;
		bool shift = true;
		bool swap_odd_signs = false; // We shift the inputs instead?
		int size = d_fft_len;
		int log_size = d_fft_logn;
		float recluster[2] = {1.0, 1.0}; // used to alter sign of "odd" fft results
		if (swap_odd_signs) {
			recluster[1] = -1.0;
		}

		// Now we call the init_xmit_fft_parameters for the target FFT HWR accelerator and the specific log_nsamples for this invocation
		int num_ffts = (n_inputs + (size - 1)) / size;
		const int fn = 0;
		const uint32_t log_nsamples = 6;
		const uint32_t do_inverse = 1;
		const uint32_t do_shift = 1;
		const uint32_t scale_factor = 1;
		FFT_DEBUG(printf("  XMIT: Calling init_xmit_fft_parms ln %u nf %u inv %u sh %u scale %u\n", log_nsamples, num_ffts, do_inverse, do_shift, scale_factor));
		init_xmit_fft_parameters(fn, log_nsamples, num_ffts, do_inverse, do_shift, scale_factor);

#ifdef INT_TIME
		gettimeofday(&(x_fHcvtin_start), NULL);
#endif // INT_TIME
		// convert input from float to fixed point
		// We also SCALE it here (but we should be able to do that in the HWR Accel later)
		{ // scope for jidx
			int jidx = 0;
			for (int k = 0; k < (n_inputs + (size - 1)); k += size) {
				for (int i = 0; i < size; i++) {
					xmit_fftHW_li_mem[fn][jidx++] = float2fx(ofdm_car_str_real[k + i] * scale, FX_IL); // NOTE: when we enable scale is HW remove it from here.
					xmit_fftHW_li_mem[fn][jidx++] = float2fx(ofdm_car_str_imag[k + i] * scale, FX_IL); // NOTE: when we enable scale is HW remove it from here.
					FFT_DEBUG(
							if ((k == 0)) { // && (i < 2)
								printf(" IN_R %u : %f * %f = %f at %p\n", k + i, scale, ofdm_car_str_real[k + i], scale * ofdm_car_str_real[k + i], &(xmit_fftHW_li_mem[fn][jidx - 2]));
								printf(" IN_I %u : %f * %f = %f at %p\n", k + i, scale, ofdm_car_str_imag[k + i], scale * ofdm_car_str_imag[k + i], &(xmit_fftHW_li_mem[fn][jidx - 1]));
								usleep(50000);
							});
							}
				}
			} // scope for jidx
#ifdef INT_TIME
			gettimeofday(&x_fHcvtin_stop, NULL);
			x_fHcvtin_sec += x_fHcvtin_stop.tv_sec - x_fHcvtin_start.tv_sec;
			x_fHcvtin_usec += x_fHcvtin_stop.tv_usec - x_fHcvtin_start.tv_usec;
#endif // INT_TIME

			// Call the FFT Accelerator
			//    NOTE: Currently this is blocking-wait for call to complete
			FFT_DEBUG(printf("XMIT: calling the HW_XMIT_FFT[%u]\n", fn); usleep(50000));

#ifdef INT_TIME
			gettimeofday(&(x_fHcomp_start), NULL);
#endif // INT_TIME
			xmit_fft_in_hw(&(xmit_fftHW_fd[fn]), &(xmit_fftHW_desc[fn]));
#ifdef INT_TIME
			gettimeofday(&x_fHcomp_stop, NULL);
			x_fHcomp_sec += x_fHcomp_stop.tv_sec - x_fHcomp_start.tv_sec;
			x_fHcomp_usec += x_fHcomp_stop.tv_usec - x_fHcomp_start.tv_usec;
#endif
			// convert output from fixed point to float
			FFT_DEBUG(printf("EHFA:   converting from fixed-point to float and reclustering...\n"));
#ifdef INT_TIME
			gettimeofday(&(x_fHcvtout_start), NULL);
#endif // INT_TIME
			{ // scope for jidx
				int jidx = 0;
				for (int k = 0; k < (n_inputs + (size - 1)); k += size) {
					for (int i = 0; i < size; i++) {
						//fft_out_real[k + i] = recluster[i&0x1]*(float)fx2float(xmit_fftHW_lmem[fn][jidx++], FX_IL);
						//fft_out_imag[k + i] = recluster[i&0x1]*(float)fx2float(xmit_fftHW_lmem[fn][jidx++], FX_IL);
						float valr = (float) fx2float(xmit_fftHW_lo_mem[fn][jidx++], FX_IL);
						float vali = (float) fx2float(xmit_fftHW_lo_mem[fn][jidx++], FX_IL);
						fft_out_real[k + i] = recluster[i & 0x1] * valr;
						fft_out_imag[k + i] = recluster[i & 0x1] * vali;
						FFT_DEBUG(
								if (k == 0) {
								printf("  OUT_R %u : rc %d : Vr %f R %f at %p\n", k + i, recluster[i & 0x1], valr, fft_out_real[k + i], &(xmit_fftHW_lo_mem[fn][jidx - 2]));
								printf("  OUT_I %u : rc %d : Vr %f I %f at %p\n", k + i, recluster[i & 0x1], vali, fft_out_imag[k + i], &(xmit_fftHW_lo_mem[fn][jidx - 1]));
								usleep(50000);
								});
					}
				}
			} // scope for jidx
#ifdef INT_TIME
			gettimeofday(&x_fHcvtout_stop, NULL);
			x_fHcvtout_sec += x_fHcvtout_stop.tv_sec - x_fHcvtout_start.tv_sec;
			x_fHcvtout_usec += x_fHcvtout_stop.tv_usec - x_fHcvtout_start.tv_usec;
			x_fHtotal_sec += x_fHcvtout_stop.tv_sec - x_fHcvtin_start.tv_sec;
			x_fHtotal_usec += x_fHcvtout_stop.tv_usec - x_fHcvtin_start.tv_usec;
#endif // INT_TIME

			// the else below corresponds to ifdef XMIT_HW_FFT
#else

			float scale = 1 / sqrt(52.0); // HPVM: Copied from do_xmit_pipeline in T6

			// Do the FFT in 64-entry windows, and add the "shift" operation to each
			//   Also add the weighting/scaling for the window
			bool inverse = true;
			bool shift = true;
			bool swap_odd_signs = false; // We shift the inputs instead?
			int size = d_fft_len;
			int log_size = d_fft_logn;
			float recluster[2] = {1.0, 1.0}; // used to alter sign of "odd" fft results
			if (swap_odd_signs) {
				recluster[1] = -1.0;
			}

#if !defined(HPVM)
			{
				int n_inputs = (*ofc_res) * d_fft_len; // max is ofdm_max_out_size
				DO_LIMITS_ANALYSIS(float min_input = 3.0e+038; float max_input = -1.17e-038);
				DEBUG(printf("Starting do_xmit_fft_work with size %u inverse %u shift %u on n_inputs %u\n", size, inverse, shift, n_inputs));
			}
#endif
			//for (int iteration = 0; iteration < (n_inputs + (size - 1)) / size; iteration += 1) // Original for-loop
			for (int iteration = 0; iteration < MAX_FFT_FRAMES; iteration += 1) { // TODO: IS THE TERMINATION CONDITION VALID?
				int n_inputs = (*ofc_res) * d_fft_len; // max is ofdm_max_out_size
				int k = iteration * size;
				if (!(k > n_inputs + (size - 1))) {
				float fft_in_real[64];
				float fft_in_imag[64];

				DEBUG(printf(" Prepping for FFT call starting at %u\n", k));
				// Set up the (scaled) inputs
				if (shift) {
					FFT_DEBUG(
							for (int i = 0; i < size / 2; i++) {
							if ((k == 0)) { // && (i < 2))
								printf(" IN_R %u : %f * %f = %f\n", k + i, scale, ofdm_car_str_real[k + i], scale * ofdm_car_str_real[k + i]);
								printf(" IN_I %u : %f * %f = %f\n", k + i, scale, ofdm_car_str_imag[k + i], scale * ofdm_car_str_imag[k + i]);
								usleep(50000);
							}
							});
							for (int i = 0; i < size / 2; i++) {
								fft_in_real[32 + i] = ofdm_car_str_real[k + i] * scale; // Copy  0 .. 31 into 32 .. 63
								DO_LIMITS_ANALYSIS(
										if (fft_in_real[32 + i] < min_input) {
										min_input = fft_in_real[32 + i];
										}
										if (fft_in_real[32 + i] > max_input) {
										max_input = fft_in_real[32 + i];
										});
								fft_in_imag[32 + i] = ofdm_car_str_imag[k + i] * scale; // Copy  0 .. 31 into 32 .. 63
								DO_LIMITS_ANALYSIS(
										if (fft_in_imag[32 + i] < min_input) {
										min_input = fft_in_imag[32 + i];
										}
										if (fft_in_imag[32 + i] > max_input) {
										max_input = fft_in_imag[32 + i];
										});
								fft_in_real[i] = ofdm_car_str_real[k + 32 + i] * scale; // Copy 32 .. 63 into  0 .. 31
								DO_LIMITS_ANALYSIS(
										if (fft_in_real[i] < min_input) {
										min_input = fft_in_real[i];
										}
										if (fft_in_real[i] > max_input) {
										max_input = fft_in_real[i];
										});
								fft_in_imag[i] = ofdm_car_str_imag[k + 32 + i] * scale; // Copy 32 .. 63 into  0 .. 31
								DO_LIMITS_ANALYSIS(
										if (fft_in_imag[i] < min_input) {
										min_input = fft_in_imag[i];
										}
										if (fft_in_imag[i] > max_input) {
										max_input = fft_in_imag[i];
										});
								DEBUG(
										if (k == 0) {
										printf("  set %u IN[ %2u ] = %11.8f * ( %11.8f + %11.8f i) = %11.8f + %11.8f i\n", k, 32 + i, scale, ofdm_car_str_real[k + i], ofdm_car_str_imag[k + i], fft_in_real[32 + i], fft_in_imag[32 + i]);
										printf("  set %u IN[ %2u ] = %11.8f * ( %11.8f + %11.8f i) = %11.8f + %11.8f i\n", k, i, scale, ofdm_car_str_real[k + 32 + i], ofdm_car_str_imag[k + 32 + i], fft_in_real[i], fft_in_imag[i]);
										});
							}
							FFT_DEBUG(
									for (int i = 0; i < size / 2; i++) {
									if ((k == 0)) { // && (i < 2))
										printf(" SHIFTED_IN_R %u : %f\n", k + i, fft_in_real[i]);
										printf(" SHIFTED_IN_I %u : %f\n", k + i, fft_in_imag[i]);
										usleep(50000);
									}
									});
									}
							else {
								for (int i = 0; i < size; i++) {
									fft_in_real[i] = ofdm_car_str_real[k + i] * scale;
									fft_in_imag[i] = ofdm_car_str_imag[k + i] * scale;
									FFT_DEBUG(
											if ((k == 0)) { // && (i < 2))
												printf(" Set IN_R at %u to %f * %f = %f\n", k + i, scale, ofdm_car_str_real[k + i], scale * ofdm_car_str_real[k + i]);
												printf(" Set IN_I at %u to %f * %f = %f\n", k + i, scale, ofdm_car_str_imag[k + i], scale * ofdm_car_str_imag[k + i]);
												usleep(50000);
											});
											DEBUG(
													if (k == 0) {
													printf(" set %u IN[ %2u ] = %11.8f * ( %11.8f + %11.8f i) = %11.8f + %11.8f i\n", k, i, scale, ofdm_car_str_real[k + i], ofdm_car_str_imag[k + i], fft_in_real[i], fft_in_imag[i]);
													});
											}
								}
								DEBUG(
										if (k < 256) {
										printf("\n Iteration %u inputs (in order):\n", k);
										for (int i = 0; i < size; i++) {
										printf("  FFT_%u_IN[ %2u ] = %11.8f * ( %11.8f + %11.8f i) = %11.8f + %11.8f i\n", k, i, scale, ofdm_car_str_real[k + i], ofdm_car_str_imag[k + i], fft_in_real[i], fft_in_imag[i]);
										}
										printf("\nCalling FFT function with inverse = %u size = %u\n", inverse, size);
										});
								// NOTE: This version over-writes the input data with output data
								fft_ri(fft_in_real, fft_in_imag, inverse, false, size, log_size); 

								for (int i = 0; i < size; i++) {
									// Swap sign on the "odd" FFT results (re-cluster energy around zero?)
									fft_out_real[k + i] = recluster[i & 0x1] * fft_in_real[i];
									fft_out_imag[k + i] = recluster[i & 0x1] * fft_in_imag[i];
									FFT_DEBUG(
											if (k == 0) {
											printf("  OUT_R %u : rc %f : Vr %f R %f\n", k + i, recluster[i & 0x1], fft_in_real[i], fft_out_real[k + i]);
											printf("  OUT_I %u : rc %f : Vr %f I %f\n", k + i, recluster[i & 0x1], fft_in_imag[i], fft_out_imag[k + i]);
											usleep(50000);
											});
								}

								DEBUG(
										if (k < 256) {
										for (int i = 0; i < size; i++) {
										printf(" FFT_%u_OUT[ %2u : %5u ] = %11.8f + %11.8f i\n", k, i, k + i, fft_out_real[k + i], fft_out_imag[k + i]);
										}
										});

								DEBUG(
										if (k < 4) {
										printf("\n");
										});
							}
							} // for (k = 0 .. n_inputs
#endif // end for ifdef / else XMIT_HW_FFT


							DO_LIMITS_ANALYSIS(printf("DO_XMIT_FFT_WORK : min_input = %.15g  max_input = %.15g\n", min_input, max_input));
							DEBUG(printf(" Done with fft calls... output:\n"));

#if !defined(HPVM)
#ifdef INT_TIME
									gettimeofday(&x_fft_stop, NULL);
									x_fft_sec += x_fft_stop.tv_sec - x_fft_start.tv_sec;
									x_fft_usec += x_fft_stop.tv_usec - x_fft_start.tv_usec;
#endif
									DO_NUM_IOS_ANALYSIS(printf("Back from do_xmit_fft_work: OUT n_out %u\n", n_ins));
									DEBUG(
											for (int i = 0; i < n_ins; i++) {
											printf(" fft_out %6u : %11.8f + %11.8f i\n", i, fft_out_real[i], fft_out_imag[i]);
											});
#endif

#if defined(HPVM)
									__hetero_task_end(T6);
#endif

#if defined(HPVM)
									void * T7 = __hetero_task_begin(5, ofc_res, ofc_res_sz, fft_out_real, fft_out_real_sz,
											fft_out_imag, fft_out_imag_sz, cycpref_out_real, cycpref_out_real_sz,
											cycpref_out_imag, cycpref_out_imag_sz,
											2, cycpref_out_real, cycpref_out_real_sz, cycpref_out_imag, cycpref_out_imag_sz,
											"cyclic_prefix_task_wrapper");
#endif
#if defined(HPVM)
						__hpvm__hint(DEVICE);
#endif

						//#include "gold_fft_outputs.c"

#ifdef INT_TIME
						gettimeofday(&x_ocycpref_start, NULL);
#endif

						int num_cycpref_outs = (*ofc_res) * (d_fft_len + d_cp_size) + 1;

						// float cycpref_out_real[41360]; // Large enough
						// float cycpref_out_imag[41360]; // Large enough

						DEBUG(printf("\nCalling do_ofdm_cyclic_prefixer_impl_work(%u, fft_output)\n", (*ofc_res)));
						DO_NUM_IOS_ANALYSIS(printf("Calling do_ofdm_cyclic_prefx: IN ofc_res %u : OUT n_cycp_out %u\n",
									(*ofc_res), num_cycpref_outs));
						//do_ofdm_cyclic_prefixer_impl_work(ofc_res, gold_fft_out_real, gold_fft_out_imag, cycpref_out_real, cycpref_out_imag);
						/* #ifdef INT_TIME */
						/*  gettimeofday(&x_ocycpref_start, NULL); */
						/* #endif */
						do_ofdm_cyclic_prefixer_impl_work(*ofc_res, fft_out_real, fft_out_imag, cycpref_out_real, cycpref_out_imag);
						//printf("Done with do_ofdm_cyclic_prefixer_impl_work\n");

#ifdef INT_TIME
						gettimeofday(&x_ocycpref_stop, NULL);
						x_ocycpref_sec += x_ocycpref_stop.tv_sec - x_ocycpref_start.tv_sec;
						x_ocycpref_usec += x_ocycpref_stop.tv_usec - x_ocycpref_start.tv_usec;
#endif
						DEBUG(
								for (int i = 0; i < num_cycpref_outs; i++) {
								printf(" ocypref_out %6u : %11.8f + %11.8f i\n", i, cycpref_out_real[i], cycpref_out_imag[i]);
								}
								printf("\n"));
#if false

									//#include "gold_fft_outputs.c"

#ifdef INT_TIME
									gettimeofday(&x_ocycpref_start, NULL);
#endif

									int num_cycpref_outs = (*ofc_res) * (d_fft_len + d_cp_size) + 1;

									// float cycpref_out_real[41360]; // Large enough
									// float cycpref_out_imag[41360]; // Large enough

									DEBUG(printf("\nCalling do_ofdm_cyclic_prefixer_impl_work(%u, fft_output)\n", (*ofc_res)));
									DO_NUM_IOS_ANALYSIS(printf("Calling do_ofdm_cyclic_prefx: IN ofc_res %u : OUT n_cycp_out %u\n",
												(*ofc_res), num_cycpref_outs));
									//do_ofdm_cyclic_prefixer_impl_work(ofc_res, gold_fft_out_real, gold_fft_out_imag, cycpref_out_real, cycpref_out_imag);
									/* #ifdef INT_TIME */
									/*  gettimeofday(&x_ocycpref_start, NULL); */
									/* #endif */
									do_ofdm_cyclic_prefixer_impl_work(*ofc_res, fft_out_real, fft_out_imag, cycpref_out_real, cycpref_out_imag);

#ifdef INT_TIME
									gettimeofday(&x_ocycpref_stop, NULL);
									x_ocycpref_sec += x_ocycpref_stop.tv_sec - x_ocycpref_start.tv_sec;
									x_ocycpref_usec += x_ocycpref_stop.tv_usec - x_ocycpref_start.tv_usec;
#endif
									DEBUG(
											for (int i = 0; i < num_cycpref_outs; i++) {
											printf(" ocypref_out %6u : %11.8f + %11.8f i\n", i, cycpref_out_real[i], cycpref_out_imag[i]);
											}
											printf("\n"));
#endif // if false

#if defined(HPVM)
									__hetero_task_end(T7);
#endif

#if defined(HPVM)
									void * T8 = __hetero_task_begin(6, num_final_outs, num_final_outs_sz, final_out_real, final_out_real_sz,
											final_out_imag, final_out_imag_sz, cycpref_out_real, cycpref_out_real_sz,
											cycpref_out_imag, cycpref_out_imag_sz, ofc_res, ofc_res_sz, 3,
											num_final_outs, num_final_outs_sz, final_out_real, final_out_real_sz,
											final_out_imag, final_out_imag_sz, "padding_task_wrapper");
#endif
#if defined(HPVM)
						__hpvm__hint(DEVICE);
#endif


						int num_cycpref_outs_cp = (*ofc_res) * (d_fft_len + d_cp_size) + 1; // copied from above task
						// The next "stage" is the "packet_pad2" which adds 500 zeros to the front (and no zeros to the rear) of the output
						//   This block may also add some time-stamp tags (for UHD?) for GnuRadio use?
						//   Not sure we care about this padding?
						bool do_add_pre_pad = false;
						int num_pre_pad = do_add_pre_pad ? 500 : 0;
						int num_post_pad = 0;

						// Now set the Final Outputs
						*num_final_outs = num_pre_pad + num_cycpref_outs_cp + num_post_pad;
						for (int i = 0; i < num_pre_pad; i++) {
							final_out_real[i] = 0.0;
							final_out_imag[i] = 0.0;
						}
						for (int i = 0; i < num_cycpref_outs_cp; i++) {
							int iidx = num_pre_pad + i;
							final_out_real[iidx] = cycpref_out_real[i];
							final_out_imag[iidx] = cycpref_out_imag[i];
						}
#if false

									int num_cycpref_outs_cp = (*ofc_res) * (d_fft_len + d_cp_size) + 1; // copied from above task
									// The next "stage" is the "packet_pad2" which adds 500 zeros to the front (and no zeros to the rear) of the output
									//   This block may also add some time-stamp tags (for UHD?) for GnuRadio use?
									//   Not sure we care about this padding?
									bool do_add_pre_pad = false;
									DEBUG(printf("\nAdd the pre-padding : %u\n", do_add_pre_pad));
									int num_pre_pad = do_add_pre_pad ? 500 : 0;
									int num_post_pad = 0;
									DEBUG(printf("\n"));

									// Now set the Final Outputs
									DEBUG(printf("\nFinal XMIT output:\n"));
									*num_final_outs = num_pre_pad + num_cycpref_outs_cp + num_post_pad;
									DO_NUM_IOS_ANALYSIS(printf("Set num_finalouts to %u = pre-pad %u + %u num_cycpref_outs\n",
												*num_final_outs, num_pre_pad, num_cycpref_outs_cp));
									for (int i = 0; i < num_pre_pad; i++) {
										final_out_real[i] = 0.0;
										final_out_imag[i] = 0.0;
										DEBUG(printf(" fin_xmit_out %6u : %11.8f + %11.8f i\n", i, final_out_real[i], final_out_imag[i]));
									}
									for (int i = 0; i < num_cycpref_outs_cp; i++) {
										int iidx = num_pre_pad + i;
										final_out_real[iidx] = cycpref_out_real[i];
										final_out_imag[iidx] = cycpref_out_imag[i];
										DEBUG(printf(" fin_xmit_out %6u : %11.8f + %11.8f i\n", iidx, final_out_real[iidx], final_out_imag[iidx]));
									}
									/* for (int i = 0; i < num_post_pad; i++) { */
									/*   int iidx = num_pre_pad + num_cycpref_outs_cp + i; */
									/*   final_out_real[iidx] = 0.0; */
									/*   final_out_imag[iidx] = 0.0; */
									/*   DEBUG(printf(" fin_xmit_out %6u : %11.8f + %11.8f i\n", iidx, final_out_real[iidx], final_out_imag[iidx])); */
									/* } */
									// These next stages do not appear to have any relationship to a physical system that we might consider accelerating.

									// The next "Stage" is the "throttle" block, which does not alter the output/message (just timing?)

									// Then there is the channel_model...

#ifdef INT_TIME
									// The time take by the whole function can't be measure from within the function (as doing so would
									// mean the calls to gettimeofday must be put in tasks which would then run in parallel with the rest
									// of the tasks in this function). The caller of this function must measure the time taken.
									// gettimeofday(&x_pipe_stop, NULL);
									// x_pipe_sec  += x_pipe_stop.tv_sec  - x_pipe_start.tv_sec;
									// x_pipe_usec += x_pipe_stop.tv_usec - x_pipe_start.tv_usec;
#endif

#endif // if false

#if defined(HPVM)
									__hetero_task_end(T8);
#endif

#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR)) && true
	__hetero_section_end(Section_Main);
	__hetero_task_end(T_CPU_Wrapper);
	__hetero_section_end(Section);
#endif
}

#ifdef USE_OLD_MODEL

void cv_root(unsigned tr_val, label_t * out_label, size_t outlabel_sz) {
#if (defined(HPVM) && defined(HPVM_CV_ROOT))
	void * Section = __hetero_section_begin();
	void * T1 = __hetero_task_begin(2, tr_val, out_label, outlabel_sz, 1, out_label, outlabel_sz, "cv_root_task");
//	__hpvm__hint(DEVICE);
#endif
	* out_label = run_object_classification(tr_val);

#if (defined(HPVM) && defined(HPVM_CV_ROOT))
	__hetero_task_end(T1);
	__hetero_section_end(Section);
#endif
}

void cv_root_wrapper(unsigned tr_val, label_t * out_label, size_t outlabel_sz) {
#if (defined(HPVM) && defined(HPVM_CV_ROOT)) && true
	void * Section = __hetero_section_begin();
	void * T1 = __hetero_task_begin(2, tr_val, out_label, outlabel_sz, 1, out_label, outlabel_sz, "cv_root_wrapper_task");
#endif

	cv_root(tr_val, out_label, outlabel_sz);

#if (defined(HPVM) && defined(HPVM_CV_ROOT)) && true
	__hetero_task_end(T1);
	__hetero_section_end(Section);
#endif
}

#else

void cv_root(uint8_t * rgb_image, size_t rgb_image_sz, dim_t * dimensions, size_t dimensions_sz,
	char * filename, size_t filename_sz, int * nboxes, size_t nboxes_sz,
	detection_t * dets, size_t dets_sz) {
#if (defined(HPVM) && defined(HPVM_CV_ROOT)) && false
	void * Section = __hetero_section_begin();
	void * T1 = __hetero_task_begin(5, rgb_image, rgb_image_sz, dimensions, dimensions_sz, filename, filename_sz,
		nboxes, nboxes_sz, dets, dets_sz, 1, dets, dets_sz, "cv_root_task");
//	__hpvm__hint(DEVICE);
#endif
	// dets = run_object_classification(rgb_image, *dimensions, filename, nboxes); // TODO: Uncomment me; this is commented cause my machine doesn't have opencv.

#if  (defined(HPVM) && defined(HPVM_CV_ROOT)) && false
	__hetero_task_end(T1);
	__hetero_section_end(Section);
#endif
}

void cv_root_wrapper(uint8_t * rgb_image, size_t rgb_image_sz, dim_t * dimensions, size_t dimensions_sz,
	char * filename, size_t filename_sz, int * nboxes, size_t nboxes_sz,
	detection_t * dets, size_t dets_sz) {
#if (defined(HPVM) && defined(HPVM_CV_ROOT)) && true
	void * Section = __hetero_section_begin();
	void * T1 = __hetero_task_begin(5, rgb_image, rgb_image_sz, dimensions, dimensions_sz, filename, filename_sz,
		nboxes, nboxes_sz, dets, dets_sz, 1, dets, dets_sz, "cv_root_wrapper_task");
#endif

	cv_root(rgb_image, rgb_image_sz, dimensions, dimensions_sz, filename, filename_sz, nboxes, nboxes_sz, dets, dets_sz);

#if  (defined(HPVM) && defined(HPVM_CV_ROOT)) && true
	__hetero_task_end(T1);
	__hetero_section_end(Section);
#endif
}

#endif // ifdef USE_OLD_MODEL


int main(int argc, char * argv[]) {
	for(int i = 0; i < MAX_PSDU_SIZE; ++i) {
		d_psdu_org[i] = 0;
	}
	printf("Initialized d_psdu_org (addr: %p) to 0 for entries indexed [0...%d]\n", d_psdu_org, MAX_PSDU_SIZE);

	struct sockaddr_in bag_servaddr;
	struct sockaddr_in xmit_servaddr;
	struct sockaddr_in recv_servaddr;
	struct sockaddr_in car_servaddr;
	unsigned char l_buffer[20] = {
		0 };
	// unsigned char buffer[200002] = {0};

	lidar_inputs_t lidar_inputs;

	// snprintf(bag_inet_addr_str, 20, "192.168.1.99");
	// snprintf(wifi_inet_addr_str, 20, "192.168.1.99");
	// snprintf(car_inet_addr_str, 20, "192.168.1.99");
	snprintf(bag_inet_addr_str, 20, "127.0.0.1");
	snprintf(wifi_inet_addr_str, 20, "127.0.0.1");
	snprintf(car_inet_addr_str, 20, "127.0.0.1");

	// hpvm: The inits below can probably all go in parallel
	printf("Initializing the OccGrid state...\n");
	init_occgrid_state(); // Initialize the occgrid functions, state, etc.
	printf("Initializing the Transmit pipeline...\n");
	xmit_pipe_init(crcTable); // Initialize the IEEE SDR Transmit Pipeline
	printf("Initializing the Receive pipeline...\n");
	recv_pipe_init(fir_input, firc_input);
	printf("Initializing the Computer Vision toolset...\n");

#ifdef USE_OLD_MODEL
	if (cv_toolset_init() != success) {
		printf("Computer Vision toolset initialization failed...\n");
		exit(0);
	}
#else
	/*****************************************************************************/
	/* NEW: PyTorch TinyYOLOv2 support (May 2022)                                */
	// TODO: Uncomment the if-statement below! This is commented as my machine doesn't have opencv installed so everything related to cv is currently being ignored on my side.
		/******* TODO: Uncomment me *******
	if (cv_toolset_init("tiny_yolov2_coco", "yolov2-tiny.weights") != 0) {
		printf("Computer Vision toolset initialization failed...\n");
		exit(1);
	}
	******* TODO: Uncomment me *******/
	/*****************************************************************************/
#endif

	signal(SIGINT, INThandler);
	signal(SIGPIPE, SIGPIPE_handler);

	// Use getopt to read in run-time options
	// put ':' in the starting of the
	// string so that program can
	// distinguish between '?' and ':'
	int opt;
	while ((opt = getopt(argc, argv, ":hB:W:C:s:")) != -1) {
		switch (opt) {
		case 'h':
			print_usage(argv[0]);
			exit(0);
		case 'B':
			snprintf(bag_inet_addr_str, 20, "%s", optarg);
			break;
		case 'W':
			snprintf(wifi_inet_addr_str, 20, "%s", optarg);
			break;
		case 'C':
			snprintf(car_inet_addr_str, 20, "%s", optarg);
			break;
		case 's':
			max_time_steps = atoi(optarg);
			DBGOUT(printf("Set maximum time steps to %u\n", max_time_steps));
			break;

		case ':':
			printf("option needs a value\n");
			break;
		case '?':
			printf("unknown option: %c\n", optopt);
			break;
		}
	}

	if (max_time_steps != ~1) {
		printf("Limiting to %u max time steps\n", max_time_steps);
	}
	else {
		printf("Running the entire bag file.\n");
	}

	printf("Connecting to bag-server at IP %s PORT %u\n", bag_inet_addr_str, BAG_PORT);
	// Open and connect to the BAG_SERVER
	if ((bag_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("BAG Socket creation failed...\n");
		exit(0);
	}
	else {
		printf("BAG Socket successfully created..\n");
	}

	bag_servaddr.sin_family = AF_INET;
	bag_servaddr.sin_addr.s_addr = inet_addr(bag_inet_addr_str);
	bag_servaddr.sin_port = htons(BAG_PORT);

	while (true) {
		if (connect(bag_sock, (struct sockaddr *) &bag_servaddr, sizeof(bag_servaddr)) != 0) {
			printf("connection with the BAG server failed...\n");
			sleep(1);
			continue;
		}
		else {
			printf("connected to the BAG server..\n");
			break;
		}
	}

	// Open and connect to the XMIT_SERVER
	printf("Connecting to xmit-server at IP %s PORT %u\n", wifi_inet_addr_str, XMIT_PORT);
	if ((xmit_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("WIFI XMIT Socket creation failed...\n");
		exit(0);
	}
	else {
		printf("WIFI XMIT Socket successfully created..\n");
	}

	xmit_servaddr.sin_family = AF_INET;
	xmit_servaddr.sin_addr.s_addr = inet_addr(wifi_inet_addr_str);
	xmit_servaddr.sin_port = htons(XMIT_PORT);

	while (true) {
		if (connect(xmit_sock, (struct sockaddr *) &xmit_servaddr, sizeof(xmit_servaddr)) != 0) {
			printf("connection with the WIFI XMIT server failed...\n");
			sleep(1);
			continue;
		}
		else {
			printf("connected to the WIFI XMIT server..\n");
			break;
		}
	}

	// Open and connect to the RECV_SERVER
	printf("Connecting to recv-server at IP %s PORT %u\n", wifi_inet_addr_str, RECV_PORT);
	if ((recv_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("WIFI RECV Socket creation failed...\n");
		exit(0);
	}
	else {
		printf("WIFI RECV Socket successfully created..\n");
	}

	recv_servaddr.sin_family = AF_INET;
	recv_servaddr.sin_addr.s_addr = inet_addr(wifi_inet_addr_str); //"127.0.0.1");
	recv_servaddr.sin_port = htons(RECV_PORT);

	while (true) {
		if (connect(recv_sock, (struct sockaddr *) &recv_servaddr, sizeof(recv_servaddr)) != 0) {
			printf("connection with the WIFI RECV server failed...\n");
			sleep(1);
			continue;
		}
		else {
			printf("connected to the WIFI RECV server..\n");
			break;
		}
	}

	printf("Connecting to car-server at IP %s PORT %u\n", car_inet_addr_str, CAR_PORT);
	// Open and connect to the CAR_SERVER
	if ((car_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("CAR Socket creation failed...\n");
		exit(0);
	}
	else {
		printf("CAR Socket successfully created..\n");
	}

	car_servaddr.sin_family = AF_INET;
	car_servaddr.sin_addr.s_addr = inet_addr(car_inet_addr_str);
	car_servaddr.sin_port = htons(CAR_PORT);

	while (true) {
		if (connect(car_sock, (struct sockaddr *) &car_servaddr, sizeof(car_servaddr)) != 0) {
			printf("connection with the CAR server failed...\n");
			sleep(1);
			continue;
		}
		else {
			printf("connected to the CAR server..\n");
			break;
		}
	}

#if PARALLEL_PTHREADS
	// Now set up the processing threads - 1 for Lidar input, one for WiFi RECV processing (fusion)
	pthread_attr_t pt_attr;
	pthread_attr_init(&pt_attr);
	// pthread_t form_occmap_thread;
	pthread_t fuse_occmap_thread;
	/* int pt_ret = pthread_create(&form_occmap_thread, &pt_attr, process_lidar_to_occgrid, NULL); */
	/* if (pt_ret != 0) { */
	/*   printf("Could not start the scheduler pthread... return value %d\n", pt_ret); */
	/*   closeout_and_exit(-1); */
	/* } */
	int pt_ret = pthread_create(&fuse_occmap_thread, &pt_attr, receive_and_fuse_maps, NULL);
	if (pt_ret != 0) {
		printf("Could not start the scheduler pthread... return value %d\n", pt_ret);
		closeout_and_exit("Couldn't allocate fuse_occmap_thread...", -1);
	}
#endif

	//#ifdef INT_TIME
	gettimeofday(&start_prog, NULL);
	//#endif
	bool hit_eof = false;

	/*************************************************************************/
	/*                           M A I N   L O O P                           */
	/*************************************************************************/
	while ((!hit_eof) && (lidar_count < max_time_steps)) {

		DBGOUT(printf("Calling read_all on the BAG socket...\n"); fflush(stdout));
		// int valread = read(bag_sock , l_buffer, 10);
		//#ifdef INT_TIME
		gettimeofday(&start_proc_rdbag, NULL);
		//#endif
		int valread = read_all(bag_sock, (char *) l_buffer, 10);
		//#ifdef INT_TIME
		gettimeofday(&stop_proc_rdbag, NULL);
		proc_rdbag_sec += stop_proc_rdbag.tv_sec - start_proc_rdbag.tv_sec;
		proc_rdbag_usec += stop_proc_rdbag.tv_usec - start_proc_rdbag.tv_usec;
		//#endif
		DBGOUT(printf("Top: read %d bytes\n", valread));
		if (valread < 10) {
			if (valread == 0) {
				// Zero bytes out implies we hit EOF
				hit_eof = true;
			}
			else {
				printf("  TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, 10);
				closeout_and_exit("Top read got too few bytes...", -1);
			}
		}

		/***********************************************************************/
		/* Checking if the received message includes lidar information         */
		/***********************************************************************/
		if (l_buffer[0] == 'L' && l_buffer[9] == 'L') {

			//#ifdef INT_TIME
			gettimeofday(&start_proc_lidar, NULL);
			//#endif
			char * ptr;
			int message_size = strtol((char *) l_buffer + 1, &ptr, 10);
			DBGOUT(printf("Lidar: expecting message size: %d\n", message_size));
			send(bag_sock, ack, 2, 0);

			int total_bytes_read = read_all(bag_sock, lidar_inputs.data, message_size);
			if (total_bytes_read < message_size) {
				if (total_bytes_read == 0) {
					printf("  Lidar read got ZERO bytes -- END of TRANSFER?\n");
					closeout_and_exit("Lidar read got zero bytes...", -1);
				}
				else {
					printf("  Lidar read got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", total_bytes_read,
						message_size);
					closeout_and_exit("Lidar read got too few bytes...", -1);
				}
			}
			if (total_bytes_read > message_size) {
				printf("NOTE: read more total bytes than expected: %u vs %u\n", total_bytes_read, message_size);
			}
			lidar_inputs.odometry[0] = odometry[0];
			lidar_inputs.odometry[1] = odometry[1];
			lidar_inputs.odometry[2] = odometry[2];
			lidar_inputs.data_size = total_bytes_read;
			DBGOUT(printf("Calling process_lidar_to_occgrid for %d total bytes\n", total_bytes_read));
			printf("Processing Lidar msg %4u data\n", lidar_count);
			//#ifdef INT_TIME
			gettimeofday(&start_proc_data, NULL);
			//#endif

			// Arguments to lidar_root
			int n_cmp_bytes = 0;
			size_t n_cmp_bytes_sz = sizeof(int);
			unsigned char cmp_data[MAX_COMPRESSED_DATA_SIZE];
			size_t cmp_data_sz = MAX_COMPRESSED_DATA_SIZE;

			// Start of arguments to cloudToOccgrid (indirectly called by lidar_root)
			double AVxyzw = 1.5;
			bool rolling_window = false;
			double min_obstacle_heght = 0.05;
			double max_obstacle_heght = 2.05;
			double raytrace_range = RAYTR_RANGE;
			unsigned int size_x = GRID_MAP_X_DIM;
			unsigned int size_y = GRID_MAP_Y_DIM;
			unsigned int resolution = GRID_MAP_RESLTN;
			// End of arguments to cloudToOccgrid (indirectly called by lidar_root)

			int timer_sequentialize = 0;

			// Start of arguments to do_xmit_pipeline (called by lidar_root)
			int n_xmit_out = 0;
			size_t n_xmit_out_sz = sizeof(int);
			float xmit_out_real[MAX_XMIT_OUTPUTS];
			size_t xmit_out_real_sz = MAX_XMIT_OUTPUTS * sizeof(float);
			float xmit_out_imag[MAX_XMIT_OUTPUTS];
			size_t xmit_out_imag_sz = MAX_XMIT_OUTPUTS * sizeof(float);
			// End of arguments to do_xmit_pipeline (called by lidar_root)

			// Start of arguments to do_xmit_pipeline (called by lidar_root) and transmit_occgrid
			int psdu_len = 0;
			size_t psdu_len_sz = sizeof(int);
			uint8_t pckt_hdr_out[64];
			size_t pckt_hdr_out_sz = 64;
			int pckt_hdr_len = 0;
			size_t pckt_hdr_len_sz = sizeof(int);
			float msg_stream_real[MAX_SIZE];
			size_t msg_stream_real_sz = MAX_SIZE * sizeof(float);
			float msg_stream_imag[MAX_SIZE];
			size_t msg_stream_imag_sz = MAX_SIZE * sizeof(float);
			float ofdm_car_str_real[ofdm_max_out_size];
			size_t ofdm_car_str_real_sz = ofdm_max_out_size * sizeof(float);
			float ofdm_car_str_imag[ofdm_max_out_size];
			size_t ofdm_car_str_imag_sz = ofdm_max_out_size * sizeof(float);
			int ofc_res = 0;
			size_t ofc_res_sz = sizeof(int);
			float fft_out_real[ofdm_max_out_size];
			size_t fft_out_real_sz = ofdm_max_out_size * sizeof(float);
			float fft_out_imag[ofdm_max_out_size];
			size_t fft_out_imag_sz = ofdm_max_out_size * sizeof(float);
			float cycpref_out_real[41360];
			size_t cycpref_out_real_sz = 41360 * sizeof(float);
			float cycpref_out_imag[41360];
			size_t cycpref_out_imag_sz = 41360 * sizeof(float);
			// End of arguments to do_xmit_pipeline (called by lidar_root) and transmit_occgrid


			printf("%s %d Calling lidar_root", __FILE__, __LINE__);

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR)) && true
			void * lidarDAG = __hetero_launch((void *) lidar_root, 43, 
				&lidar_inputs, sizeof(lidar_inputs_t),
				&observationsArr[next_obs], sizeof(Observation),
				&n_cmp_bytes, sizeof(int),
				cmp_data, MAX_COMPRESSED_DATA_SIZE,
				// Global vars passed for internal use in lidar_root
				(int *) &curr_obs, sizeof(int),
				(int *) &next_obs, sizeof(int),
				(int *) &lmap_count, sizeof(unsigned),
				// Args to cloudToOccgrid
				&AVxyzw, sizeof(double),
				&rolling_window, sizeof(bool),
				&min_obstacle_heght, sizeof(double),
				&max_obstacle_heght, sizeof(double),
				&raytrace_range, sizeof(double),
				&size_x, sizeof(unsigned int),
				&size_y, sizeof(unsigned int),
				&resolution, sizeof(unsigned int),
				// Args to sequentialized timer start and end calls along with the code being time
				&timer_sequentialize, sizeof(int),
				// Args for encode_occgrid
				&n_xmit_out, n_xmit_out_sz,
				xmit_out_real, xmit_out_real_sz,
				xmit_out_imag, xmit_out_imag_sz,
				// Start of local variables for do_xmit_pipeline
				&psdu_len, psdu_len_sz,
				pckt_hdr_out, pckt_hdr_out_sz,
				&pckt_hdr_len, pckt_hdr_len_sz,
				msg_stream_real, msg_stream_real_sz,
				msg_stream_imag, msg_stream_imag_sz,
				ofdm_car_str_real, ofdm_car_str_real_sz,
				ofdm_car_str_imag, ofdm_car_str_imag_sz,
				&ofc_res, ofc_res_sz,
				fft_out_real, fft_out_real_sz,
				fft_out_imag, fft_out_imag_sz,
				cycpref_out_real, cycpref_out_real_sz,
				cycpref_out_imag, cycpref_out_imag_sz,
				d_occupied_carriers_org, d_occupied_carriers_org_sz,
				&d_psdu_org, d_psdu_org_size,
				d_map_out_copy_org, d_map_out_org_size,
				&d_seq_nr_org, d_seq_nr_org_sz/*=sizeof(uint16_t)*/,
				&d_scrambler_org, d_scrambler_org_sz,
				d_symbols_org, d_symbols_org_sz,
				&d_symbols_offset_org, d_symbols_offset_org_sz,
				&d_symbols_len_org, d_symbols_len_org_sz,
				&d_ofdm_org, d_ofdm_org_sz,
                         	&d_frame_org, d_frame_org_sz,
				d_pilot_carriers_org, d_pilot_carriers_org_sz,
				crcTable, crcTable_sz,
				// Outputs
				17, &n_cmp_bytes, sizeof(int),
				cmp_data, MAX_COMPRESSED_DATA_SIZE,
				&observationsArr[next_obs], sizeof(Observation),
				&d_psdu_org, d_psdu_org_size,
				d_map_out_copy_org, d_map_out_org_size,
				&d_seq_nr_org, d_seq_nr_org_sz/*=sizeof(uint16_t)*/,
				&d_scrambler_org, d_scrambler_org_sz,
				d_symbols_org, d_symbols_org_sz,
				&d_symbols_offset_org, d_symbols_offset_org_sz,
				&d_symbols_len_org, d_symbols_len_org_sz,
				&d_ofdm_org, d_ofdm_org_sz,
                                &d_frame_org, d_frame_org_sz,
				d_pilot_carriers_org, d_pilot_carriers_org_sz,
				&n_xmit_out, n_xmit_out_sz,
				xmit_out_real, xmit_out_real_sz,
				xmit_out_imag, xmit_out_imag_sz,
				crcTable, crcTable_sz
			);
			__hetero_wait(lidarDAG);
#else
			lidar_root(
				&lidar_inputs, sizeof(lidar_inputs_t),
				&observationsArr[next_obs], sizeof(Observation),
				&n_cmp_bytes, sizeof(int),
				cmp_data, MAX_COMPRESSED_DATA_SIZE,
				// Global vars passed for internal use in lidar_root
				(int *) &curr_obs, sizeof(int),
				(int *) &next_obs, sizeof(int),
				(int *) &lmap_count, sizeof(unsigned),
				// Args to cloudToOccgrid
				&AVxyzw, sizeof(double),
				&rolling_window, sizeof(bool),
				&min_obstacle_heght, sizeof(double),
				&max_obstacle_heght, sizeof(double),
				&raytrace_range, sizeof(double),
				&size_x, sizeof(unsigned int),
				&size_y, sizeof(unsigned int),
				&resolution, sizeof(unsigned int),
				// Args to sequentialized timer start and end calls along with the code being time
				&timer_sequentialize, sizeof(int),
				// Args for encode_occgrid
				&n_xmit_out, n_xmit_out_sz,
				xmit_out_real, xmit_out_real_sz,
				xmit_out_imag, xmit_out_imag_sz,
				// Start of local variables for do_xmit_pipeline
				&psdu_len, psdu_len_sz,
				pckt_hdr_out, pckt_hdr_out_sz,
				&pckt_hdr_len, pckt_hdr_len_sz,
				msg_stream_real, msg_stream_real_sz,
				msg_stream_imag, msg_stream_imag_sz,
				ofdm_car_str_real, ofdm_car_str_real_sz,
				ofdm_car_str_imag, ofdm_car_str_imag_sz,
				&ofc_res, ofc_res_sz,
				fft_out_real, fft_out_real_sz,
				fft_out_imag, fft_out_imag_sz,
				cycpref_out_real, cycpref_out_real_sz,
				cycpref_out_imag, cycpref_out_imag_sz,
				d_occupied_carriers_org, d_occupied_carriers_org_sz,
				&d_psdu_org, d_psdu_org_size,
				d_map_out_copy_org, d_map_out_org_size,
				&d_seq_nr_org, d_seq_nr_org_sz/*=sizeof(uint16_t)*/,
				&d_scrambler_org, d_scrambler_org_sz,
				d_symbols_org, d_symbols_org_sz,
				&d_symbols_offset_org, d_symbols_offset_org_sz,
				&d_symbols_len_org, d_symbols_len_org_sz,
				&d_ofdm_org, d_ofdm_org_sz,
                         	&d_frame_org, d_frame_org_sz,
				d_pilot_carriers_org, d_pilot_carriers_org_sz,
				crcTable, crcTable_sz
				);
#endif

			printf("%s %d Calling transmit occgrid \n", __FILE__, __LINE__);
			printf("n_xmit_out: %d\n", n_xmit_out);
			printf("xmit_out_real:");
		       	for(int i = 0; i < MAX_XMIT_OUTPUTS; ++i){
				printf("%f", xmit_out_real[i]);
			} 
			printf("\n");
			printf("xmit_out_imag:");
		       	for(int i = 0; i < MAX_XMIT_OUTPUTS; ++i){
				printf("%f", xmit_out_imag[i]);
			}
			printf("\n");


			// Send the occgrid through the socket
			transmit_occgrid(
				n_xmit_out,
				xmit_out_real, xmit_out_real_sz,
				xmit_out_imag, xmit_out_imag_sz
				);
#if PARALLEL_PTHREADS
			; // nothing to do here...
#else

			printf("%s %d Calling recieve_and_fuse_maps ", __FILE__, __LINE__);

			receive_and_fuse_maps(NULL, 0);
			DBGOUT(printf("Returning from process_lidar_to_occgrid\n"); fflush(stdout));
#endif
			DBGOUT(printf("Back from process_lidar_to_occgrid for Lidar\n"); fflush(stdout));
			lidar_count++;
			//#ifdef INT_TIME
			gettimeofday(&stop_proc_lidar, NULL);
			proc_data_sec += stop_proc_lidar.tv_sec - start_proc_data.tv_sec;
			proc_data_usec += stop_proc_lidar.tv_usec - start_proc_data.tv_usec;
			proc_lidar_sec += stop_proc_lidar.tv_sec - start_proc_lidar.tv_sec;
			proc_lidar_usec += stop_proc_lidar.tv_usec - start_proc_lidar.tv_usec;
			//#endif
		}
		/***********************************************************************/
		/* Checking if the received message includes odometry information      */
		/***********************************************************************/
		else if (l_buffer[0] == 'O' && l_buffer[9] == 'O') {

#ifdef INT_TIME
			gettimeofday(&start_proc_odo, NULL);
#endif
			char * ptr;
			int message_size = strtol((char *) l_buffer + 1, &ptr, 10);
			DBGOUT(printf("Odometry: expecting message size: %d\n", message_size));
			send(bag_sock, ack, 2, 0);

			int total_bytes_read = 0;

			// valread = read(bag_sock , buffer, 10000);
			valread = read_all(bag_sock, (char *) l_buffer, message_size);
			DBGOUT(printf("read %d bytes\n", valread));
			if (valread < message_size) {
				if (valread == 0) {
					printf("  Odo read got ZERO bytes -- END of TRANSFER?\n");
					closeout_and_exit("Odometry read got zero bytes...", -1);
				}
				else {
					printf("  Odo read got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, message_size);
					closeout_and_exit("Odometry read got too few bytes...", -1);
				}
			}

			odometry[0] = *((float *) (l_buffer));
			odometry[1] = *((float *) (l_buffer + 4));
			odometry[2] = *((float *) (l_buffer + 8));
			printf("Odometry msg %4u: %.2f %.2f %.2f\n", odo_count, odometry[0], odometry[1], odometry[2]);
			odo_count++;
#ifdef INT_TIME
			gettimeofday(&stop_proc_odo, NULL);
			proc_odo_sec += stop_proc_odo.tv_sec - start_proc_odo.tv_sec;
			proc_odo_usec += stop_proc_odo.tv_usec - start_proc_odo.tv_usec;
#endif
		}
		else {

			/*DBGOUT(printf("BUFFER : '");
				for (int ii = 0; ii < 8; ii++) {
				printf("%c", l_buffer[ii]);
				}
				printf("'\n");
				fflush(stdout));*/
		}

		/***********************************************************************/
		/* We next execute the computer vision task:                           */
		/*   - We use YOLO on images coming from a source TBD.                 */
		/*   - For testing purposes, these images can be taken initially from  */
		/*     the ATR dataset, COCO dataset, or some other available one.     */
		/*   - Ideally, we want these images to come from the actual car       */
		/*     simulator (e.g. CARLA).                                         */
		/***********************************************************************/

#ifdef USE_OLD_MODEL

#ifdef INT_TIME
		gettimeofday(&start_proc_cv, NULL);
#endif

		// HPVM task
		label_t out_label;
		unsigned tr_val = 1;  // TODO: What is the parameter 'tr_val' passed to  run_object_classification()?
#if (defined(HPVM) && defined(HPVM_CV_ROOT))
		void * dfg = __hetero_launch((void *) cv_root_wrapper, 2, tr_val, &out_label, sizeof(out_label),
			1, &out_label, sizeof(out_label));
		__hetero_wait(dfg);
#else
		cv_root(tr_val, &out_label, sizeof(label_t));
#endif
		cv_count++;
#ifdef INT_TIME
		gettimeofday(&stop_proc_cv, NULL);
		proc_cv_sec += stop_proc_cv.tv_sec - start_proc_cv.tv_sec;
		proc_cv_usec += stop_proc_cv.tv_usec - start_proc_cv.tv_usec;
		printf("run_object_classification time in usec %ld\n",
			(stop_proc_cv.tv_sec - start_proc_cv.tv_sec) * 1000000 + (stop_proc_cv.tv_usec -
				start_proc_cv.tv_usec));
#endif
		printf("run_object_classification returned %u\n", out_label);
#else

		/*****************************************************************************/
		/* NEW: PyTorch TinyYOLOv2 support (May 2022)                                */
		int width, height, channels;
		uint8_t * rgb_image = stbi_load("test.jpg", &width, &height, &channels, 3);

		if (rgb_image != NULL) {

			char filename[300];
			int nboxes = 0;
			snprintf(filename, 270, "test_output.jpg");

			dim_t dimensions;
			dimensions.width = width;
			dimensions.height = height;
			dimensions.c = channels;

			detection_t * dets;
			//dets = run_object_classification(rgb_image, dimensions, filename, &nboxes);
#if (defined(HPVM) && defined(HPVM_CV_ROOT))
			size_t rgb_image_sz = width * height * channels;
			void * dfg = __hetero_launch((void *) cv_root_wrapper, 5, rgb_image, rgb_image_sz,
				&dimensions, sizeof(dim_t), filename, (size_t) 500,
				&nboxes, sizeof(int), dets, sizeof(detection_t),
				0);
			__hetero_wait(dfg);
#else
			cv_root_wrapper(rgb_image, width * height * channels, &dimensions, sizeof(dim_t), filename, 500,
				&nboxes, sizeof(int), dets, sizeof(detection_t));
#endif

			if (dets == NULL)
				printf("run_object_classification failed (skipping this frame)\n");

			stbi_image_free(rgb_image);


		}
		else {
			printf("test.jpg image not found\n");
		}
		/*****************************************************************************/
#endif

	}

	dump_final_run_statistics();
	close(bag_sock);
	close(xmit_sock);
	close(recv_sock);
}

void dump_final_run_statistics() {
	//printf("\nFinal Run Stats, %u, Odo, %u, Lidar, %u, LMAP, %u, XMIT, %u, RECV, %u, CAR-SEND, %u, CV\n",
	//		odo_count, lidar_count, lmap_count, xmit_count, recv_count, car_send_count, cv_count);
	printf("\nFinal Run Stats, %u, Odo, %u, Lidar, %u, LMAP, %u, XMIT, %u, RECV, %u, CAR-SEND\n",
		odo_count, lidar_count, lmap_count, xmit_count, recv_count, car_send_count);
	printf("Occ-Map Dimensions, %u, by, %u, grid, res, %lf, ray_r, %u\n", GRID_MAP_X_DIM,
		GRID_MAP_Y_DIM, GRID_MAP_RESLTN, RAYTR_RANGE);

	printf("Timing (in usec):");
#ifdef XMIT_HW_FFT
	printf(" and %u HW_XMIT_FFT", NUM_XMIT_FFT_ACCEL);
#else
	printf(" and NO HW_XMIT_FFT");
#endif
#ifdef RECV_HW_FFT
	printf(" and %u HW_RECV_FFT", NUM_RECV_FFT_ACCEL);
#else
	printf(" and NO HW_RECV_FFT");
#endif
	printf("\n");

	gettimeofday(&stop_prog, NULL);
	uint64_t total_exec = (uint64_t) (stop_prog.tv_sec - start_prog.tv_sec) * 1000000 + (uint64_t) (stop_prog.tv_usec - start_prog.tv_usec);
	uint64_t proc_rdbag = (uint64_t) (proc_rdbag_sec) * 1000000 + (uint64_t) (proc_rdbag_usec);
	uint64_t proc_odo = (uint64_t) (proc_odo_sec) * 1000000 + (uint64_t) (proc_odo_usec);
	uint64_t proc_lidar = (uint64_t) (proc_lidar_sec) * 1000000 + (uint64_t) (proc_lidar_usec);
	uint64_t proc_data = (uint64_t) (proc_data_sec) * 1000000 + (uint64_t) (proc_data_usec);
	uint64_t proc_cv = (uint64_t) (proc_cv_sec) * 1000000 + (uint64_t) (proc_cv_usec);

#ifdef INT_TIME
	uint64_t pd_cloud2grid = (uint64_t) (pd_cloud2grid_sec) * 1000000 + (uint64_t) (pd_cloud2grid_usec);
	uint64_t pd_lz4_cmp = (uint64_t) (pd_lz4_cmp_sec) * 1000000 + (uint64_t) (pd_lz4_cmp_usec);
	uint64_t pd_wifi_pipe = (uint64_t) (pd_wifi_pipe_sec) * 1000000 + (uint64_t) (pd_wifi_pipe_usec);
	uint64_t pd_wifi_send = (uint64_t) (pd_wifi_send_sec) * 1000000 + (uint64_t) (pd_wifi_send_usec);
	uint64_t pd_wifi_send_rl = (uint64_t) (pd_wifi_send_rl_sec) * 1000000 + (uint64_t) (pd_wifi_send_rl_usec);
	uint64_t pd_wifi_send_im = (uint64_t) (pd_wifi_send_im_sec) * 1000000 + (uint64_t) (pd_wifi_send_im_usec);
	uint64_t pd_wifi_recv_th = (uint64_t) (pd_wifi_recv_th_sec) * 1000000 + (uint64_t) (pd_wifi_recv_th_usec);
	uint64_t pd_wifi_lmap_wait = (uint64_t) (pd_wifi_lmap_wait_sec) * 1000000 + (uint64_t) (pd_wifi_lmap_wait_usec);
	uint64_t pd_wifi_recv_wait = (uint64_t) (pd_wifi_recv_wait_sec) * 1000000 + (uint64_t) (pd_wifi_recv_wait_usec);
	uint64_t pd_wifi_recv_all = (uint64_t) (pd_wifi_recv_all_sec) * 1000000 + (uint64_t) (pd_wifi_recv_all_usec);
	uint64_t pd_wifi_recv_rl = (uint64_t) (pd_wifi_recv_rl_sec) * 1000000 + (uint64_t) (pd_wifi_recv_rl_usec);
	uint64_t pd_wifi_recv_im = (uint64_t) (pd_wifi_recv_im_sec) * 1000000 + (uint64_t) (pd_wifi_recv_im_usec);
	uint64_t pd_recv_pipe = (uint64_t) (pd_recv_pipe_sec) * 1000000 + (uint64_t) (pd_recv_pipe_usec);
	uint64_t pd_lz4_uncmp = (uint64_t) (pd_lz4_uncmp_sec) * 1000000 + (uint64_t) (pd_lz4_uncmp_usec);
	uint64_t pd_combGrids = (uint64_t) (pd_combGrids_sec) * 1000000 + (uint64_t) (pd_combGrids_usec);
	uint64_t pd_carSend = (uint64_t) (pd_wifi_car_sec) * 1000000 + (uint64_t) (pd_wifi_car_usec);

	// This is the cloud2grid breakdown
	uint64_t ocgr_cl2g_total = (uint64_t) (ocgr_c2g_total_sec) * 1000000 + (uint64_t) (ocgr_c2g_total_usec);
	uint64_t ocgr_cl2g_initCM = (uint64_t) (ocgr_c2g_initCM_sec) * 1000000 + (uint64_t) (ocgr_c2g_initCM_usec);
	uint64_t ocgr_cl2g_updOrig = (uint64_t) (ocgr_c2g_updOrig_sec) * 1000000 + (uint64_t) (ocgr_c2g_updOrig_usec);
	uint64_t ocgr_cl2g_updBnds = (uint64_t) (ocgr_c2g_updBnds_sec) * 1000000 + (uint64_t) (ocgr_c2g_updBnds_usec);

	uint64_t ocgr_upBd_total = (uint64_t) (ocgr_upBd_total_sec) * 1000000 + (uint64_t) (ocgr_upBd_total_usec);
	uint64_t ocgr_upBd_rayFSp = (uint64_t) (ocgr_upBd_rayFSp_sec) * 1000000 + (uint64_t) (ocgr_upBd_rayFSp_usec);
	uint64_t ocgr_upBd_regObst = (uint64_t) (ocgr_upBd_regObst_sec) * 1000000 + (uint64_t) (ocgr_upBd_regObst_usec);

	/** No need to do this here -- provides no more info than exterior measure, really
		uint64_t ocgr_ryFS_total  = (uint64_t)(ocgr_ryFS_total_sec)  * 1000000 + (uint64_t)(ocgr_ryFS_total_usec);
		uint64_t ocgr_ryFS_rtLine = (uint64_t)(ocgr_ryFS_rtLine_sec) * 1000000 + (uint64_t)(ocgr_ryFS_rtLine_usec); **/

		// This is the xmit_pipe.c breakdown
	uint64_t x_pipe = (uint64_t) (x_pipe_sec) * 1000000 + (uint64_t) (x_pipe_usec);
	uint64_t x_genmacfr = (uint64_t) (x_genmacfr_sec) * 1000000 + (uint64_t) (x_genmacfr_usec);
	uint64_t x_domapwk = (uint64_t) (x_domapwk_sec) * 1000000 + (uint64_t) (x_domapwk_usec);
	uint64_t x_phdrgen = (uint64_t) (x_phdrgen_sec) * 1000000 + (uint64_t) (x_phdrgen_usec);
	uint64_t x_ck2sym = (uint64_t) (x_ck2sym_sec) * 1000000 + (uint64_t) (x_ck2sym_usec);
	uint64_t x_ocaralloc = (uint64_t) (x_ocaralloc_sec) * 1000000 + (uint64_t) (x_ocaralloc_usec);
	uint64_t x_fft = (uint64_t) (x_fft_sec) * 1000000 + (uint64_t) (x_fft_usec);
	uint64_t x_ocycpref = (uint64_t) (x_ocycpref_sec) * 1000000 + (uint64_t) (x_ocycpref_usec);

#ifdef XMIT_HW_FFT
	uint64_t x_fHtotal = (uint64_t) (x_fHtotal_sec) * 1000000 + (uint64_t) (x_fHtotal_usec);
	uint64_t x_fHcvtin = (uint64_t) (x_fHcvtin_sec) * 1000000 + (uint64_t) (x_fHcvtin_usec);
	uint64_t x_fHcomp = (uint64_t) (x_fHcomp_sec) * 1000000 + (uint64_t) (x_fHcomp_usec);
	uint64_t x_fHcvtout = (uint64_t) (x_fHcvtout_sec) * 1000000 + (uint64_t) (x_fHcvtout_usec);
#endif

	// This is the Xmit doMapWork breakdown
	uint64_t xdmw_total = (uint64_t) (xdmw_total_sec) * 1000000 + (uint64_t) (xdmw_total_usec);
	uint64_t xdmw_cnvEnc = (uint64_t) (xdmw_cnvEnc_sec) * 1000000 + (uint64_t) (xdmw_cnvEnc_usec);
	uint64_t xdmw_punct = (uint64_t) (xdmw_punct_sec) * 1000000 + (uint64_t) (xdmw_punct_usec);
	uint64_t xdmw_intlv = (uint64_t) (xdmw_intlv_sec) * 1000000 + (uint64_t) (xdmw_intlv_usec);
	uint64_t xdmw_symbols = (uint64_t) (xdmw_symbls_sec) * 1000000 + (uint64_t) (xdmw_symbls_usec);
	uint64_t xdmw_mapout = (uint64_t) (xdmw_mapout_sec) * 1000000 + (uint64_t) (xdmw_mapout_usec);

	// This is the recv_pipe.c breakdown
	uint64_t r_pipe = (uint64_t) (r_pipe_sec) * 1000000 + (uint64_t) (r_pipe_usec);
	uint64_t r_cmpcnj = (uint64_t) (r_cmpcnj_sec) * 1000000 + (uint64_t) (r_cmpcnj_usec);
	uint64_t r_cmpmpy = (uint64_t) (r_cmpmpy_sec) * 1000000 + (uint64_t) (r_cmpmpy_usec);
	uint64_t r_firc = (uint64_t) (r_firc_sec) * 1000000 + (uint64_t) (r_firc_usec);
	uint64_t r_cmpmag = (uint64_t) (r_cmpmag_sec) * 1000000 + (uint64_t) (r_cmpmag_usec);
	uint64_t r_cmpmag2 = (uint64_t) (r_cmpmag2_sec) * 1000000 + (uint64_t) (r_cmpmag2_usec);
	uint64_t r_fir = (uint64_t) (r_fir_sec) * 1000000 + (uint64_t) (r_fir_usec);
	uint64_t r_div = (uint64_t) (r_div_sec) * 1000000 + (uint64_t) (r_div_usec);
	uint64_t r_sshort = (uint64_t) (r_sshort_sec) * 1000000 + (uint64_t) (r_sshort_usec);
	uint64_t r_slong = (uint64_t) (r_slong_sec) * 1000000 + (uint64_t) (r_slong_usec);
	uint64_t r_fft = (uint64_t) (r_fft_sec) * 1000000 + (uint64_t) (r_fft_usec);
	uint64_t r_eqlz = (uint64_t) (r_eqlz_sec) * 1000000 + (uint64_t) (r_eqlz_usec);
	uint64_t r_decsignl = (uint64_t) (r_decsignl_sec) * 1000000 + (uint64_t) (r_decsignl_usec);
	uint64_t r_descrmbl = (uint64_t) (r_descrmbl_sec) * 1000000 + (uint64_t) (r_descrmbl_usec);

	// This is the receiver Hardware FFT breakdown
#ifdef RECV_HW_FFT
	uint64_t r_fHtotal = (uint64_t) (r_fHtotal_sec) * 1000000 + (uint64_t) (r_fHtotal_usec);
	uint64_t r_fHcvtin = (uint64_t) (r_fHcvtin_sec) * 1000000 + (uint64_t) (r_fHcvtin_usec);
	uint64_t r_fHcomp = (uint64_t) (r_fHcomp_sec) * 1000000 + (uint64_t) (r_fHcomp_usec);
	uint64_t r_fHcvtout = (uint64_t) (r_fHcvtout_sec) * 1000000 + (uint64_t) (r_fHcvtout_usec);
#endif

	// This is the sync_short.c "equalize" breakdown
	uint64_t rssh_total = (uint64_t) (sysh_total_sec) * 1000000 + (uint64_t) (sysh_total_usec);
	uint64_t rssh_search = (uint64_t) (sysh_search_sec) * 1000000 + (uint64_t) (sysh_search_usec);
	uint64_t rssh_frame = (uint64_t) (sysh_frame_sec) * 1000000 + (uint64_t) (sysh_frame_usec);

	// This is the synch_long.c "equalize" breakdown
	uint64_t rslg_total = (uint64_t) (sylg_total_sec) * 1000000 + (uint64_t) (sylg_total_usec);
	uint64_t rslg_firG = (uint64_t) (sylg_firG_sec) * 1000000 + (uint64_t) (sylg_firG_usec);
	uint64_t rslg_search = (uint64_t) (sylg_search_sec) * 1000000 + (uint64_t) (sylg_search_usec);
	uint64_t rslg_outgen = (uint64_t) (sylg_outgen_sec) * 1000000 + (uint64_t) (sylg_outgen_usec);

	// This is the gr_equalizer.c "equalize" breakdown
	uint64_t reql_total = (uint64_t) (reql_total_sec) * 1000000 + (uint64_t) (reql_total_usec);
	uint64_t reql_sym_set = (uint64_t) (reql_symset_sec) * 1000000 + (uint64_t) (reql_symset_usec);
	uint64_t reql_ls_eql = (uint64_t) (reql_lseq_call_sec) * 1000000 + (uint64_t) (reql_lseq_call_usec);
	uint64_t reql_out_sym = (uint64_t) (reql_outsym_sec) * 1000000 + (uint64_t) (reql_outsym_usec);
	uint64_t reql_ds_fld = (uint64_t) (reql_decSF_sec) * 1000000 + (uint64_t) (reql_decSF_usec);

	// This is the ofdm.c decode-signal breakdown
	uint64_t rdec_total = (uint64_t) (rdec_total_sec) * 1000000 + (uint64_t) (rdec_total_usec);
	uint64_t rdec_map_bitr = (uint64_t) (rdec_map_bitr_sec) * 1000000 + (uint64_t) (rdec_map_bitr_usec);
	uint64_t rdec_get_bits = (uint64_t) (rdec_get_bits_sec) * 1000000 + (uint64_t) (rdec_get_bits_usec);
	uint64_t rdec_dec_call = (uint64_t) (rdec_dec_call_sec) * 1000000 + (uint64_t) (rdec_dec_call_usec);
#endif
	printf(" Total workload main-loop : %10lu usec\n", total_exec);
	printf("   Total proc Read-Bag      : %10lu usec\n", proc_rdbag);
	printf("   Total proc Odometry      : %10lu usec\n", proc_odo);
	printf("   Total proc Lidar         : %10lu usec\n", proc_lidar);
	printf("     Total proc Data          : %10lu usec\n", proc_data);
	printf("     Total proc CV          : %10lu usec\n", proc_cv);
#ifdef INT_TIME
	printf("       Total pd cloud2grid      : %10lu usec\n", pd_cloud2grid);
	printf("         Ocgr_Cl2gr Total Time         : %10lu usec\n", ocgr_cl2g_total);
	printf("         Ocgr_Cl2gr InitCM Time        : %10lu usec\n", ocgr_cl2g_initCM);
	printf("         Ocgr_Cl2gr Upd-Origin Time    : %10lu usec\n", ocgr_cl2g_updOrig);
	printf("         Ocgr_Cl2gr Upd-Bounds Time    : %10lu usec\n", ocgr_cl2g_updBnds);
	printf("           Ocgr_UpBnds Total Time        : %10lu usec\n", ocgr_upBd_total);
	printf("           Ocgr_UpBnds Ray_FreeSp Time   : %10lu usec\n", ocgr_upBd_rayFSp);
	/** No need to do this here -- provides no more info than exterior measure, really
		printf("             Ocgr_RyFSp Total Time         : %10lu usec\n", ocgr_ryFS_total);
		printf("             Ocgr_RyFSp RayTrace-Line      : %10lu usec\n", ocgr_ryFS_rtLine); **/
	printf("       Total pd lz4_cmp         : %10lu usec\n", pd_lz4_cmp);
	printf("       Total pd xmit_pipe       : %10lu usec\n", pd_wifi_pipe);
	printf("         X-Pipe Total Time        : %10lu usec\n", x_pipe);
	printf("         X-Pipe GenMacFr Time     : %10lu usec\n", x_genmacfr);
	printf("         X-Pipe doMapWk Time      : %10lu usec\n", x_domapwk);
	printf("           XdoMW Total Time         : %10lu usec\n", xdmw_total);
	printf("           XdoMW ConvEncode Time    : %10lu usec\n", xdmw_cnvEnc);
	printf("           XdoMW Puncture Time      : %10lu usec\n", xdmw_punct);
	printf("           XdoMW Interleave Time    : %10lu usec\n", xdmw_intlv);
	printf("           XdoMW Gen-Symbols Time   : %10lu usec\n", xdmw_symbols);
	printf("           XdoMW Gen-Map-Out Time   : %10lu usec\n", xdmw_mapout);
	printf("         X-Pipe PckHdrGen Time    : %10lu usec\n", x_phdrgen);
	printf("         X-Pipe Chnk2Sym Time     : %10lu usec\n", x_ck2sym);
	printf("         X-Pipe CarAlloc Time     : %10lu usec\n", x_ocaralloc);
	printf("         X-Pipe Xm-FFT Time       : %10lu usec\n", x_fft);
#ifdef XMIT_HW_FFT
	printf("           X-Pipe xHfft_total Time  : %10lu usec\n", x_fHtotal);
	printf("           X-Pipe xHfft_cvtin Time  : %10lu usec\n", x_fHcvtin);
	printf("           X-Pipe xHfft_comp  Time  : %10lu usec\n", x_fHcomp);
	printf("           X-Pipe xHfft_cvtout Time : %10lu usec\n", x_fHcvtout);
#endif
	printf("         X-Pipe CycPrefix Time    : %10lu usec\n", x_ocycpref);
	printf("       Total pd xmit_send       : %10lu usec\n", pd_wifi_send);
	printf("         Total pd xmit_send_rl    : %10lu usec\n", pd_wifi_send_rl);
	printf("         Total pd xmit_send_im    : %10lu usec\n", pd_wifi_send_im);
	printf("       Total pd xmit_recv_th    : %10lu usec\n", pd_wifi_recv_th);
	printf("         Total pd xmit_lmap_wait  : %10lu usec\n", pd_wifi_lmap_wait);
	printf("         Total pd xmit_recv_wait  : %10lu usec\n", pd_wifi_recv_wait);
	printf("         Total pd xmit_recv_all   : %10lu usec\n", pd_wifi_recv_all);
	printf("           Total pd xmit_recv_rl    : %10lu usec\n", pd_wifi_recv_rl);
	printf("           Total pd xmit_recv_im    : %10lu usec\n", pd_wifi_recv_im);
	printf("       Total pd recv_pipe       : %10lu usec\n", pd_recv_pipe);
	printf("         R-Pipe Total Time        : %10lu usec\n", r_pipe);
	printf("         R-Pipe CmplCnjg Time     : %10lu usec\n", r_cmpcnj);
	printf("         R-Pipe CmplMult Time     : %10lu usec\n", r_cmpmpy);
	printf("         R-Pipe FIRC Time         : %10lu usec\n", r_firc);
	printf("         R-Pipe CmplMag Time      : %10lu usec\n", r_cmpmag);
	printf("         R-Pipe CmplMag^2 Time    : %10lu usec\n", r_cmpmag2);
	printf("         R-Pipe FIR Time          : %10lu usec\n", r_fir);
	printf("         R-Pipe DIV Time          : %10lu usec\n", r_div);
	printf("         R-Pipe SyncShort Time    : %10lu usec\n", r_sshort);
	printf("           R-SySht Total Time         : %10lu usec\n", rssh_total);
	printf("           R-SySht Search Time        : %10lu usec\n", rssh_search);
	printf("           R-SySht Frame Time         : %10lu usec\n", rssh_frame);
	printf("         R-Pipe SyncLong Time     : %10lu usec\n", r_slong);
	printf("           R-SyLng Total Time         : %10lu usec\n", rslg_total);
	printf("           R-SyLng FIR-G Time         : %10lu usec\n", rslg_firG);
	printf("           R-SyLng Search Time        : %10lu usec\n", rslg_search);
	printf("           R-SyLng OutGen Time        : %10lu usec\n", rslg_outgen);
	printf("         R-Pipe Rc-FFT Time       : %10lu usec\n", r_fft);
#ifdef RECV_HW_FFT
	printf("           R-Pipe rHfft_total Time  : %10lu usec\n", r_fHtotal);
	printf("           R-Pipe rHfft_cvtin Time  : %10lu usec\n", r_fHcvtin);
	printf("           R-Pipe rHfft_comp  Time  : %10lu usec\n", r_fHcomp);
	printf("           R-Pipe rHfft_cvtout Time : %10lu usec\n", r_fHcvtout);
#endif
	printf("         R-Pipe Equalize Time     :  %10lu usec\n", r_eqlz);
	printf("           R-Eql Total Time         : %10lu usec\n", reql_total);
	printf("           R-Eql Set-Symbol Time    : %10lu usec\n", reql_sym_set);
	printf("           R-Eql LS-EQ Time         : %10lu usec\n", reql_ls_eql);
	printf("           R-Eql Output-Sym Time    : %10lu usec\n", reql_out_sym);
	printf("           R-Eql DecSigFld Time     : %10lu usec\n", reql_ds_fld);
	printf("         R-Pipe DecSignal Time    : %10lu usec\n", r_decsignl);
	printf("           R-Dec Total Time         : %10lu usec\n", rdec_total);
	printf("           R-Dec Map-BitR Time      : %10lu usec\n", rdec_map_bitr);
	printf("           R-Dec Get-Bits Time      : %10lu usec\n", rdec_get_bits);
	printf("           R-Dec Decode Call        : %10lu usec\n", rdec_dec_call);
	printf("         R-Pipe DeScramble Time   : %10lu usec\n", r_descrmbl);
	printf("       Total pd lz4_uncmp       : %10lu usec\n", pd_lz4_uncmp);
	printf("       Total pd combGrids       : %10lu usec\n", pd_combGrids);
	printf("       Total pd carSend         : %10lu usec\n", pd_carSend);
	printf("\n");
#else
	printf(" NO more detailed timing information on this run...\n");
#endif
	printf("\nDone with the run...\n");
}

#undef HPVM

