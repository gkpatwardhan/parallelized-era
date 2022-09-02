#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>
#include <math.h>
#include <unistd.h>
#include <assert.h>

/* This is XMIT PIPE internal Timing information (gathering resources) */
extern struct timeval x_pipe_stop, x_pipe_start;
extern uint64_t x_pipe_sec;
extern uint64_t x_pipe_usec;

extern struct timeval x_genmacfr_stop, x_genmacfr_start;
extern uint64_t x_genmacfr_sec;
extern uint64_t x_genmacfr_usec;

extern struct timeval x_domapwk_stop, x_domapwk_start;
extern uint64_t x_domapwk_sec;
extern uint64_t x_domapwk_usec;

extern struct timeval xdmw_total_stop, xdmw_total_start;
extern uint64_t xdmw_total_sec;
extern uint64_t xdmw_total_usec;

extern struct timeval xdmw_genDF_stop, xdmw_genDF_start;
extern uint64_t xdmw_genDF_sec;
extern uint64_t xdmw_genDF_usec;

extern struct timeval xdmw_scrmbl_stop, xdmw_scrmbl_start;
extern uint64_t xdmw_scrmbl_sec;
extern uint64_t xdmw_scrmbl_usec;

extern struct timeval xdmw_cnvEnc_stop, xdmw_cnvEnc_start;
extern uint64_t xdmw_cnvEnc_sec;
extern uint64_t xdmw_cnvEnc_usec;

extern struct timeval xdmw_punct_stop, xdmw_punct_start;
extern uint64_t xdmw_punct_sec;
extern uint64_t xdmw_punct_usec;

extern struct timeval xdmw_intlv_stop, xdmw_intlv_start;
extern uint64_t xdmw_intlv_sec;
extern uint64_t xdmw_intlv_usec;

extern struct timeval xdmw_symbls_stop, xdmw_symbls_start;
extern uint64_t xdmw_symbls_sec;
extern uint64_t xdmw_symbls_usec;

extern struct timeval xdmw_mapout_stop, xdmw_mapout_start;
extern uint64_t xdmw_mapout_sec;
extern uint64_t xdmw_mapout_usec;

extern struct timeval x_phdrgen_stop, x_phdrgen_start;
extern uint64_t x_phdrgen_sec;
extern uint64_t x_phdrgen_usec;

extern struct timeval x_ck2sym_stop, x_ck2sym_start;
extern uint64_t x_ck2sym_sec;
extern uint64_t x_ck2sym_usec;

extern struct timeval x_ocaralloc_stop, x_ocaralloc_start;
extern uint64_t x_ocaralloc_sec;
extern uint64_t x_ocaralloc_usec;

extern struct timeval x_fft_stop, x_fft_start;
extern uint64_t x_fft_sec;
extern uint64_t x_fft_usec;

extern struct timeval x_ocycpref_stop, x_ocycpref_start;
extern uint64_t x_ocycpref_sec;
extern uint64_t x_ocycpref_usec;

extern uint8_t d_psdu_org[MAX_PSDU_SIZE];

extern uint8_t d_map_out_copy_org[32768];

extern uint16_t d_seq_nr_org; 
extern uint8_t d_scrambler_org; 
extern char d_symbols_org[24528]; 
extern int d_symbols_offset_org; 
extern int d_symbols_len_org; 


extern ofdm_param d_ofdm_org; 
extern frame_param d_frame_org; 

#define d_num_pilot_carriers 1
#define d_size_pilot_carriers_val 4
extern int d_pilot_carriers_org[d_num_pilot_carriers][d_size_pilot_carriers_val]; /* = {
	{
		-21, -7, 7, 21
	}
};*/

#define d_num_occupied_carriers 1
#define d_size_occupied_carriers 48
extern int d_occupied_carriers_org[d_num_occupied_carriers][d_size_occupied_carriers]; /* = {
        {
                -26, -25, -24, -23, -22, -20, -19, -18, //  8
                -17, -16, -15, -14, -13, -12, -11, -10, // 16
                -9, -8, -6, -5, -4, -3, -2, -1, // 24
                1, 2, 3, 4, 5, 6, 8, 9, // 32
                10, 11, 12, 13, 14, 15, 16, 17, // 40
                18, 19, 20, 22, 23, 24, 25, 26
        }
}; // 48*/

extern size_t d_psdu_org_size; // = MAX_PSDU_SIZE;
extern size_t d_map_out_org_size; // = 32768;

extern size_t d_seq_nr_org_sz; // = sizeof(uint16_t);
extern size_t d_scrambler_org_sz; // = sizeof(uint8_t);
extern size_t d_symbols_org_sz; // = 24528;
extern size_t d_symbols_offset_org_sz; // = sizeof(int);
extern size_t d_symbols_len_org_sz; // = sizeof(int);

extern size_t d_ofdm_org_sz; // = sizeof(ofdm_param);
extern size_t d_frame_org_sz; // = sizeof(frame_param);

extern size_t d_pilot_carriers_org_sz; // = d_num_pilot_carriers*d_size_pilot_carriers_val*sizeof(int);
extern size_t d_occupied_carriers_org_sz; // = d_num_occupied_carriers*d_size_occupied_carriers*sizeof(int);

