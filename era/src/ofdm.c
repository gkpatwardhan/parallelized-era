#include <complex.h>
//#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/time.h>

/* #ifndef DEBUG_MODE */
/*  #define DEBUG_MODE */
/* #endif */
#include "debug.h"

#include "sdr_type.h"
#include "sdr_base.h"
#include "ofdm.h"
#include "sdr_viterbi.h"
#include "globalsSDRViterbi.h"

// typedef ap_fixed<32,15> fx_pt1_ext1;
// typedef ap_fixed<64,16> fx_pt1_ext2;
// typedef complex< fx_pt1_ext1 > fx_pt_ext1;
// typedef complex< fx_pt1_ext2 > fx_pt_ext2;
//

#define HPVM_OFDM

#if defined(HPVM)
#include "hpvm.h"
#include "hetero.h"
#endif

#undef HPVM

//extern int d_frame_bytes;
//extern int d_frame_encoding;

#ifdef INT_TIME
/* This is RECV-DECODE (SIGNAL) internal Timing information (gathering resources) */
struct timeval rdec_total_stop, rdec_total_start;
uint64_t rdec_total_sec  = 0LL;
uint64_t rdec_total_usec = 0LL;

struct timeval rdec_map_bitr_stop, rdec_map_bitr_start;
uint64_t rdec_map_bitr_sec  = 0LL;
uint64_t rdec_map_bitr_usec = 0LL;

struct timeval rdec_get_bits_stop, rdec_get_bits_start;
uint64_t rdec_get_bits_sec  = 0LL;
uint64_t rdec_get_bits_usec = 0LL;

struct timeval rdec_dec_call_stop, rdec_dec_call_start;
uint64_t rdec_dec_call_sec  = 0LL;
uint64_t rdec_dec_call_usec = 0LL;
#endif


void decode_signal_start_Wrapper(unsigned* num_inputs, size_t num_inputs_sz /*= sizeof(unsigned)*/,
		fx_pt* constellation, size_t constellation_sz /*= DECODE_IN_SIZE_MAX*/, 
		unsigned* num_outputs, size_t num_outputs_sz /*= sizeof(unsigned)*/,
		uint8_t * output_data, size_t output_data_sz /*= MAX_ENCODED_BITS * 3 / 4*/,
		/*Local variablse used by decode_signal*/
		uint8_t* bit_r, size_t bit_r_sz /*= DECODE_IN_SIZE_MAX*/,
		uint8_t* bit, size_t bit_sz /*= DECODE_IN_SIZE_MAX + OFDM_PAD_ENTRIES*/,
		ofdm_param* ofdm, size_t ofdm_sz /*= sizeof(ofdm_param)*/,
		frame_param* frame, size_t frame_sz /*= sizeof(frame_param)*/) {

#if defined(HPVM) && defined(HPVM_OFDM)
	void* Section = __hetero_section_begin();
	void* T = __hetero_task_begin(8, num_inputs, num_inputs_sz, constellation, constellation_sz, num_outputs, num_outputs_sz, 
			output_data, output_data_sz, bit_r, bit_r_sz, bit, bit_sz, ofdm, ofdm_sz, frame, frame_sz, 
			8, num_inputs, num_inputs_sz, constellation, constellation_sz, num_outputs, num_outputs_sz, 
			output_data, output_data_sz, bit_r, bit_r_sz, bit, bit_sz, ofdm, ofdm_sz, frame, frame_sz, 
			"decode_signal_start_task");
#endif

	{
		// JDW : REPLACING ALL of this with our viterbi_flat
		// ap_uint<1> bit_r[CHUNK];
		// ap_uint<1> bit[CHUNK];
		unsigned num_sym = (*num_inputs)/48;
		//  uint8_t bit_r[DECODE_IN_SIZE_MAX];
		//  uint8_t bit[DECODE_IN_SIZE_MAX + OFDM_PAD_ENTRIES]; // This is oversize becuase decode uses extra space (?)

		DEBUG(printf("In the decode_signal routine with num_inputs = %u\n", *num_inputs));
#ifdef INT_TIME
		gettimeofday(&rdec_total_start, NULL);
#endif
		// map to the nearest one
		for(unsigned i = 0; i < *num_inputs /*DECODE_IN_SIZE_MAX*/;i++) {
			if( crealf(constellation[i]) > 0 ) {
				bit_r[i] = 1;
			} else {
				bit_r[i] = 0;
			}
			DEBUG(printf(" OFDM_BIT_R %5u : CONS %12.8f %12.8f : BIT_R %u\n", i, crealf(constellation[i]), cimagf(constellation[i]), bit_r[i]));
		}
#ifdef INT_TIME
		gettimeofday(&rdec_map_bitr_stop, NULL);
		rdec_map_bitr_sec  += rdec_map_bitr_stop.tv_sec  - rdec_total_start.tv_sec;
		rdec_map_bitr_usec += rdec_map_bitr_stop.tv_usec - rdec_total_start.tv_usec;
#endif

		DEBUG(printf("     at the interleaving...\n"));
		// interleaving
		const unsigned inter[]=
		{ 0,3,6,9,12,15,18,21,24,27,30,33,36,
			39,42,45,1,4,7,10,13,16,19,22,25,28,
			31,34,37,40,43,46,2,5,8,11,14,17,20,23,
			26,29,32,35,38,41,44,47 };

		for (unsigned i = 0; i < num_sym; i++) {
			for (unsigned j = 0; j < 48; j++) {
				unsigned index = inter[j]+i*48;
				bit[ j+i*48 ] = bit_r[index];
				DEBUG(printf(" OFDM_BIT %5u : BIT %5u = BIR_R %5u = %u\n", i, (j+i*48), index, bit[j+i*48]));
			}
		}
		// Initialize the pad entries
		for (int ti = 0; ti < OFDM_PAD_ENTRIES; ti++) {
			bit[48*num_sym + ti] = 0;
		}
#ifdef INT_TIME
		gettimeofday(&rdec_get_bits_stop, NULL);
		rdec_get_bits_sec  += rdec_get_bits_stop.tv_sec  - rdec_map_bitr_stop.tv_sec;
		rdec_get_bits_usec += rdec_get_bits_stop.tv_usec - rdec_map_bitr_stop.tv_usec;
#endif

		DEBUG(printf("     at the call to our decode...\n"));
		unsigned num_out_bits = *num_inputs/2; 			// for BPSK_1_2

		ofdm->encoding =  0; //d_frame_encoding; 	  	// encoding   : 0 = BPSK_1_2
		ofdm->rate_field = 13; 					// rate_field : rate field of SIGNAL header //Taken constant
		ofdm->n_bpsc = 1;        				// n_bpsc     : coded bits per subcarrier
		ofdm->n_cbps = 48;       				// n_cbps     : coded bits per OFDM symbol
		ofdm->n_dbps = 24;     					// n_dbps     : data bits per OFDM symbol

		frame->psdu_size = 0; //d_frame_bytes; 			// psdu_size      : PSDU size in bytes
		// Not sure if the above assignment will lead to issues; psdu_size isn't really being used any where
		// and I couldn't figure out where d_frame_bytes was getting updated. So, I am just initializing this variable
		// to some constant value and trying to see how it works.
		
		frame->n_sym = (int)(num_sym);				// n_sym          : number of OFDM symbols
		frame->n_pad = 18;					// n_pad          : number of padding bits in DATA field
		frame->n_encoded_bits = (int)*num_inputs; //24528	// n_encoded_bits : number of encoded bits
		frame->n_data_bits = (int)num_out_bits;   //12264	// n_data_bits    : number of data bits, including service and padding
		printf("Calling decode with OFDM_PARMS %u %2u %u %2u %2u\n", ofdm->encoding, 13, ofdm->n_bpsc, ofdm->n_cbps, ofdm->n_dbps);
				printf("               and FRAME_PARMS %4u %3u %2u %5u %5u\n",  frame->psdu_size, frame->n_sym, frame->n_pad, frame->n_encoded_bits, frame->n_data_bits);
		/* DEBUG(printf("Calling decode : n_inputs = %u \n", num_inputs); */
		/* 	  printf("OFDM : enc %u   rate %u  n_bpsc %u  n_cbps %u  n_dbps %u\n", ofdm->encoding, ofdm->rate_field, ofdm->n_bpsc, ofdm->n_cbps, ofdm->n_dbps); */
		/* 	  printf("FRAME: psdu %u  n_sym %u  n_pad %u  n_encb %u  n_dtab %u\n", frame->psdu_size, frame->n_sym, frame->n_pad, frame->n_encoded_bits, frame->n_data_bits)); */
		// Always use hardware accelerator IF available...
		printf("Calling sdr_decode_ofdm from ofdm.c\n");
	}

#if defined(HPVM) && defined(HPVM_OFDM)
	__hetero_task_end(T);
	__hetero_section_end(Section);
#endif
}

void decode_signal_end_Wrapper(unsigned* num_inputs, size_t num_inputs_sz /*= sizeof(unsigned)*/,
		unsigned* num_outputs, size_t num_outputs_sz /*= sizeof(unsigned)*/) {
	
#if defined(HPVM) && defined(HPVM_OFDM) 
	void* Section = __hetero_section_begin();
	void* T = __hetero_task_begin(2, num_inputs, num_inputs_sz, num_outputs, num_outputs_sz, 
					1, num_outputs, num_outputs_sz, "decode_signal_end_task");
#endif
	{
		unsigned num_out_bits = (*num_inputs)/2; // for BPSK_1_2 // HPVM: Copied from T1 as num_inputs nor num_out_bits is modified in T1/T2

		// end of decode (viterbi) function, but result bits need to be "descrambled"
		*num_outputs = num_out_bits;
#ifdef INT_TIME
		gettimeofday(&rdec_dec_call_stop, NULL);
		rdec_dec_call_sec  += rdec_dec_call_stop.tv_sec  - rdec_get_bits_stop.tv_sec;
		rdec_dec_call_usec += rdec_dec_call_stop.tv_usec - rdec_get_bits_stop.tv_usec;

		rdec_total_sec  += rdec_dec_call_stop.tv_sec  - rdec_total_start.tv_sec;
		rdec_total_usec += rdec_dec_call_stop.tv_usec - rdec_total_start.tv_usec;
#endif

		DEBUG(printf("  done and leaving ofdm.c\n"));
	}

#if defined(HPVM) && defined(HPVM_OFDM) 
	__hetero_task_end(T);
	__hetero_section_end(Section);
#endif
}


void decode_signal(unsigned* num_inputs, size_t num_inputs_sz /*= sizeof(unsigned)*/,
		fx_pt* constellation, size_t constellation_sz /*= DECODE_IN_SIZE_MAX*/, 
		unsigned* num_outputs, size_t num_outputs_sz /*= sizeof(unsigned)*/,
		uint8_t * output_data, size_t output_data_sz /*= MAX_ENCODED_BITS * 3 / 4*/,
		/*Local variablse used by decode_signal*/
		uint8_t* bit_r, size_t bit_r_sz /*= DECODE_IN_SIZE_MAX*/,
		uint8_t* bit, size_t bit_sz /*= DECODE_IN_SIZE_MAX + OFDM_PAD_ENTRIES*/,
		ofdm_param* ofdm, size_t ofdm_sz /*= sizeof(ofdm_param)*/,
		frame_param* frame, size_t frame_sz /*= sizeof(frame_param)*/,
		int* n_res_char, size_t n_res_char_sz /*= sizeof(int)*/,
		 /* Local variables for sdr_decode_ofdm*/
                uint8_t* inMemory, size_t inMemory_sz /*= 24852*/,
                uint8_t* outMemory, size_t outMemory_sz /*18585*/,
                int* d_ntraceback_arg, size_t d_ntraceback_arg_sz /*= sizeof(int)*/
		) // hls::stream< ap_uint<1> > &output_data  )
{

#if defined(HPVM) && defined(HPVM_OFDM)
	void* Section = __hetero_section_begin();
#endif

#if defined(HPVM) && defined(HPVM_OFDM)
	void* T1 = __hetero_task_begin(8, num_inputs, num_inputs_sz, constellation, constellation_sz, num_outputs, num_outputs_sz, 
			output_data, output_data_sz, bit_r, bit_r_sz, bit, bit_sz, ofdm, ofdm_sz, frame, frame_sz, 
			8, num_inputs, num_inputs_sz, constellation, constellation_sz, num_outputs, num_outputs_sz, 
			output_data, output_data_sz, bit_r, bit_r_sz, bit, bit_sz, ofdm, ofdm_sz, frame, frame_sz, 
			"decode_signal_start_task");
#endif

	{
		decode_signal_start_Wrapper(num_inputs, num_inputs_sz, constellation, constellation_sz, num_outputs, num_outputs_sz,
                        output_data, output_data_sz, bit_r, bit_r_sz, bit, bit_sz, ofdm, ofdm_sz, frame, frame_sz);
	}

#if defined(HPVM) && defined(HPVM_OFDM)
	__hetero_task_end(T1);
#endif


#if defined(HPVM) && defined(HPVM_OFDM) 
	void* T2 = __hetero_task_begin(8, ofdm, ofdm_sz, frame, frame_sz, bit, bit_sz, n_res_char, n_res_char_sz, 
					output_data, output_data_sz, inMemory, inMemory_sz, outMemory, outMemory_sz,
					d_ntraceback_arg, d_ntraceback_arg_sz, 
					5, ofdm, ofdm_sz, frame, frame_sz, bit, bit_sz, n_res_char, n_res_char_sz, 
					output_data, output_data_sz, "sdr_decode_task");
#endif	
	{
		sdr_decode_ofdm(ofdm, ofdm_sz, frame, frame_sz, bit /*input_data*/, bit_sz, n_res_char, n_res_char_sz, 
				output_data, output_data_sz, inMemory, inMemory_sz, outMemory, outMemory_sz,
				d_ntraceback_arg, d_ntraceback_arg_sz);
	}
#if defined(HPVM) && defined(HPVM_OFDM) 
	__hetero_task_end(T2);
#endif

#if defined(HPVM) && defined(HPVM_OFDM) 
	void* T3 = __hetero_task_begin(2, num_inputs, num_inputs_sz, num_outputs, num_outputs_sz, 
					1, num_outputs, num_outputs_sz, "decode_signal_end_task");
#endif
	{
		decode_signal_end_Wrapper(num_inputs, num_inputs_sz, num_outputs, num_outputs_sz);
	}

#if defined(HPVM) && defined(HPVM_OFDM) 
	__hetero_task_end(T3);
#endif


#if defined(HPVM) && defined(HPVM_OFDM)
	__hetero_section_end(Section);
#endif

}

