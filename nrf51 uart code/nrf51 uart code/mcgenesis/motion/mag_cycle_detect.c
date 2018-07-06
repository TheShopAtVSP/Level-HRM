/*
 * mag_cycle_detect.c
 *
 * Created: 4/7/2015 2:22:40 PM
 *  Author: matt
 */ 

#include "data_crunch.h"

extern bool inv_debug;
extern TMOTION Motion;

int32_t butterworth_bpf( int32_t input );

///
//! \fn
/// 4th order Bandpass IIR Butterworth filter. Fc1 = 0.67Hz, Fc2 = 15Hz
/// Filter design from: http://www-users.cs.york.ac.uk/~fisher/mkfilter/
/// \param
/// \return .
///
#define NZEROS  8
#define NPOLES  8
int32_t x[NZEROS+1], y[NPOLES+1];
int32_t a[NZEROS+1] = { 0, -0.0241687697*1024, -0.0272320121*1024, -0.1525263087*1024, 0.9375233839*1024,
						-1.3709797052*1024, 1.8765541669*1024, -3.2914862515*1024, 3.0521835563*1024 };
int32_t b[NPOLES+1] = { 1, 0, -4, 0, 6, 0, -4, 0, 1 };
const uint32_t GAIN = 6.916374639*1024;
int32_t butterworth_bpf( int32_t input )
{
	static uint8_t ptr = 0;
	uint8_t temp;
	
	//fc1 = 0.67Hz, fc2 = 15Hz
	if( ++ptr > NPOLES ) ptr = 0;
	x[ptr] = 1024*input/GAIN;
	
	temp = ptr;
	y[ptr] = 0;
	for(int i=0; i<= NPOLES; i++)
	{
		y[ptr] += y[temp]*a[i]/1024;
		y[ptr] += x[temp]*b[i];
		if( ++temp > NPOLES ) temp = 0;
	}
	
	return y[ptr];
}

