//-----------------------------------------------------------------------------
// misc.h
//-----------------------------------------------------------------------------
// Copyright 2013, Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Created on: May 6, 2013
//     Author: sgghang
//-----------------------------------------------------------------------------


#ifndef MISC_H_
#define MISC_H_



// Macro to calculate integer division rounded to nearest integer
//
#define ROUND_DIV(N, D)     ( ( (N) + ((D)>>1) )/(D) )

// Macro to calculate integer remainder rounded to nearest integer
#define ROUND_REM(N, D)     ( ( (N) + ((D)>>1) ) % (D) )


// Macro to calculate fractional value scale to 65536, given
// numerator N and denominator/divider D
//
// K = INT(N/D)*65536 + REM(N/D)*65536/D
//
// The purpose of this is to simplify the calculations like:
//
// y = x * N/D to become
//
// y = x * K/65536

#define SCALE_65536(N, D)   ( ( (U32)((N)/(D)) << 16 ) + \
                              ROUND_DIV( ((U32)((N)%(D)) << 16), (D) ) )


#endif // MISC_H_
