/*
  @file FFT.h
  @version: 1.0
  @author: Suky
  @web www.micros-designs.com.ar
  @date 10/02/11
*/
#include "mbed.h"

// Extracted from Numerical Recipes in C
void vFFT(float data[], unsigned int nn);
// Extracted from Numerical Recipes in C
void vRealFFT(float data[], unsigned int n);

void vCalPowerf(float Input[],float Power[], unsigned int n);

void vCalPowerInt(float Input[],unsigned char Power[], unsigned int n);

void vCalPowerLog(float Input[],unsigned char Power[], unsigned int n);
