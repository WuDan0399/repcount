/*
  @file FFT.cpp
  @version: 1.0
  @author: Suky
  @web www.micros-designs.com.ar
  @date 10/02/11
*/
#include "FFT.h"

// Extracted from Numerical Recipes in C
void vFFT(float data[], unsigned int nn){
/*Replaces data[1..2*nn] by its discrete Fourier transform, if isign is input as 1; or replaces
data[1..2*nn] by nn times its inverse discrete Fourier transform, if isign is input as -1.
data is a complex array of length nn or, equivalently, a real array of length 2*nn. nn MUST
be an integer power of 2 (this is not checked for!).*/
    unsigned int n,mmax,m,j,istep,i;
    double wtemp,wr,wpr,wpi,wi,theta; 
    float tempr,tempi;
    
    #define SWAP(a,b) tempr=(a);(a)=(b);(b)=tempr
    
    n=nn << 1;
    j=1;
    for (i=1;i<n;i+=2) { 
        if(j>i){
            SWAP(data[j],data[i]); 
            SWAP(data[j+1],data[i+1]);
        }
        m=n >> 1;
        while (m >= 2 &&j>m){
            j-=m;
            m >>= 1;
        }
        j+=m;
    }
    
    mmax=2;
    while (n > mmax) { 
        istep=mmax << 1;
        theta=(6.28318530717959/mmax); 
        wtemp=sin(0.5*theta);
        wpr = -2.0*wtemp*wtemp;
        wpi=sin(theta);
        wr=1.0;
        wi=0.0;
        for (m=1;m<mmax;m+=2) { 
            for (i=m;i<=n;i+=istep) {
                j=i+mmax; 
                tempr=wr*data[j]-wi*data[j+1];
                tempi=wr*data[j+1]+wi*data[j];
                data[j]=data[i]-tempr;
                data[j+1]=data[i+1]-tempi;
                data[i] += tempr;
                data[i+1] += tempi;
            }
            wr=(wtemp=wr)*wpr-wi*wpi+wr;
            wi=wi*wpr+wtemp*wpi+wi;
        }
        mmax=istep;
    }
}

// Extracted from Numerical Recipes in C
void vRealFFT(float data[], unsigned int n){
/*Calculates the Fourier transform of a set of n real-valued data points. Replaces this data (which
is stored in array data[1..n]) by the positive frequency half of its complex Fourier transform.
The real-valued rst and last components of the complex transform are returned as elements
data[1] and data[2], respectively. n must be a power of 2. This routine also calculates the
inverse transform of a complex data array if it is the transform of real data. (Result in this case
must be multiplied by 2/n.)*/
    unsigned long i,i1,i2,i3,i4,np3;
    float c1=0.5,c2,h1r,h1i,h2r,h2i;
    double wr,wi,wpr,wpi,wtemp,theta; 
    theta=3.141592653589793/(double) (n>>1); 
    
    c2 = -0.5;
    vFFT(data,n>>1); 
    wtemp=sin(0.5*theta);
    wpr = -2.0*wtemp*wtemp;
    wpi=sin(theta);
    wr=1.0+wpr;
    wi=wpi;
    np3=n+3;
    for (i=2;i<=(n>>2);i++) {         
        i4=1+(i3=np3-(i2=1+(i1=i+i-1)));
        h1r=c1*(data[i1]+data[i3]); 
        h1i=c1*(data[i2]-data[i4]);
        h2r = -c2*(data[i2]+data[i4]);
        h2i=c2*(data[i1]-data[i3]);
        data[i1]=h1r+wr*h2r-wi*h2i; 
        data[i2]=h1i+wr*h2i+wi*h2r;
        data[i3]=h1r-wr*h2r+wi*h2i;
        data[i4] = -h1i+wr*h2i+wi*h2r;
        wr=(wtemp=wr)*wpr-wi*wpi+wr; 
        wi=wi*wpr+wtemp*wpi+wi;
    }
    data[1] = (h1r=data[1])+data[2]; 
    data[2] = h1r-data[2];

}


void vCalPowerf(float Input[],float Power[], unsigned int n){
    unsigned char k,j;
    
    for(k=0,j=0;k<n;k++,j+=2){
        Power[k]=sqrt(Input[j]*Input[j]+Input[j+1]*Input[j+1]);
    }    
}

void vCalPowerInt(float Input[],unsigned char Power[], unsigned int n){
    unsigned char k,j;
    
    for(k=0,j=0;k<n;k++,j+=2){
        Power[k]=sqrt(Input[j]*Input[j]+Input[j+1]*Input[j+1]);
    }    
}

void vCalPowerLog(float Input[],unsigned char Power[], unsigned int n){
    unsigned char k,j;
    float Temp;
    
    for(k=0,j=0;k<n;k++,j+=2){
        if((Input[j]!=0) && (Input[j+1]!=0)){
            Temp=sqrt(Input[j]*Input[j]+Input[j+1]*Input[j+1]);
            Power[k]=10.0*log10(Temp);
        }else{
            Power[k]=0;
        }    
    }

}
