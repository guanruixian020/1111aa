#include "helpers.h"

//volatile float arrble[]={0,0,0,0,0,0};//进度条
//volatile bool zt[]={false,false,false,false};//true正转，false反转
//volatile int bflag=1;

float mapfloat(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
/*
float minfloat(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (int)(in_max-in_min)/(out_max - out_min)*x;
}
*/
float rateLimit(float currentSample,float lastSample)
{
  float output;
  float dif = currentSample - lastSample;
  float maxRate = .001;
    
  if (dif > 0)
  {
    output = lastSample + maxRate;
  }
  else if(dif < 0)
  {
    output = lastSample - maxRate;
  }

  return output;
}

//驱动AX
/*
void blerawdata(){

  //i=轴的编号
  for(int i=0; i<6; i++)
  {           
      if(arrble[0]<50)
      {
        if(i<2)
        {
          zt[i]=false;
        }
        else if(i>1)
        {
          zt[i]=true;
        }
      }
      else if(arrble[0]>50)
      {
        if(i<2){
          zt[i]=false;
        }
        else if(i>1){
          zt[i]=true;
        }
    }

    if(!bflag)      //前后
    {             
      if(arrble[1]<50)
      {
        if(i<1||i>2){
          zt[i]=false;
        }
        else if(0<i<3){
          zt[i]=true;
        }
      }
      else if(arrble[1]>50)
      {
        if(i<1||i>2){
          zt[i]=false;
        }
        else if(0<i<3){
          zt[i]=true;
        }
      }
    }
  }
}
*/
