/*
 * m_OpticalFlow.h
 *
 *  Created on: May 15, 2013
 *      Author: fengzh
 */

#ifndef M_OPTICALFLOW_H_
#define M_OPTICALFLOW_H_

#include "macros.h"

#include "extractForeGround.h"

typedef struct Point6f{
    float x,y,z,w,v, u;
    Point6f(float *f){
        x=f[0],y=f[1],z=f[2],w=f[3],v=f[4], u = f[5];
    }
}Point6f;


// for angle convert

void optiflowDescriptor( int gid, int vid, vector<int>&label, vector<vector<float> > &pfeat, 
        vector<vector<float> > &nfeat);

void EarlyDetection(int gid);
void EarlyRegression(int gid);
void ConfusionMatrix(int gid);

#endif /* M_OPTICALFLOW_H_ */
