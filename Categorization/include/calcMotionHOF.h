/*
 * calcMotion_HOF.h
 *
 *  Created on: May 27, 2013
 *      Author: fengzh
 */

#ifndef CALCMOTION_HOF_H_
#define CALCMOTION_HOF_H_

#include "macros.h"
#include "mImgproc.h"
#include "opticalFlow.h"

#define TRAINRATIO 5

void estiCamMotionHOF(const vector<Mat> &frames);

void motionRecogHOF( );

void motionRecogHOF(int gid, int vid);


#endif /* CALCMOTION_HOF_H_ */
