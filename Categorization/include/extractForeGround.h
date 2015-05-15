/*
 * findForeGround.h
 *
 *  Created on: Mar 5, 2013
 *      Author: fengzh
 */

#ifndef EXTRACTFOREGROUND_H_
#define EXTRACTFOREGROUND_H_

#include "incheader.h"

// for each scan
// warp length
#define SC_AV_WARP_SIZE 1		// sliding window warp size
#define NUMPROJFRAME 10			// sliding window unit
#define NUMCASCADE 1			// total # of slides
#define INITPSIZEH 8			// patch size
#define INITPSIZEW 8
#define TOPTHRES_SCALAR 0.3	// threshold for each frame difference
#define PROJ_MODE_VAL 1

void findForeGround ( const vector<Mat> &frames,
					  vector<vector<Rect> > &Blob,
						vector<Point2f> &optiPt,
					  vector<vector<bool> >&Label,
					  Mat &mask);

void getSIFTHomoMat(const Mat &fir, const Mat &sec, Mat &H, bool sc = false, double scalar = 1);

void descriptorSIFT(const Mat &src,
					vector<KeyPoint> &keypoint,
					Mat &descriptor,
					bool sc = false,
					double scalar = 1);
#endif /* EXTRACTFOREGROUND_H_ */
