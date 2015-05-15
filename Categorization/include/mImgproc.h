/*
 * mImgproc.h
 *
 *  Created on: Dec 6, 2012
 *      Author: fengzh
 */

#ifndef IMGPROCC_H_
#define IMGPROCC_H_

#include "macros.h"

/* read line config from disk file, for each frame.
 * store as point set
 */
void ldFrameLines(const string infile, videoLineStruct& out, int width, int height,int );

/*
 * debug info.
 */
void drawLines(videoLineStruct *vls, const char* cVName);
void drawLines( int frameID, const videoLineStruct &vls, Mat &frame);
/* find nearest lines
 * distance: say, 50 units? (between two middle points of two lines)
 * or just load neighboring lines according to its line id?
 */
vector<vector<vector<float> > > findNeighborLines(vector<vector<vector<float> > > lSet);

// draw lines
//void drawLines(IplImage *dst, float *line, CvScalar color);

// draw each point on the same line
//void drawPoints(IplImage *dst, const vector<CvPoint> &pset, CvScalar color);

// get points on the same line
void getPoints(float *line, int width, int height, vector<CvPoint> &);

void rotateVideo(string viName);

string getimgName(int n);


#endif /* IMGPROCC_H_ */
