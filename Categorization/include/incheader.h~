/*
 * incheader.h
 *
 *  Created on: Dec 6, 2012
 *      Author: fengzh
 */

#ifndef INCHEADER_H_
#define INCHEADER_H_


// c
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <time.h>
#include <functional>

// for string s = boost::lexical_cast<string>( number );
#include <boost/lexical_cast.hpp>

// for gsl
#include <gsl/gsl_sf.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

// cv
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/gpu/gpu.hpp>

// std
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stack>
#include <algorithm>
#include <iomanip>
#include <time.h>

/////////////////////////////////////////////////////////////
//
// others
/////////////////////////////////////////////////////////////

using namespace std;
using namespace cv;

/////////////////////////////////////////////////////////////
//
// others
/////////////////////////////////////////////////////////////
#define CLIP2(minv, maxv, value) (min(maxv, max(minv, value)))
#define MAX_CORNERPOINT_NUM 1000 // max number of detected corner pts
#define T_SMALLEST_EIG 	60 // thres. for the smallest eigenvalue method
#define W_SIZE 			7  // window size used in corner detection
#define EUC_DISTANCE 	10 // thres. for Euclidean distance for uniquness_corner
#define B_SIZE 			30 // size for excluding boundary pixel
#define W_SIZE_MATCH 	30 // window size used in NCC
#define T_DIST 		 	30 // thres. for distance in RANSAC algorithm
#define PDISTTHRES   	30 // THRES FOR SELECTING NEIGHBORING POINTS WHEH MATCHING


/**********************************************************************
* MACRO DEFINITION FOR KD-TREE
*/
/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 200

/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.49

#define NEIGHTHBORDIST 50


////////////////////////////////////////////////////////////
//
// for vector writer
////////////////////////////////////////////////////////////
enum VEC_TYPE{ CAMLEFT, CAMRIGHT, CAMUP, CAMDOWN, CAMZOOMIN, CAMZOOMOUT, CAMSTILL, BACKGROUND };
// macro definition
#define MAX_DTW_COST 1e10
//#define MAX(a,b)( a > b? a:b)
//#define MIN(a,b)( a < b? a:b)
struct point3
{
	point3( CvPoint p, int Z)
	{
		x = p.x;
		y = p.y;
		z = Z;
	}

	point3( CvPoint p)
	{
		x = p.x;
		y = p.y;
		z = 1;
	}
	point3 ( )
	{
		x = y = z = 0;
	}

	int x;
	int y;
	int z;
};
struct pathNode
{
	pathNode(Point P, float w, float tw)
	{
		p = P;
		weight = w;
		totalWeight = tw;
	}

	bool operator < ( const pathNode p) const
	{
		return ( weight < p.weight);
	}

	Point p;
	float weight;
	float totalWeight;
};

// for line storage
struct singleLine
{
	singleLine(int id, float ftheta, float ftho)
	{
		lineId 	= id;
		theta 	= ftheta;
		tho   	= ftho;
	}

	singleLine()
	{
		lineId 	= 0;
		theta 	= .0f;
		tho   	= .0f;
	}

	vector<CvPoint> pointSet;
	float theta, tho;
	int lineId;
};

// for each frame
struct frameLine
{
	frameLine(int num_line, int fid, vector<singleLine> set)
	{
		numLines = num_line;
		frameId  = fid;
		lineSet  = set;
	}
	frameLine( )
	{
		numLines = 0;
		frameId = 1;
	}

	int numLines;
	int frameId;
	vector<singleLine> lineSet;
};

// for each pair's dtw path storage
struct dtwPairCost
{
	dtwPairCost()
	{
		lid1 = lid2 = 0;
	}
	int lid1;
	int lid2;
	vector<pathNode> path;
	bool operator == (const CvPoint p) const
	{
		return ( p.y == lid1 && p.x == lid2);
	}
};
// for dtw matrix construction
struct videoLineStruct
{
	int maxLineId;
	vector<frameLine> videoLineSet;
};

// for each pair. coordinate and cost
struct dtwPairPoint
{
	dtwPairPoint()
	{
		cost = .0f;
	}
	dtwPairPoint( point3 P1, point3 P2, float C)
	{
		p1 = P1;
		p2 = P2;
		cost = C;
	}
	bool operator < ( const dtwPairPoint dpp) const
	{
		return cost < dpp.cost;
	}
	point3 p1, p2;
	float cost;
};

inline void CrossProduct( vector<int> v1, vector<int> v2, vector<int> &out)
{
	out[0] = v1[1] * v2[2] - v1[2] * v2[1];
	out[1] = v1[2] * v2[0] - v1[0] * v2[2];
	out[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 4-d array
struct int4
{

	int a[4];


	int4()
	{
		a[0] = a[1] = a[2] = a[3] = 0;
	}

	int4( int a0, int a1, int a2, int a3)
	{
		a[0] = a0;
		a[1] = a1;
		a[2] = a2;
		a[3] = a3;
	}
};

// for curve fitting
struct idxpt
{
	idxpt( double v, int id)
	{
		val = v;
		idx = id;
	}
	idxpt( )
	{
		val = 0;
		idx = 0;
	}
	double val;
	int idx;
};
#endif /* INCHEADER_H_ */
