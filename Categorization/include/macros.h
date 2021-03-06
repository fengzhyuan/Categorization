/*
 * macros.h
 *
 *  Created on: May 27, 2013
 *      Author: fengzh
 */

#ifndef MACROS_H_
#define MACROS_H_

#include "incheader.h"

/***********************************************************
 * macro definition.
 */

#define NCLUSTER 500 // cluster size for feature engineering
#define DATASET_SIZE 100 // dataset size
// ratio of pos and neg
//#define TRAINRATIO 5

//#define TESTBLOCKSIZE 50

/***********************************************************
 * func.
 */

void playVideos();

void loadVideo( const string &capName, vector<Mat> &frame,int &vlen, int &width, int &height);

void getPtCoord( int width, int height, vector<Point2f> &pts);

void ldVP( int seq, int vlen,const string &rtPath, const string &prefix, const string&pofix,vector<Point2f> &vp);

void ldKickingAnno( vector<Point2i> &kick);
void ldLabel(int, vector<int>&);
string ldFolder( int n , int,bool addPrefix, const string &rtPath, const string &r, const string &pofix );


// for classifier training
void calcFeatConsecuFrame(const vector<Mat> &frames,const vector<Point2f> &ptSet,	const videoLineStruct &vls,	vector<vector<float> >&feat );

void estiCamMotion_Learn(const vector<Mat> &frames);

void featVec2Mat( const vector<vector<vector<float> > > &feat_pos,
        const vector<vector<vector<float> > > &feat_neg,
        const vector<vector<vector<float> > > &test_pos,
        const vector<vector<vector<float> > > &test_neg,
        Mat &labelMat,Mat &instanceMat,Mat &center);

void calcTestFeat( const vector<vector<vector<float> > > &feat,const vector<vector<vector<float> > > &nfeat, const Mat &center,Mat &instanceMat,bool pos_neg);

void predictMotionClassifier( const CvSVM &svm, const Mat &label, const Mat &instance);

void calcTestFeat ( const vector<vector<vector<float> > > &pfeat,
						const Mat &center,
						Mat &instanceMat);

void calcTestFeatPOS ( const vector<vector<vector<float> > > &pfeat,
						const Mat &center,
						Mat &instanceMat);

void calcTestFeatNEG ( const vector<vector<vector<float> > > &nfeat,
						const Mat &center,
						Mat &instanceMat);

void kmeansCluster( const vector<vector<float> > &descriptor, int n, vector<int> &label, Mat &center);


void descriptorSIFT(const Mat &src,
					vector<KeyPoint> &keypoint,
					Mat &descriptor,
					bool sc,
					double scalar);

void getSIFTHomoMat(const Mat &fir, const Mat &sec, Mat &H, bool sc , double scalar);

string exec(char* cmd);
#endif /* MACROS_H_ */
