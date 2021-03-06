/*
 * macros.cpp
 *
 *  Created on: May 27, 2013
 *      Author: fengzh
 */
#include "macros.h"
using namespace std;

string num2str( int base, int num)
{
	vector<string> strnum(base);
	for ( int i = 0; i < base; ++i)
		strnum[i] = "0";
	if ( num < 10)
	{
		ostringstream out;
		out << num;
		strnum[0] = out.str();
	}
	else if ( num >= 10 && num < 100)
	{
		ostringstream ba, ten;
		int t = floor(double(num) / 10), b = num % 10;
		ba << b;
		ten << t;
		strnum[0] = ba.str();
		strnum[1] = ten.str();
	}
	else
	{
		ostringstream ba, ten, hun;
		int b = num % 10, t = int(floor( double(num / 10))) % 10, h = num / 100;
		ba << b, ten << t, hun << h;
		strnum[0] = ba.str();
		strnum[1] = ten.str();
		strnum[2] = hun.str();
	}
	string name;
	for ( int i = base - 1; i >= 0; --i)
	{
		name += strnum[i];
	}

	return name;
}

void loadVideo( const string &capName, vector<Mat> &frame,
				int &vlen, int &width, int &height)
{
	VideoCapture cap ( capName);
	vlen = cap.get ( CV_CAP_PROP_FRAME_COUNT);
	width= cap.get( CV_CAP_PROP_FRAME_WIDTH);
	height = cap.get( CV_CAP_PROP_FRAME_HEIGHT);
	frame.clear();frame.reserve( vlen);
	for(int n = 0; n < vlen;++n) {Mat f; cap>>f; frame.push_back(f.clone());}
}

void getPtCoord( int width, int height, vector<Point2f> &pts)
{
	pts.reserve( width * height);
	for ( int w = 0; w < width; ++w)
		for( int h = 0; h < height; ++h)
		{
			pts.push_back(Point2f(w,h));
		}
}

void ldLabel(int gid, vector<int>&label){
    char path[512];
    sprintf(path,"/home/fengzy/Projects/XProject/dataset/annotation/Set%d/id.txt",gid);
    FILE *fp = fopen(path, "r");
    if(!fp){
    	printf("error, cannot open file. @%s,%d\n",__FILE__,__LINE__);
    	return;
	}
    int mos;
    while(fscanf(fp,"%d",&mos)!= EOF){
        label.push_back(mos);
    }
    fclose(fp);
}

void Vect2Mat(const vector<vector<float> > &vect, Mat &mtx)
{
	if (mtx.empty())
		mtx = Mat::zeros( vect.size(), vect[0].size(), CV_32FC1);

    for (uint i=0; i<vect.size(); i++)
        for (uint j=0; j<vect[i].size(); j++)
            mtx.at<float>(i,j) = vect[i][j];
}

float vecdist( const vector<float> &v1, const vector<float> &v2){
	float sum = 0;
    if(v1.size() != v2.size()){
        printf("vector dim not equal!\n"); return -1;
    }
	for ( uint n = 0; n < v1.size(); ++n)
		sum += (v1[n] - v2[n]) * (v1[n] - v2[n]);
	return sqrt(sum);
}

template <class NumType>
cv::Mat Vect2Mat(std::vector<std::vector<NumType> > vect)
{
    Mat mtx = Mat::zeros(vect.size(), vect[0].size(), DataType<NumType>::type);
    // copy data
    for (uint i=0; i<vect.size(); i++)
        for (uint j=0; j<vect[i].size(); j++)
            mtx.at<NumType>(i,j) = vect[i][j];
    
    return mtx;
}

void kmeansCluster( const vector<vector<float> > &descriptor, int nCluster,vector<int> &label, Mat &centers)
{
	printf("K-Means parameter: %d clusters\n",nCluster);
	Mat mLabel( descriptor.size(), 1, CV_32SC1);;
	Mat mat_dsp = Vect2Mat(descriptor);

	TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 1e-6);

	Mat center( nCluster, descriptor[0].size(), CV_32FC1);
	CvMat cmCenter = (CvMat)center;
	CvMat matdsp = (CvMat)mat_dsp;
	CvMat cmLabel = (CvMat)mLabel;

	cvKMeans2( &matdsp, nCluster, &cmLabel, criteria, 3, 0, KMEANS_PP_CENTERS, &cmCenter);

	mLabel = Mat(&cmLabel, true);
	center = Mat(&cmCenter, true);

//	kmeans( mat_dsp, nCluster, mLabel, criteria, 3, KMEANS_PP_CENTERS, center);

	for( int n = 0; n < mLabel.rows; ++n) label.push_back(mLabel.at<int>(n));

	centers = center.clone();

}

void calcFeatConsecuFrame(const vector<Mat> &frames,
					const vector<Point2f> &ptSet,
					const videoLineStruct &vls,
					vector<vector<float> > &feat)
{
	vector<Mat> homography(frames.size() - 1);
	vector<Point2f> mean_displace(frames.size() - 1);
	vector<float> meanLAngle(frames.size() - 1);
	feat.reserve( frames.size() - 1);
	for ( uint n = 0; n < frames.size() - 1; n++)
	{
		printf("%d/%d\n",n,frames.size() - 2);
		// homography
		Mat H ;
		getSIFTHomoMat( frames[n], frames[n+1], H, false, 0.4);
//		H = Mat::eye(3,3,CV_32FC1);
		homography[n] = H.clone();

		// displacement
		vector<Point2f> ptTrans;
		perspectiveTransform ( ptSet, ptTrans, H);
		vector<Point2f> diff;
		subtract( ptTrans, ptSet, diff);
		Mat mean_mv;
		reduce(diff, mean_mv, CV_REDUCE_AVG, 1);
		Point2f mean_pt(mean_mv.at<float>(0,0), mean_mv.at<float>(0,1));
		mean_displace[n] = mean_pt;

		// mean line angle changed
		vector<singleLine> lfir = vls.videoLineSet[n].lineSet,
						   lsec = vls.videoLineSet[n + 1].lineSet;

		float m_angle = 0; int ct = 0;

		for ( uint nlf = 0; nlf < lfir.size(); ++nlf){
			for ( uint nls = 0; nls < lsec.size(); ++nls){
				if ( lfir[nlf].lineId == lsec[nls].lineId){
					m_angle += abs(lfir[nlf].theta - lsec[nls].theta);
					++ct;
				}
			}
		}
		meanLAngle[n] = m_angle / max(ct,1);
	}
	float fst = .0f, fed = .0f;
	vector<singleLine> lst = vls.videoLineSet[0].lineSet,
					   led = vls.videoLineSet[frames.size() - 2].lineSet;
	for ( uint nlf = 0; nlf < lst.size(); ++nlf){
		fst += lst[nlf].theta;
	}
	fst /= lst.size();
	for ( uint nlf = 0; nlf < led.size(); ++nlf){
			fed += led[nlf].theta;
		}
	fed /= led.size();

	printf("construct feature\n");
	// construct feature
	for ( uint n = 0; n < frames.size() - 2; ++n){
		vector<float> feature;
		// mean homogarphy
		Mat mH = abs(homography[n]+homography[n+1])/2;
		vector<double> reshaped = mH.reshape(1,1);
		feature.insert( feature.end(), reshaped.begin(), reshaped.end());

		// mean change of line angle
		feature.push_back( abs(meanLAngle[n] - meanLAngle[n+1]));

		// mean change of pixels
		Point2f mp = (mean_displace[n] + mean_displace[n+1]);
		mp.x = mp.x < 0 ? -mp.x/2:mp.x/2;
		mp.y = mp.y < 0 ? -mp.y/2:mp.y/2;
		feature.push_back(mp.x); feature.push_back(mp.y);
//		 mean change of start and end of angle of sideline
//		feature.push_back(abs(fed - fst));
		feat.push_back(feature);

	}

	printf("construct feature finished\n");
	homography.clear();
}

void getSIFTHomoMat(const Mat &fir, 
					const Mat &sec, 
					Mat &H, 
					bool sc , 
					double scalar) /*sc: scaling or not*/
{
	Mat in = sec.clone();

	vector<vector<float> > dsp1, dsp2;
	vector<KeyPoint> kp1, kp2;
	Mat dpmat1, dpmat2;

	vector<int4> bound1, bound2;

	IplImage nfir = (IplImage)fir, nsec = (IplImage)sec;

	Mat contour;
	descriptorSIFT(fir,kp1,dpmat1, sc, scalar);
	descriptorSIFT(sec,kp2,dpmat2, sc, scalar);

	if ( kp1.size() == 0 || kp2.size() == 0)
	{
		printf(" # key points 0. exit\n");
		H = Mat::eye(3,3,CV_32FC1);
		return;
	}
	/******************************************************
	 * matching
	 */
	BruteForceMatcher<L2<float> > matcher;
	vector<DMatch> matches;
	matcher.match(dpmat1, dpmat2, matches);

	vector<int> pairOfsrcKP(matches.size()), pairOfdstKP(matches.size());
	for (size_t i = 0; i < matches.size(); i++)
	{
		pairOfsrcKP[i] = matches[i].queryIdx;
		pairOfdstKP[i] = matches[i].trainIdx;
	}

	vector<Point2f> sPoints;
	KeyPoint::convert(kp1, sPoints, pairOfsrcKP);
	vector<Point2f> dPoints;
	KeyPoint::convert(kp2, dPoints, pairOfdstKP);

	/******************************************************
	 * homography matrix
	 */
	vector<uchar> inlier;
	H.release();
	// if less than 4 points
	if ( dPoints.size() < 4 || dPoints.size() < 4)
	{
//		printf(" points not enough. exit\n");
		H = Mat::eye(Size(3,3), CV_32FC1);
		return ;
	}
	H = findHomography(sPoints, dPoints, inlier, CV_RANSAC);

}

void descriptorSIFT(const Mat &src,
					vector<KeyPoint> &keypoint,
					Mat &descriptor,
					bool sc,
					double scalar)
{

	Mat in = src.clone();
	if ( sc)
	{
		resize(in, in, Size(int(in.rows * scalar), 
						int(in.cols * scalar)), 0, 0);
	}

	SiftFeatureDetector detector;

	SiftDescriptorExtractor extractor;

	detector.detect( in, keypoint );

	int imgHeight = in.rows;

	// rm edge area points
	int bdValue = 50;
	for ( vector<KeyPoint>::iterator itr = keypoint.begin();
			itr != keypoint.end();)
	{
		Point2f curPt = (*itr).pt;
		if ( curPt.y < bdValue || curPt.y > imgHeight - bdValue){
			itr = keypoint.erase(itr);
		}
		else{
			++itr;
		}
	}

	// compute sift descriptor
	extractor.compute( in, keypoint, descriptor );

}

//simulate command line
string exec(char* cmd){
	FILE* pipe = popen(cmd, "r");
	if (!pipe)return "ERROR";
	char buffer[128];
	string result = "";
	while (!feof(pipe)) {
		if (fgets(buffer, 128, pipe) != NULL)result += buffer;
	}
	pclose(pipe);
	return result;
}
