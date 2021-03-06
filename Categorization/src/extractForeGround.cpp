
/*
 * extractForeGround.cpp
 *
 *  Created on: May 27, 2013
 *      Author: fengzh
 */

#include "extractForeGround.h"

void alignRect( Rect &r1, Rect &r2, int imgW, int imgH)
{

	r1.width = r2.width = max( r1.width, r2.width);
	r1.height = r2.height = max( r1.height, r2.height);

	r1.x = r1.x < 0 ? 0 : r1.x;
	r1.y = r1.y < 0 ? 0 : r1.y;
	r2.x = r2.x < 0 ? 0 : r2.x;
	r2.y = r2.y < 0 ? 0 : r2.y;

	r1.x = r1.x + r1.width > imgW - 1 ? r1.x + r1.width - (imgW - 1) : r1.x;
	r1.y = r1.y + r1.height > imgH - 1 ? r1.y + r1.height - (imgH - 1) : r1.y;
	r2.x = r2.x + r2.width > imgW - 1 ? r2.x + r2.width - (imgW - 1) : r2.x;
	r2.y = r2.y + r2.height > imgH - 1 ? r2.y + r2.height - (imgH - 1) : r2.y;

}

void coord2rect( const vector<vector<Point2f> > &projcoord,
				 vector<Rect> &rect, int width,int height)
{
	int num = projcoord[0].size();
	for ( int n = 0; n < num; ++n)
	{
		int w = projcoord[1][n].x - projcoord[0][n].x;
		int h = projcoord[2][n].y - projcoord[0][n].y;

		Rect r;
		r.x = max( 0,  int(projcoord[0][n].x));
		r.y = max( 0,  int(projcoord[0][n].y));
		r.x = min( r.x, width - 1 - w);
		r.y = min( r.y, height- 1 - h);

		r.width = w;
		r.height= h;
		rect.push_back(r);
	}

}
//
void coord2rect( const vector<vector<Point2f> > &projcoord, vector<Rect> &rect,
				 int width,int height,int pchSizeW,int pchSizeH)
{
	int num = projcoord[0].size();
	for ( int n = 0; n < num; ++n)
	{
//		int w = pchSizeW;
//		int h = pchSizeH;
		int w = projcoord[1][n].x - projcoord[0][n].x;
		int h = projcoord[2][n].y - projcoord[0][n].y;

		Rect r;

		r.x = max( 0,  int(projcoord[0][n].x));
		r.y = max( 0,  int(projcoord[0][n].y));
		r.x = min( r.x, width - 1 - w);
		r.y = min( r.y, height- 1 - h);

		r.width = w;
		r.height= h;

		rect.push_back(r);
	}
}

bool mRecEqual( const Rect &r1, const Rect &r2){
	return( r1.x == r2.x && r1.y == r2.y);
}

void scanWindow(const int &numPh,
				const int &curSlide,
				const int &nstart, /* used for scanning the whole video, ith outer-loop scan*/
				const vector<Mat> &bckPMat,
				const vector<Mat> &bckFrame,
				const vector<Mat> &frame_diff,
				const vector<double> thresVec,
				const vector<vector<Point2f> > &UL,
				const vector<vector<Point2f> > &UR,
				const vector<vector<Point2f> > &LL,
				const vector<vector<Point2f> > &LR,
				const Mat &SoM,
				Mat &preMask,
				vector<Rect> &blobs,
				bool *label,
				Mat &mask)
{

	int dynProjNum = NUMPROJFRAME;

	int width = bckFrame[0].cols, height = bckFrame[0].rows;

	Mat curFrame = bckFrame[curSlide].clone();
	Mat filter ( SoM.rows, SoM.cols, CV_8UC3, CV_RGB(0,255,0));
	Mat binT (SoM.rows, SoM.cols, CV_8UC1, CV_RGB(255,255,255));
	if ( !preMask.cols)
		preMask = Mat::zeros(SoM.size(), CV_8UC1);

//	size_t t1 = clock();

	vector<vector<Point2f> > ul(dynProjNum),ur(dynProjNum ),
							 ll(dynProjNum),lr(dynProjNum );

	ul[0].insert(ul[0].end(), UL[0].begin(), UL[0].end());
	ur[0].insert(ur[0].end(), UR[0].begin(), UR[0].end());
	ll[0].insert(ll[0].end(), LL[0].begin(), LL[0].end());
	lr[0].insert(lr[0].end(), LR[0].begin(), LR[0].end());

	for ( int npMat = 0; npMat < dynProjNum - 1; ++npMat)
	{
		Mat curPMat = bckPMat[npMat + curSlide].clone();

		// get projected points
		perspectiveTransform ( ul[npMat], ul[npMat + 1], curPMat);
		perspectiveTransform ( ur[npMat], ur[npMat + 1], curPMat);
		perspectiveTransform ( ll[npMat], ll[npMat + 1], curPMat);
		perspectiveTransform ( lr[npMat], lr[npMat + 1], curPMat);
	}

	vector<vector<Rect> > rectset( dynProjNum );
	vector<Rect> bck_rectset;

	for ( int i = 0; i < dynProjNum; ++i)
	{
		rectset[i].reserve( numPh);
		vector<vector<Point2f> >projCoordVec(4);

		projCoordVec[0].insert( projCoordVec[0].end(), ul[i].begin(), ul[i].end());
		projCoordVec[1].insert( projCoordVec[1].end(), ur[i].begin(), ur[i].end());
		projCoordVec[2].insert( projCoordVec[2].end(), ll[i].begin(), ll[i].end());
		projCoordVec[3].insert( projCoordVec[3].end(), lr[i].begin(), lr[i].end());

		coord2rect( projCoordVec, rectset[i], width, height);
//		coord2rect( projCoordVec, rectset[i], width, height, INITPSIZEW, INITPSIZEH);
	}

	// back-proj
	for ( int npMat = 0; npMat < curSlide; ++npMat)
	{
		Mat invHomo = bckPMat[npMat].inv();
		// get projected points
		perspectiveTransform ( ul[npMat], ul[npMat + 1], invHomo);
		perspectiveTransform ( ur[npMat], ur[npMat + 1], invHomo);
		perspectiveTransform ( ll[npMat], ll[npMat + 1], invHomo);
		perspectiveTransform ( lr[npMat], lr[npMat + 1], invHomo);
	}
	vector<vector<Point2f> >projCoordVec(4);
	bck_rectset.reserve( numPh);

	projCoordVec[0].insert( projCoordVec[0].end(), ul[curSlide].begin(), ul[curSlide].end());
	projCoordVec[1].insert( projCoordVec[1].end(), ur[curSlide].begin(), ur[curSlide].end());
	projCoordVec[2].insert( projCoordVec[2].end(), ll[curSlide].begin(), ll[curSlide].end());
	projCoordVec[3].insert( projCoordVec[3].end(), lr[curSlide].begin(), lr[curSlide].end());
	coord2rect( projCoordVec, bck_rectset, width, height);

//	printf("scan window, pre-process time:%g\n",double(clock() - t1)/1000000 );

//	for ( int nr = numPh - 1; nr >= 0; --nr)
	for ( int nr = 0; nr < numPh; ++nr)
	{
		int i0 = 0, i1 = 0;
		double curscore = 0, maxscore = 0;
		bool rst = false;
		if ( !label[nr])
		{
			for ( int n = 0; n < dynProjNum - 1; ++n)
			{
				Rect cr = rectset[n + 1][nr];
				if (cr.x < 0 || cr.x > width - 1 ||
					cr.y < 0 || cr.y > height - 1 ||
					cr.width < 0 || cr.width > width - 1||
					cr.height< 0 || cr.height> height- 1)
					continue;

				Mat grayDiff = frame_diff[n + curSlide](cr);
//				gpu::GpuMat grayDiffGPU; grayDiffGPU.upload( grayDiff);
				double maxval = norm( grayDiff, NORM_INF);
				if ( maxval >= thresVec[n + curSlide] ){
					++i1; ++curscore;
					maxscore =curscore > maxscore ? curscore : maxscore;
				}
				else{
//					++i0;
					curscore = 0;
				}
			}
			rst = ( i1 >= PROJ_MODE_VAL /*&& maxscore >= 2*/ );
		}

		// back-proj
		if ( rst &&	rectset[0][nr].y > 50 && rectset[0][nr].y < height - 50)
		{
			label[nr] = 1;

			Rect bckRstRect = bck_rectset[nr];
			alignRect( bckRstRect, bckRstRect, width, height);
			// store blobs
			blobs.push_back(bckRstRect);

			// debug mode; show result
			Mat binM = binT(bckRstRect);
			binM = binM + preMask( bckRstRect);
			binM.copyTo( preMask( bckRstRect));
		}
	}

	vector<Rect>::iterator itr = unique( blobs.begin(), blobs.end(), mRecEqual);
	blobs.resize( distance( blobs.begin(), itr));

	// debug mode; show result
	Mat curSoM = SoM.clone();

	for ( unsigned int n = 0; n < blobs.size(); ++n)
	{
		float tp = 0.5;
		curSoM(blobs[n]) = curSoM(blobs[n]) * (1-tp)+ filter(blobs[n]) * tp;
	}
//	namedWindow("SOM");
//	imshow("SOM", curSoM); waitKey(10);

	if ( curSlide == NUMCASCADE - 1)
		mask = preMask.clone();

}

void findForeGround(const vector<Mat> &frames,
					vector<vector<Rect> > &Blob,
					vector<Point2f> &optiPt,
					vector<vector<bool> > &Label,
					Mat &mask)
{
	// add grass and line filter | update: or hlsFilter | update:rm *
	bool init = true;
	int patchH = INITPSIZEH, patchW = INITPSIZEW;
	int patchWarpH = max(1,patchH >> 0), patchWarpW = max(1, patchW >> 0);
	int vlen, width, height;
	vlen = frames.size(), width = frames[0].cols, height = frames[0].rows;
	int numPh =  (width-patchH) * (height-patchH)/(patchWarpW * patchWarpH); // total # of patches
	int tSlides = NUMCASCADE;
	int nNumProj = NUMPROJFRAME;
	bool rescale = false; double featScalar = .5;

	vector<double> bu_thresVec;
	vector<Mat> bu_frame_diff, bu_bckPMat;
	bu_thresVec.reserve( nNumProj + tSlides - 1);
	bu_frame_diff.reserve ( nNumProj + tSlides - 1); // store older
	bu_bckPMat.reserve ( nNumProj + tSlides - 1);    // information

	vector<vector<Point2f> > ul(nNumProj),ur(nNumProj ),
							 ll(nNumProj),lr(nNumProj );

	ul[0].reserve( numPh);
	ur[0].reserve( numPh);
	ll[0].reserve( numPh);
	lr[0].reserve( numPh);

	optiPt.reserve( numPh);
	// save origin position
	for ( int h = 0; h < height - patchH; h += patchWarpH)
	{
		for ( int w = 0; w < width - patchW; w += patchWarpW)
		{
			ul[0].push_back ( Point2f( w, h)); //upper-left
			ur[0].push_back ( Point2f( w + patchW, h)); //upper-right
			ll[0].push_back ( Point2f( w, h + patchH)); //lower-left
			lr[0].push_back ( Point2f( w + patchW, h + patchH)); //upper-left

			optiPt.push_back( Point2f( w + patchW/2, h + patchH/2));
		}
	}

	for ( int nstart = 0, bindx = 0; nstart <  vlen - NUMPROJFRAME - tSlides + 1; nstart += SC_AV_WARP_SIZE, ++bindx)
	{
//		printf("%d\n",nstart);
		if ( nstart == 93)
			int dbg =1;
		Mat H, m_SOM, m_Mask;
		vector<Mat> bckFrame, bckPMat, frame_diff;
		vector<double> thresVec;

		thresVec.reserve( nNumProj + tSlides - 1);
		frame_diff.reserve( nNumProj + tSlides - 1);
		bckPMat.reserve( tSlides + nNumProj-1);
		bckFrame.insert( bckFrame.end(), frames.begin() + nstart,
						 frames.begin() + nstart + tSlides + nNumProj );

		m_SOM = bckFrame[0];

		size_t t = clock();
		// init first sliding window data
		if ( init)
		{
			for ( int n = 0; n < nNumProj + tSlides - 1; ++n)
			{
				Mat allOriDiff = abs(bckFrame[n] - bckFrame[n + 1]);
				Mat GrayDiff;
				cvtColor ( allOriDiff, GrayDiff, CV_RGB2GRAY);
				frame_diff.push_back ( GrayDiff);
				bu_frame_diff.push_back( GrayDiff);

				vector<double> reshaped = GrayDiff.reshape(1, 1);
				set<double> s;
				unsigned size = reshaped.size();
				for( unsigned i = 0; i < size; ++i ) s.insert( reshaped[i] );
				reshaped.assign( s.begin(), s.end() );
				reverse( reshaped.begin(), reshaped.end());
				int numTopN = TOPTHRES_SCALAR * reshaped.size();
				double thres = reshaped[numTopN - 1];
				thresVec.push_back( thres);

				Mat curH;
				getSIFTHomoMat(bckFrame[n], bckFrame[n + 1], curH, rescale, featScalar);
//				getSURFHomoMat(bckFrame[n], bckFrame[n + 1], curH, true, featScalar);
				bckPMat.push_back( curH);bu_bckPMat.push_back(curH);
			}
			bu_thresVec = thresVec;
//			printf("init, pre-process:%g\n", double(clock() - t)/1000000);
		}

		else{
			thresVec.insert( thresVec.end(), bu_thresVec.begin() + SC_AV_WARP_SIZE , bu_thresVec.end());
			frame_diff.insert( frame_diff.end(), bu_frame_diff.begin() + SC_AV_WARP_SIZE, bu_frame_diff.end());
			bckPMat.insert( bckPMat.end(), bu_bckPMat.begin() + SC_AV_WARP_SIZE, bu_bckPMat.end());

			for ( int n = nNumProj + tSlides - SC_AV_WARP_SIZE - 1; n < nNumProj + tSlides - 1; ++n)
			{
//				size_t t1 = clock();
				Mat allOriDiff = abs(bckFrame[n] - bckFrame[n + 1]);
				Mat GrayDiff;
				cvtColor ( allOriDiff, GrayDiff, CV_RGB2GRAY);
				frame_diff.push_back ( GrayDiff);

				vector<double> reshaped = GrayDiff.reshape(1, 1);
				set<double> s;unsigned size = reshaped.size();
				for( unsigned i = 0; i < size; ++i ) s.insert( reshaped[i] );
				reshaped.assign( s.begin(), s.end() );
				reverse( reshaped.begin(), reshaped.end());
				int numTopN = TOPTHRES_SCALAR * reshaped.size();
				double thres = reshaped[numTopN - 1];
				thresVec.push_back( thres);

				Mat curH;
				getSIFTHomoMat(bckFrame[n], bckFrame[n + 1], curH, rescale, featScalar);

				bckPMat.push_back( curH);
			}
			bu_thresVec = thresVec;
			for ( unsigned int nb = 0; nb < bckPMat.size(); ++nb)
			{
				bu_frame_diff[nb] = frame_diff[nb].clone();
				bu_bckPMat[nb]    = bckPMat[nb].clone();
			}
//			printf("new position, pre-process:%g\n", double(clock() - t)/1000000);
		}

		Blob[bindx].reserve( numPh);
		Label[bindx].reserve( numPh);

		bool *label = (bool *)malloc(sizeof(bool) * numPh);
		memset( label, 0, sizeof(bool) * numPh);

		for ( int nslide = 0; nslide < tSlides; ++nslide)
		{

			scanWindow ( numPh, nslide, nstart,
						 bckPMat, bckFrame, frame_diff,
						 thresVec, ul,ur, ll, lr, m_SOM, m_Mask, Blob[bindx], label,mask);
		}
		init = false;
		for ( int i = 0; i < numPh; ++i) Label[bindx].push_back(label[i]);
		free(label);label = NULL;
	}

}


