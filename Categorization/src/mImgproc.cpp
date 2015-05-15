/*
 * imgprocc.cpp
 *
 *  Created on: Dec 6, 2012
 *      Author: fengzh
 */
#include "mImgproc.h"

void drawLines(IplImage *dst, float *line, CvScalar color)
{
	float rho = line[0];
	float theta = line[1];
	CvPoint pt1, pt2;
	double cosTheta = cos(theta), sinTheta = sin(theta);
//	double a = -cosTheta / sinTheta, b = rho / sinTheta;
	double x0 = cosTheta*rho, y0 = sinTheta*rho;
	pt1.x = cvRound(x0 + 1000*(-sinTheta));
	pt1.y = cvRound(y0 + 1000*(cosTheta));
	pt2.x = cvRound(x0 - 1000*(-sinTheta));
	pt2.y = cvRound(y0 - 1000*(cosTheta));
	cvLine(dst, pt1, pt2, color, 3, 8);
}

void drawPoints(IplImage *dst, const vector<CvPoint> &pset, CvScalar color)
{

	for (vector<CvPoint>::const_iterator itr = pset.begin(); itr != pset.end(); ++itr)
	{
		CvPoint p = *itr;
		cvLine(dst, p, p, color, 3, 8);
	}
}

void drawPoints(Mat &dst, const vector<CvPoint> &pset, const Scalar &color)
{

	for (vector<CvPoint>::const_iterator itr = pset.begin(); itr != pset.end(); ++itr)
	{
		Point p = *itr;
		line(dst, p, p, color, 3, 8);
	}
}

void getPoints(float *line, int width, int height, vector<CvPoint> &result)
{
	float rho = line[0];
	float theta = line[1];
	CvPoint pt1, pt2, pt3;
	double a = -cos(theta)/sin(theta), b = rho/sin(theta);
	pt1.x = cvRound(rho/cos(theta));
	pt1.y = 0;
	pt2.x = 0;
	pt2.y = cvRound(rho/sin(theta));
	if ((pt1.x > 0) && (pt2.y > 0))
	{
		if (pt1.x >= width)
		{
			pt1.x = width - 1;
			pt1.y = cvRound(pt1.x*a+b);
		}
		if (pt2.y >= height)
		{
			pt2.y = height - 1;
			pt2.x = cvRound((pt2.y-b)/a);
		}
	}
	else
	{
		if (a*width+b > height)
		{
			pt3.y = cvRound(height);
			pt3.x = cvRound((pt3.y-b) / a);
		}
		else
		{
			pt3.x = cvRound(width);
			pt3.y = cvRound(a*pt3.x+b);
		}
		if (pt2.y < 0)
		{
			pt2.x = pt3.x;
			pt2.y = pt3.y;
		}
		else
		{
			pt1.x = pt3.x;
			pt1.y = pt3.y;
		}
	}

	if (pt1.x > pt2.x)
	{
		swap(pt1, pt2);
	}

	CvPoint p;
	p.x = pt1.x;
	p.y = pt1.y;
	while (p.x < pt2.x && p.y < height)
	{
		CvPoint pt;
		pt.x = p.x;
		pt.y = p.y;
		result.push_back(pt);
		int rd = 0;
		while (p.y == pt.y)
		{
			p.x++;
			p.y = cvRound(a*p.x + b);
			++rd;
			if(rd > 1e4) break;
		}
		// avoid trival
		if ( result.size() > max(width,height)) break;
	}
//	return result;
}

void ldFrameLines(const string name, videoLineStruct &vls, int width, int height, int nframe)
{
	bool DEBUG = false;

	/*
	 * fstream for loading values
	 */
	ifstream infile;
	infile.open(name.c_str());

	// if failed
	if(!infile)
	{
		printf("cannot load line file, exit\n");
		return;
	}

//	 vector for line storage; vector for access convenience
//	videoLineStruct vls;

	// frame id
	int fID = 1;

	// for each line
	int numLine = 0;

	// for max line id
	int maxlineId = 0;

	vls.videoLineSet.clear();
	vls.videoLineSet.reserve(nframe);

	// load how many lines in current frame
	for( ; infile >> numLine != 0; )
	{
		// current frame
		frameLine fset;
		fset.numLines = numLine;
		fset.frameId  = fID++;

//		printf("frame_id:%d\n",fset.frameId);

		// load each line
		for (int nLine = 0; nLine < numLine; ++nLine)
		{
			singleLine line;
			infile >> line.tho;
			infile >> line.theta;
			if(name.substr(name.find_last_of(".") + 1) == "line")
			{
				line.lineId = -1;
			}
			else
			{
				infile >> line.lineId;
			}

			maxlineId = max(maxlineId, line.lineId);

			// get point set
			float fCurLine[2] = { line.tho, line.theta };
			getPoints(fCurLine, width, height, line.pointSet);
			// for each line in the same frame
			fset.lineSet.push_back(line);
		}

		// for each frame
		vls.videoLineSet.push_back(fset);
	}

	if (DEBUG)
	{
		bool m_debug_var = true;
	}

//	return videoLine;

	if(name.substr(name.find_last_of(".") + 1) == "line")
	{
		vls.maxLineId = 12;
	}
	else
	{
		vls.maxLineId = maxlineId;
	}

}

void drawLines( videoLineStruct *vls, const char* cVName)
{
	CvCapture *cap = NULL;
	vector<CvPoint> points;
	cap = cvCaptureFromFile(cVName);


	for ( int frameID = 0; ;++frameID )
	{
		CvScalar color[12] = {	CV_RGB(255,0,0), CV_RGB(0,255,0), CV_RGB(0,0,255),
								CV_RGB(255,0,255), CV_RGB(255,255,0), CV_RGB(0,255,255),
								CV_RGB(0,0,0), CV_RGB(50,50,0), CV_RGB(0,50,50),
								CV_RGB(50,0,50), CV_RGB(100,50,0), CV_RGB(150,150,0)};

		IplImage *frame = NULL;

		frame = cvQueryFrame(cap);
		if (!frame)
		{
			printf("total frame:%d\n", frameID - 1);
			break;
		}

		// for each frame
		frameLine curframe = vls->videoLineSet[frameID];

		for (int nline = 0; nline < curframe.numLines; ++nline)
		{
			singleLine curline = curframe.lineSet[nline];
			drawPoints(frame, curline.pointSet, color[curline.lineId]);
		}

		string winName = "frame";
		cvNamedWindow( winName.c_str(), 1);
		cvShowImage( winName.c_str(), frame);
//		printf("current frame:%d\n", frameID);
 		cvWaitKey();
	}

}

void drawLines( int frameID, const videoLineStruct &vls, Mat &frame)
{

	Scalar color(255,0,0);

	// for each frame
	frameLine curframe = vls.videoLineSet[frameID];

	for (int nline = 0; nline < curframe.numLines; ++nline)
	{
		singleLine curline = curframe.lineSet[nline];
		drawPoints(frame, curline.pointSet, color);
		ostringstream lid; lid<<curline.lineId;
		CvPoint p = curline.pointSet[curline.pointSet.size()/2];
		putText( frame, lid.str(), p, FONT_HERSHEY_SIMPLEX,
									 1.0, Scalar(0,0,255));
	}


}

void rotateVideo(string viName)
{
	VideoCapture cap(viName);

	string fName = "up.avi";

	if( !cap.isOpened())
	{
		printf("cannot open video. exit\n");
		exit(-1);
	}

	for ( ; ; )
	{
		Mat src, dst;
		cap.read(src);// >> src;
		transpose(src, dst);
		flip(dst, dst, 1);
		imshow("rotated", dst);
		waitKey(30);

		static VideoWriter wt(fName, CV_FOURCC('D','I','V','X'), 30, dst.size(), true);
		wt.write(dst);

	}
}

void saveVideo( string videoName)
{
	VideoCapture cap(videoName);

	string fName = "sample.avi";

	if( !cap.isOpened())
	{
		printf("cannot open video. exit\n");
		exit(-1);
	}

	for ( ; ; )
	{
		Mat src, dst;
		cap.read(src);// >> src;
		transpose(src, dst);
		flip(dst, dst, 1);
		imshow("rotated", dst);
		waitKey(30);

		static VideoWriter wt(fName, CV_FOURCC('D','I','V','X'), 30, dst.size(), true);
		wt.write(dst);

	}
}

// get image name; output frame_xxx where xxx is a number
string getimgName(int n)
{
	string prefix = "frame_";
	ostringstream c0, c1, c2; // stream used for the conversion

	if (n < 10)
	{
		c0 << n;
		c1 << 0;
		c2 << 0;
	}
	else if (n >= 10 && n < 100)
	{
		c0 << n - int(n / 10) * 10;
		c1 << int(n / 10);
		c2 << 0;
	}
	else
	{
		c2 << int(n / 100);
		c1 << int(n - int(n / 100) * 100) / 10;
		c0
				<< int(
						n - int(n / 100) * 100
								- int(n - int(n / 100) * 100) / 10 * 10);
	}

	string imgName = prefix + c2.str() + c1.str() + c0.str();
	return imgName;
}

