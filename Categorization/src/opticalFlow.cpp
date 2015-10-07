/*
 * m_OpticalFlow.cpp
 *
 *  Created on: May 15, 2013
 *      Author: fengzh
 */

#include "opticalFlow.h"

#define USEDENSE false


inline double square( double a){
	return double( a * a);
}

inline double dist( Point a, Point b){
	return double( sqrt(double(a.x - b.x) * double(a.x - b.x) + double(a.y - b.y) * double(a.y - b.y)));
}
inline double dist( Point2f &a, Point2f &b){
	return double( sqrt(double(a.x - b.x) * double(a.x - b.x) + double(a.y - b.y) * double(a.y - b.y)));
}

Point2i SamplingOpticalFlow( const Mat &fir,
		const Mat &sec,
        const Mat &mask,
		vector<Point2f> &pt_src,
		vector<Point2f> &flo)
{
	bool useDense = USEDENSE;
    double scalar = 1;
	Mat src; cvtColor( fir, src, CV_RGB2GRAY);
	Mat dst; cvtColor( sec, dst, CV_RGB2GRAY);
    resize(src, src, Size((int)src.rows*scalar, (int)src.cols*scalar));
    resize(dst, dst, Size((int)dst.rows*scalar, (int)dst.cols*scalar));
    Point2i imgsize(src.cols, src.rows);
//	
    if (!useDense)
	{
		vector<Point2f> pt_dst; vector<uchar> status;vector<float>err;
        pt_src.clear();
        for ( int h = 0; h <src.rows; h+=2)
            for ( int w = 0; w < src.cols; w+=2)
                pt_src.push_back( Point2f(w,h));
        
		pt_dst.reserve( pt_src.size());
		flo.reserve( pt_src.size());
		calcOpticalFlowPyrLK( src, dst, pt_src, pt_dst, status, err);
		for ( uint n = 0; n < pt_src.size(); ++n)
		{
			flo.push_back(Point2f(pt_src[n].x - pt_dst[n].x, pt_src[n].y - pt_dst[n].y));
            if(!status[n] || dist(Point( pt_src[n].x, pt_src[n].y), Point(pt_src[n].x + flo[n].x, pt_src[n].y + flo[n].y)) >= INITPSIZEW)
				flo[n].x = flo[n].y = 0;
		}
	}
	else
	{
		Mat floMat;
		calcOpticalFlowFarneback( src, dst, floMat, 0.75, 3, 30, 10, 5, 1.5, OPTFLOW_FARNEBACK_GAUSSIAN);
		for (int y = 0; y<floMat.rows; y++) {
			for (int x = 0; x < floMat.cols; x++) {
				if ( !mask.at<int>(y,x))
					flo.push_back(Point2f(floMat.at<float>(y,2*x), floMat.at<float>(y,2*x+1)));
				else
					flo.push_back(Point2f(0.0f,0.0f));
			}
		}
	}
    return imgsize;
}

inline int tan2g ( const float & x, const float & y)
{
	return int(floor(atan(float(y)/x)*180/M_PI));
}

inline float floLen( const float &x, const float &y)
{
	return float( sqrt(x * x + y * y));
}

vector<float> difFeat(vector<float>&f1, vector<float>&f2){
    vector<float> dif;
    if(f1.size() != f2.size()) return dif;
    for(int i = 0; i < f1.size(); ++i) dif.push_back(f1[i]-f2[i]);
    return dif;
}
// get optical flow field descriptor
void optiflowDescriptor( int gid, int vid, vector<int>&label, vector<vector<float> > &pfeat, 
        vector<vector<float> > &nfeat)
{
	bool useDenseOF = USEDENSE;
    int flen = 50;
    vector<int> mos;
    ldLabel(gid, mos);
    char vname[512];
    Point2i isize;
 
    for(int v = vid; v < vid+1; ++v){
        sprintf(vname, "/home/fengzy/Projects/XProject/dataset/Set%.02d/video/%d.avi", gid, v);
        CvCapture *cap = cvCaptureFromFile(vname);
        if(!cap) continue;

        vector<vector<Point2f> > flo(flen);

        int width = cvGetCaptureProperty(cap, CV_CAP_PROP_FRAME_WIDTH);
        int height= cvGetCaptureProperty(cap, CV_CAP_PROP_FRAME_HEIGHT);
        vector<Point2f> densePt; densePt.reserve( width * height);
        for ( int h = 0; h <height; ++h)
            for ( int w = 0; w < width; ++w)
                densePt.push_back( Point2f(w,h));

        IplImage *pre, *nex;
        nex= cvQueryFrame(cap); pre = cvCreateImage(cvGetSize(nex), 8, 3);
        for(int i = 1; i <= flen; ++i){
            printf("[%d/%d]page\n",i,flen);
            cvCopy(nex, pre);
            nex = cvQueryFrame(cap);
            if(!nex) break;
            Mat mPre(pre), mNex(nex), mask;

            Point2i imgsize = SamplingOpticalFlow(mPre,mNex,mask,densePt, flo[i-1]);
            if(!isize.x) isize=imgsize;
        }
        //	 create histogram
        vector<float> floHist(flo.size()); //// remove the 1st and last frame's optical flow.
        char buff[512];
        sprintf(buff,"/home/fengzy/Projects/XProject/dataset/Set%.02d/feature/%d.txt",gid,v);
        FILE *fp = fopen(buff,"w+");

        vector<float> preFeat;
        vector<vector<float> >derivHist;
        for ( unsigned int nlen = 0; nlen < flo.size(); ++nlen)
        {
            vector<float> hist[4]; float count[4] = {0};
            for( int i = 0; i < 4; ++i) {hist[i] = vector<float>(9, 0);}

            for ( unsigned int ne = 0; ne < flo[nlen].size(); ++ne){
    //			// convert into angle
                float angle = 0, flolen = 1;
                if ( flo[nlen][ne].y ){
                    angle = tan2g(flo[nlen][ne].x, flo[nlen][ne].y);
                    angle = angle > 0 ? angle : 360 + angle;
                    // use flo length as weight
                    flolen = floLen( flo[nlen][ne].x, flo[nlen][ne].y);
                    flolen = flolen == 0 ? 1.0f : flolen;
                }
                int iy = ne/isize.x, ix = ne%isize.x;
                int indx = iy*2/isize.y + ix*2/isize.x;
                int inda = floor(angle/45.0f);
                if (int(angle) == 360) inda = 7;
                if(indx >= 4 || inda >= 8)
                    int db = 1;
                hist[0][inda] += flolen;
                count[0] += flolen;
            }
            vector<float> curFeat; 
            for ( int i = 0; i < 1; ++i)
            {
                // normalize
                if(!count[i]) count[i] = 1;
                transform(hist[i].begin(), hist[i].end(), hist[i].begin(),bind2nd( multiplies<float>(), float(1)/*/(count[i])*/ ));
                hist[i].back() = count[i];
                floHist.insert( floHist.end(), hist[i].begin(), hist[i].end());
                curFeat.insert(curFeat.end(), hist[i].begin(), hist[i].end());
                for(int j = 0; j < hist[i].size(); ++j)
                fprintf(fp,"%.08lf\t",hist[i][j]);
            }
            if( nlen) derivHist.push_back(difFeat(curFeat, preFeat));
            preFeat = curFeat;
            fprintf(fp,"\n");
        }
        for(int nf = 0; nf < derivHist.size(); ++nf){
            for(int ne = 0; ne < derivHist[nf].size(); ++ne)
                fprintf(fp,"%.08lf\t", derivHist[nf][ne]);
            fprintf(fp,"\n");
        }
        fclose(fp);
        
        if(mos[v] >= 0 && mos[v] <= 50) 
            pfeat.push_back(floHist); // for each video
        else nfeat.push_back(floHist);
    }
}

void EarlyDetection(int gid){
    
    vector<int> mos; ldLabel(gid, mos);

    char cmd[512], vname[512]; FILE *_fp = NULL;

    sprintf(cmd,"mkdir -p FlowFeature/"); exec(cmd);
    sprintf(cmd,"FlowFeature/%d.txt",gid);
    
    _fp = fopen(cmd,"w+"); 
     
    for(int vid = 1; vid <300; ++vid){

        sprintf(vname, "/home/fengzy/Projects/XProject/dataset/Set%.02d/video/video%.03d.mp4", gid, vid);
        CvCapture *cap = cvCaptureFromFile(vname);
        if(!cap) return;

        printf("[gid:%d; vid:%d]\n", gid, vid);
        int width = cvGetCaptureProperty(cap, CV_CAP_PROP_FRAME_WIDTH);
        int height= cvGetCaptureProperty(cap, CV_CAP_PROP_FRAME_HEIGHT);
        int vlen  = cvGetCaptureProperty(cap, CV_CAP_PROP_FRAME_COUNT);
        double qualityLevel = 0.01,minDistance = 10,k = 0.04;
        int blockSize = 3, maxCorners = 300;
        bool useHarrisDetector = false;

        IplImage *pre = cvCreateImage(cvSize(width,height),8,3);
        IplImage *cur = cvCreateImage(cvSize(width,height),8,3);
        IplImage *frame = cvQueryFrame(cap);
        cvCopy(frame,cur);

        vector<float> tflen, tfratio;
        // mean; var
        float fTol = 0, fMean = 0, fVar = 0, fNum = 0;
        float rTol(0), rMean(0), rMax(-INFINITY);
        for(int idx = 0; idx < 50;++idx){
            frame = cvQueryFrame(cap);
            if(!frame) break;
            cvCopy(cur, pre); cvCopy(frame, cur);
            vector<Point2f> p_pre, p_cur, set1, set2;
            vector<uchar> status, inlier;
            vector<float>err;
            Mat m_pre, m_cur;
            cvtColor(Mat(pre), m_pre, CV_RGB2GRAY);
            cvtColor(Mat(cur), m_cur, CV_RGB2GRAY);
            goodFeaturesToTrack( m_pre,
                     p_pre,
                     maxCorners,
                     qualityLevel,
                     minDistance,
                     Mat(),
                     blockSize,
                     useHarrisDetector,
                     k );
            calcOpticalFlowPyrLK(m_pre, m_cur, p_pre, p_cur, status, err);
            
            for(int flen = 0; flen < status.size(); ++flen){
                if(!status[flen]) continue;
                float d = dist(p_pre[flen], p_cur[flen]);
                if(d > 21) continue; // default search window size
                set1.push_back(p_pre[flen] ); set2.push_back(p_cur[flen]);
            }
            p_pre.clear(), p_cur.clear();
            float tol = 0, tnum = 0;
            if(set1.size() > 4){
                Mat H = findHomography(set1, set2, inlier, CV_RANSAC);
                for(int plen = 0; plen < set1.size(); ++plen)
                    if(!inlier[plen]){ 
                        float d = dist(set1[plen],set2[plen]);
                        tol += d; tnum += 1;
                    }
                fTol += tol/max(1.0f,tnum);
                tflen.push_back(tol);
                float tratio = tnum / float(set1.size());
                rMean += tratio; tfratio.push_back(tratio); rMax = max(rMax, tratio);
            }
        }
        fMean = fTol / float(tflen.size()); 
        rTol = rMean; rMean /= float(tfratio.size()); 
        for(int flen = 0; flen < tflen.size(); ++flen){
            fVar += pow(tflen[flen]-fMean,2);
        }
        fVar /= float(tflen.size()-1);
        
        float rVar = 0;
        for(int flen = 0; flen < tfratio.size(); ++flen){
            rVar += pow(tfratio[flen]-rMean,2);
        }
        rVar /= float(tfratio.size()-1);
        
       // total: zoom in/out;mean: same; variance: a little
        float fPre(0), fNex(0), rPre(0), rNex(0);
        for(int flen =0 ; flen < 10; ++flen){
            fPre+= tflen[flen], fNex += tflen[tflen.size()-1-flen];
            rPre+= tfratio[flen], rNex += tfratio[tfratio.size()-1-flen];
        }
         
       fprintf(_fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", 
               float(fNex-fPre)/10.0f, fMean , fVar, float(rNex-rPre)/10.0f, rMean, rVar);
       cvReleaseCapture(&cap);
    }
    fclose(_fp);

}

void wtSVMData(int gid){
    vector<int> mos; ldLabel(gid, mos);
    char buf[256]; FILE *_fp = NULL;
    vector<Point6f> feat; vector<int> vlabel;
    sprintf(buf, "FlowFeature/%i.txt", gid);
    _fp = fopen(buf, "r");
    float f[6];
    while(fscanf(_fp,"%f\t%f\t%f\t%f\t%f\t%f\n",&f[0],&f[1],&f[2],&f[3],&f[4],&f[5]) != EOF){
        feat.push_back(Point6f(f));
    } fclose(_fp);


    sprintf(buf,"FlowFeature/%i_svm.txt",gid);
    _fp = fopen(buf,"w+");
    for(int i = 0; i < feat.size(); ++i){
        if(mos[i] == -2) continue;
        int label = 1; if(mos[i] >= 50 || mos[i] == -1) label = -1;
        vlabel.push_back(label);
        fprintf(_fp,"%d 1:%f 2:%f\n", label, 
                feat[i].z,feat[i].u);
    } fclose(_fp);
    
    sprintf(buf,"FlowFeature/%d_gt.txt",gid);
    _fp = fopen(buf,"w+");
    for(int i = 0 ; i < vlabel.size(); ++i) fprintf(_fp,"%d\n",vlabel[i]);
    fclose(_fp);
}

Point6f min(Point6f p1, Point6f p2){
    float f[6];
    f[0]=min(p1.x,p2.x);f[1]=min(p1.y,p2.y);f[2]=min(p1.z,p2.z);
    f[3]=min(p1.w,p2.w);f[4]=min(p1.v,p2.v);f[5]=min(p1.u,p2.u);
    return Point6f(f);
}
Point6f max(Point6f p1, Point6f p2){
    float f[6];
    f[0]=max(p1.x,p2.x);f[1]=max(p1.y,p2.y);f[2]=max(p1.z,p2.z);
    f[3]=max(p1.w,p2.w);f[4]=max(p1.v,p2.v);f[5]=max(p1.u,p2.u);
    return Point6f(f);
}

float F_Measure(float beta, float prec, float rec){
    return (1+beta*beta)*prec*rec/(beta*beta*prec+rec);
}
float F_Measure(float beta, float tp, float fp, float fn){
    return(1+beta*beta)*tp/((1+beta*beta)*tp+beta*beta*fn+fp);
}
void EarlyRegression(int Gid){

    char buf[512]; FILE *_fp = NULL;
    vector<Point6f> feat; vector<int> label;
    for(int gid = 2; gid < 3; ++gid){
        sprintf(buf, "FlowFeature/%i.txt", gid);
        _fp = fopen(buf, "r");
        float f[6] = {0};
        while(fscanf(_fp,"%f\t%f\t%f\t%f\t%f\t%f\n",&f[0],&f[1],&f[2],&f[3],&f[4], &f[5]) != EOF){
            feat.push_back(Point6f(f));
        } fclose(_fp);
        sprintf(buf, "FlowFeature/%d_gt.txt", gid);
        _fp = fopen(buf, "r"); int t = 0;
        while(fscanf(_fp,"%d\n", &t) != EOF) label.push_back(t);
        fclose(_fp);
    }
    float _fmin[6]={INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY};
    float _fmax[6]={-1,-1,-1,-1,-1,-1};
    Point6f fmin(_fmin), fmax(_fmax);
    for(int i = 0; i < feat.size(); ++i){ 
        fmin = min(fmin,feat[i]);
        fmax = max(fmax,feat[i]);
    }
    float fscore = 0, opt_rec = 0, opt_prec = 0, grid = 1000;
    for(float th = fmin.u; th < fmax.u; th += (fmax.u-fmin.u)/grid){
        for(float th2 = fmin.x; th2 < fmax.x; th2 += (fmax.x-fmin.x)/grid){
            int tp(0),fp(0),fn(0),tn(0);
            for(int i = 0 ; i < feat.size(); ++i){
                if(feat[i].u > th && feat[i].x > th2 ){
                    if(label[i] == 1)++tp;
                    else ++fp;
                }else{
                    if(label[i] == 1)++fn;
                    else ++tn;
                }
            }
            float _prec = float(tp)/float(max(1,tp+fp)), _rec = float(tp)/float(max(1,tp+fn));
            float _f1 = F_Measure(2.0, _prec, _rec);
            if(_f1 > fscore){ fscore = _f1, opt_rec = th;}
        }
    }
    
    for(int g = 0; g < DATASET_SIZE; g++){ /*for all data*/
        sprintf(buf, "FlowFeature/%i.txt", g);
        _fp = fopen(buf, "r");
        float f[6]; feat.clear(); label.clear();
        while(fscanf(_fp,"%f\t%f\t%f\t%f\t%f\t%f\n",&f[0],&f[1],&f[2],&f[3],&f[4], &f[5]) != EOF){
            feat.push_back(Point6f(f));
        } fclose(_fp);
        sprintf(buf, "FlowFeature/%d_gt.txt", g);
        _fp = fopen(buf, "r"); int t = 0;
        while(fscanf(_fp,"%d\n", &t) != EOF) label.push_back(t);
        fclose(_fp);
        
        ////////////////////////////
        int tp(0),fp(0),fn(0),tn(0);
        for(int i = 0 ; i < feat.size(); ++i){
            
            if(feat[i].u > opt_rec && feat[i].x > opt_prec){
                if(label[i] == 1)++tp;
                else ++fp;
            }else{
                if(label[i] == 1)++fn;
                else ++tn;
            }
        }
        float _prec = float(tp)/float(max(1,tp+fp)), _rec = float(tp)/float(max(1,tp+fn));
        float _f1 = 2.0*_prec*_rec/max(1.0f,(_prec+_rec));
        printf("\n[%d]CMAT[+/-: %d,%d][PRF:%f; %f; %f]\n", g, int(tp+fn), int(fp+tn), _prec, _rec, _f1);
        printf("%d\t%d\n%d\t%d\n", tp, fp, fn, tn);
        printf("Accuracy:[%f]\n\n", float(tp+tn)/float(tp+fp+fn+tn));
    }
}

void ConfusionMatrix(int gid){
    vector<int> gt, pred;
    char buf[512]; FILE *_fp = NULL; int label;
    sprintf(buf,"FlowFeature/%d_gt.txt", gid);
    _fp = fopen(buf, "r");
    while(fscanf(_fp,"%d\n",&label) != EOF) gt.push_back(label);
    fclose(_fp);
    sprintf(buf,"FlowFeature/%d", gid);
    _fp = fopen(buf, "r");
    while(fscanf(_fp,"%d\n",&label) != EOF)  pred.push_back(label);
    fclose(_fp);
    
    float fp(0), tp(0), fn(0), tn(0);
    if(gt.size() != pred.size()) {printf("err: invalid length\n"); return;}
    for(int i = 0; i < gt.size(); ++i){
        if(gt[i] == 1){
            if(pred[i] == 1) tp+= 1;
            else fn += 1;
        }
        else{
            if(pred[i] == 1) fp+= 1;
            else tn += 1;
        }
    }
    printf("\n[%d]CMAT[+/-: %d,%d][PR:%f; %f]\n", gid, int(tp+fn), int(fp+tn), tp/(tp+fp), tp/(tp+fn));
    printf("%f\t%f\n%f\t%f\n", tp, fp, fn, tn);
    printf("Accuracy:[%f]\n\n", (tp+tn)/(tp+fp+fn+tn));
}


