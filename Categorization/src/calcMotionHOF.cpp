/*
 * calcMotionHOF.cpp
 *
 *  Created on: May 27, 2013
 *      Author: fengzh
 */ 

#include "calcMotionHOF.h"

// return cluster-vector distance
float vDist( vector<float>&v, Mat &m, int idx){
    float dist = 0;
    if(v.size() != m.row(idx).cols) {
    	printf("invalid dim\n"); return INFINITY;
    }
    for(int i = 0; i < v.size(); ++i){
     	dist += v[i]-m.at<float>(idx, i)*v[i]-m.at<float>(idx, i);
 	}
    return sqrt(dist);
}

// with k-means for bow
void featVec2Mat(  vector<vector<vector<float> > > &featBlock_pos,
				 vector<vector<vector<float> > > &featBlock_neg,
                 vector<vector<vector<float> > > &test_pos,
                 vector<vector<vector<float> > > &test_neg,
				Mat &labelMat,
				Mat &instanceMat,
                Mat &testPMat,
                Mat &testNMat,
				Mat &center)
{

	// k-means
	int npos = 0, nneg = 0 ;
	const int narr = NCLUSTER;
	
	vector<vector<float> > feat_uncluster;

	for( uint nv = 0; nv < featBlock_pos.size(); ++nv)
		feat_uncluster.insert( feat_uncluster.end(), featBlock_pos[nv].begin(), featBlock_pos[nv].end());

	// # of positive features(each frame)
	npos = feat_uncluster.size();

	// for each video block
	for( uint nv = 0; nv < featBlock_neg.size(); ++nv)
		feat_uncluster.insert( feat_uncluster.end(), featBlock_neg[nv].begin(), featBlock_neg[nv].end());
	nneg = feat_uncluster.size() - npos;

	vector<int> label; int nCluster = NCLUSTER;

	kmeansCluster( feat_uncluster, nCluster, label, center);

	feat_uncluster.clear();
	printf("k-means finished\n");

	// bow
    int tPos = featBlock_pos.size(), tNeg = featBlock_neg.size();
	vector<vector<float> >bowfeat_pos(tPos);
    vector<vector<float> >bowfeat_neg(tNeg);
    vector<vector<float> > svmTrain;
    
    labelMat = Mat::zeros( tPos+tNeg, 1, CV_32FC1);
	for(int n = 0; n < tPos; ++n) labelMat.at<float>(n,0) = 1.0f;
	for(int n = tPos; n < tPos+tNeg; ++n) labelMat.at<float>(n,0) = -1.0f;
	// init feature dimension
	for (uint i = 0; i < tPos; ++i)
		bowfeat_pos[i] = vector<float>(nCluster, 0);

	for (int i = 0; i < tNeg; ++i)
		bowfeat_neg[i] = vector<float>(nCluster, 0);

	// positive feature
    int pIdx = 0;
	for( int nv = 0; nv < tPos; ++nv){
		int nInstance = featBlock_pos[nv].size();
		for ( int n = 0; n < nInstance; ++n){
            if(label[pIdx + n] >= bowfeat_pos[nv].size()||!nInstance) printf("invalid dim\n");
			bowfeat_pos[nv][label[pIdx + n]] += 1;
        }
        
		for( int n = 0; n < nCluster;++n) bowfeat_pos[nv][n] /= (float)nInstance;
        pIdx += nInstance;
	}

    int nIdx = pIdx;
    for(int nv = 0; nv < tNeg; ++nv){
        int nInstance = featBlock_neg[nv].size();
		for ( int n = 0; n < nInstance; ++n){
            if(label[nIdx + n] >= bowfeat_neg[nv].size()||!nInstance) printf("invalid dim\n");
			bowfeat_neg[nv][label[nIdx + n]] += 1;
        }
        
		for( int n = 0; n < nCluster;++n) bowfeat_neg[nv][n] /= nInstance;
        nIdx += nInstance;
    }

    /*
     * test feature
     */
    int tsPos = test_pos.size(), tsNeg = test_neg.size();
    vector<vector<float> >testP(tsPos), testN(tsNeg);
    for(int nvideo = 0; nvideo < tsPos; ++nvideo){
        int len = test_pos[nvideo].size();
        if(!len) {printf("nil feature\n");continue;}
        
        int tlabel[len]; memset(tlabel, 0, sizeof(tlabel));
        for(int nframe = 0; nframe < len; ++nframe){
            if(test_pos[nvideo][nframe].empty()){printf("nil feat\n"); continue;}
            float dist = INFINITY;
            for(int ct = 0; ct < center.rows; ++ct){
                float tdist = vDist(test_pos[nvideo][nframe], center, ct);
                if(tdist < dist){ dist = tdist; tlabel[nframe] = ct;}
            }
        }
        vector<float> tfeat(nCluster, 0);
        for(int i = 0; i < len; ++i) tfeat[tlabel[i]] += 1;
        for(int i = 0; i < nCluster; ++i) tfeat[i] /= (float)len;
        testP[nvideo] = (tfeat);
    }
    
    for(int nvideo = 0; nvideo < tsNeg; ++nvideo){
        int len = test_neg[nvideo].size();
        if(!len) {printf("nil feature\n");continue;}
        
        int tlabel[len]; memset(tlabel, 0, sizeof(tlabel));
        for(int nframe = 0; nframe < len; ++nframe){
            if(test_neg[nvideo][nframe].empty()){printf("NIL feat@%s%d\n",__FILE__,__LINE__); continue;}
            float dist = INFINITY;
            for(int ct = 0; ct < center.rows; ++ct){
                float tdist = vDist(test_neg[nvideo][nframe], center, ct);
                if(tdist < dist){ dist = tdist; tlabel[nframe] = ct;}
            }
        }
        vector<float> tfeat(nCluster, 0);
        for(int i = 0; i < len; ++i) tfeat[tlabel[i]] += 1;
        for(int i = 0; i < nCluster; ++i) tfeat[i] /= (float)len;
        testN[nvideo] = (tfeat);
    }	printf("BoW feature finished\n");

	svmTrain.insert( svmTrain.end(), bowfeat_pos.begin(), bowfeat_pos.end());
	svmTrain.insert( svmTrain.end(), bowfeat_neg.begin(), bowfeat_neg.end());

	instanceMat = Mat::zeros(tPos+tNeg, nCluster, CV_32FC1);
    testPMat = Mat::zeros(tsPos, nCluster, CV_32FC1);
    testNMat = Mat::zeros(tsNeg, nCluster, CV_32FC1);
	// ensambling data
    for (int i=0; i<tPos+tNeg; i++)
    {
        if(svmTrain[i].size() != instanceMat.row(i).cols){printf("invalid dim@%s%d\n",__FILE__,__LINE__);}
        for(int j = 0; j < nCluster; ++j) instanceMat.at<float>(i,j) = svmTrain[i][j];
    } 
    for (int i=0; i<tsPos; i++){
        if(testP[i].size() != testPMat.row(i).cols){printf("invalid dim@%s%d\n",__FILE__,__LINE__);}
        for(int j = 0; j < nCluster; ++j) testPMat.at<float>(i,j) = testP[i][j];
    }
    for (int i=0; i<tsNeg; i++){
//        if(testN[i].size() != testNMat.row(i).cols){printf("invalid dim\n");}        
        for(int j = 0; j < nCluster; ++j) testNMat.at<float>(i,j) = testN[i][j];
//        for(int j = 0; j < nCluster; ++j) cvmSet(testNMat, i, j, testN[i][j]);
    }
}

//without k-means
void featVec2Mat2(  vector<vector<vector<float> > > &featBlock_pos,
				 vector<vector<vector<float> > > &featBlock_neg,
                 vector<vector<vector<float> > > &test_pos,
                 vector<vector<vector<float> > > &test_neg,
				Mat &labelMat,
				Mat &instanceMat,
                Mat &testPMat,
                Mat &testNMat,
				Mat &center)
{
    int tpos = featBlock_pos.size(), tneg = featBlock_neg.size();
    int tepos = test_pos.size(), teneg = test_neg.size();
    int featlen = 0;
    for(int i = 0; i < featBlock_pos[0].size(); ++i)
        for(int j = 0; j < featBlock_pos[0][i].size(); ++j)++featlen;
    labelMat = Mat::zeros(tpos+tneg, 1, CV_32FC1);
    instanceMat = Mat::zeros(tpos+tneg, featlen, CV_32FC1);
    testPMat = Mat::zeros(tepos,featlen, CV_32FC1);
    testNMat = Mat::zeros(teneg, featlen, CV_32FC1);
    
    for(int i = tpos; i < tpos+tneg; ++i) labelMat.at<float>(i,0)=-1;
    for(int i = 0; i < tpos; ++i){
        vector<float>feat;
        for(int j = 0; j < featBlock_pos[i].size(); ++j)
            feat.insert(feat.end(), featBlock_pos[i][j].begin(), featBlock_pos[i][j].end());
        for(int k = 0; k < feat.size(); ++k)
            instanceMat.at<float>(i,k) = feat[k];
    }
    for(int i = 0; i < tneg; ++i){
        vector<float>feat;
        for(int j = 0; j < featBlock_neg[i].size(); ++j)
            feat.insert(feat.end(), featBlock_neg[i][j].begin(), featBlock_neg[i][j].end());
        for(int k = 0; k < feat.size(); ++k)
            instanceMat.at<float>(i+tpos,k) = feat[k];
    }
    for(int i = 0; i < tepos; ++i){
        vector<float>feat;
        for(int j = 0; j < test_pos[i].size(); ++j)
            feat.insert(feat.end(), test_pos[i][j].begin(), test_pos[i][j].end());
        for(int k = 0; k < feat.size(); ++k)
            testPMat.at<float>(i,k) = feat[k];
    }
    for(int i = 0; i < teneg; ++i){
        vector<float>feat;
        for(int j = 0; j < test_neg[i].size(); ++j)
            feat.insert(feat.end(), test_neg[i][j].begin(), test_neg[i][j].end());
        for(int k = 0; k < feat.size(); ++k)
            testNMat.at<float>(i,k) = feat[k];
    }    
}

void predictMotionClassifier( const CvSVM &svm, const Mat &label, const Mat &instance)
{
	Mat response;
	svm.predict( instance, response);

	float fp = 0, fn = 0, tp = 0, tn = 0;
    float gp = 0, gn = 0;
	for ( int n = 0; n < instance.rows; ++n){
		if ( label.at<float>(n,0) * response.at<float>(n,0)>= 0){
            if(label.at<float>(n, 0) > 0) ++tp;
            else ++tn;
        }else{
            if(label.at<float>(n,0) > 0) ++fn;
            else ++fp;
        }
        if(label.at<float>(n,0) > 0) ++gp; else ++gn;
    }

    printf("[# instance:%d; # pos:%lf; #neg:%lf]\n",label.rows,gp,gn);

    printf("\n \t1\t-1\n 1\t%lf\t%lf\n -1\t%lf\t%lf\n=======\n",tp,fp,fn,tn);
	float acc = float(tp+tn) / label.rows;
	printf("acc:%f\n", acc);
}

void writeSVMData(const Mat&data, const Mat&label, string pname){
    FILE *fp = fopen(pname.c_str(), "w+");
    for(int i = 0; i < data.rows; ++i){
        fprintf(fp,"%d ", (int)label.at<float>(i, 0));
        for(int j = 0; j < data.cols; ++j)fprintf(fp, "%d:%lf ", j+1, data.at<float>(i,j));
        fprintf(fp,"\n");
    }
    fclose(fp);
}

void writeSVMData(const Mat&data, const Mat&label, string pname, Point2i size, Point2i offset){
    FILE *fp = fopen(pname.c_str(), "w+");
    int i = 0, j = 0, ipos = 0, ineg = 0, idx = 0;
    while(idx < data.rows){
        if(label.at<float>(idx, 0) > 0){ 
            if(i < offset.x) ++i;
            else if(!ipos) ipos = idx;
        }else{ 
            if(j < offset.y) ++j;
            else if(!ineg) ineg = idx;
        }
        if(ipos && ineg) break;
        ++idx;
    }
    for(int i = min(ipos, ineg), m = 0, n = 0; i < data.rows; ++i, ++m, ++n){
        if(label.at<float>(i, 0) > 0) ++m;
        else ++n;
        if(m > size.x && label.at<float>(i, 0) > 0) continue;
        if(n > size.y && label.at<float>(i, 0) < 0) continue;
        fprintf(fp,"%d ", (int)label.at<float>(i, 0));
        for(int j = 0; j < data.cols; ++j)fprintf(fp, "%d:%lf ", j+1, data.at<float>(i,j));
        fprintf(fp,"\n");
    }
    fclose(fp);
}

void trainMotionClassifier( vector<vector<vector<float> > > feat_pos, 
        vector<vector<vector<float> > > feat_neg, 
        vector<vector<vector<float> > > test_pos,
        vector<vector<vector<float> > > test_neg,
        Mat &center, 
        CvSVM &svm, int tStartId, vector<Point2i> &nsample){
    
	printf("Train SVM\n");
	Mat svmlsMat,tlpMat, tlnMat, svmTrainMat, testPMat, testNMat;
//	CvMat *TrainMat = NULL, *tPMat= NULL, *tNMat= NULL;

	featVec2Mat2( feat_pos,feat_neg,test_pos, test_neg, svmlsMat, svmTrainMat, testPMat, testNMat, center);
    ostringstream ncluster; ncluster << NCLUSTER;
    string tname = "train"+ncluster.str()+".txt";
    writeSVMData(svmTrainMat, svmlsMat, tname);
    tlpMat = Mat::ones(testPMat.rows, 1, CV_32FC1);
//    predictMotionClassifier(svm, tlpMat, testPMat);
    tlnMat = Mat::ones(testNMat.rows, 1, CV_32FC1)*(-1.0f);
//    predictMotionClassifier(svm, tlnMat, testNMat);
    Mat tMat, lMat;
    vconcat(testPMat, testNMat, tMat);
    vconcat(tlpMat, tlnMat, lMat);
   
    Point2i offset(0, 0);
    //for(int i = tStartId; i < 12; ++i){
    //    ostringstream str; str << i;
    string tsname = "test"+ncluster.str()+".txt";
    writeSVMData(tMat, lMat, tsname);//, nsample[i], offset);
        //offset.x += nsample[i].x, offset.y += nsample[i].y;
    //}
    
//    svmTrainMat = Mat(TrainMat), testPMat = Mat(tPMat), testNMat = Mat(tNMat);
	CvSVMParams params = svm.get_params();
	params.svm_type    = CvSVM::C_SVC;
	params.kernel_type = CvSVM::RBF;
	params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 10000, 1e-10);
	params.C = 32768;
    params.gamma = 8;
    CvParamGrid CvParamGrid_C(pow(2.0,-5), pow(2.0,32), pow(2.0,1));
    CvParamGrid CvParamGrid_gamma(pow(2.0,-15), pow(2.0,5), pow(2.0,1));
    if (!CvParamGrid_C.check() || !CvParamGrid_gamma.check())
        cout<<"The grid is NOT VALID."<<endl;
    svm.train_auto(svmTrainMat, svmlsMat, Mat(), Mat(), params, 5, CvParamGrid_C, CvParamGrid_gamma, CvSVM::get_default_grid(CvSVM::P), CvSVM::get_default_grid(CvSVM::NU), CvSVM::get_default_grid(CvSVM::COEF), CvSVM::get_default_grid(CvSVM::DEGREE), true);
//    svm.train_auto(svmTrainMat, svmlsMat, Mat(), Mat(), params, 10);
//    svm.train(svmTrainMat, svmlsMat, Mat(), Mat(), params);
    predictMotionClassifier(svm, svmlsMat, svmTrainMat);
	printf("Training SVM... finished\n");
    predictMotionClassifier(svm, lMat, tMat);
}

inline bool vcomp(const vector<float>&v1, const vector<float>&v2){
    int l1, l2;
    for(int i = 0; i < v1.size(); ++i){
        l1 += v1[i]*v1[i]; l2 += v2[i]*v2[i];
    }
    return l1 < l2;
}

void motionRecogHOF(int gid, int vid){
    
    int trainGameID = gid;
    vector<int> label;
    ldLabel(trainGameID, label);                                        
    vector<vector<float> > feat_pos, feat_neg;
    optiflowDescriptor( trainGameID, vid, label, feat_pos,feat_neg);
}

void motionRecogHOF( )
{
	Mat center;
	CvSVM SVM ;
    vector<vector<vector<float> > >feat_pos, feat_neg,test_pos, test_neg;
    int nTrain = 3; // training size
    const char *_chTPath="/home/fengzy/Projects/XProject/dataset/Set%.02d/feature/%d.txt";
    for(int gid = 0; gid < nTrain; ++gid){
        vector<int> label;
        ldLabel(gid, label);
        for(int i = 0; i < label.size(); ++i){
            char buf[512];
            sprintf(buf, _chTPath, gid, i+1);
            ifstream infile(buf);
            if(!infile.is_open()){ 
                printf("open file failed[%s]\n",buf);continue;
            }
            vector<vector<float> >video;
            while(!infile.eof()){
                vector<float> frame; float e;
                for(int j = 0; !infile.eof() && j < 9; ++j) {
                    infile >> e; frame.push_back(e);
                }
                if(frame.size() == 9) video.push_back(frame);
                else 
                    printf("invalid length[%d]\n",frame.size());
            }

            if(label[i] >= 0 && label[i] <= 50) feat_pos.push_back(video);
            else if(label[i] > 50) feat_neg.push_back(video);
        }
    }
    vector<Point2i> nsample(12);
    for(int gid = 2+nTrain; gid < 12; ++gid){
        vector<int> label;
        ldLabel(gid, label); 
        Point2i sz(0,0);
        for(int i = 0; i < label.size(); ++i){
            char buf[512];
            sprintf(buf, _chTPath, gid, i+1);
            ifstream infile(buf);
            if(!infile.is_open()) continue;
            vector<vector<float> >video;
            while(!infile.eof()){
                vector<float> frame; float e;
                for(int j = 0; !infile.eof() && j < 9; ++j) {
                    infile >> e; frame.push_back(e);
                }
                if(frame.size() == 9) video.push_back(frame);
            }
            infile.close();

            if(label[i] >= 0 && label[i] <= 50){ ++sz.x; test_pos.push_back(video);}
            else if(label[i] > 50){ ++sz.y; test_neg.push_back(video);}
        }
        nsample[gid] = sz;
    }
    FILE *fp = fopen("GroundTruth.txt","w+");
    for(int i = nTrain; i < DATASET_SIZE; ++i){
        for(int j = 0; j < nsample[i].x; ++j)
            fprintf(fp,"%d\t%d\n", i, 1);
    }
    for(int i = nTrain; i < DATASET_SIZE; ++i){
        for(int j = 0; j < nsample[i].y; ++j)
            fprintf(fp,"%d\t%d\n", i, -1);
    }
    fclose(fp);
    trainMotionClassifier( feat_pos, feat_neg, test_pos, test_neg, center, SVM, 2+nTrain, nsample);

}


