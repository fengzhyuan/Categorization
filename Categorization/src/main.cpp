/*
 * main.cpp
 *
 *  Created on: May 15, 2013
 *      Author: fengzh
 */


#include "main.h" 

int main (int argc, char** argv)
{
    setvbuf(stdout,0,_IONBF,0);
    if(argc < 3){
    	printf("invalid format: ./bin <dataset_id> <type> \n");
    	printf("<type> : \n1: early_detection\n2:early_regression\n[other]Output ConfusionMat\n");
    	exit(-1);
    }
    int gid = atoi(argv[1]);//, vid = atoi(argv[2]);
    int type = atoi(argv[2]);
    
    if(type == 1)
    EarlyDetection(gid);
    else if(type == 2)
    EarlyRegression(gid);
    else 
    ConfusionMatrix(gid);
    /***********************/
    return 0;
}


