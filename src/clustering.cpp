#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <map>
#include <algorithm>
#include <assert.h>
#include "clustering.h"

using namespace std;


double ClusterPoint::getDis(const ClusterPoint & ot) {
	double dis = 0.;
	assert(ot._data.size() == _data.size());
	for(int i=0; i<_data.size(); i++)
	{
		dis += pow(_data[i] - ot._data[i], 2);
	}
	return sqrt(dis);
}


void DBCAN::run () {
	checkNearPoints();

	for(int i=0;i<size;i++) {
		//cout<<"points[i].cluster = "<<points[i].cluster<<endl;
	    if(points[i].cluster != NOT_CLASSIFIED) 
		{
			//cout<<"point = NOT_CLASSIFIED"<<endl;
			continue;
		}
	    if(isCoreObject(i)) {
			//cout<<"point = dfs"<<endl;
			dfs(i, ++clusterIdx);
	    } else {
			//cout<<"point = NOISE"<<endl;
			points[i].cluster = NOISE;
	    }
	}

	vv_cluster.resize(clusterIdx+1);
	for(int i=0;i<size;i++) {
	    if(points[i].cluster != NOISE) {
		vv_cluster[points[i].cluster].push_back(i);
	    }
	}
}
    
void DBCAN::dfs (int now, int c) {
	points[now].cluster = c;
	if(!isCoreObject(now)) return;

	for(auto&next:adjPoints[now]) {
	    if(points[next].cluster != NOT_CLASSIFIED) continue;
	    dfs(next, c);
	}
}
    
//compute point's num with in distance eps
void DBCAN::checkNearPoints() {
	for(int i=0;i<size;i++) {
	    for(int j=0;j<size;j++) {
			if(i==j) continue;
			//cout<<points[i]._data[0]<<" --" <<points[j]._data[0]<<"  distance = "<<points[i].getDis(points[j])<<endl;
			if(points[i].getDis(points[j]) <= eps) {
				points[i].ptsCnt++;
				adjPoints[i].push_back(j);
			}
	    }
	}
}



