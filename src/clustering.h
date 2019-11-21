#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <map>
#include <algorithm>
#include <assert.h>

using namespace std;

const int NOISE = -2;
const int NOT_CLASSIFIED = -1;

class ClusterPoint {
public:
    //double x, y;
    ClusterPoint(vector<double>& data):_data(data),ptsCnt(0),cluster(-1){}
    vector<double> _data;
    int ptsCnt, cluster;
    double getDis(const ClusterPoint & ot);
};

class DBCAN {
public:
    int n, minPts;
    double eps;
    vector<ClusterPoint> points;
    int size;
    vector<vector<int> > adjPoints;
    vector<bool> visited;
    vector<vector<int> > vv_cluster;
    int clusterIdx;
    
    DBCAN(int n, double eps, int minPts, vector<ClusterPoint> points) {
        this->n = n;
        this->eps = eps;
        this->minPts = minPts;
        this->points = points;
        this->size = (int)points.size();
        adjPoints.resize(size);
        this->clusterIdx=-1;
    }
    void run ();
    
    void dfs (int now, int c);
    
    //compute point's num with in distance eps
    void checkNearPoints();
    // is idx'th point core object?
    bool isCoreObject(int idx) {
        return points[idx].ptsCnt >= minPts;
    }
    
    vector<vector<int> > getCluster() {
        return vv_cluster;
    }
};

class InputReader {
private:
    ifstream fin;
    vector<ClusterPoint> points;
public:
    InputReader(string filename) {
        fin.open(filename);
        if(!fin) {
            cout << filename << " file could not be opened\n";
            exit(0);
        }
        parse();
    }
    void parse();
};


class OutputPrinter {
private:
    ofstream fout;
    vector<vector<int> > cluster;
    string filename;
    int n;
public:

    static bool compare_my(const vector<int>& a, const vector<int>& b)
    {
        return a.size() > b.size();
    }
    OutputPrinter(int n, string filename, vector<vector<int> > cluster) {
        this->n = n;
        this->cluster = cluster;
        
        // remove ".txt" from filename
        if(filename.size()<4){
            cout << filename << "input file name's format is wrong\n";
            exit(0);
        }
        for(int i=0;i<4;i++) filename.pop_back();
        this->filename = filename;
        
        // sort by size decending order
        // sort(cluster.begin(), cluster.end(), [&](const vector<int> i, const vector<int> j) {
        //     return (int)i.size() > (int)j.size();
        // });
        sort(cluster.begin(), cluster.end(), compare_my);
    }
    void print() {
        for(int i=0;i<n;i++) {
            fout.open(filename+"_cluster_"+to_string(i)+".txt");
            
            for(int j=0;j<cluster[i].size();j++) {
                fout << cluster[i][j] << endl;
            }
            
            fout.close();
        }
    }

};

/*
int main(int argc, const char * argv[]) {
    if(argc!=5) {
        cout << "Please follow this format. clustering.exe [intput] [n] [eps] [minPts]";
        return 0;
    }
    
    string inputFileName(argv[1]);
    string n(argv[2]);
    string eps(argv[3]);
    string minPts(argv[4]);
    
    InputReader inputReader(inputFileName);
    
    DBCAN dbScan(stoi(n), stod(eps), stoi(minPts), inputReader.getPoints());
    dbScan.run();
    
    OutputPrinter outputPrinter(stoi(n), inputFileName, dbScan.getCluster());
    outputPrinter.print();
    
    return 0;
}
*/
