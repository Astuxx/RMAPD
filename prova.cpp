#include <stdio.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <bits/stdc++.h>
//#include "boost/graph/adjacency_list.hpp"
//#include "boost/graph/topological_sort.hpp"
//#include "boost/graph/graphviz.hpp"
#include "fstream"
#include <set>
#include <utility>
//#include <boost/test/minimal.hpp>
#include <string>
//#include <boost/graph/iteration_macros.hpp>
#include <limits>
using namespace std;
//using namespace boost;
/*
typedef boost::adjacency_list<vecS, vecS, directedS,
                              property<vertex_name_t, std::string>,
                              property<edge_weight_t, double> > Digraph;

typedef boost::adjacency_list<vecS, vecS, undirectedS,
                              property<vertex_name_t, std::string>,
                              property<edge_weight_t, double> > Graph;

*/
ofstream myfile;


vector<vector<unsigned int> > getw(const char* wayp_file){

vector<vector<unsigned int> > wayp;
vector < unsigned int > waypointsR;
    unsigned int i, j;
    int temp;
    int start,goal;
    vector <int> goals;
    int r;
    int c;
    FILE *file;
    file = fopen(wayp_file, "r");
    int counter_wayp=0;
    
    ifstream File(wayp_file);
    string s;
    istringstream is(s);
    //if (file != NULL)
        //{
            for (i = 0; i < 3; i++){

                c = 2;
                while(getline(File, s) && !s.empty())
                {
                    r = fscanf(file, "%d",&temp);
                    //cout << "fsdwcdr" << temp;
                    
                    if(c == 3){
                        //cout << "altro" << temp;
                        //cout << endl;
                        waypointsR.push_back(temp);
                    }

                    if(c == 2){
                        myfile << temp;
                        myfile << " [label= ";
                        myfile << "R";
                        myfile << i;
                        myfile << "];\n";
                        waypointsR.push_back(temp);
                        cout << "start" << temp;
                        c=3;
                    }
                }
                cout << "\n goal" << waypointsR[waypointsR.size()-1];
                myfile << waypointsR[waypointsR.size()-1];
                myfile << " [label= ";
                myfile << "G";
                myfile << i;
                myfile << "];\n";
                
                wayp.push_back(waypointsR);
                waypointsR.clear();
                counter_wayp = 0;
                cout << endl;
            }
            /*for (i = 0; i < TEAM_SIZE; i++){
        while(getline(File, s) && !s.empty()){
        
            r = fscanf(file, "%d",&temp);
            waypointsR.push_back(temp);
            //cout << temp << endl;
            counter_wayp = counter_wayp+1;
        }
        //cout << endl;
    
    wayp.push_back(waypointsR);
    waypointsR.clear();
    counter_wayp = 0;
    }*/
            fclose(file);

    return wayp;
}


int main (int argc,char *argv[]){

    myfile.open("/home/riccardoastolfi/minisim2/test_graph.gv");
    myfile << "graph G{\n"; 

string file = "src/logistic_sim/maps/"+string(argv[1])+"/waypoints.txt";
cout << file << endl;
vector<vector<unsigned int> > vec = getw(file.c_str());


    set<pair<int,int>> edge_list;
    //set<pair<int,int>> std::iterator it;
    set<int> vertex_list;
    //set<int> std::iterator ot;
    pair <int,int> P;
    pair <int,int> temp;

    
    P.first=1;
    P.second=2;
    vertex_list.insert(P.first);
    vertex_list.insert(P.second);
    edge_list.insert(make_pair(P.first,P.second));
    //auto 
    decltype(edge_list)::iterator it;
    decltype(edge_list)::iterator itr;

    P.first=1;
    P.second=3;
    it = edge_list.find(pair<int,int>(P.first,P.second));
    itr = edge_list.find(pair<int,int>(P.second,P.first));
    //vertex_list.find(P.first);
    

    //for(auto const &var : edge_list){
        if (vertex_list.find(P.first)== vertex_list.end() && ( vertex_list.find(P.second)== vertex_list.end())){
            vertex_list.insert(P.first);
            vertex_list.insert(P.second);
            
            edge_list.insert(make_pair(P.first,P.second));
        }
        else if(vertex_list.find(P.first)!= vertex_list.end() && vertex_list.find(P.second)== vertex_list.end()) {
            vertex_list.insert(P.second);
            
            edge_list.insert(make_pair(P.first,P.second));
        }
        else if(vertex_list.find(P.second)!= vertex_list.end() && vertex_list.find(P.first)== vertex_list.end()) {
            vertex_list.insert(P.second);
            
            edge_list.insert(make_pair(P.first,P.second));
        }
        else if(vertex_list.find(P.second)!= vertex_list.end() && vertex_list.find(P.first)!= vertex_list.end()) {
            if((it == edge_list.end()) && (itr == edge_list.end())){
                cout << "test" << endl;
                edge_list.insert(make_pair(P.first,P.second));
            }
        }


    //}
      
    P.first=2;
    P.second=3;
    
        if (vertex_list.find(P.first)== vertex_list.end() && ( vertex_list.find(P.second)== vertex_list.end())){
            vertex_list.insert(P.first);
            vertex_list.insert(P.second);
            
            edge_list.insert(make_pair(P.first,P.second));
        }
        else if(vertex_list.find(P.first)!= vertex_list.end() && vertex_list.find(P.second)== vertex_list.end()) {
            vertex_list.insert(P.second);
            
            edge_list.insert(make_pair(P.first,P.second));
        }
        else if(vertex_list.find(P.second)!= vertex_list.end() && vertex_list.find(P.first)== vertex_list.end()) {
            vertex_list.insert(P.second);
            
            edge_list.insert(make_pair(P.first,P.second));
        }
        else if(vertex_list.find(P.second)!= vertex_list.end() && vertex_list.find(P.first)!= vertex_list.end()) {
            if((it == edge_list.end()) && (itr == edge_list.end())){
                cout << "test" << endl;
                edge_list.insert(make_pair(P.first,P.second));
            }
        }
    
    P.first = 4;
    P.second = 3;
    //it = edge_list.find(pair<int,int>(P.first,P.second));
    //itr = edge_list.find(pair<int,int>(P.second,P.first));
    
        if (vertex_list.find(P.first) == vertex_list.end() && ( vertex_list.find(P.second) == vertex_list.end())){
            vertex_list.insert(P.first);
            vertex_list.insert(P.second);
            edge_list.insert(make_pair(P.first,P.second));
        }
        else if(vertex_list.find(P.first)!= vertex_list.end() && vertex_list.find(P.second)== vertex_list.end()) {
            vertex_list.insert(P.second);
            
            edge_list.insert(make_pair(P.first,P.second));
        }
        else if(vertex_list.find(P.second)!= vertex_list.end() && vertex_list.find(P.first)== vertex_list.end()) {
            vertex_list.insert(P.first);
            edge_list.insert(make_pair(P.first,P.second));
        }
        else if(vertex_list.find(P.second)!= vertex_list.end() && vertex_list.find(P.first)!= vertex_list.end()) {
            if((it == edge_list.end()) && (itr == edge_list.end())){
                edge_list.insert(make_pair(P.first,P.second));
            }
        }


  

    
    P.first=2;
    P.second=1;
    it = edge_list.find(pair<int,int>(P.first,P.second));
    itr = edge_list.find(pair<int,int>(P.second,P.first));

    for(auto const &var : vertex_list){
            cout << "(" << var << ")\n";
        }

    
        if (vertex_list.find(P.first)== vertex_list.end() && ( vertex_list.find(P.second)== vertex_list.end())){
            vertex_list.insert(P.first);
            vertex_list.insert(P.second);
            edge_list.insert(make_pair(P.first,P.second));
        }
        else if(vertex_list.find(P.first)!= vertex_list.end() && vertex_list.find(P.second)== vertex_list.end()) {
            vertex_list.insert(P.second);
            cout << "3" << endl;
            edge_list.insert(make_pair(P.first,P.second));
        }
        else if(vertex_list.find(P.second)!= vertex_list.end() && vertex_list.find(P.first)== vertex_list.end()) {
            vertex_list.insert(P.second);
            edge_list.insert(make_pair(P.first,P.second));
        }
        else if(vertex_list.find(P.second)!= vertex_list.end() && vertex_list.find(P.first)!= vertex_list.end()) {
            if((it == edge_list.end()) && (itr == edge_list.end())){
                cout << "testjhvjk" << endl;
                edge_list.insert(make_pair(P.first,P.second));
            }
        }
    


        for(auto const &var : edge_list){
            cout << "(" << var.first << "," << var.second << ")\n";
        }

       


    
    
    myfile << " }";
    myfile.close();
    system("neato -Tpng test_graph.gv -o test_graph.png");
    system("display test_graph.png");


}