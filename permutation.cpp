// next_permutation example
#include <iostream>     // std::cout
#include <algorithm>    // std::next_permutation, std::sort
#include <vector>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <limits>
#include <fstream>
#include <set>
#include <utility>
#include <regex>
using namespace std;

int main (int argc , char *argv[]) {
  /*string mypaths[] = {"1\n30","3\n33","8\n31","11\n34","9\n28"};
  //vector <pair < int,int > > paths ;
  std::sort (mypaths,mypaths+5);
  fstream myfile;
  fstream myfile_output;
  fstream myfile_trunc;
  //myfile_output.open("astar_output.txt",ios::out);//|ios::trunc);
  myfile.open("waypoints.txt",ios::out);//|ios::trunc);
int c;
  std::cout << "The 5! possible permutations with 5 elements:\n"; 
  do {
    //myfile.open("waypoints.txt",ios::out);//|ios::trunc);

    myfile << mypaths[0] ;
    myfile << "\n";
    myfile << "\n";
    myfile << mypaths[1] ;
    myfile << "\n";
    myfile << "\n";
    myfile << mypaths[2] ;
    myfile << "\n";
    myfile << "\n";
    myfile << mypaths[3] ;
    myfile << "\n";
    myfile << "\n";
    myfile << mypaths[4] ;
    myfile << "\n";
    myfile << "\n";
    
    myfile.close();
c++;
    //myfile_trunc.open("waypoints.txt",ios::out|ios::trunc);

    //myfile_trunc.close();

  }while ( std::next_permutation(mypaths,mypaths+5) );
  cout << c<<endl;;
    //myfile.close();
    //myfile_trunc.close(); .erase(std::remove(input.begin(), input.end(), '\n'), input.end());
    string prova ="1\n2";
    regex new_line ("\n+");

  auto result = std::regex_replace(prova, new_line, "");
  //prova.erase(remove(prova.begin(),prova.end(),"\n"),prova.end());
  cout << "   "<< result <<endl;

  vector < vector < int > > prova2;*/
  vector < int > test;
  //vector < int > test2;
  test.push_back(51);
  test.push_back(53);
  test.push_back(57);
  test.push_back(60);
  test.push_back(64);
  test.push_back(70);

  float media = (51+53+57+60+64+70)/6;
  float sigma = 0;
  float partial_sigma = 0;
  /*test2.push_back(3);
  prova2.push_back(test);
  prova2.push_back(test2);

  prova2.clear();

  test.clear();
  if(prova2.empty()) cout << "ocnfev" << endl;*/
  //float x = atof(argv[1]);
  for(int i=0;i<test.size();i++){
    cout << i <<endl;
    partial_sigma = test[i]-media;
    sigma = sigma + pow(partial_sigma,2);
  }
  sigma = sqrt(sigma/test.size());
  cout << sigma << endl;
}