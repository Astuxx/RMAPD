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
#include <iomanip>
using namespace std;

unsigned int WIDTH_PX;
 unsigned int HEIGHT_PX;
 float RESOLUTION;
 float OFFSET_X;
 float OFFSET_Y;
  
  unsigned int ****prev_paths;
  unsigned int ***path_sizes;
  unsigned int ***visited;
  vector< vector< unsigned int > > map_graph;
  vector<vector< unsigned int> > vec;
  vector< unsigned int> temp;

struct vertex{
    unsigned int id, num_neigh;
    float x,y;
    unsigned int id_neigh[8], cost[8];
    double cost_m[8];
    bool visited[8];
    char dir[8][3];
};

struct st_location
{
  unsigned int vertex;
  unsigned int time;
  unsigned int waypoint;

  st_location(unsigned int vertex = 0, unsigned int time = 0, unsigned int waypoint = 0)
    : vertex(vertex), time(time), waypoint(waypoint)
  {
  }
  st_location(const st_location &ref) : vertex(ref.vertex), time(ref.time), waypoint(ref.waypoint)
  {
  }
  bool operator<(const st_location &loc) const
  {
    if (time < loc.time)
    {
      return true;
    }
    else if (time == loc.time)
    {
      if (waypoint > loc.waypoint)
      {
        return true;
      }
    }
    return false;
  }
};

st_location *queue;
vertex *vertex_web;

int FIRST_VID = 7;
unsigned int TEAM_SIZE;
string modalita;
unsigned int dimension;
const unsigned int WHITE = 0, GRAY = 1, BLACK = 2, MAX_TIME = 300U, MAX_WAYPOINTS = 64U;
vector < unsigned int > waypoints;

ofstream myfile;
int guard=0;
set<pair<int,int>> edge_list;
set<int> vertex_list;
pair <int,int> P;
decltype(edge_list)::iterator it;
decltype(edge_list)::iterator itr;
vector <int> tmp_casa ;



template <class T>
vector<unsigned int> spacetime_dijkstra(//const vector< Path > &other_paths, 
                                                          const vector <vector <unsigned int> > &vec,
                                                          const vector<vector<unsigned int> > &graph,
                                                          const vector<unsigned int> &waypoints,int start_time,
                                                          vector<unsigned int> *last_leg,
                                                          vector<unsigned int> *first_leg, 
                                                          T *cmp_function,int ID_ROBOT)
                                                          
{
  
  /*for(int l = 0; l<TEAM_SIZE;l++){
        for(int k = 0;k<vec[l].size();k++){
        cout << "  "<< vec[l][k];
        }cout << endl;
      }*/
  
  /*cout << "SPACETIME DIJKSTRA --- WAYPOINTS:";
  for (int i = 0; i < waypoints.size(); i++)
  {
    cout << " " << waypoints[i];
  }
  */
  //cout <<endl ;
  
  unsigned int source = waypoints.front();
  
  auto it_waypoints = waypoints.begin() +1;
  // inizializzazione strutture
  
  for (unsigned int i = 0; i < graph.size(); i++)
  {
    for (unsigned int j = 0; j < MAX_TIME; j++)
    {
      for (unsigned int k = 0; k < MAX_WAYPOINTS; k++)
      {
        path_sizes[i][j][k] = 0;
      }
    }
  }
  prev_paths[source][0][1][0] = source;
  path_sizes[source][0][1] = 1;

  for (unsigned int i = 0; i < graph.size(); i++)
  {
    for (unsigned int j = 0; j < MAX_TIME; j++)
    {
      for (unsigned int k = 0; k < MAX_WAYPOINTS; k++)
      {
        visited[i][j][k] = WHITE;
      }
    }
  }
  
  // inizializzazione coda
  for (unsigned int i = 0; i < MAX_TIME; i++)
  {
    queue[i] = st_location();
  }
  st_location st(source, start_time, 1);
  queue[0] = st;
  int queue_size = 1;

  // individuo la lunghezza del percorso maggiore
  unsigned int max_path_size = 0;
  for (int i = 0; i < TEAM_SIZE; i++)
  {
      //cout <<" size: " << vec[i].size() << endl;
    if (i != ID_ROBOT && vec[i].size() > max_path_size) //
    {
      max_path_size = vec[i].size();
    }
  }
  
  while (queue_size > 0)
  {
    st_location current_st = queue[--queue_size];
    unsigned int u = current_st.vertex;
    unsigned int time = current_st.time - start_time;
    unsigned int current_waypoint = current_st.waypoint;
    unsigned int next_next_waypoint;

    visited[u][time][current_waypoint] = BLACK;

    if (u == waypoints[current_waypoint])
    {
      if (current_waypoint + 1 == waypoints.size())
      {
        bool good = true;
        for (int i=0; i<TEAM_SIZE && good; i++)
        {
          if (i != ID_ROBOT && vec[i].size() > current_st.time + 1)
          {
            for (int j=current_st.time; j<vec[i].size(); j++)
            {
              if (vec[i][j] == u)
              {
                good = false;
                break;
              }
            }
          }
        }
        
        //if (time + 1 + start_time >= max_path_size)
        if (good)
        {
          vector<unsigned int> result =
              vector<unsigned int>(prev_paths[u][time][current_waypoint],
                                        prev_paths[u][time][current_waypoint] + path_sizes[u][time][current_waypoint]);
          ///*
          // spezzo il percorso in due parti
          if (last_leg != nullptr && first_leg != nullptr)
          {
            last_leg->clear();
            first_leg->clear();
            bool last_part = true;
            for (auto it = result.rbegin(); it != result.rend(); it++)
            {
              if (*it == waypoints[current_waypoint - 1])
              {
                last_part = false;
              }

              if (last_part)
              {
                last_leg->insert(last_leg->begin(), *it);
              }
              else
              {
                first_leg->insert(first_leg->begin(), *it);
              }
            }
          }
          //*/
          return result;
        }
        else
        {
          next_next_waypoint = current_waypoint;
        }
      }
      else
      {
        next_next_waypoint = current_waypoint + 1;
      }
    }
    else  // u non è waypoint
    {
      next_next_waypoint = current_waypoint;
    }

    // considero l'attesa sul posto
    unsigned int next_time = time + 1;
    if (next_time < MAX_TIME && visited[u][next_time][next_next_waypoint] == WHITE)
    {
      bool good = true;
      for (int i = 0; i < TEAM_SIZE; i++)
      {
        if (i != ID_ROBOT)
        {
          if (next_time + start_time < vec[i].size())
          {
            if (vec[i][next_time + start_time] == u)
            {
              // cout << "can't stand in " << u << " at time " << time << endl;
              good = false;
              break;
            }
          }
          else if (!vec[i].empty() /* && still_robots[i] */
          
          
           
          )

         


          {
            if (vec[i].back() == u)
            {
              // cout << "can't stand in " << u << " at time " << time << endl;
              good = false;
              break;
            }
          }
        }
      }

      if (good)
      {
        st_location next_st(u, next_time + start_time, next_next_waypoint);
        queue_size = insertion_sort(queue, queue_size, next_st, cmp_function);
        //cout << "o" << endl;
        visited[u][next_time][next_next_waypoint] = GRAY;

        unsigned int psize = path_sizes[u][time][current_waypoint];
        for (unsigned int i = 0; i < psize; i++)
        {
          prev_paths[u][next_time][next_next_waypoint][i] = prev_paths[u][time][current_waypoint][i];
        }
        
        prev_paths[u][next_time][next_next_waypoint][psize] = u;
        path_sizes[u][next_time][next_next_waypoint] = path_sizes[u][time][current_waypoint] + 1;
        
      }
    }
    //cout <<"test2613"<<endl;
    // considero i vicini
    for (auto it = graph[u].begin(); it != graph[u].end(); it++)
    {
      const unsigned int v = *it;
      const unsigned int next_time = time + 1;

      if (next_time < MAX_TIME && visited[v][next_time][next_next_waypoint] == WHITE)
      {
        bool good = true;
        for (int i = 0; i < TEAM_SIZE; i++)
        {
          if (i != ID_ROBOT)
          {
            if (next_time + start_time < vec[i].size())
            {
              if (vec[i][next_time + start_time] == v)
              {
                // cout << "can't go in " << v << " at time " << time << endl;
                good = false;
                break;
              }
              if (vec[i][next_time + start_time] == u && vec[i][time + start_time] == v)
              {
                // cout << "can't go in " << v << " at time " << time << endl;
                good = false;
                break;
              }
            }
            else if (!vec[i].empty() /* && still_robots[i] */
            
            
            )


            { 
              if (vec[i].back() == v) //return ultimo elemento del vettore
              {
                // cout << "can't go in " << v << " at time " << time << ", there is a still robot" << endl;
                good = false;
                break;
              }
            }
          }
        }
        if (good)
        { 
          st_location next_st(v, next_time + start_time, next_next_waypoint);
          queue_size = insertion_sort(queue, queue_size, next_st, cmp_function); 
          visited[v][next_time][next_next_waypoint] = GRAY;
          unsigned int psize = path_sizes[u][time][current_waypoint];
          for (unsigned int i = 0; i < psize; i++)
          {
            prev_paths[v][next_time][next_next_waypoint][i] = prev_paths[u][time][current_waypoint][i];
          }
          prev_paths[v][next_time][next_next_waypoint][psize] = v;
          path_sizes[v][next_time][next_next_waypoint] = path_sizes[u][time][current_waypoint] + 1;
          
        }
      }
    }
  }
  throw string("Can't find path!!!");
}

template <class T>
unsigned int insertion_sort(st_location *queue, unsigned int size, st_location loc, T *cmp_function)
{
  
  int i;
  for (i = size - 1; i >= 0; i--)
  {
    if (cmp_function != nullptr)
    { 
      bool cmp_result = (*cmp_function)(loc, queue[i]);
      
      if (cmp_result)
        break;
    }
    else if (loc < queue[i])
    {
      break;
    }
    queue[i + 1] = queue[i];
    //cout << "vertex: " << queue->vertex << endl;
  }
  queue[i + 1] = loc;
  // std::cout << std::endl;
  return size + 1;
}


void allocate_memory()
{
  cout << "allocating memory..." << endl;
  cout << dimension << " " << MAX_TIME<< endl;
  
  
  prev_paths = new unsigned int ***[dimension];
  path_sizes = new unsigned int **[dimension];
  
  for (unsigned int i = 0; i < dimension; i++)
  {
    prev_paths[i] = new unsigned int **[MAX_TIME];
    path_sizes[i] = new unsigned int *[MAX_TIME];
    for (unsigned int j = 0; j < MAX_TIME; j++)
    {
      prev_paths[i][j] = new unsigned int *[MAX_WAYPOINTS];
      path_sizes[i][j] = new unsigned int[MAX_WAYPOINTS];
      for (unsigned int k = 0; k < MAX_WAYPOINTS; k++)
      {
        prev_paths[i][j][k] = new unsigned int[MAX_TIME];
      }
    }
  }
   
  visited = new unsigned int **[dimension];
  for (unsigned int i = 0; i < dimension; i++)
  {
    visited[i] = new unsigned int *[MAX_TIME];
    for (unsigned int j = 0; j < MAX_TIME; j++)
    {
      visited[i][j] = new unsigned int[MAX_WAYPOINTS];
    }
  }

  queue = new st_location[MAX_TIME * MAX_TIME];
}


bool astar_cmp_function(const vector < vector < unsigned int > > &min_hops_matrix,
                        const vector<unsigned int> &waypoints, const st_location &lhs, const st_location &rhs)
{
  // uso la min_hops_matrix come euristica sulla lunghezza del percorso rimanente
  unsigned int lhs_h = min_hops_matrix[lhs.vertex][waypoints[lhs.waypoint]];
  unsigned int rhs_h = min_hops_matrix[rhs.vertex][waypoints[rhs.waypoint]];
  // TEST
  // cout << "[DEBUG]\tVertex: " << rhs.vertex << "\tTime: " << rhs.time
  //           << "\tWaypoint: " << waypoints[rhs.waypoint]
  //           << "\tf-value: " << rhs.time + rhs_h << endl;
  if (lhs.time + lhs_h < rhs.time + rhs_h)
  {
    return true;
  }
  // else if (lhs.time + lhs_h == rhs.time + rhs_h)
  // {
  //   if (lhs.waypoint > rhs.waypoint)
  //   {
  //     return true;
  //   }
  // }

  return false;
}



vector< vector< unsigned int > > build_graph()
{
  vector< vector< unsigned int > > result = vector< vector< unsigned int > >(dimension);
  for (int i = 0; i < dimension; i++)
  {
    for (int j = 0; j < vertex_web[i].num_neigh; j++)
    {
      result[i].push_back(vertex_web[i].id_neigh[j]);
    }
  }

  return result;
}


unsigned int GetGraphDimension(const char *graph_file)
{

  FILE *file;
  file = fopen(graph_file, "r");
  unsigned int dimension;

  if (file == NULL)
  {
    printf("Can not open filename %s", graph_file);
    
  }
  else
  {
    printf("Graph File Opened. Reading Dimensions.\n");
    int r;
    r = fscanf(file, "%u", &dimension);

    //Initialize other dimension variables:
    r = fscanf(file, "%u", &WIDTH_PX);
    r = fscanf(file, "%u", &HEIGHT_PX);
    r = fscanf(file, "%f", &RESOLUTION);
    r = fscanf(file, "%f", &OFFSET_X);
    r = fscanf(file, "%f", &OFFSET_Y);
    //WIDTH_M = (float) WIDTH_PX * RESOLUTION;
    //HEIGHT_M = (float) HEIGHT_PX * RESOLUTION;
    fclose(file);
    
  }
  
  return dimension;
}


void GetGraphInfo(vertex *vertex_web, unsigned int dimension, const char *graph_file){
  
  FILE *file;
  file = fopen(graph_file, "r");

  // FILE *debug;
  // debug = fopen("test-map.graph", "w");

    unsigned int i, j;
    float temp;
    int r;

    //Start Reading the File from FIRST_VID On:
    for (i = 0; i < FIRST_VID - 1; i++)
    {
      r = fscanf(file, "%f", &temp);
      // fprintf(debug, "%f\n", temp);
    }
    // fprintf(debug, "\n");

    for (i = 0; i < dimension; i++)
    {

      r = fscanf(file, "%u", &vertex_web[i].id);
      // fprintf(debug, "%u\n", vertex_web[i].id);

      

      r = fscanf(file, "%f", &vertex_web[i].x);
      // fprintf(debug, "%f\n", vertex_web[i].x);
      vertex_web[i].x *= RESOLUTION; //convert to m
      vertex_web[i].x += OFFSET_X;

      r = fscanf(file, "%f", &vertex_web[i].y);
      // fprintf(debug, "%f\n", vertex_web[i].y);
      vertex_web[i].y *= RESOLUTION; //convert to m
      vertex_web[i].y += OFFSET_Y;


      r = fscanf(file, "%u", &vertex_web[i].num_neigh);
      // fprintf(debug, "%u\n", vertex_web[i].num_neigh);

      //( pose [%f %f 0 0] name \"point%u\" color \"black\") \n", vertex_web[i].x, vertex_web[i].y, vertex_web[i].id);
      
      for (j = 0; j < vertex_web[i].num_neigh; j++)
      {
        r = fscanf(file, "%u", &vertex_web[i].id_neigh[j]);
        // fprintf(debug, "%u\n", vertex_web[i].id_neigh[j]);
        //myfile <<" -- "+vertex_web[i].id_neigh[j]+" ;\n";
        P.first=vertex_web[i].id;
        P.second=vertex_web[i].id_neigh[j];
        it = edge_list.find(pair<int,int>(P.first,P.second));
        itr = edge_list.find(pair<int,int>(P.second,P.first));
               
        if (vertex_list.find(P.first)== vertex_list.end() && ( vertex_list.find(P.second)== vertex_list.end())){ // entrambi i nodi NON presenti nel set
            vertex_list.insert(P.first);
            vertex_list.insert(P.second);
            edge_list.insert(make_pair(P.first,P.second));
          //myfile << vertex_web[i].id;
          //myfile << " -- ";
          //myfile << vertex_web[i].id_neigh[j];
          //myfile << ";\n";
        }
        else if(vertex_list.find(P.first)!= vertex_list.end() && vertex_list.find(P.second)== vertex_list.end()) { // il primo nodo c'è già, il secondo no
            vertex_list.insert(P.second);
            edge_list.insert(make_pair(P.first,P.second));
          //myfile << vertex_web[i].id;
          //myfile << " -- ";
          //myfile << vertex_web[i].id_neigh[j];
          //myfile << ";\n";
        }
        else if(vertex_list.find(P.second)!= vertex_list.end() && vertex_list.find(P.first)== vertex_list.end()) { //il primo non c'è, il secondo è già presente
            vertex_list.insert(P.first);
            edge_list.insert(make_pair(P.first,P.second));
          //myfile << vertex_web[i].id;
          //myfile << " -- ";
          //myfile << vertex_web[i].id_neigh[j];
          //myfile << ";\n";
        }
        else if(vertex_list.find(P.second)!= vertex_list.end() && vertex_list.find(P.first)!= vertex_list.end()) { // entrambi i nodi presenti
            if((it == edge_list.end()) && (itr == edge_list.end())){
                edge_list.insert(make_pair(P.first,P.second));
            //myfile << vertex_web[i].id;
            //myfile << " -- ";
            //myfile << vertex_web[i].id_neigh[j];
            //myfile << ";\n";
          }
        }      

        r = fscanf(file, "%s", vertex_web[i].dir[j]);
        // fprintf(debug, "%s\n", vertex_web[i].dir[j]);
        r = fscanf(file, "%u", &vertex_web[i].cost[j]); //can eventually be converted to meters also...
        // fprintf(debug, "%u\n", vertex_web[i].cost[j]);
      }

      // fprintf(debug, "\n");
    }
  

  // printf("[v=10], x = %f (meters)\n", vertex_web[10].x);

  // fclose(debug);
  fclose(file);
}


unsigned int GetNumberEdges(vertex *vertex_web, unsigned int dimension)
{

  unsigned int result = 0;

  for (unsigned int i = 0; i < dimension; i++)
  {
    for (unsigned int j = 0; j < vertex_web[i].num_neigh; j++)
    {
      if (vertex_web[i].id < vertex_web[i].id_neigh[j])
      {
        result++;
      }
    }
  }

  return result;
}

vector < vector < unsigned int > > calculate_min_hops_matrix()
{
  unsigned int infinity = numeric_limits < unsigned int > ::max();
  vector < vector < unsigned int > > result(dimension,
                                                // std::vector<unsigned int>(dimension, 0));
                                                vector<unsigned int>(dimension, infinity));
  for (unsigned int u = 0; u < dimension; u++)
  {
    // self loop is 1 because the robot must wait for the others to do their moves
    result[u][u] = 1;
    for (unsigned int v : map_graph[u])
    {
      result[u][v] = 1;
    }
  }

  for (unsigned int k = 0; k < dimension; k++)
  {
    for (unsigned int i = 0; i < dimension; i++)
    {
      for (unsigned int j = 0; j < dimension; j++)
      {
        if (result[i][k] != infinity && result[k][j] != infinity && result[i][j] > result[i][k] + result[k][j])
        {
          result[i][j] = result[i][k] + result[k][j];
        }
      }
    }
  }

  return result;
}


vector<vector<unsigned int>> GetWaypoints(const char *waypoint_file, string modalita, string ar)
{

  vector<vector<unsigned int> > wayp;
  vector < unsigned int > waypointsR;
    unsigned int i, j;
    int temp,start;
    int r,c;
    FILE *file;
    file = fopen(waypoint_file, "r");
    ifstream File(waypoint_file);
    string s;
    istringstream is(s);
    int d=2;
    string tmp_path;
 
    for (i = 0; i < TEAM_SIZE; i++){
      std::vector<unsigned int> pathR;
      c=2;
        while(getline(File, s) && !s.empty()){
          r = fscanf(file, "%d",&temp);
          if(c == 3){
              waypointsR.push_back(temp);
              //tmp_path = tmp_path+"\n"+to_string(temp);
          }

          if(c == 2){
              start = temp;
              //myfile << start;
              //myfile << " [label= \"";
              //myfile << start;
              //myfile << ",R";
              //myfile << i;
              //myfile << "\"];\n";
              waypointsR.push_back(start);
              //tmp_path = to_string(start);
                          
              if(modalita == "S"){
                pathR.push_back(start);
                vec[i]=pathR; 
              }
              c=3;
          }       
        }
        //tmp_path = tmp_path+"\n"+to_string(waypointsR[waypointsR.size()-1]);
        //myfile << waypointsR[waypointsR.size()-1];
        //myfile << " [label= \"";
        //myfile << waypointsR[waypointsR.size()-1];
        //myfile << ",G";
        //myfile << i;
        //myfile << "\"];\n";
        if (ar == "r"){
          switch (start)
          {
            case 0:
            waypointsR.push_back(0);
            break;
          case 5:
            waypointsR.push_back(5);
            break;
          case 7:
            waypointsR.push_back(7);
            break; 
          case 21:
            waypointsR.push_back(8);
            break;
          case 16:
            waypointsR.push_back(3);
            break;
          case 29:
            waypointsR.push_back(10);
            break;
          case 32:
            waypointsR.push_back(9);
            break;
          case 12:
            waypointsR.push_back(8);
            break;
          case 25:
            waypointsR.push_back(2);
            break;
          case 1:
            waypointsR.push_back(1);
            break;
          case 3:
            waypointsR.push_back(3);
            break;
          case 13:
            waypointsR.push_back(13);
            break;
          case 9:
            waypointsR.push_back(9);
            break;
          case 8:
            waypointsR.push_back(8);
            break;
          case 11:
            waypointsR.push_back(11);
            break;
          case 2:
            waypointsR.push_back(2);
            break;
          case 10:
            waypointsR.push_back(10);
            break;
           
          default:
            break;
          }
        } //rientro a casa, nel nodo iniziale
    
    wayp.push_back(waypointsR);
    //cout << "current path: "<<tmp_path<< endl;
    //mypaths[i]=tmp_path;
    waypointsR.clear();
    }
  fclose(file);
   // riscrivi il fle waypoint con la nuova combinazione
  return wayp;
}

int main(int argc,char *argv[]){
   
  TEAM_SIZE = atoi(argv[2]);
  string modalita =  argv[3];
  string ar = "r";//argv[4];

  ofstream myfile_output;
  myfile_output.open("astar_output2.txt",ios::out);
  float niteration=0;
  fstream myfile_trunc;
  string mypaths[] = {"0\n6","1\n9","7\n3"};
  std::sort (mypaths,mypaths+TEAM_SIZE);
  regex new_line ("\n+");

  if(modalita=="A"){    
    for (int k = 0; k<TEAM_SIZE;k++){
      std::vector<unsigned int> init_vector ;
      vec.push_back(init_vector);
    }
  }
    
  vector<unsigned int> last_leg, first_leg;
  //myfile.open("/home/riccardoastolfi/minisim2/test_graph.gv");
  //myfile << "graph G{\n"; il resto viene scritto dentro getwaypoint

  string mappa = "src/logistic_sim/maps/"+string(argv[1])+"/"+string(argv[1])+".graph";
  string lista_wayp = "src/logistic_sim/maps/"+string(argv[1])+"/waypoints.txt";
  vector<vector<unsigned int> > waypoints;

  /*if(modalita=="S"){
    for(int l = 0; l<TEAM_SIZE;l++){
        cout << "stato iniziale R" << l << ":"; 
        for(int k = 0;k<vec[l].size();k++){
        cout << " "<< vec[l][k];
        }
        cout << endl;
    }
  }*/
  dimension = GetGraphDimension(mappa.c_str());    
  
  vertex_web = new vertex[dimension];
  
  GetGraphInfo(vertex_web, dimension,mappa.c_str());
  //myfile << " }";
  //myfile.close();
  unsigned int nedges = GetNumberEdges(vertex_web, dimension);
  cout << "number of edges "<< nedges << endl;
  cout << "number of nodes "<< dimension << endl;
  //cout << ("Loaded graph %s with %d nodes and %d edges\n", "test", dimension, nedges) << endl;
  map_graph = build_graph();

  vector<vector< unsigned int> > min_hops_matrix = calculate_min_hops_matrix(); 
  allocate_memory();
      
  vector<unsigned int>astar_result;
  vector<unsigned int> init_vector ;
      
  int ok=0;
  int errori = 0;
  int max_timesteps=0;
  vector <string>  tmp_max_steps;
  string tmp_max_array;
  int counter_max_array =0;
  int media_robot_path = 0;
  float media_medie_path = 0;
  float each_robot_media[TEAM_SIZE]={0};
  vector < int > tmp_robot_media;
  float sigma =0;
  vector <float>sigma_collection;

  do{

    myfile_trunc.open(lista_wayp,ios::out);
    for(int f=0;f<TEAM_SIZE;f++){
      myfile_trunc << mypaths[f] ;
      myfile_trunc << "\n";
      myfile_trunc << "\n";
    }
    
    myfile_trunc.close();

    waypoints = GetWaypoints(lista_wayp.c_str(),modalita,ar);
    
    //if(modalita=="A"){
      /*for(int l = 0; l<TEAM_SIZE;l++){
        cout << "stato iniziale R" << l << ":"; 
        for(int k = 0;k<vec[l].size();k++){
          cout << " "<< vec[l][k];
        }
        cout << endl;
      }*/
    //}
    tmp_max_array = "";
    //scrivi sul file l'istanza
    for(int i = 0;i<TEAM_SIZE;i++){
      auto result = std::regex_replace(mypaths[i], new_line, "-");
      myfile_output << result << " ";
      tmp_max_array = tmp_max_array +" " + result;    
    }
    myfile_output << endl;
    myfile_output << endl;

    for (int j=0;j<TEAM_SIZE;j++){
      try{
        auto f = boost::bind(astar_cmp_function, min_hops_matrix, waypoints[j], _1, _2);
        astar_result = spacetime_dijkstra(vec,map_graph, waypoints[j],0,&last_leg,&first_leg,&f,j);
        myfile_output << "\tpath R"<< j << ":" << "("<<astar_result.size() << ") [";
          
        for (int i = 0; i < astar_result.size(); i++){
            myfile_output << " "<< astar_result[i];
        }
        myfile_output << " ]" << endl;
        
        vec[j] = astar_result;

        //statistical data
        if(astar_result.size() >= max_timesteps){
          counter_max_array ++;
          max_timesteps = astar_result.size();
          tmp_max_array + to_string(j);
          tmp_max_steps.push_back(tmp_max_array);
        }

        each_robot_media[j]= each_robot_media[j] + astar_result.size();
        media_robot_path = media_robot_path + astar_result.size();
        sigma_collection.push_back(astar_result.size());

        myfile_output << "\t";
        for(int l = 0; l<=j;l++){
          for(int k = 0;k<vec[l].size();k++){ 
              myfile_output <<setw(3)<< vec[l][k];
          }
          myfile_output << endl;
          myfile_output << "\t";
        }

        myfile_output << endl;

        //plot grafo
        //system("dot -Tpng test_graph.gv -o test_graph.png");
        //system("display test_graph.png");
      
      }catch(string errore){ 
        errori++;
        //scrivi path errore e modalità  
        /*for(int i = 0;i<TEAM_SIZE;i++){
          auto result = std::regex_replace(mypaths[i], new_line, "-");
          myfile_output << result << " ";  
        }*/

        myfile_output << endl;
        myfile_output << "result: R"<< j << " non può completare";
        myfile_output << endl;
        myfile_output << " MODE: " << modalita << endl;
        break;
      }//catch  
    } //fine for (per ogni robot all'interno di un ordine)
    
    media_robot_path = media_robot_path/TEAM_SIZE;
    myfile_output << "timesteps medio corrente: " << media_robot_path << endl;
    media_medie_path = media_medie_path + media_robot_path;
    media_robot_path = 0;  
    myfile_output << "___________________________________________________________________________________________________________" << endl;
    myfile_output << endl;
    myfile_output << endl;
    myfile_output << endl;
    
    //ri inizializza tutta la struttura dopo ogni combinazione
    for (int k = 0; k<TEAM_SIZE;k++){
      vec[k]=init_vector;
    }

    //cout << "n iteration: "<< niteration << endl;
    /*for(int i=0;i<TEAM_SIZE;i++){
      cout << mypaths[i] << endl;
    }*/
    niteration++;

  } while ( std::next_permutation(mypaths,mypaths+TEAM_SIZE));//fine do
  media_medie_path = media_medie_path/niteration;
  myfile_output.close();
  cout << "errori : "<<errori<<endl;
  cout << "max timesteps: " << max_timesteps << " "<< counter_max_array<< "volte" << endl;
  cout << "ottenuto nelle seguenti configurazioni" << endl;
  
  /*for(int l = 0; l<tmp_max_steps.size();l++){
    //cout << "\t "<< tmp_max_steps[l].substr(0,(sizeof(tmp_max_steps[l])/sizeof(tmp_max_steps[l][0]))-1)<<endl;
    cout << "\t "<< tmp_max_steps[l]<<endl;
    //cout <<" Robot R" << tmp_max_steps[l].substr((sizeof(tmp_max_steps[l])/sizeof(tmp_max_steps[l][0]))-2,(sizeof(tmp_max_steps[l])/sizeof(tmp_max_steps[l][0]))-1) << endl;
    cout << endl;
  }*/
  cout << "media medie path: " << media_medie_path << endl;
  
  for(int i=0;i<TEAM_SIZE;i++){
      cout << "robot R"<< i << " path globale medio: " << each_robot_media[i]/niteration << endl; //media dei percorsi fatti da ciascun robot
  }

  //deviazione standard
  for(int i = 0;i<sigma_collection.size();i++){
    sigma = sigma + ( pow((sigma_collection[i]-media_medie_path),2) );
  }
  sigma = sqrt(sigma/niteration);
  cout << "deviazione standard: " << sigma << endl;
}
