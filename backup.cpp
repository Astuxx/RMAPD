#include <stdio.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <limits>
#include <fstream>
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
unsigned int dimension;
const unsigned int WHITE = 0, GRAY = 1, BLACK = 2, MAX_TIME = 300U, MAX_WAYPOINTS = 64U;
vector < unsigned int > waypoints;

template <class T>
vector<unsigned int> spacetime_dijkstra(//const vector< Path > &other_paths, 
                                                          const vector <vector <unsigned int> > &vec,
                                                          const vector<vector<unsigned int> > &graph,
                                                          const vector<unsigned int> &waypoints,int start_time,
                                                          vector<unsigned int> *last_leg,
                                                          vector<unsigned int> *first_leg, 
                                                          T *cmp_function,int ID_ROBOT)
                                                          
{
  
  for(int l = 0; l<TEAM_SIZE;l++){
        for(int k = 0;k<vec[l].size();k++){
        cout << "  "<< vec[l][k];
        }cout << endl;
      }
  
  cout << "SPACETIME DIJKSTRA --- WAYPOINTS:";
  for (int i = 0; i < waypoints.size(); i++)
  {
    cout << " " << waypoints[i];
  }
  
  cout <<endl ;
  
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
        //cout <<"test2813"<<endl;
        if (good)
        { //cout <<"test2913"<<endl;
          st_location next_st(v, next_time + start_time, next_next_waypoint);
          //cout << "pre insertion sort" << endl;
          queue_size = insertion_sort(queue, queue_size, next_st, cmp_function); 
          //cout << "queue size " << queue_size << endl;
          visited[v][next_time][next_next_waypoint] = GRAY;
          //cout << "visited " << ***visited << endl;
          unsigned int psize = path_sizes[u][time][current_waypoint];
          for (unsigned int i = 0; i < psize; i++)
          {
            prev_paths[v][next_time][next_next_waypoint][i] = prev_paths[u][time][current_waypoint][i];
          }
          //cout <<"test2213"<<endl;
          prev_paths[v][next_time][next_next_waypoint][psize] = v;
          path_sizes[v][next_time][next_next_waypoint] = path_sizes[u][time][current_waypoint] + 1;
          
        }
      }
    }
  }
  //cout << "test" << endl;
  throw string("Can't find path!!!");
  //cout << " no path" << endl;
  //cout <<"test2713"<<endl;
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
  //cout << "pre return insertion sort" << endl;
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


void GetGraphInfo(vertex *vertex_web, unsigned int dimension, const char *graph_file)
{

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


vector<vector<unsigned int>> GetWaypoints(const char *waypoint_file)
{

  vector<vector<unsigned int> > wayp;
  vector < unsigned int > waypointsR;
    unsigned int i, j;
    int temp;
    int r;
    FILE *file;
    file = fopen(waypoint_file, "r");
    ifstream File(waypoint_file);
    string s;
    istringstream is(s);
    int counter_wayp=0;
    
    for (i = 0; i < TEAM_SIZE; i++){
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
    }
  fclose(file);
  return wayp;
}

int main(int argc,char *argv[]){
   
    TEAM_SIZE = atoi(argv[2]);
    vector<vector< unsigned int> > vec; 
    
    for (int k = 0; k<TEAM_SIZE;k++){
      std::vector<unsigned int> pathR ;
      vec.push_back(pathR);
    }
    

    

    vector<unsigned int> last_leg, first_leg;

    string mappa = "src/logistic_sim/maps/"+string(argv[1])+"/"+string(argv[1])+".graph";
    string lista_wayp = "src/logistic_sim/maps/"+string(argv[1])+"/waypoints.txt";
    vector<vector<unsigned int> > waypoints = GetWaypoints(lista_wayp.c_str());
    dimension = GetGraphDimension(mappa.c_str());    
    
    vertex_web = new vertex[dimension];
    
    GetGraphInfo(vertex_web, dimension,mappa.c_str());
    unsigned int nedges = GetNumberEdges(vertex_web, dimension);
    cout << "number of edges "<< nedges << endl;
    cout << "number of nodes "<< dimension << endl;
    //cout << ("Loaded graph %s with %d nodes and %d edges\n", "test", dimension, nedges) << endl;
    map_graph = build_graph();

    vector<vector< unsigned int> > min_hops_matrix = calculate_min_hops_matrix(); 
    allocate_memory();
    //auto f = boost::bind(astar_cmp_function, min_hops_matrix, waypoints[j], _1, _2); 
    
    
    for (int j=0;j<TEAM_SIZE;j++){ //per ogni robot
      auto f = boost::bind(astar_cmp_function, min_hops_matrix, waypoints[j], _1, _2);
      vector<unsigned int>astar_result = spacetime_dijkstra(vec,map_graph, waypoints[j],0,&last_leg,&first_leg,&f,j);
      cout << "result: R"<< j ;
      for (int i = 0; i < astar_result.size(); i++){
          cout << "  " << astar_result[i];
      }
      cout << endl;

      vec.erase(vec.begin()+j);
      vec.insert(vec.begin()+j,astar_result);
      //vec.push_back();
      
      for(int l = 0; l<=j;l++){
        for(int k = 0;k<vec[l].size();k++){
        cout << "  "<< vec[l][k];
        }cout << endl;
      }
      cout << endl;
    }
    
  }

