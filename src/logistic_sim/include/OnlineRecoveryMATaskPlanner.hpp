#pragma once
#include "OnlineTaskPlanner.hpp"
#include "logistic_sim/ChangeEdge.h"

namespace onlinerecoverymataskplanner
{
class OnlineRecoveryMATaskPlanner : public onlinetaskplanner::OnlineTaskPlanner
{
public:
  OnlineRecoveryMATaskPlanner(ros::NodeHandle &nh_, const std::string &name = "OnlineRecoveryMATaskPlanner");

  void token_callback(const logistic_sim::TokenConstPtr &msg) override;

protected:

  ros::ServiceServer change_edge_service;
  void print_graph();   // for test
  void advertise_change_edge_service(ros::NodeHandle &nh) override;
  bool change_edge(logistic_sim::ChangeEdge::Request &msg, logistic_sim::ChangeEdge::Response &res);

  // edges that must be removed in the next token cycle
  std::vector<logistic_sim::Edge> removed_edges;
  boost::mutex edges_mutex; // used to avoid conflicts with the token thread and the main thread

  std::vector<bool> _check_conflict_free_impl(uint task_endpoint);
  bool check_conflict_free_property();

  bool check_feasible_recovery_vertex(uint vertex);
  // check if a configuration is good for recovery
  bool check_valid_recovery_configuration(const std::vector<uint> &configuration,
                  const std::vector<uint> &robot_ids,
                  const std::vector<std::vector<uint> > &waypoints,
                  const std::vector<uint> &unfeasible_vertices);

  std::vector<std::vector<uint> > find_all_recovery_configs(const std::vector<std::vector<uint> > &waypoints, const std::vector<uint> &robot_ids);
  std::vector<uint> find_best_recovery_config(const std::vector<std::vector<uint> > &waypoints
                                            , const std::vector<uint> &robot_ids
                                            , const std::vector<uint> &current_config
                                            , const std::vector<std::vector<uint> > &other_paths);


  void init_token(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
  void multi_agent_repair(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);

  std::vector<std::vector<unsigned int>> map_graph, fw_matrix;

  std::vector<std::vector<unsigned int> > build_graph();
  void build_fw_matrix();
};
}