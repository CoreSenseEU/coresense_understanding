#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "coresense_msgs/action/query_reasoner.hpp"
#include "coresense_msgs/action/understand.hpp"
#include "coresense_msgs/srv/add_knowledge.hpp"
#include "coresense_msgs/srv/pop_knowledge.hpp"
#include "coresense_msgs/srv/list_knowledge.hpp"

class Problem {
public:
  std::string id{""};
  std::string content{""};
  std::vector<std::string> includes{};
};

class Theory {
public:
  std::string id{""};
  std::string content{""};
  std::vector<std::string> includes{};
  bool added{false};
};

namespace coresense_understanding_cpp {
class UnderstandingSystemNode : public rclcpp::Node 
{
  using UnderstandAction = coresense_msgs::action::Understand;
  using UnderstandActionGoalHandle = rclcpp_action::ServerGoalHandle<UnderstandAction>;
  using QueryReasonerAction = coresense_msgs::action::QueryReasoner;
  using QueryReasonerActionGoalHandle = rclcpp_action::ClientGoalHandle<QueryReasonerAction>;

public:
  UnderstandingSystemNode() : Node("understanding_system_node")
  {
    query_reasoner_action_client_ptr = rclcpp_action::create_client<QueryReasonerAction>(this, "/vampire");
    add_knowledge_client_ptr = create_client<coresense_msgs::srv::AddKnowledge>("/add_knowledge");
    pop_knowledge_client_ptr = create_client<coresense_msgs::srv::PopKnowledge>("/pop_knowledge");
    list_knowledge_client_ptr = create_client<coresense_msgs::srv::ListKnowledge>("/list_knowledge");
    
    RCLCPP_INFO(this->get_logger(), "Connected to vampire_node");
    understand_action_server_ptr = rclcpp_action::create_server<UnderstandAction>(
      this,
      "understand",
      std::bind(&UnderstandingSystemNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&UnderstandingSystemNode::cancel_goal, this, std::placeholders::_1),
      std::bind(&UnderstandingSystemNode::handle_accepted, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Created understanding action");
    this->read_logic();
    this->add_knowledge(this->get_theories());
    Problem p = this->read_problem(ament_index_cpp::get_package_share_directory("coresense_understanding") + "/understanding-logic/tff/tests/test-wind-scenario.tff");
    //this->send_goal(p.content, "--selection 10 --output_mode vampire --proof off --theory_axioms off --saturation_algorithm fmb --term_ordering lpo");
    //this->send_goal(p.content, "--selection 10 --output_mode vampire --proof off --theory_axioms on --saturation_algorithm fmb");
    this->send_goal(p.content, "--selection 1011 --proof off");
    //this->send_goal(p.content, "--selection 1011 --output_mode vampire --proof off");
    //RCLCPP_INFO(this->get_logger(), this->get_theories().c_str());
  }

private:
  rclcpp_action::Client<QueryReasonerAction>::SharedPtr query_reasoner_action_client_ptr;
  rclcpp::Client<coresense_msgs::srv::AddKnowledge>::SharedPtr add_knowledge_client_ptr;
  rclcpp::Client<coresense_msgs::srv::PopKnowledge>::SharedPtr pop_knowledge_client_ptr;
  rclcpp::Client<coresense_msgs::srv::ListKnowledge>::SharedPtr list_knowledge_client_ptr;
  rclcpp_action::Server<UnderstandAction>::SharedPtr understand_action_server_ptr;
  std::map<std::string, Theory> theories;
  std::regex include_regex = std::regex("^include\\('(.*)'\\)\\..*$");

  Problem read_problem(std::string path) {
    // read tptp files and sort them by their includes
    std::cout << path << std::endl;
    std::ifstream file_handle;
    std::stringstream string_handle;
    file_handle.open(path);
    string_handle << file_handle.rdbuf();
    Problem p;
    // separate includes
    // TODO do we have to split theories if there are includes in the middle of a file?
    //      if not, we can just run the regex over the whole string rather than individual lines
    //      nope, we have to REMOVE the include lines
    std::string line;
    while (std::getline(string_handle, line, '\n') ) {
      //extract includes
      std::smatch include_match;
      if (std::regex_search(line, include_match, include_regex)) {
        for (std::size_t i = 0; i < include_match.size(); ++i) {
          p.includes.push_back(include_match[i]);
        } 
      } else {
        //add line to cleaned string
        p.content += line + '\n';
      }
    }
    file_handle.close();
    return p;
  }

  void read_logic() {
    // read tptp files and sort them by their includes
    std::string path = ament_index_cpp::get_package_share_directory("coresense_understanding") + "/understanding-logic/tff/model/";
    for (const auto & entry : std::filesystem::recursive_directory_iterator(path)) {
      std::cout << entry.path() << std::endl;
      std::ifstream file_handle;
      std::stringstream string_handle;
      file_handle.open(entry.path());
      string_handle << file_handle.rdbuf();
      auto it = theories.find(entry.path());
      Theory t;
      if (it == theories.end()) {
        t.id = entry.path();
        theories[t.id] = t;
      } else {
        t = it->second;
      }
      // separate includes
      // TODO do we have to split theories if there are includes in the middle of a file?
      //      if not, we can just run the regex over the whole string rather than individual lines
      //      nope, we have to REMOVE the include lines
      std::string line;
      while (std::getline(string_handle, line, '\n') ) {
        //extract includes
        std::smatch include_match;
        if (std::regex_search(line, include_match, include_regex)) {
          for (std::size_t i = 0; i < include_match.size(); ++i) {
            t.includes.push_back(include_match[i]);
          } 
        } else {
          //add line to cleaned string
          t.content += line + '\n';
        }
      }
      theories[t.id] = t;
      file_handle.close();
    }

    // ...so create a graph structure
    // TODO make sure the tptp files are actually copied to where we expect them by colcon
    // TODO create STL tptp file dependency graph
    // TODO create file reading thing
  }

  std::string get_theories() {
    std::string body = "";
    for (auto& [key, t] : theories) {
      this->get_theory(t, body);
    }
    return body;
  }
  
  void get_theory(Theory &t, std::string &body) {
    if (!t.added) {
      for (std::string key : t.includes) {
        this->get_theory(theories[key], body);
      }
      body += t.content;
      t.added = true;
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const UnderstandAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", goal->target_modelet.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancel_goal(
    const std::shared_ptr<UnderstandActionGoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<UnderstandActionGoalHandle> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&UnderstandingSystemNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void add_knowledge(std::string theory) {
    while (!add_knowledge_client_ptr->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    auto request = std::make_shared<coresense_msgs::srv::AddKnowledge::Request>();
    request->tptp = theory;
    auto result_future = add_knowledge_client_ptr->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "service call failed :(");
      add_knowledge_client_ptr->remove_pending_request(result_future);
    }
    //auto result = result_future.get();
  }


  void execute(const std::shared_ptr<UnderstandActionGoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const std::string modelet = goal->target_modelet;
    // TODO generate query from target modelet
    const std::string query = modelet;
    // manage understanding process
    // - manage used theories
    // TODO create new reasoner session
    // load reasoner with context theories
    //   this->add_knowledge("context");
    this->send_goal(query, "");
    // return result
  }


  void send_goal(std::string query, std::string config)
  {
    if (!this->query_reasoner_action_client_ptr->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = QueryReasonerAction::Goal();
    goal_msg.query = query;
    goal_msg.configuration = config;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<QueryReasonerAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const QueryReasonerActionGoalHandle::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](
      QueryReasonerActionGoalHandle::SharedPtr,
      const std::shared_ptr<const QueryReasonerAction::Feedback> feedback)
    {
      std::stringstream ss;
      ss << "Current state of reasoning is " << feedback->status;
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    };

    send_goal_options.result_callback =[this](const QueryReasonerActionGoalHandle::WrappedResult & wrapped_result) {
      switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Goal succeeded. with code %d", wrapped_result.result->code);
          RCLCPP_INFO(this->get_logger(), "Reasoner output is:\n%s", wrapped_result.result->std_output.c_str());
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
      rclcpp::shutdown();
    };
    this->query_reasoner_action_client_ptr->async_send_goal(goal_msg, send_goal_options);
  }


};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("Initializing CoreSense Understanding System"), "pid: %i", getpid());
  rclcpp::spin(std::make_shared<coresense_understanding_cpp::UnderstandingSystemNode>());
  rclcpp::shutdown();
  return 0;
}
