#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "coresense_msgs/action/query_reasoner.hpp"
#include "coresense_msgs/action/understand.hpp"
#include "coresense_msgs/srv/test_understanding.hpp"

#include "coresense_msgs/srv/start_session.hpp"
#include "coresense_msgs/srv/list_session.hpp"
#include "coresense_msgs/srv/add_to_session.hpp"

#include "coresense_understanding/model.hpp"
#include "coresense_understanding/agent_model.hpp"
#include "coresense_understanding/understanding_graph.hpp"
#include "coresense_understanding/theory.hpp"
#include "coresense_understanding/vampire_interface.hpp"


using namespace std::chrono_literals;
namespace ns_vampire = coresense::understanding::interfaces::vampire;
namespace ns_theory = coresense::understanding::theory;

class Problem {
public:
  std::string id{""};
  std::string content{""};
  std::vector<std::string> includes{};
};



namespace coresense::understanding::node {
class UnderstandingSystemNode : public rclcpp::Node {
  using UnderstandAction = coresense_msgs::action::Understand;
  using UnderstandActionGoalHandle = rclcpp_action::ServerGoalHandle<UnderstandAction>;
  using QueryReasonerAction = coresense_msgs::action::QueryReasoner;
  using QueryReasonerActionGoalHandle = rclcpp_action::ClientGoalHandle<QueryReasonerAction>;

public:
  UnderstandingSystemNode() : Node("understanding_system_node") {
    set_coresense_parameter();
    if (true) {
      query_reasoner_action_client_ptr = rclcpp_action::create_client<QueryReasonerAction>(this, "/query_reasoner");
      start_session_client_ptr = create_client<coresense_msgs::srv::StartSession>("/start_session");
      add_to_session_client_ptr = create_client<coresense_msgs::srv::AddToSession>("/add_to_session");
      list_session_client_ptr = create_client<coresense_msgs::srv::ListSession>("/list_session");
      // wait for clients to become available 
      while (!add_to_session_client_ptr->wait_for_service(1s) && !list_session_client_ptr->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the add_knowledge service. Exiting.");
          return; //TODO: find better way to kill this.
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "/add_knowledge service not available, waiting again...");
      }
      RCLCPP_INFO(get_logger(), "Connected to vampire_node");
    understand_action_server_ptr = rclcpp_action::create_server<UnderstandAction>(
      this,
      "/understanding/understand",
      std::bind(&UnderstandingSystemNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&UnderstandingSystemNode::cancel_goal, this, std::placeholders::_1),
      std::bind(&UnderstandingSystemNode::handle_accepted, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Created understanding action");
    }
    read_logic();
    start_session_server_ptr = create_service<coresense_msgs::srv::StartSession>("/understanding/start_session", std::bind(&UnderstandingSystemNode::start_session, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    //test_understanding_service_server_ptr = create_service<coresense_msgs::srv::TestUnderstanding>("/understanding/run_test", std::bind(&UnderstandingSystemNode::test_understanding, this, std::placeholders::_1, std::placeholders::_2));
    //test_reasoner();
    analyse_ros_system();
    if (false) {
      std::string theo = theory_reader.get_theories();
      std::ofstream out0("/home/alex/plansys2_ws/theory.tff");
      out0 << theo;
      out0.close();
      //TODO do this later
      //add_to_session(session_id, "understanding_theory_static", theo);


      std::ofstream out("/home/alex/plansys2_ws/agent_model.tff");
      out << agent_model.model;
      out.close();
      //rclcpp::sleep_for(std::chrono::milliseconds(3000));
      //add_knowledge("all", knowledge.str());
      //std::ofstream out("/tmp/all_knowlegde");
      //out << knowledge.str();
      //out.close();
      //test_response_parsing();
      //add_knowledge_internal("agent_model", agent_model);
      //std::cout << knowledge.str();
      //add_knowledge("theory", knowledge.str());
      //read_package_logic("coresense_understanding", "understanding-logic/tff/modelets");
    }
  }

private:
  rclcpp_action::Client<QueryReasonerAction>::SharedPtr query_reasoner_action_client_ptr;
  rclcpp::Service<coresense_msgs::srv::StartSession>::SharedPtr start_session_server_ptr;
  //rclcpp::Service<coresense_msgs::srv::TestUnderstanding>::SharedPtr test_understanding_service_server_ptr;
  rclcpp::Client<coresense_msgs::srv::StartSession>::SharedPtr start_session_client_ptr;
  rclcpp::Client<coresense_msgs::srv::AddToSession>::SharedPtr add_to_session_client_ptr;
  rclcpp::Client<coresense_msgs::srv::ListSession>::SharedPtr list_session_client_ptr;
  rclcpp_action::Server<UnderstandAction>::SharedPtr understand_action_server_ptr;
  coresense::understanding::agent_model::AgentModel agent_model;
  ns_theory::TheoryReader theory_reader{std::filesystem::path(ament_index_cpp::get_package_share_directory("coresense_understanding"))};
  ns_vampire::VampireInterface vampire_interface;

  std::map<rclcpp_action::GoalUUID, std::shared_ptr<UnderstandActionGoalHandle>> goals;
  std::stringstream knowledge;

  // Methods

  void set_coresense_parameter() {
    const std::string path = ament_index_cpp::get_package_share_directory("coresense_understanding") + "/config/coresense_engine.json";
    const std::ifstream input_stream(path, std::ios_base::binary);

    if (input_stream.fail()) {
      throw std::runtime_error("Failed to open coresense_engine.json");
    }
    std::stringstream buffer;
    buffer << input_stream.rdbuf();
    declare_parameter("coresense_engine", buffer.str());
  }
  
  void read_logic() {
    
    theory_reader.read_package_logic(ament_index_cpp::get_package_share_directory("coresense_understanding"), "understanding-logic/tff/model");
  }
  
  void analyse_ros_system() {
    RCLCPP_INFO(get_logger(), "Waiting for 2.5s");
    rclcpp::sleep_for(std::chrono::milliseconds(2500));
    RCLCPP_INFO(get_logger(), "Waiting done");
    for (std::string node_name : get_node_names()) {
      if (node_name.rfind("/_ros2cli_", 0) != 0) {
        RCLCPP_INFO(get_logger(), "Scanning node %s", node_name.c_str());
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, node_name);
        try {
        
          std::string engine_annotation = parameters_client->get_parameter<std::string>("coresense_engine");
          RCLCPP_INFO(get_logger(), "Creating engine model for %s", node_name.c_str());
          agent_model.create_engine_model(node_name, engine_annotation);
          RCLCPP_INFO(get_logger(), "Created engine model for %s", node_name.c_str());
        } catch (std::runtime_error& e) {
          RCLCPP_WARN(get_logger(), "Node %s produced error %s", node_name.c_str(), e.what());
        } catch (std::exception &e ) {
          RCLCPP_ERROR(get_logger(), "Could not read Node %s annotation. Caught error: %s", node_name.c_str(), e.what());
        }
      }
    }
    agent_model.create_engine_relations();
  }
  
  void start_session(std::shared_ptr<rclcpp::Service<coresense_msgs::srv::StartSession>> start_session_server_ptr,
               const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<coresense_msgs::srv::StartSession::Request> incoming_request) {
    // special service that can call a service
    auto start_session_cb = [start_session_server_ptr, request_header, incoming_request, this](rclcpp::Client<coresense_msgs::srv::StartSession>::SharedFuture start_session_future) {
      //(void)start_session_request;
      std::stringstream logic;
      logic << this->theory_reader.get_theories();
      logic << this->agent_model.model;
      auto add_to_session_request = std::make_shared<coresense_msgs::srv::AddToSession::Request>();
      add_to_session_request->tptp = logic.str();
      add_to_session_request->session_id = start_session_future.get()->session_id;
      auto add_to_session_cb = [start_session_server_ptr, request_header, add_to_session_request, this](rclcpp::Client<coresense_msgs::srv::AddToSession>::SharedFuture add_to_session_future) {
        //(void)add_to_session_request;
        coresense_msgs::srv::StartSession::Response start_session_response;
        start_session_response.session_id = add_to_session_request->session_id;
        start_session_server_ptr->send_response(*request_header, start_session_response);
        RCLCPP_INFO(get_logger(), "Created Session %s", start_session_response.session_id.c_str());
      };
      add_to_session_client_ptr->async_send_request(add_to_session_request, add_to_session_cb);

    };
    auto start_session_request = std::make_shared<coresense_msgs::srv::StartSession::Request>();
    start_session_client_ptr->async_send_request(start_session_request, start_session_cb);
  }

  void add_to_session(std::string session_id, std::string theory) {
    while (!add_to_session_client_ptr->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Client interrupted while waiting for service to appear.");
      }
      RCLCPP_INFO(get_logger(), "Waiting for service to appear...");
    }
    auto request = std::make_shared<coresense_msgs::srv::AddToSession::Request>();
    request->tptp = theory +"\n\n";
    request->session_id = session_id;
    auto result_future = add_to_session_client_ptr->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Calling add_knowledge service failed :(");
      add_to_session_client_ptr->remove_pending_request(result_future);
    }
    else {
      auto result = result_future.get();
      RCLCPP_INFO(get_logger(), "Adding Knowledge returned %s", (result->success? "true" : "false"));
    }
  }
  
  





  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const UnderstandAction::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with order %s in session %s.", goal->target_modelet.c_str(), goal->session_id.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancel_goal(
    const std::shared_ptr<UnderstandActionGoalHandle> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<UnderstandActionGoalHandle> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&UnderstandingSystemNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void add_knowledge_internal(std::string id, std::string theory) {
    RCLCPP_WARN(get_logger(), "Adding knowledge (internal): %s", id.c_str());
    knowledge << theory;
  }


  void execute(const std::shared_ptr<UnderstandActionGoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();
    const std::string modelet = goal->target_modelet;
    std::stringstream ss;
    ss << "tff(query, question," << std::endl;
    ss << modelet << std::endl;
    ss << ").";
    std::string query = ss.str();

    // manage understanding process
    // - manage used theories
    
    //auto result = std::make_shared<UnderstandAction::Result>();
    std::string config = "";
    send_goal_from_action(goal->session_id, query, config, goal_handle);
    // return result
  }


  void send_goal_from_action(std::string session_id, std::string query, std::string config, const std::shared_ptr<UnderstandActionGoalHandle> understanding_goal_handle) {
    auto future = send_goal(session_id, query, config);
    if (future.get() != nullptr) {
      goals[future.get()->get_goal_id()] = understanding_goal_handle;
    } else {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting or rejected goal (this needs better error handling)");
      auto understanding_result = std::make_shared<UnderstandAction::Result>();
      understanding_result->result = "Reasoner not available";
      //TODO create separate error and result fields?
      understanding_goal_handle->abort(understanding_result);
    } 
  }

  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<coresense_msgs::action::QueryReasoner>>> send_goal(std::string session_id, std::string query, std::string config) {
    RCLCPP_INFO(get_logger(), "Looking for Reasoner to send query.");

    //std::cout << "waiting for reasoner" << std::endl;
    if (!query_reasoner_action_client_ptr->wait_for_action_server()) {
      RCLCPP_WARN(get_logger(), "Failed waiting for reasoner.");
      //need to throw some exeception here to denote the failing wait
      return std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<coresense_msgs::action::QueryReasoner>>>();
    }
    RCLCPP_INFO(get_logger(), "Found Reasoner.");
    //std::cout << "have reasoner" << std::endl;

    auto goal_msg = QueryReasonerAction::Goal();
    goal_msg.session_id = session_id;
    goal_msg.query = query;
    goal_msg.configuration = config;
    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<QueryReasonerAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const QueryReasonerActionGoalHandle::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Query goal was rejected by reasoner");
        auto understanding_result = std::make_shared<UnderstandAction::Result>();
        understanding_result->result = "Reasoner rejected query";
        auto id = goal_handle->get_goal_id();
        if (!(goals[id] == nullptr)) {
          goals[id]->abort(understanding_result);
        }
        goals.erase(id);
        return;
      } else {
        RCLCPP_INFO(get_logger(), "Query goal accepted by reasoner, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](QueryReasonerActionGoalHandle::SharedPtr, const std::shared_ptr<const QueryReasonerAction::Feedback> feedback) { // this is so odd, but corresponds to documentation
      //std::stringstream ss;
      //ss << "Current state of reasoning is " << feedback->status;
      //RCLCPP_INFO(get_logger(), ss.str().c_str());
      //TODO PRIORITY forward feedback in some form
    };

    send_goal_options.result_callback = [this](const QueryReasonerActionGoalHandle::WrappedResult & wrapped_result) {
      auto understanding_result = std::make_shared<UnderstandAction::Result>();
      switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: {
          RCLCPP_INFO(get_logger(), "Goal succeeded with code %d", wrapped_result.result->code);
          RCLCPP_INFO(get_logger(), "Reasoner output is:\n%s", wrapped_result.result->result.c_str());
          std::vector<std::string> trees = vampire_interface.parse_output(wrapped_result.result->result);
          std::stringstream result;
          for (std:: string tree : trees) {
            result << tree << std::endl;
          }
          understanding_result->result = result.str();
          if (!(goals[wrapped_result.goal_id] == nullptr)) {
            goals[wrapped_result.goal_id]->succeed(understanding_result);
          }
          break;
        }
        case rclcpp_action::ResultCode::ABORTED: {
          RCLCPP_ERROR(get_logger(), "Goal was aborted");
          understanding_result->result = "Reasoner aborted query, aborting understanding goal.";
          if (!(goals[wrapped_result.goal_id] == nullptr)) {
            goals[wrapped_result.goal_id]->abort(understanding_result);
          }
          break;
        }
        case rclcpp_action::ResultCode::CANCELED: {
          RCLCPP_ERROR(get_logger(), "Goal was canceled");
          understanding_result->result = "Query goal to reasoner got cancelled, cancelling understanding goal.";
          if (!(goals[wrapped_result.goal_id] == nullptr)) {
            goals[wrapped_result.goal_id]->canceled(understanding_result);
          }
          break;
        }
        default: {
          RCLCPP_ERROR(get_logger(), "Unknown result code");
          understanding_result->result = "Reasoner reacted with unknown result code, aborting understanding goal.";
          if (!(goals[wrapped_result.goal_id] == nullptr)) {
            goals[wrapped_result.goal_id]->abort(understanding_result);
          }
        }
      }
      goals.erase(wrapped_result.goal_id);
    }; 
    return query_reasoner_action_client_ptr->async_send_goal(goal_msg, send_goal_options);
    
  }
};
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("Initializing CoreSense Understanding System"), "pid: %i", getpid());
  rclcpp::executors::MultiThreadedExecutor executor;

  auto node = std::make_shared<coresense::understanding::node::UnderstandingSystemNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
