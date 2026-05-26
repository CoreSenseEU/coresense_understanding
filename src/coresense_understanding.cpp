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

#include "std_srvs/srv/trigger.hpp"

#include "coresense_msgs/action/query_reasoner.hpp"
#include "coresense_msgs/action/understand.hpp"
#include "coresense_msgs/srv/test_understanding.hpp"

#include "coresense_msgs/srv/start_session.hpp"
#include "coresense_msgs/srv/end_session.hpp"
#include "coresense_msgs/srv/list_session.hpp"
#include "coresense_msgs/srv/add_to_session.hpp"
#include "coresense_msgs/srv/remove_from_session.hpp"

#include "coresense_understanding/model.hpp"
#include "coresense_understanding/agent_model.hpp"
#include "coresense_understanding/knowledge_model.hpp"
#include "coresense_understanding/understanding_graph.hpp"
#include "coresense_understanding/theory.hpp"
#include "coresense_understanding/vampire_interface.hpp"
#include "coresense_understanding/session.hpp"

#include "triplestar_msgs/srv/sparql_query.hpp"
#include "triplestar_msgs/srv/select_query.hpp"

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
      query_triplestar_kb_ptr = create_client<triplestar_msgs::srv::SPARQLQuery>("/triplestar_core/query");
      get_modelets_client_ptr = create_client<triplestar_msgs::srv::SelectQuery>("/triplestar_core/query_services/get_modelets");
      query_reasoner_action_client_ptr = rclcpp_action::create_client<QueryReasonerAction>(this, "/query_reasoner");
      start_session_client_ptr = create_client<coresense_msgs::srv::StartSession>("/start_session");
      end_session_client_ptr = create_client<coresense_msgs::srv::EndSession>("/end_session");
      add_to_session_client_ptr = create_client<coresense_msgs::srv::AddToSession>("/add_to_session");
      list_session_client_ptr = create_client<coresense_msgs::srv::ListSession>("/list_session");
      // wait for clients to become available 
      while (!add_to_session_client_ptr->wait_for_service(1s)
         && !remove_from_session_client_ptr->wait_for_service(1s) 
         && !list_session_client_ptr->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the add_knowledge service. Exiting.");
          return; //TODO: find better way to kill this.
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "/add_knowledge service not available, waiting again...");
      }
      RCLCPP_INFO(get_logger(), "Connected to vampire_node");
      while (!get_modelets_client_ptr->wait_for_service(1s) ) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for triplestar get_modelets service. Exiting.");
          return; //TODO: find better way to kill this.
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "/triplestar_core/query_services/get_modelets service not available, waiting again...");
      }
      while (!query_triplestar_kb_ptr->wait_for_service(1s) ) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for triplestar query service. Exiting.");
          return; //TODO: find better way to kill this.
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "/triplestar_core/query service not available, waiting again...");
      }
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
    end_session_server_ptr = create_service<coresense_msgs::srv::EndSession>("/understanding/end_session", std::bind(&UnderstandingSystemNode::end_session, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    mark_agent_server_ptr = create_service<std_srvs::srv::Trigger>("/understanding/mark_agent_model_dirty", std::bind(&UnderstandingSystemNode::mark_agent_model_dirty, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    mark_knowledge_server_ptr = create_service<std_srvs::srv::Trigger>("/understanding/mark_knowledge_model_dirty", std::bind(&UnderstandingSystemNode::mark_knowledge_model_dirty, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    //test_understanding_service_server_ptr = create_service<coresense_msgs::srv::TestUnderstanding>("/understanding/run_test", std::bind(&UnderstandingSystemNode::test_understanding, this, std::placeholders::_1, std::placeholders::_2));
    //test_reasoner();
    RCLCPP_INFO(get_logger(), "Waiting for 2.5s");
    rclcpp::sleep_for(std::chrono::milliseconds(2500));
    update_models();
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
  rclcpp::Service<coresense_msgs::srv::EndSession>::SharedPtr end_session_server_ptr;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mark_agent_server_ptr;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mark_knowledge_server_ptr;
  //rclcpp::Service<coresense_msgs::srv::TestUnderstanding>::SharedPtr test_understanding_service_server_ptr;
  rclcpp::Client<triplestar_msgs::srv::SPARQLQuery>::SharedPtr query_triplestar_kb_ptr;
  rclcpp::Client<triplestar_msgs::srv::SelectQuery>::SharedPtr get_modelets_client_ptr;
  rclcpp::Client<coresense_msgs::srv::StartSession>::SharedPtr start_session_client_ptr;
  rclcpp::Client<coresense_msgs::srv::EndSession>::SharedPtr end_session_client_ptr;
  rclcpp::Client<coresense_msgs::srv::AddToSession>::SharedPtr add_to_session_client_ptr;
  rclcpp::Client<coresense_msgs::srv::RemoveFromSession>::SharedPtr remove_from_session_client_ptr;
  rclcpp::Client<coresense_msgs::srv::ListSession>::SharedPtr list_session_client_ptr;
  rclcpp_action::Server<UnderstandAction>::SharedPtr understand_action_server_ptr;
  coresense::understanding::agent_model::AgentModel agent_model;
  std::chrono::seconds agent_model_update_interval = std::chrono::seconds(5);
  coresense::understanding::knowledge_model::KnowledgeModel knowledge_model;
  std::chrono::seconds knowledge_model_update_interval = std::chrono::seconds(1);
  ns_theory::TheoryReader theory_reader{std::filesystem::path(ament_index_cpp::get_package_share_directory("coresense_understanding"))};
  ns_vampire::VampireInterface vampire_interface;

  std::map<std::string, rclcpp::Client<triplestar_msgs::srv::SelectQuery>::SharedPtr> selectQueryClients;

  std::map<rclcpp_action::GoalUUID, std::shared_ptr<UnderstandActionGoalHandle>> goals;
  std::stringstream knowledge;
  std::map<std::string, coresense::understanding::session::Session> sessions;

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


  
  void get_modelet_snapshot() {
    RCLCPP_INFO(get_logger(), "Creating knowledge model.");
    for (std::string klass : knowledge_model.klasses) {
      selectQueryClients[klass] = create_client<triplestar_msgs::srv::SelectQuery>("/triplestar_core/query_services/"+klass+"_query");
      auto request = std::make_shared<triplestar_msgs::srv::SelectQuery::Request>();
      auto callback = [this, klass](rclcpp::Client<triplestar_msgs::srv::SelectQuery>::SharedFuture future) {
        this->knowledge_model.add_klass(klass, future.get()->result);
        knowledge_model.updated[klass] = true;
        selectQueryClients.erase(klass);
        RCLCPP_DEBUG(get_logger(), "Knowledge Model updated class: %s", klass.c_str());
      };
      selectQueryClients[klass]->async_send_request(request, callback);
    }
    auto get_modelets_request = std::make_shared<triplestar_msgs::srv::SelectQuery::Request>();
    auto get_modelets_cb = [this](rclcpp::Client<triplestar_msgs::srv::SelectQuery>::SharedFuture get_modelets_future) {
      this->knowledge_model.create_knowledge_model(get_modelets_future.get()->result);
      RCLCPP_DEBUG(get_logger(), "Knowledge Model updated modelets");
    };
    //TODO enable multithreaded executor to make this work
    //while (!(knowledge_model.updated["concept"] && knowledge_model.updated["representation_class"] && knowledge_model.updated["formalism"])){
    //  rclcpp::sleep_for(std::chrono::milliseconds(50));
    //}
    get_modelets_client_ptr->async_send_request(get_modelets_request, get_modelets_cb);
  }
  
  void analyse_ros_system() {
    RCLCPP_INFO(get_logger(), "Creating agent model.");
    for (std::string node_name : get_node_names()) {
      if ((node_name.rfind("/_ros2cli_", 0) != 0) && (node_name.rfind("/launch_ros", 0) != 0)) {
        RCLCPP_DEBUG(get_logger(), "Scanning node %s", node_name.c_str());
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, node_name);
        try {       
          std::string engine_annotation = parameters_client->get_parameter<std::string>("coresense_engine");
          if (engine_annotation.length() > 0) {
            agent_model.create_engine_model(node_name, engine_annotation);
            RCLCPP_DEBUG(get_logger(), "Created engine model for %s", node_name.c_str());
          } else {
            RCLCPP_DEBUG(get_logger(), "Node %s is not annotated", node_name.c_str());
          }
        } catch (std::runtime_error& e) {
          RCLCPP_DEBUG(get_logger(), "Node %s produced error %s", node_name.c_str(), e.what());
        } catch (std::exception &e ) {
          RCLCPP_ERROR(get_logger(), "Could not read Node %s annotation. Caught error: %s", node_name.c_str(), e.what());
        }
      }
    }
    agent_model.create_engine_relations();
    RCLCPP_INFO(get_logger(), "Created agent model.");
  }


  void update_models() {
    analyse_ros_system();
    get_modelet_snapshot();
  }

  void mark_agent_model_dirty(std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> mark_agent_model_server_ptr,
               const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<std_srvs::srv::Trigger::Request> incoming_request) {
    agent_model.dirty = true;
    RCLCPP_INFO(get_logger(), "Agent Model marked dirty.");
    std_srvs::srv::Trigger::Response response;
    response.success = true;
    response.message = "";
    mark_agent_server_ptr->send_response(*request_header, response);
  }
  
  void mark_knowledge_model_dirty(std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> mark_knowledge_model_server_ptr,
               const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<std_srvs::srv::Trigger::Request> incoming_request) {
    knowledge_model.dirty = true;
    RCLCPP_INFO(get_logger(), "Knowledge Model marked dirty.");
    std_srvs::srv::Trigger::Response response;
    response.success = true;
    response.message = "";
    mark_knowledge_server_ptr->send_response(*request_header, response);
  }
  
  void start_session(std::shared_ptr<rclcpp::Service<coresense_msgs::srv::StartSession>> start_session_server_ptr,
               const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<coresense_msgs::srv::StartSession::Request> incoming_request) {
    // special service that can call a service
    auto start_session_cb = [this, start_session_server_ptr, request_header, incoming_request](rclcpp::Client<coresense_msgs::srv::StartSession>::SharedFuture start_session_future) {
      std::string session_id = start_session_future.get()->session_id;
      RCLCPP_INFO(get_logger(), "Creating Session %s", session_id.c_str());
      sessions.emplace(session_id, coresense::understanding::session::Session(session_id));

      //TODO refactor the next three lines for consistency and efficiency (session manager)
      add_to_session(session_id, "understanding_theory", theory_reader.get_theories());
      add_to_session(session_id, "agent_model", agent_model.model);
      add_to_session(session_id, "knowledge_model", knowledge_model.model);
      sessions[session_id].last_agent_update = agent_model.last_update;
      sessions[session_id].last_knowledge_update = knowledge_model.last_update;
      coresense_msgs::srv::StartSession::Response response;
      response.session_id = session_id;
      start_session_server_ptr->send_response(*request_header, response);
    };
    auto start_session_request = std::make_shared<coresense_msgs::srv::StartSession::Request>();
    start_session_client_ptr->async_send_request(start_session_request, start_session_cb);
  }

  void end_session(std::shared_ptr<rclcpp::Service<coresense_msgs::srv::EndSession>> end_session_server_ptr,
               const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<coresense_msgs::srv::EndSession::Request> incoming_request) {
    // special service that can call a service
    auto end_session_cb = [this, end_session_server_ptr, request_header, incoming_request](rclcpp::Client<coresense_msgs::srv::EndSession>::SharedFuture end_session_future) {
      coresense_msgs::srv::EndSession::Response response;
      response.success = end_session_future.get()->success;
      if (response.success) {
        sessions.erase(incoming_request->session_id);
        RCLCPP_INFO(get_logger(), "Ended session %s", incoming_request->session_id.c_str());

      } else {
        RCLCPP_WARN(get_logger(), "Failed to end session %s", incoming_request->session_id.c_str());
      }
      end_session_server_ptr->send_response(*request_header, response);
    };
    auto end_session_request = std::make_shared<coresense_msgs::srv::EndSession::Request>();
    end_session_request->session_id = incoming_request->session_id;
    end_session_client_ptr->async_send_request(end_session_request, end_session_cb);
  }

  void add_to_session(std::string session_id, std::string theory_id, std::string theory) {
    while (!add_to_session_client_ptr->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Client interrupted while waiting for service to appear.");
      }
      RCLCPP_INFO(get_logger(), "Waiting for service to appear...");
    }
    auto request = std::make_shared<coresense_msgs::srv::AddToSession::Request>();
    request->tptp = theory +"\n\n";
    request->session_id = session_id;
    request->formula_set_id = theory_id;
    auto callback = [this](rclcpp::Client<coresense_msgs::srv::AddToSession>::SharedFuture future) {
      auto result = future.get();
      RCLCPP_DEBUG(get_logger(), "Adding Knowledge returned %s", (result->success? "true" : "false"));
    };
    auto result_future = add_to_session_client_ptr->async_send_request(request, callback);
  }
  

  void remove_from_session(std::string session_id, std::string theory_id) {
    while (!remove_from_session_client_ptr->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Client interrupted while waiting for service to appear.");
      }
      RCLCPP_INFO(get_logger(), "Waiting for service to appear...");
    }
    auto request = std::make_shared<coresense_msgs::srv::RemoveFromSession::Request>();
    request->session_id = session_id;
    request->formula_set_id = theory_id;
    auto callback = [this](rclcpp::Client<coresense_msgs::srv::RemoveFromSession>::SharedFuture future) {
      auto result = future.get();
      RCLCPP_DEBUG(get_logger(), "Removing Knowledge returned %s", (result->success? "true" : "false"));
    };
    auto result_future = remove_from_session_client_ptr->async_send_request(request, callback);
  }

  void replace_in_session(std::string session_id, std::string theory_id, std::string theory) {
    while (!remove_from_session_client_ptr->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Client interrupted while waiting for service to appear.");
      }
      RCLCPP_INFO(get_logger(), "Waiting for service to appear...");
    }
    auto request = std::make_shared<coresense_msgs::srv::RemoveFromSession::Request>();
    request->session_id = session_id;
    request->formula_set_id = theory_id;
    auto callback = [this, session_id, theory_id, theory](rclcpp::Client<coresense_msgs::srv::RemoveFromSession>::SharedFuture future) {
      auto result = future.get();
      RCLCPP_DEBUG(get_logger(), "Removing Knowledge returned %s", (result->success? "true" : "false"));
      add_to_session(session_id, theory_id, theory);
    };
    auto result_future = remove_from_session_client_ptr->async_send_request(request, callback);
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
    //check if understanding system theories are too old or known to have changed
    coresense::understanding::session::Session session = sessions[goal->session_id];
    std::chrono::time_point now = std::chrono::system_clock::now();
    if ((agent_model.dirty) || (now > (agent_model.last_update + agent_model_update_interval))) {
      //analyse_ros_system();
    }
    if ((knowledge_model.dirty) || (now > (knowledge_model.last_update + knowledge_model_update_interval))) {
      //get_modelet_snapshot();
    }
    //check if session theories are too old
    if (now > (agent_model.last_update + session.agent_model_max_age)) {
      //replace_in_session(goal->session_id, "agent_model", agent_model.model);
    }
    if (now > (knowledge_model.last_update + session.knowledge_model_max_age)) {
      //replace_in_session(goal->session_id, "knowledge_model", knowledge_model.model);
    }

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
    RCLCPP_DEBUG(get_logger(), "Looking for Reasoner to send query.");

    //std::cout << "waiting for reasoner" << std::endl;
    if (!query_reasoner_action_client_ptr->wait_for_action_server()) {
      RCLCPP_WARN(get_logger(), "Failed waiting for reasoner.");
      //need to throw some exeception here to denote the failing wait
      return std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<coresense_msgs::action::QueryReasoner>>>();
    }
    RCLCPP_DEBUG(get_logger(), "Found Reasoner.");
    //std::cout << "have reasoner" << std::endl;

    auto goal_msg = QueryReasonerAction::Goal();
    goal_msg.session_id = session_id;
    goal_msg.query = query;
    goal_msg.configuration = config;
    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<QueryReasonerAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const QueryReasonerActionGoalHandle::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_WARN(get_logger(), "Query goal was rejected by reasoner");
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
          if (wrapped_result.result->code == 0) {
            RCLCPP_WARN(get_logger(), "Reasoner output is:\n%s", wrapped_result.result->result.c_str());
          } else if (wrapped_result.result->code == 1) {
            RCLCPP_INFO(get_logger(), "Result message is: %s", wrapped_result.result->code_msg.c_str());
            RCLCPP_DEBUG(get_logger(), "Reasoner output is:\n%s", wrapped_result.result->result.c_str());
          } else {
            RCLCPP_WARN(get_logger(), "Result message is: %s", wrapped_result.result->code_msg.c_str());
            RCLCPP_WARN(get_logger(), "Reasoner output is:\n%s", wrapped_result.result->result.c_str());
          }
          std::vector<std::string> trees = vampire_interface.parse_output(wrapped_result.result->result);
          std::stringstream result;
          for (std:: string tree : trees) {
            result << tree;
          }
          RCLCPP_INFO(get_logger(), "Resulting tree is:\n%s", result.str().c_str());

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
