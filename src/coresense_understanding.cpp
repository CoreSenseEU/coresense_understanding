#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <regex>
#include <list>
#include <chrono>

#include <nlohmann/json.hpp>
#include "tinyxml2.h"
#include "uuid/uuid.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "coresense_msgs/action/query_reasoner.hpp"
#include "coresense_msgs/action/understand.hpp"
#include "coresense_msgs/srv/add_knowledge.hpp"
#include "coresense_msgs/srv/pop_knowledge.hpp"
#include "coresense_msgs/srv/list_knowledge.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
namespace us {




struct requirement {
  std::string name;
  std::string datatype;
  std::string value_range;
  std::string to_tff() {
    std::stringstream req_decl, req_type, req_value_range, output;
    req_decl << "tff(" << name << "_requirement, type, " << name << "Req: requirement).\n";
    req_type << "tff(" << name << "Req_has_type, axiom, type_of_r(" << name << "Req) = " << datatype << ").\n";
    req_value_range << "tff(" << name << "Req_is_permissable, axiom, is_permissible(" << name << "Req, " << value_range << ")).\n";
    output << req_decl.str() << req_type.str() << req_value_range.str();
    return output.str();
  };
};
void from_json(const json& j, requirement& r) {
  j.at("name").get_to(r.name);
  j.at("datatype").get_to(r.datatype);
  j.at("value_range").get_to(r.value_range);
};
struct templat {
  std::string name;
  std::string formalism;
  std::list<us::requirement> requirements;
  std::string to_tff() {
    std::stringstream output, req_set;
    output << "tff(" << name << "_template_decl, type, " << name << "_template : template).\n" 
        << "tff(" << name << "_template_formalism, axiom, template_formalism_requirement(" << name << "_template) = " << formalism << ").\n";
    if (requirements.size()) { // template has requirements
      req_set << "tff(define_requirement_set1, axiom,! [R : requirement]:( is_part_of(R, " << name << ") => ( ";
      for (us::requirement req : requirements) {
        std::stringstream req_link_template;
        req_link_template << "tff(" << req.name << "Req_is_part_of_" << name << ", axiom, is_part_of(" << req.name << "Req, " << name << ")).\n";
        output << req.to_tff() << req_link_template.str();
        req_set << " (R = " << req.name << "Req) |";
      }
      req_set.seekp(-1, req_set.cur);
      req_set << "))).\n";
      output << req_set.str();
    } else { // template has no requirements
      output << "tff(" << name << "_template_no_requirements, axiom, ~?[R : requirement]: is_part_of(R, " << name << "_template)).\n";
    }
    return output.str();
  };
};
void from_json(const json& j, templat& t) {
  j.at("name").get_to(t.name);
  j.at("formalism").get_to(t.formalism);
  j.at("requirements").get_to(t.requirements);

};
struct property {
  std::string name;
  std::string datatype;
  std::string value;
};
void from_json(const json& j, property& p) {
  j.at("name").get_to(p.name);
  j.at("datatype").get_to(p.datatype);
  j.at("value").get_to(p.value);
};
struct modelet {
  std::string name;
  std::string formalism;
  std::list<us::property> properties;
};
void from_json(const json& j, modelet& m) {
  j.at("name").get_to(m.name);
  j.at("formalism").get_to(m.formalism);
  j.at("properties").get_to(m.properties);
};
struct resource {
  std::string name;
  int percentage;
};
void from_json(const json& j, resource& r) {
  j.at("name").get_to(r.name);
  j.at("percentage").get_to(r.percentage);
};
struct engine {
  std::string name;
  std::list<us::templat> inputs;
  modelet engine_output;
  int time_delay;
  int energy_cost;
  std::list<us::resource> resources_used;
  std::list<us::resource> resources_blocked;
  std::string to_tff() {
    std::stringstream output, template_set;
    output << "tff(" << name << "_engine_decl, type, " << name << "_engine : engine).\n";
    output << "tff(" << name << "_template_set_decl, type, " << name << "_template_set : template_set).\n";
    output << "tff(" << name << "_engine_output_formalism, axiom, output_modelet_formalism(" << name << "_engine) = " << engine_output.formalism << ").\n";
    //TODO define output properties
    output << "tff(" << name << "_template_set_axiom, axiom, interface_of(" << name << "_engine) = " << name << "_template_set).\n";
    if (inputs.size()) {
      template_set << "tff(" << name << "_template_set_parts, axiom, ![T : template]:( is_in_template_set(T, " << name << "_template_set)<=>( ";
      for (us::templat templ : inputs) {
        output << templ.to_tff();
        template_set << " (T = " << templ.name << "_template) |";
      }
      template_set.seekp(-1, template_set.cur);
      template_set << "))).\n";
      output << template_set.str();
    } else { // engine has no inputs
      output << "tff(" << name << "_engine_no_inputs, axiom, ~?[T : template]: is_in_template_set(T, " << name << "_template_set)).\n";
    }
    return output.str();
  };
};
void from_json(const json& j, engine& e) {
  j.at("name").get_to(e.name);
  j.at("inputs").get_to(e.inputs);
  j.at("time_delay").get_to(e.time_delay);
  j.at("energy_cost").get_to(e.energy_cost);
  j.at("engine_output").get_to(e.engine_output);
  j.at("resources_used").get_to(e.resources_used);
  j.at("resources_blocked").get_to(e.resources_blocked);
};
} // namespace us

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
    set_coresense_parameter();
    query_reasoner_action_client_ptr = rclcpp_action::create_client<QueryReasonerAction>(this, "/vampire");
    add_knowledge_client_ptr = create_client<coresense_msgs::srv::AddKnowledge>("/add_knowledge");
    pop_knowledge_client_ptr = create_client<coresense_msgs::srv::PopKnowledge>("/pop_knowledge");
    list_knowledge_client_ptr = create_client<coresense_msgs::srv::ListKnowledge>("/list_knowledge");
    // wait for clients to become available 
    while (!add_knowledge_client_ptr->wait_for_service(1s) && !pop_knowledge_client_ptr->wait_for_service(1s) && !list_knowledge_client_ptr->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the add_knowledge service. Exiting.");
        return; //TODO: find better way to kill this.
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "/add_knowledge service not available, waiting again...");
    }
    RCLCPP_INFO(get_logger(), "Connected to vampire_node");
    analyse_ros_system();

    understand_action_server_ptr = rclcpp_action::create_server<UnderstandAction>(
      this,
      "understand",
      std::bind(&UnderstandingSystemNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&UnderstandingSystemNode::cancel_goal, this, std::placeholders::_1),
      std::bind(&UnderstandingSystemNode::handle_accepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Created understanding action");

    read_logic();
    add_knowledge(get_theories());
  }

private:
  rclcpp_action::Client<QueryReasonerAction>::SharedPtr query_reasoner_action_client_ptr;
  rclcpp::Client<coresense_msgs::srv::AddKnowledge>::SharedPtr add_knowledge_client_ptr;
  rclcpp::Client<coresense_msgs::srv::PopKnowledge>::SharedPtr pop_knowledge_client_ptr;
  rclcpp::Client<coresense_msgs::srv::ListKnowledge>::SharedPtr list_knowledge_client_ptr;
  rclcpp_action::Server<UnderstandAction>::SharedPtr understand_action_server_ptr;
  std::map<std::string, Theory> theories;
  const std::regex include_regex = std::regex("^include\\('(.*)'\\)\\..*$");

  // Classes
  class GraphNode {
  public:
    std::string id;
    GraphNode() {
      uuid_t uuid;
      char tmp[37];
      uuid_generate(uuid);
      uuid_unparse(uuid, tmp);
      id = std::string(tmp);
    };
    virtual std::string print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map)=0;
  };

  class SubsetNode : public GraphNode {
  public:
    std::string element;
    std::string set;
    SubsetNode(std::smatch match): GraphNode() {
      element = match[1];
      set = match[2];
    };

    std::string print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map)  override {
      std::string tree = map[set]->print(map) + map[element]->print(map);
      //  "<Sequence/>";
      return tree;
    };
  };


  class ExertNode : public GraphNode {
  public:
    std::string engine;
    std::string modelet_set;
    ExertNode(std::smatch match): GraphNode() {
      engine = match[1];
      modelet_set = match[2];
    };
    std::string print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map)  override {
      std::string tree = map[modelet_set]->print(map) + "\n" +"      <SubTree ID='" + engine +"'/>";
      return tree;
    };
  };

  class ConceptNode : public GraphNode {
  public:
    std::string name;
    ConceptNode(std::string name): GraphNode(), name(name) {};

    std::string print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map)  override {
      return "";
    };
  };


  // Methods
  void set_coresense_parameter() {
    const std::string path = ament_index_cpp::get_package_share_directory("coresense_understanding") + "/config/coresense_engine1.json";
    const std::ifstream input_stream(path, std::ios_base::binary);

    if (input_stream.fail()) {
      throw std::runtime_error("Failed to open coresense_engine.json");
    }
    std::stringstream buffer;
    buffer << input_stream.rdbuf();
    declare_parameter("coresense_engine", buffer.str());
  };

  void create_engine_model(std::string engine_annotation) {
    json j = json::parse(engine_annotation);
    auto eng = j.get<us::engine>();
    //std::cout << eng.to_tff() << std::endl;
    add_knowledge(eng.to_tff());

    //json annotation to tff engine model
    // https://json.nlohmann.me/features/arbitrary_types/
    // start with tff model components, then create json structure
    // engine id/name
    // template set
    //  templates
    //   requirements
    // output properties
    // fixed output things
    //  formalism
    // engine exertion properties
    //  cost
    //  delay
    // use templating system? https://github.com/jinja2cpp/Jinja2Cpp
  };

  void analyse_ros_system() {
    for (std::string node_name : get_node_names()) {
      auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, node_name);
      std::string engine_annotation = parameters_client->get_parameter<std::string>("coresense_engine");
      create_engine_model(engine_annotation);
    }
  }

  Problem read_problem(std::string path) {
    // read tptp files and sort them by their includes
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
    const int package_path_length = ament_index_cpp::get_package_share_directory("coresense_understanding").length();
    std::string path = ament_index_cpp::get_package_share_directory("coresense_understanding") + "/understanding-logic/tff/model/";
    for (const auto & entry : std::filesystem::recursive_directory_iterator(path)) {
      std::ifstream file_handle;
      std::stringstream string_handle;
      file_handle.open(entry.path());
      string_handle << file_handle.rdbuf();
      auto it = theories.find(entry.path());
      Theory t;
      if (it == theories.end()) {
        t.id = entry.path().string().substr(package_path_length+1);
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
          for (std::size_t i = 1; i < include_match.size(); ++i) { // skip over whole line match, thus starting at 1
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

    // TODO make sure the tptp files are actually copied to where we expect them by colcon
  }

  std::string get_theories() {
    std::string body = "";
    for (auto& [key, t] : theories) {
      get_theory(t, body);
    }
    return body;
  }
  
  void get_theory(Theory &t, std::string &body) {
    if (!t.added) {
      for (std::string key : t.includes) {
        get_theory(theories[key], body);
      }
      RCLCPP_INFO(get_logger(), "Adding Theory %s", t.id.c_str());
      body += t.content;
      t.added = true;
    }
  }

  void test_reasoner() {
    Problem p = read_problem(ament_index_cpp::get_package_share_directory("coresense_understanding") + "/understanding-logic/tff/tests/test-wind-scenario.tff");
    //send_goal(p.content, "--selection 10 --output_mode vampire --proof off --theory_axioms off --saturation_algorithm fmb --term_ordering lpo");
    //send_goal(p.content, "--selection 10 --output_mode vampire --proof off --theory_axioms on --saturation_algorithm fmb");
    send_goal(p.content, "--selection 1011 --proof off");
    auto response = "% SZS answers Tuple [(∀X1,X0.[M->exert(we_engine,s(exert(dse_engine,ius),s(X0,s(exert(dsee_engine,s(exert(dse_engine,ius),ius)),X1))))]|∀X0.[M->exert(we_engine,s(exert(dsee_engine,s(exert(dse_engine,ius),ius)),s(exert(dse_engine,ius),X0)))])|_] for test-wind-scenario";
    parse_reasoner_output(response);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const UnderstandAction::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with order %s", goal->target_modelet.c_str());
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

  void add_knowledge(std::string theory) {
    while (!add_knowledge_client_ptr->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Client interrupted while waiting for service to appear.");
      }
      RCLCPP_INFO(get_logger(), "Waiting for service to appear...");
    }
    auto request = std::make_shared<coresense_msgs::srv::AddKnowledge::Request>();
    request->tptp = theory;
    request->id = "1";
    auto result_future = add_knowledge_client_ptr->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Calling add_knowledge service failed :(");
      add_knowledge_client_ptr->remove_pending_request(result_future);
    }
    else {
      auto result = result_future.get();
      RCLCPP_INFO(get_logger(), "Adding Knowledge returned %s", (result->success? "true" : "false"));
    }
  }

  tinyxml2::XMLElement * add_parallel_node(tinyxml2::XMLDocument doc, tinyxml2::XMLElement* target_node) {
    tinyxml2::XMLElement * parallel_node = doc.NewElement("Parallel");
    target_node->InsertEndChild(parallel_node);
    return parallel_node;
  }

  tinyxml2::XMLElement * add_sequence_node(tinyxml2::XMLDocument doc, tinyxml2::XMLElement* target_node) {
    tinyxml2::XMLElement * sequence_node = doc.NewElement("Sequence");
    target_node->InsertEndChild(sequence_node);
    return sequence_node;
  }

  void add_engine_node(tinyxml2::XMLDocument doc, tinyxml2::XMLElement* root, tinyxml2::XMLElement* target_node, std::string engine_name, std::string ros_pkg, std::string path) {
    tinyxml2::XMLElement * subtree_node = doc.NewElement("Subtree");
    subtree_node->SetAttribute("ID", engine_name.c_str());
    target_node->InsertEndChild(subtree_node);
    tinyxml2::XMLElement * include_node = doc.NewElement("include");
    include_node->SetAttribute("ros_pkg", ros_pkg.c_str());
    include_node->SetAttribute("path", path.c_str());
    root->InsertFirstChild(include_node);
  }


  bool consume_subset(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map, std::string & answer) {
    const std::regex subset_regex = std::regex("s\\(([\\w-]+)\\,([\\w-]+)\\)");
    std::smatch match;
    if (std::regex_search(answer, match, subset_regex)) {
      if (map.find(match[1]) == map.end()) {
        map[match[1]] = std::make_shared<ConceptNode>(match[1]);
        std::cout << "created ConceptNode " << match[1] << std::endl;
      }
      if (map.find(match[2]) == map.end()) {
        map[match[2]] = std::make_shared<ConceptNode>(match[2]);
        std::cout << "created ConceptNode " << match[2] << std::endl;
      }
      auto n = std::make_shared<SubsetNode>(match);
      map[n->id] = n;
      std::cout << "Created SubsetNode " << n->id << " from s(" << match[1] << "," << match[2] << ")" << std::endl;
      answer = std::regex_replace(answer, subset_regex, n->id, std::regex_constants::format_first_only);
      return true;
    } else {
      return false;
    }
  }

  bool consume_exert(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map, std::string & answer) {
    const std::regex exert_regex = std::regex("exert\\(([\\w-]+)\\,([\\w-]+)\\)");
    std::smatch match;
    //TODO: construct map
    //TODO: how to create 'parallel' parts
    if (std::regex_search(answer, match, exert_regex)) {
      if (map.find(match[1]) == map.end()) {
        map[match[1]] = std::make_shared<ConceptNode>(match[1]);
        std::cout << "created ConceptNode " << match[1] << std::endl;
      }
      if (map.find(match[2]) == map.end()) {
        map[match[2]] = std::make_shared<ConceptNode>(match[2]);
        std::cout << "created ConceptNode " << match[2] << std::endl;
      }
      auto n = std::make_shared<ExertNode>(match); 
      map[n->id] = n;
      std::cout << "created ExertNode " << n->id << " from exert(" << match[1] << "," << match[2] << ")" << std::endl;
      answer = std::regex_replace(answer, exert_regex, n->id, std::regex_constants::format_first_only);
      return true;
    } else {
      return false;
    }
  }

  void consume_answer(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map, std::string & answer) {
    if (consume_exert(map, answer)) {
      consume_answer(map, answer);
    } else if (consume_subset(map, answer)) {
      consume_answer(map, answer);
    }
    return;
  }

  void parse_reasoner_output(std::string output) {
    const std::regex answer_line_regex = std::regex("^% SZS answers Tuple \\[\\((.*)\\)\\|_\\] for [\\-\\w]+$");
    const std::regex answer_regex = std::regex("\\.\\[\\w->(.*?)\\]\\|?");
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement* rootXML = doc.NewElement("root");
    rootXML->SetAttribute("BTCPP_format", 4);
    doc.InsertFirstChild(rootXML);
    std::string line;
    std::stringstream string_handle;

    string_handle << output;
    while (std::getline(string_handle, line, '\n') ) {
      //extract includes
      std::smatch answer_match;
      std::unordered_map<std::string, std::shared_ptr<GraphNode>> map;
      if (std::regex_search(line, answer_match, answer_line_regex)) {
        //std::cout << answer_match[1] << std::endl;
        std::string answers = answer_match[1];
        const std::sregex_token_iterator End;
        for (std::sregex_token_iterator it(answers.begin(), answers.end(), answer_regex, 1); it != End; ++it) {
          std::string answer = *it;
          std::cout << "----- Before -----" << std::endl;
          std::cout << "Answer: " << answer << std::endl;
          consume_answer(map, answer);
          std::cout << "----- After  -----" << std::endl;
          std::cout << "Root: " << answer << std::endl;
          std::cout << build_behavior_tree(map, answer) << std::endl;
        }
      } else {
        std::cout << "no match" << std::endl;
      }
    }
  }

  std::string build_behavior_tree(std::unordered_map<std::string, std::shared_ptr<GraphNode>> map, std::string root_id) {
    
    return "<root BTCPP_format='4'>\n  <BehaviorTree ID='Understanding Solution XYZ'>\n    <Sequence>" + map[root_id]->print(map) + "\n    </Sequence>\n  </BehaviorTree>\n</root>";
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
    //   add_knowledge("context");
    send_goal(query, "");
    // return result
  }


  void send_goal(std::string query, std::string config)
  {
    if (!query_reasoner_action_client_ptr->wait_for_action_server()) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = QueryReasonerAction::Goal();
    goal_msg.query = query;
    goal_msg.configuration = config;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<QueryReasonerAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const QueryReasonerActionGoalHandle::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](
      QueryReasonerActionGoalHandle::SharedPtr,
      const std::shared_ptr<const QueryReasonerAction::Feedback> feedback)
    {
      std::stringstream ss;
      ss << "Current state of reasoning is " << feedback->status;
      RCLCPP_INFO(get_logger(), ss.str().c_str());
    };

    send_goal_options.result_callback =[this](const QueryReasonerActionGoalHandle::WrappedResult & wrapped_result) {
      switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "Goal succeeded with code %d", wrapped_result.result->code);
          RCLCPP_INFO(get_logger(), "Reasoner output is:\n%s", wrapped_result.result->std_output.c_str());
          parse_reasoner_output(wrapped_result.result->std_output);
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(get_logger(), "Unknown result code");
          return;
      }
      rclcpp::shutdown();
    };
    query_reasoner_action_client_ptr->async_send_goal(goal_msg, send_goal_options);
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
