#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <regex>
#include <list>

#include <nlohmann/json.hpp>
#include "tinyxml2.h"
#include "uuid/uuid.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "coresense_msgs/action/query_reasoner.hpp"
#include "coresense_msgs/action/understand.hpp"
#include "coresense_msgs/srv/test_understanding.hpp"
#include "coresense_msgs/srv/add_knowledge.hpp"
#include "coresense_msgs/srv/pop_knowledge.hpp"
#include "coresense_msgs/srv/list_knowledge.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

std::string create_relation_exist1(std::string relation, std::string individual, std::string set_klass,  std::list<std::string> set) {
  std::stringstream ss;
  ss << "tff(axiom_" << individual << "_" << relation << "_existence, axiom," << std::endl 
     << "  ![X: " << set_klass << "]: " << std::endl 
     << "  (" << std::endl
     << "    (";
  for (std::string name : set) {
    ss << std::endl <<"      (X = " << name << ")" << std::endl << "      |"; 
  }
  ss.seekp(-3, ss.cur);
  ss <<     ")" << std::endl
     << "    =>" << std::endl
     << "    " << relation << "(" << individual << ", X)" << std::endl
     << "  )" << std::endl << ")." << std::endl;
  return ss.str();
}

std::string create_relation_limit1(std::string relation, std::string individual, std::string set_klass,  std::list<std::string> set) {
  std::stringstream ss;
  ss << "tff(axiom_" << individual << "_" << relation << "_existence, axiom," << std::endl 
     << "  ![X: " << set_klass << "]: " << std::endl 
     << "  (" << std::endl
     << "    " << relation << "(" << individual << ", X)" << std::endl
     << "    =>" << std::endl
     << "    (";
  for (std::string name : set) {
    ss << std::endl <<"      (X = " << name << ")" << std::endl << "      |"; 
  }
  ss.seekp(-3, ss.cur);
  ss << ")" << std::endl << "  )" << std::endl << ")." << std::endl;
  return ss.str();
}
std::string create_relation_exist2(std::string relation, std::string individual, std::string set_klass,  std::set<std::string> set) {
  std::stringstream ss;
  ss << "tff(axiom_" << individual << "_" << relation << ", axiom," << std::endl 
     << "  ![X: " << set_klass << "]: " << std::endl 
     << "  (" << std::endl
     << "    (";
  for (std::string name : set) {
    ss << std::endl <<"      (X = " << name << ")" << std::endl << "      |"; 
  }
  ss.seekp(-3, ss.cur);
  ss <<     ")" << std::endl
     << "    =>" << std::endl
     << "    " << relation << "(X, " << individual << ")" << std::endl
     << "  )" << std::endl << ")." << std::endl;
  return ss.str();
}

std::string create_relation_limit2(std::string relation, std::string individual, std::string set_klass,  std::list<std::string> list) {
  std::stringstream ss;
  ss << "tff(axiom_" << individual << "_" << relation << ", axiom," << std::endl 
     << "  ![X: " << set_klass << "]: " << std::endl 
     << "  (" << std::endl
     << "    " << relation << "(X, " << individual << ")" << std::endl
     << "    =>" << std::endl
     << "    (";
  for (std::string name : list) {
    ss << std::endl <<"      (X = " << name << ")" << std::endl << "      |"; 
  }
  ss.seekp(-3, ss.cur);
  ss << ")" << std::endl << "  )" << std::endl << ")." << std::endl;
  return ss.str();
}

std::string create_has_no_relation1(std::string relation, std::string individual, std::string set_klass) {
  std::stringstream ss;
  ss << "tff(" << individual << "_has_no_" << relation <<", axiom, " << std::endl
     << "  ~?[X : "<< set_klass << "]:" << std::endl
//     << "  (" << std::endl
     << "    " << relation << "(" << individual << ", X)" << std::endl
//     << "  )" << std::endl
     << ")." << std::endl;
  return ss.str();
}
std::string create_has_no_relation2(std::string relation, std::string individual, std::string set_klass) {
  std::stringstream ss;
  ss << "tff(" << individual << "_has_no_" << relation <<", axiom, " << std::endl
     << "  ~?[X : "<< set_klass << "]:" << std::endl
//     << "  (" << std::endl
     << "    " << relation << "(X, " << individual << ")" << std::endl
//     << "  )" << std::endl
     << ")." << std::endl;
  return ss.str();
}
namespace us {

struct requirement {
  std::string name;
  std::string datatype;
  std::string value_range;
  std::string to_tff() {
    std::stringstream req_decl, req_type, req_value_range, output;
    req_type << "tff(" << name << "_has_type, axiom, type_of_requirement(" << name << ") = " << datatype << ").\n";
    req_value_range << "tff(" << name << "_is_permissable, axiom, is_permissible(" << name << ", " << value_range << ")).\n";
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
  std::string creator;
  std::list<std::string> representation_classes;
  std::list<std::string> concepts;
  //std::list<std::string> locations;
  //std::list<std::string> extents;
  std::list<us::requirement> requirements;
  std::string to_tff() {
    std::stringstream output, req_set;
    output << "tff(" << name << "_formalism_requirement, axiom,\n  template_formalism_requirement(" << name << ") = " << formalism << "\n).\n";
    if (representation_classes.empty()) {
      output << create_has_no_relation1("template_has_representation_class_requirement", name, "representation_class");
    } else { // template has representation_classes
      output << create_relation_limit1("template_has_representation_class_requirement", name, "representation_class", representation_classes);
    }
    if (concepts.empty()) {
      output << create_has_no_relation1("template_has_concept_requirement", name, "concept");
    } else { // template has concepts
      output << create_relation_limit1("template_has_concept_requirement", name, "concept", concepts);
    }
    if (creator != "") {
      output << "tff(" << name << "_template_has_creator_requirement, axiom,\n  template_has_creator_requirement(" << name << ", " << creator << ")\n).\n";
    } else { // template has no creators
      output << "tff(" << name << "_template_has_no_creator_requirement, axiom,\n  ~?[E: engine]:\n    template_has_creator_requirement(" << name << ", E)\n).\n";
    }
    // property requirements
    if (requirements.empty()) { // template has no requirements
      output << create_has_no_relation2("is_part_of", name, "requirement");
    } else { // template has requirements
      std::set<std::string> requirement_names;
      for (us::requirement req : requirements) {
        output << req.to_tff();
        requirement_names.insert(req.name);
      }
      output << create_relation_exist2("is_part_of", name, "requirement", requirement_names);
    }
    return output.str();
  };
};
void from_json(const json& j, templat& t) {
  j.at("name").get_to(t.name);
  j.at("creator").get_to(t.creator);
  j.at("formalism").get_to(t.formalism);
  j.at("requirements").get_to(t.requirements);
  j.at("concepts").get_to(t.concepts);
  j.at("representation_classes").get_to(t.representation_classes);
//  j.at("extents").get_to(t.extents);
//  j.at("locations").get_to(t.locations);
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
  std::list<std::string> representation_classes;
  std::list<std::string> concepts;
  std::list<us::property> properties;
};
void from_json(const json& j, modelet& m) {
  j.at("name").get_to(m.name);
  j.at("formalism").get_to(m.formalism);
  j.at("properties").get_to(m.properties);
  j.at("representation_classes").get_to(m.representation_classes);
  j.at("concepts").get_to(m.concepts);
//  j.at("extents").get_to(m.extents);
//  j.at("locations").get_to(m.locations);
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
  std::list<std::string> transit_properties;
  std::list<us::resource> resources_consumed;
  std::list<us::resource> resources_blocked;
  std::string to_tff() {
    std::stringstream output, templates;
    output << "tff(" << name << "_imparts_formalism, axiom,\n  output_modelet_formalism(" << name << ") = " << engine_output.formalism << "\n).\n";
    if (!inputs.empty()) {
      templates << "tff(" << name << "_input_definition_axiom, axiom,\n  defines_input" << inputs.size() << "(";
      for (us::templat templ : inputs) {
        //TODO this can lead to duplicate template definitions when engines define identically named templates
        output << templ.to_tff();
        templates << templ.name << ", ";
      }
      templates << name << ")\n).\n";
      output << templates.str();
    } else { // engine has no inputs
    }
    // These encode the engine->property direction 1->n 
    if (!engine_output.representation_classes.empty()) {
      output << create_relation_exist1("engine_imparts_representation_class", name, "representation_class", engine_output.representation_classes);
    } else { // engine imparts no representation_classes
      output << create_has_no_relation1("engine_imparts_representation_class", name, "representation_class");
    }
    if (!engine_output.concepts.empty()) {
      output << create_relation_exist1("engine_imparts_concept", name, "concept", engine_output.concepts);
    } else { // engine imparts no concepts
      output << create_has_no_relation1("engine_imparts_concept", name, "concept");
    }
    if (!engine_output.properties.empty()) {
      std::list<std::string> properties;
      for (us::property property : engine_output.properties) {
        properties.push_back(property.name);
      }
      output << create_relation_exist1("engine_imparts_property", name, "property", properties);
    } else { // engine imparts no propertys
      output << create_has_no_relation1("engine_imparts_property", name, "property");
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
  j.at("resources_consumed").get_to(e.resources_consumed);
  j.at("resources_blocked").get_to(e.resources_blocked);
  j.at("transit_properties").get_to(e.transit_properties);
};
} // namespace 'us' ends

class Problem {
public:
  std::string id{""};
  std::string content{""};
  std::vector<std::string> includes{};
};

class Theory {
public:
  std::filesystem::path id;
  std::string content{""};
  std::vector<std::string> includes{};
  std::vector<std::string> queries{};
  bool added{false};
};

namespace coresense_understanding_cpp {
class UnderstandingSystemNode : public rclcpp::Node {
  using UnderstandAction = coresense_msgs::action::Understand;
  using UnderstandActionGoalHandle = rclcpp_action::ServerGoalHandle<UnderstandAction>;
  using QueryReasonerAction = coresense_msgs::action::QueryReasoner;
  using QueryReasonerActionGoalHandle = rclcpp_action::ClientGoalHandle<QueryReasonerAction>;

public:
  UnderstandingSystemNode() : Node("understanding_system_node") {
    set_coresense_parameter();
    if (true) {
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

    understand_action_server_ptr = rclcpp_action::create_server<UnderstandAction>(
      this,
      "understand",
      std::bind(&UnderstandingSystemNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&UnderstandingSystemNode::cancel_goal, this, std::placeholders::_1),
      std::bind(&UnderstandingSystemNode::handle_accepted, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Created understanding action");
    }
    analyse_ros_system();
    read_logic();
    test_understanding_service_server_ptr = create_service<coresense_msgs::srv::TestUnderstanding>("run_test", std::bind(&UnderstandingSystemNode::test_understanding, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    //test_reasoner();
    if (true) {
      //get_theories();
      add_knowledge("understanding_theory", get_theories());
      std::ofstream out("/tmp/agent_model");
      out << agent_model;
      out.close();
      rclcpp::sleep_for(std::chrono::milliseconds(3000));
      //std::ifstream in("/tmp/agent_model");
      //std::string am;
      //in >> am;
      //add_knowledge("agent_model", am);
      add_knowledge("agent_model", agent_model);
    }
  }

private:
  rclcpp_action::Client<QueryReasonerAction>::SharedPtr query_reasoner_action_client_ptr;
  rclcpp::Service<coresense_msgs::srv::TestUnderstanding>::SharedPtr test_understanding_service_server_ptr;
  rclcpp::Client<coresense_msgs::srv::AddKnowledge>::SharedPtr add_knowledge_client_ptr;
  rclcpp::Client<coresense_msgs::srv::PopKnowledge>::SharedPtr pop_knowledge_client_ptr;
  rclcpp::Client<coresense_msgs::srv::ListKnowledge>::SharedPtr list_knowledge_client_ptr;
  rclcpp_action::Server<UnderstandAction>::SharedPtr understand_action_server_ptr;
  std::map<std::string, Theory> theories;
  std::map<rclcpp_action::GoalUUID, std::shared_ptr<UnderstandActionGoalHandle>> goals;
  const std::regex include_regex = std::regex("^include\\s*\\('(.*)'\\)\\.$");
  const std::regex query_regex = std::regex("tff\\s*\\([\\s\\w-]+\\, question\\s*\\,[^\\.]+\\)\\.");
  std::string agent_model;
  std::vector<us::engine> engines;
  std::map<std::string, std::set<std::string>> distinct_instances {
    {"engine", std::set<std::string>()},
    {"template", std::set<std::string>()},
    {"formalism", std::set<std::string>()},
    {"representation_class", std::set<std::string>()},
    {"requirement", std::set<std::string>()},
    {"proptype", std::set<std::string>()},
    {"property", std::set<std::string>()},
    {"resource", std::set<std::string>()},
    {"concept", std::set<std::string>()},
    {"modelet", std::set<std::string>()},
   // {"spacetime_point", std::set<std::string>()},
   // {"extent", std::set<std::string>()},
    {"", std::set<std::string>()}
  };

  // Classes
  class GraphNode {
  public:
    std::string id;
    std::string name;
    GraphNode() {
      uuid_t uuid;
      char tmp[37];
      uuid_generate(uuid);
      uuid_unparse(uuid, tmp);
      id = std::string(tmp);
    }
    virtual std::string print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map)=0;
    virtual std::string get_id()=0;
  };

  class SubsetNode : public GraphNode {
  public:
    std::string element;
    std::string set;
    SubsetNode(std::smatch match): GraphNode() {
      element = match[1];
      set = match[2];
    }

    std::string print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map) override {
      std::string tree = map[set]->print(map) + map[element]->print(map);
      return tree;
    }

    std::string get_id() {
      return name + "_" + id;
    }
  };

  class ExertNode : public GraphNode {
  public:
    std::string modelet_set;
    ExertNode(std::smatch match): GraphNode() {
      name = match[1];
      modelet_set = match[2];
    }
    std::string print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map) override {
      std::string tree = map[modelet_set]->print(map) + "\n" +"      <SubTree ID='" + name +"'/>";
      return tree;
    }
    std::string get_id() {
      return "EXERT_" + name + "_" + id;
    }
  };

  class ConceptNode : public GraphNode {
  public:
    ConceptNode(std::string modelet_name): GraphNode() {
      name = modelet_name;
    };

    std::string print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map) override {
      return name;
    }
    std::string get_id() {
      return "MODELET_" + name + "_" + id;
    }
  };


  class ExertnNode : public GraphNode {
  public:
    std::string engine;
    std::vector<std::string> modelets;
    std::vector<std::string> exerts;
    ExertnNode(std::string engine_name): GraphNode() {
      name = engine_name;
    }

    void add_node(std::shared_ptr<GraphNode> & node) {
      if (node->get_id().rfind("MODELET_", 0) == 0) {
        modelets.push_back(node->name);
      } else if (node->get_id().rfind("EXERT_", 0) == 0) {
        exerts.push_back(node->id);
        modelets.push_back(node->id);
      }
    }

    std::string print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map) override {
      std::ostringstream tree;
      //TODO think about where sequence nodes are put
      //  top level sequence
      //  moving in levels down is in fact putting things into the sequence at an earlier spot
      //  so there should only be one sequence? no. if a subtree (in a parallel node) has another subtree, that should be new sequence
      // first: add required subtree exertions, either in parallel or just on a newline in the parent sequence
      if (exerts.size() > 1) {
        // this should only happen if modelets are in fact exerts
        tree << "<Parallel failure_count='1' success_count='" << exerts.size() << "'>" << std::endl;
        for (auto exert : exerts) {
          tree << "<Sequence>" << std::endl;
          tree << map[exert]->print(map) << std::endl;
          tree << "</Sequence>" << std::endl;
        }
        tree << "</Parallel>" << std::endl;
      } else if (!exerts.empty()) {
        tree << map[exerts[0]]->print(map) << std::endl;
      } 
      tree << "<SubTree ID='" << name << "' output='" << get_id() << "_port' ";
      // TODO make sure this defines the correct output port
      //tree << map[modelets[0]]->print(map) << std::endl << "<SubTree ID='" << map[modelets[0]]->get_id()  << "' ";
      int count = 1;
      for (auto modelet : modelets) {
        tree << "port_" << count++ << "='{" << map[modelet]->get_id() << "_port}' ";
      }

      //for (auto exert : exerts) {
      //  tree << map[exert]->get_id() << "_port='{" << map[exert]->get_id() << "_port}' ";
      //}
      tree << "/>" << std::endl;
      //std::cout << tree.str() << std::endl;
      //tree << subtree_port_name << "='{" << parent_tree_output_port_name << "}'" << subtree_output_port_name << "='{" << id << "}'/>\n";
      return tree.str();
    }

    std::string get_id() {
      return "EXERT_" + name + "_" + id;
    }
  };



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

  void create_engine_model(std::string node_name, std::string engine_annotation) {
    RCLCPP_INFO(get_logger(), "Creating engine model for %s", node_name.c_str());
    try {
      json j = json::parse(engine_annotation);
      auto eng = j.get<us::engine>();
      engines.push_back(eng);
      //std::cout << eng.to_tff() << std::endl;
    } catch (const json::parse_error& e) {
      //std::stringstream ss;
      //ss << "Could not read Node" << node_name << "annotation. Caught error: " << e.what();
      //RCLCPP_ERROR(get_logger(), ss.str());
      RCLCPP_ERROR(get_logger(), "Could not read Node %s annotation. Caught error: %s", node_name.c_str(), e.what());
      //std::cout << e.what();
    } catch (const json::type_error& e) {
      RCLCPP_ERROR(get_logger(), "Could not read Node %s annotation. Caught error: %s", node_name.c_str(), e.what());
    } catch (const json::invalid_iterator& e) {
      RCLCPP_ERROR(get_logger(), "Could not read Node %s annotation. Caught error: %s", node_name.c_str(), e.what());
    } catch (const json::out_of_range& e) {
      RCLCPP_ERROR(get_logger(), "Could not read Node %s annotation. Caught error: %s", node_name.c_str(), e.what());
    } catch (const json::other_error& e) {
      RCLCPP_ERROR(get_logger(), "Could not read Node %s annotation. Caught error: %s", node_name.c_str(), e.what());
    } catch (...) {
      std::cout << "caught some unknown error from json parsing" << std::endl;
    }
    RCLCPP_INFO(get_logger(), "Created engine model for %s", node_name.c_str());

    //add_knowledge(eng.to_tff());

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
  }

  void analyse_ros_system() {
    RCLCPP_INFO(get_logger(), "Waiting for 2.5s");
    rclcpp::sleep_for(std::chrono::milliseconds(2500));
    RCLCPP_INFO(get_logger(), "Waiting done");
    for (std::string node_name : get_node_names()) {
      RCLCPP_INFO(get_logger(), "Scanning node %s", node_name.c_str());
      auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, node_name);
      try {
        RCLCPP_INFO(get_logger(), "Reading parameter");
        std::string engine_annotation = parameters_client->get_parameter<std::string>("coresense_engine");
        //std::cout << engine_annotation << std::endl;
        create_engine_model(node_name, engine_annotation);
      } catch (std::runtime_error& e) {
        RCLCPP_WARN(get_logger(), "Node %s produced error %s", node_name.c_str(), e.what());
        // Silently fail reading coresense_engine parameters if they don't exist
        //std::cout << node_name << std::endl;
        //std::cout << e.what();
      }
    }
    agent_model = create_engine_relations();
  }

  std::string create_engine_relations() {
    std::stringstream part_relations;
    std::stringstream all_relations;
    std::stringstream engine_relations;
    std::set<int> engine_input_sizes;
    engine_input_sizes.insert(1);
    engine_input_sizes.insert(2);
    engine_input_sizes.insert(3);
    // TODO collect further data to ensure distinctness
    // concepts?
    // modelets?
    // locations?
    // extents?
     
    //collect data per needed relation from all the engines
    std::map<std::string, std::set<std::string>> imparted_rcs;
    std::map<std::string, std::set<std::string>> imparted_concepts;
    std::map<std::string, std::set<std::string>> imparted_properties;
    std::map<std::string, std::set<std::string>> templates_rc;
    std::map<std::string, std::set<std::string>> templates_concept;
    std::map<std::string, std::set<std::string>> templates_requirement;
//    std::map<std::string, std::set<std::string>> requirements_location;
//    std::map<std::string, std::set<std::string>> requirements_extent;

    for (us::engine eng : engines) {
      //add engine ttf to output
      engine_relations << eng.to_tff();
      // collect data for inter-engine relations
      distinct_instances["engine"].insert(eng.name);
      // Inputs
      engine_input_sizes.insert(eng.inputs.size());
      for (auto input : eng.inputs) {
        distinct_instances["template"].insert(input.name);
        distinct_instances["formalism"].insert(input.formalism);
        for (std::string representation_class : input.representation_classes) {
          distinct_instances["representation_class"].insert(representation_class);
          if (templates_rc.find(representation_class) == templates_rc.end()) {
            templates_rc[representation_class] = std::set<std::string>();
          }
          templates_rc[representation_class].insert(input.name);
        }
        for (std::string concept : input.concepts) {
          distinct_instances["concept"].insert(concept);
          if (templates_concept.find(concept) == templates_concept.end()) {
            templates_concept[concept] = std::set<std::string>();
          }
          templates_concept[concept].insert(input.name);
        }
//        for (std::string location : input.locations) {
//          distinct_instances["location"].insert(location);
//          if (requirements_location.find(location) == requirements_location.end()) {
//            requirements_location[location] = std::set<std::string>();
//          }
//          requirements_location[location].insert(input.name);
//        }
//        for (std::string extent : input.extents) {
//          distinct_instances["extent"].insert(extent);
//          if (requirements_extent.find(extent) == requirements_extent.end()) {
//            requirements_extent[extent] = std::set<std::string>();
//          }
//          requirements_extent[extent].insert(input.name);
//        }
        for (auto requirement : input.requirements) {
          distinct_instances["requirement"].insert(requirement.name);
          distinct_instances["proptype"].insert(requirement.datatype);
          if (templates_requirement.find(requirement.name) == templates_requirement.end()) {
            templates_requirement[requirement.name] = std::set<std::string>();
          }
          templates_requirement[requirement.name].insert(input.name);
        }
      } 
      // Outputs
      distinct_instances["formalism"].insert(eng.engine_output.formalism);
      for (std::string representation_class : eng.engine_output.representation_classes) {
        distinct_instances["representation_class"].insert(representation_class);
        if (imparted_rcs.find(representation_class) == imparted_rcs.end()) {
          imparted_rcs[representation_class] = std::set<std::string>();
        }
        imparted_rcs[representation_class].insert(eng.name);
      }
      for (std::string concept : eng.engine_output.concepts) {
        distinct_instances["concept"].insert(concept);
        if (imparted_concepts.find(concept) == imparted_concepts.end()) {
         imparted_concepts[concept] = std::set<std::string>();
        }
        imparted_concepts[concept].insert(eng.name);
      }
      for (auto property : eng.engine_output.properties) {
        distinct_instances["property"].insert(property.name);
        distinct_instances["proptype"].insert(property.datatype);
        if (imparted_properties.find(property.name) == imparted_properties.end()) {
         imparted_properties[property.name] = std::set<std::string>();
        }
        imparted_properties[property.name].insert(eng.name);
      } 
      for (auto resource : eng.resources_consumed) {
        distinct_instances["resource"].insert(resource.name);
      } 
      for (auto resource : eng.resources_blocked) {
        distinct_instances["resource"].insert(resource.name);
      } 
      //distinct_instances[""].insert(eng.engine_output.);
    }
    // create instance declarations and distinctness axioms
    for (auto& [key, set] : distinct_instances) {
      part_relations << create_existence_declarations(key, set);
      part_relations << create_distinct_axiom(key + "_distinctness", set);
    }

    std::string inter_relations = create_inter_engine_relations(engine_input_sizes);
    std::ofstream out1("/home/alex/plansys2_ws/inter_relations.tff");
    out1 << inter_relations;
    out1.close();
    add_knowledge("inter_relations", inter_relations);
    //all_relations << create_inter_engine_relations(engine_input_sizes);
    std::string part_rels = part_relations.str();
    std::ofstream out2("/home/alex/plansys2_ws/part_relations.tff");
    out2 << part_rels;
    out2.close();
    add_knowledge("parts", part_rels);
    //add_knowledge("parts", part_relations.str());
    std::string engine_rels = engine_relations.str();
    std::ofstream out3("/home/alex/plansys2_ws/engine_relations.tff");
    out3 << engine_rels;
    out3.close();
    add_knowledge("engine_relations", engine_rels);
    //add_knowledge("engine_relations", engine_relations.str());
    //all_relations << part_relations.rdbuf();
    //all_relations << engine_relations.rdbuf();
    
    // These encode the property->engine direction 1->n
    // these should go over multiple engines
    // 
    for (const auto& [requirement, templates] : templates_requirement) {
      if (!templates.empty()) {
        std::list<std::string> li(templates.begin(), templates.end());
        all_relations << create_relation_limit1("is_part_of", requirement, "template", li);
      }
    }
    for (const auto& [rc, engines] : imparted_rcs) {
      if (!engines.empty()) {
        //TODO this is wrongly filled because it does not collect all the engine per rc ALEX
        std::list<std::string> li(engines.begin(), engines.end());
        all_relations << create_relation_limit2("engine_imparts_representation_class", rc, "engine", li);
      }
    }
    // create concept  -> engine relation
    for (const auto& [concept, engines] : imparted_concepts) {
      if (!engines.empty()) {
        std::list<std::string> li(engines.begin(), engines.end());
        all_relations << create_relation_limit2("engine_imparts_concept", concept, "engine", li);
      }
    }
    // create concept  -> engine relation
    for (const auto& [property, engines] : imparted_properties) {
      if (!engines.empty()) {
        std::list<std::string> li(engines.begin(), engines.end());
        all_relations << create_relation_limit2("engine_imparts_property", property, "engine", li);
      }
    }
    // create representation_class -> template relation
    for (const auto& [rc, templates] : templates_rc) {
      if (!templates.empty()) {
        std::list<std::string> li(templates.begin(), templates.end());
        all_relations << create_relation_limit2("template_has_representation_class_requirement", rc, "template", li);
      }
    }
    // create concept -> template relation
    for (const auto& [concept, templates] : templates_concept) {
      if (!templates.empty()) {
        std::list<std::string> li(templates.begin(), templates.end());
        all_relations << create_relation_limit2("template_has_concept_requirement", concept, "template", li);
      }
    }
//    // create location -> template relation
//    for (const auto& [location, templates] : requirements_location) {
//      if (!templates.empty()) {
//        all_relations << create_relation2("template_has_location_requirement", location, "template", templates);
//      }
//    }
//    // create extent -> template relation
//    for (const auto& [extent, templates] : requirements_extent) {
//      if (!templates.empty()) {
//        all_relations << create_relation2("template_has_extent_requirement", extent, "template", templates);
//      }
//    }
    all_relations << std::endl;
    std::string rest_rels = all_relations.str();
    std::ofstream out4("/home/alex/plansys2_ws/rest_relations.tff");
    out4 << rest_rels;
    out4.close();
    return all_relations.str();
  }

  std::string create_inter_engine_relations(std::set<int> sizes) {
    std::stringstream ss;
    for(int size : sizes) {
      std::stringstream modelets;
      std::string axiom_end = "\n  )\n).\n";
      std::string E_def = "  ![E: engine";
      std::stringstream M_def;
      std::stringstream T_def;
      std::string def_end = "]:\n  (";
      std::stringstream M_use;
      std::stringstream T_use;
      std::stringstream exert_use;
      std::stringstream exertable_and_exert;
      std::stringstream modelet_has_property;
      std::stringstream exertable_def;
      std::stringstream inputs_match;
      ss << "tff(decl_defines_input" << size << ", type," << std::endl 
         << "  defines_input" << size << ": (";

      exert_use << "      &" << std::endl
                << "      exert" << size << "(E";
      exertable_def << "  ![E: engine";
      modelet_has_property << "      (";
      M_def << ", OM" << ": modelet";
      for(int i =1; i<= size; i++) {
        modelet_has_property << std::endl << "        modelet_has_property(IM" << i << ", P)" << std::endl
                             << "        |";
        modelets << " * modelet";
        ss << "template * ";
        M_def << ", IM" << i << ": modelet";
        T_def << ", T" << i << ": template";
        M_use << ", IM" << i;
        exertable_def << ", IM" << i << ": modelet, T" << i << ": template";
        inputs_match << "      inputs_match" << "(IM" << i << ", T" << i << ")" << std::endl
                     << "      &" << std::endl;
        T_use << "T" << i << ", ";
        
      }
      if (size >0) {
        modelet_has_property.seekp(-3, modelet_has_property.cur);
        inputs_match.seekp(-7, inputs_match.cur);
      }
      modelet_has_property << ") " << std::endl;
      exertable_and_exert << "    (" << std::endl
                          << "      exertable" << size << "(E" << M_use.str() << ")" << std::endl
                          << "      &" << std::endl
                          << "      exert" << size << "(E" << M_use.str() << ") = OM";
      exertable_def << ", OM: modelet]:";

      // declarations
      ss << "engine) > $o" << std::endl 
         << ")." << std::endl;
      ss << "tff(decl_exert" << size << ", type," << std::endl 
         << "  exert" << size << ": (engine" << modelets.str() << ") > modelet" << std::endl 
         << ")." << std::endl;
      ss << "tff(decl_exertable" << size << ", type," << std::endl 
         << "  exertable" << size << ": (engine" << modelets.str() << ") > $o" << std::endl 
         << ")." << std::endl;
      // axioms
      ss << "tff(axiom_modelet_is_exertable" << size << ", axiom, " << std::endl
         << E_def << M_def.str() << T_def.str() << ", R: resource" << def_end << std::endl
         << "    (" << std::endl
         << inputs_match.str()
         << "      defines_input" << size << "(" << T_use.str() << "E)" << std::endl
         << "      &" << std::endl
         << "      ~engine_unavailable(E)" << std::endl
         << "      &" << std::endl
         << "      (resource_unavailable(R) => ~engine_uses_resource(E, R))" << std::endl
         << "    )" << std::endl
         << "    =>" << std::endl
         << "    exertable" << size << "(E" << M_use.str() << ")" << axiom_end;

      ss << "tff(axiom_engines_impart_formalism" << size << ", axiom," << std::endl
         << E_def << M_def.str() << def_end << std::endl
         << exertable_and_exert.str() << std::endl
         << "    )" << std::endl
         << "    =>" << std::endl
         << "    formalism_of_modelet(OM) = output_modelet_formalism(E)" << axiom_end;

      ss << "tff(axiom_engines_are_creators_of_modelets" << size << ", axiom," << std::endl
         << E_def << M_def.str() << def_end << std::endl
         << exertable_and_exert.str() << std::endl
         << "    )" << std::endl
         << "    =>" << std::endl
         << "    modelet_has_creator(OM, E)" << axiom_end;

      ss << "tff(axiom_engines_may_impart_representation_classes" << size << ", axiom," << std::endl
         << E_def << M_def.str() << ", R: representation_class"<< def_end << std::endl
         << exertable_and_exert.str() << std::endl
         << "      &" << std::endl
         << "      engine_imparts_representation_class(E, R)" << std::endl
         << "    )" << std::endl
         << "    =>" << std::endl
         << "    modelet_has_representation_class(OM, R)" << axiom_end;
         
      ss << "tff(axiom_engines_may_impart_concepts" << size << ", axiom," << std::endl
         << E_def << M_def.str() << ", C: concept"<< def_end << std::endl
         << exertable_and_exert.str() << std::endl
         << "      &" << std::endl
         << "      engine_imparts_concept(E, C)" << std::endl
         << "    )" << std::endl
         << "    =>" << std::endl
         << "    modelet_models_concept(OM, C)" << axiom_end;

      ss << "tff(axiom_engines_may_impart_properties" << size << ", axiom," << std::endl
         << E_def << M_def.str() << ", P: property"<< def_end << std::endl
         << exertable_and_exert.str() << std::endl
         << "      &" << std::endl
         << "      engine_imparts_property(E, P)" << std::endl
         << "    )" << std::endl
         << "    =>" << std::endl
         << "    modelet_has_property(OM, P)" << axiom_end;

      if (size > 0) {
        ss << "tff(axiom_engines_may_transit_properties" << size << ", axiom," << std::endl
           << E_def << M_def.str() << ", P: property"<< def_end << std::endl
           << exertable_and_exert.str() << std::endl
           << "      &" << std::endl
           << modelet_has_property.str()
           << "    )" << std::endl
           << "    =>" << std::endl
           << "    modelet_has_property(OM, P)" << axiom_end;
      }
    }
    return ss.str();
  }

  std::string create_existence_declarations(std::string klass, std::set<std::string> items) {
    std::stringstream ss;
    for (std::string item: items) {
      ss << "tff(decl_"<< item << ", type, " << item << ": " << klass << ")." << std::endl;
    }
    return ss.str();
  }

  std::string create_distinct_axiom(std::string label, std::set<std::string> items) {
    std::stringstream ss;
    if (items.size() > 1) {
      ss << "tff(axiom_"<< label << ", axiom, $distinct(";
      for (std::string item: items) {
        ss << " " << item << ",";
      }
      ss.seekp(-1, ss.cur);
      ss << "))." << std::endl;
    }
    return ss.str();
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
    read_package_logic("coresense_understanding", "understanding-logic/tff/model");
  }

  void read_package_logic(std::string package_name, std::string logic_directory) {
    // read tptp files and sort them by their includes
    std::filesystem::path path = std::filesystem::path(ament_index_cpp::get_package_share_directory(package_name)) / std::filesystem::path(logic_directory);
//    RCLCPP_INFO(get_logger(), "Reading package logic: path is %s", path.string().c_str());
    for (const auto & entry : std::filesystem::recursive_directory_iterator(path)) { // for files in path
      if (entry.is_regular_file()) {
        import_tptp_file(entry.path());
      }
    }
  }

  Theory import_tptp_file(std::filesystem::path path) {
    std::ifstream file_handle;
    std::stringstream string_handle;
    file_handle.open(path);
    string_handle << file_handle.rdbuf();
    Theory t = get_theory(path);
    parse_tptp_string(t, string_handle);
    file_handle.close();
    return t;
  }

  Theory get_theory(std::filesystem::path path) {
    Theory t;
    // find or create theory
    auto it = theories.find(path.string());
    if (it == theories.end()) { // create a new theory
      t.id = path;
      theories[t.id.string()] = t;
    } else { // set current theory to found theory
      t = it->second;
    }
    return t;
  }

  void parse_tptp_string(Theory t, std::stringstream& string_handle) {
    std::string content = string_handle.str();
    std::string line;
    // TODO find these declarations and collect them
    // TODO find the respective distinct axioms and remove them
    // engines
    // concepts
    // requirements
    // properties
    // modelets
    // representation_class
    // datatypes
    // formalisms
    // locations
    // extents
    // template sets
    while (std::getline(string_handle, line, '\n') ) {
    //extract includes
      std::smatch include_match;
      if (std::regex_search(line, include_match, include_regex)) {
        // TODO SECONDARY PRIORITY: this implies includes live on single lines, rather than spread over multiple lines.
        for (std::size_t i = 1; i < include_match.size(); ++i) { // find includes
          // handle includes
          // TODO SECONDARY PRIORITY: this assumes all the includes are magically also being added to the pile of theories...but there is no guarantee of that.
          // we need to parse them and make sure they are read
          // TODO SECONDARY PRIORITY includes are relative to some TPTP_HOME. how can we make sure it's the same home when they come from different packages?
          // TODO SECONDARY PRIORITY the include path strings are likely used as ids, to find corresponding theories, make sure this correspondence still holds (it likely does not)
          t.includes.push_back(include_match[i]);
        } 
      } else {
        //add line to cleaned string
        t.content += line + '\n';
      }
    } // TODO: SECONDARY PRIORITY t.content now still potentially includes queries and conjectures
    // idea: use regex match-ranges to remove the offending parts
    std::smatch query_match;
    if (std::regex_search(content, query_match, query_regex)) { // find queries
      for (std::size_t i = 0; i < query_match.size(); ++i) {
        std::cout << "found query:" << std::endl << query_match[i] << std::endl;
        t.queries.push_back(query_match[i]);
      }
    }
    theories[t.id.string()] = t;
  }

  std::string get_theories() {
    std::string body = "";
    for (auto& [key, t] : theories) {
      body = "";
      get_theory(t, body);
      //add_knowledge(t.id.string(), body);
    }
    return body;
  }
  
  void get_theory(Theory &t, std::string &body) {
    std::filesystem::path package_path = std::filesystem::path(ament_index_cpp::get_package_share_directory("coresense_understanding"));
    if (!t.added) {
      for (std::string key : t.includes) {
        std::filesystem::path file_path = package_path / std::filesystem::path(key);
        if (std::filesystem::exists(file_path)) {
          get_theory(theories[file_path.string()], body);
        } else {
          RCLCPP_WARN(get_logger(), "get_theory: Path does not exist.\n Missing: %s,\n Imported by: %s", file_path.string().c_str(), t.id.string().c_str());
        }
      }
      RCLCPP_INFO(get_logger(), "get_theory: Adding Theory:\n%s", t.id.string().c_str());
      body += t.content;
      t.added = true;
    }
  }

  void test_understanding(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<coresense_msgs::srv::TestUnderstanding::Request> request,
      const std::shared_ptr<coresense_msgs::srv::TestUnderstanding::Response> response) {
    std::cout << "receiving call" << std::endl;
    std::filesystem::path package_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(request->package_name));
    std::filesystem::path file_path = std::filesystem::path(request->file_path);
    Theory t = import_tptp_file(package_path / file_path);
    std::cout << "imported theory " << t.id << std::endl;
    for (std::string include : t.includes) {
      // TODO SECONDARY PRIORITY make sure we have imported the file's includes
    }
    // TODO SECONDARY PRIORITY can we ensure this is only one? or shall we support multiple queries?
    for (std::string query : t.queries) {
      std::cout << "running query:" << std::endl << query << std::endl;
      std::string config = "";
      auto future = send_goal(query, config);
      std::cout << "sent goal" << std::endl;
      if (future.get() != nullptr) {
        goals[future.get()->get_goal_id()] = std::shared_ptr<UnderstandActionGoalHandle>();
        response->success = true;
      } else {
        response->success = false;
      }
    }
    // TODO when should goals be erased
    //goals.erase(wrapped_result.goal_id);


    // manage understanding process
    // - manage used theories
    // TODO WAITING PRIORITY create new reasoner session
    // load reasoner with context theories
    //   add_knowledge("context");
    return;
  }

  void test_response_parsing() {
    //Problem p = read_problem(ament_index_cpp::get_package_share_directory("coresense_understanding") + "/understanding-logic/tff/tests/test-wind-scenario.tff");
    //send_goal(p.content, "--selection 10 --output_mode vampire --proof off --theory_axioms off --saturation_algorithm fmb --term_ordering lpo");
    //send_goal(p.content, "--selection 10 --output_mode vampire --proof off --theory_axioms on --saturation_algorithm fmb");
    //send_goal(p.content, "--selection 1011 --proof off");
    //auto response = "% SZS answers Tuple [(∀X1,X0.[M->exert(we_engine,s(exert(dse_engine,ius),s(X0,s(exert(dsee_engine,s(exert(dse_engine,ius),ius)),X1))))]|∀X0.[M->exert(we_engine,s(exert(dsee_engine,s(exert(dse_engine,ius),ius)),s(exert(dse_engine,ius),X0)))])|_] for test-wind-scenario";
    std::string response = "% SZS answers Tuple [([M->exert1(accept_size_engine,exert1(eliminate_worst_engine,exert1(order_lexicographical_engine,exert1(aggregate_preferences_engine,exert2(assess_engine,given_alternatives_modelet,given_cues_modelet)))))]|[M->exert1(accept_size_engine,exert1(eliminate_worst_engine,exert1(order_condorcet_engine,exert1(aggregate_preferences_engine,exert2(assess_engine,given_alternatives_modelet,given_cues_modelet)))))]|[M->exert1(accept_size_engine,exert1(eliminate_worst_engine,exert1(order_dominating_engine,exert1(aggregate_preferences_engine,exert2(assess_engine,given_alternatives_modelet,given_cues_modelet)))))]|[M->exert1(accept_size_engine,exert1(eliminate_worst_engine,exert1(order_lexicographical_engine,exert1(aggregate_utility_sum_engine,exert2(assess_engine,given_alternatives_modelet,given_cues_modelet)))))]|[M->exert1(accept_size_engine,exert1(eliminate_worst_engine,exert1(order_dominating_engine,exert1(aggregate_utility_sum_engine,exert2(assess_engine,given_alternatives_modelet,given_cues_modelet)))))]|[M->exert1(accept_size_engine,exert1(eliminate_worst_engine,exert1(order_condorcet_engine,exert1(aggregate_utility_sum_engine,exert2(assess_engine,given_alternatives_modelet,given_cues_modelet)))))]|[M->exert1(accept_size_engine,exert1(eliminate_worst_engine,exert1(order_lexicographical_engine,exert1(aggregate_utility_boolean_engine,exert2(assess_engine,given_alternatives_modelet,given_cues_modelet)))))]|[M->exert1(accept_size_engine,exert1(eliminate_worst_engine,exert1(order_dominating_engine,exert1(aggregate_utility_boolean_engine,exert2(assess_engine,given_alternatives_modelet,given_cues_modelet)))))]|[M->exert1(accept_size_engine,exert1(eliminate_worst_engine,exert1(order_condorcet_engine,exert1(aggregate_utility_boolean_engine,exert2(assess_engine,given_alternatives_modelet,given_cues_modelet)))))]|[M->exert1(accept_size_engine,exert1(eliminate_worst_engine,exert1(order_lexicographical_engine,exert1(aggregate_utility_signed_engine,exert2(assess_engine,given_alternatives_modelet,given_cues_modelet)))))]|[M->exert1(accept_size_engine,exert1(eliminate_worst_engine,exert1(order_dominating_engine,exert1(aggregate_utility_signed_engine,exert2(assess_engine,given_alternatives_modelet,given_cues_modelet)))))]|[M->exert1(accept_size_engine,exert1(eliminate_worst_engine,exert1(order_condorcet_engine,exert1(aggregate_utility_signed_engine,exert2(assess_engine,given_alternatives_modelet,given_cues_modelet)))))])|_] for test-decision-questions";
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

  void add_knowledge(std::string id, std::string theory) {
    while (!add_knowledge_client_ptr->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Client interrupted while waiting for service to appear.");
      }
      RCLCPP_INFO(get_logger(), "Waiting for service to appear...");
    }
    auto request = std::make_shared<coresense_msgs::srv::AddKnowledge::Request>();
    request->tptp = theory;
    request->id = id;
    auto result_future = add_knowledge_client_ptr->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
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
        std::cout << "Created ConceptNode " << match[2] << std::endl;
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
    //TODO PRIORITY: how to create 'parallel' parts
    if (std::regex_search(answer, match, exert_regex)) {
      if (map.find(match[1]) == map.end()) {
        map[match[1]] = std::make_shared<ConceptNode>(match[1]);
        std::cout << "Created ConceptNode " << match[1] << std::endl;
      }
      if (map.find(match[2]) == map.end()) {
        map[match[2]] = std::make_shared<ConceptNode>(match[2]);
        std::cout << "Created ConceptNode " << match[2] << std::endl;
      }
      auto n = std::make_shared<ExertNode>(match); 
      map[n->id] = n;
      std::cout << "Created ExertNode " << n->id << " from exert(" << match[1] << "," << match[2] << ")" << std::endl;
      answer = std::regex_replace(answer, exert_regex, n->id, std::regex_constants::format_first_only);
      return true;
    } else {
      return false;
    }
  }

  bool consume_exertn(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map, std::string & answer) {
    const std::regex exert_regex = std::regex("exert(\\d+)\\(([\\w-]+)([,\\w-]+)\\)");
    const std::regex modelet_regex = std::regex("([\\w-]+)");
    std::smatch match;
    if (std::regex_search(answer, match, exert_regex)) {
      //int modelet_count = stoi(match[1]);
      //TODO SECONDARY PRIORITY: how do we decide whether an engine was run yet
      // idea: create an id based on used inputs
      // not sure this scales to complex cases though (ones where the same engine is run with different configurations)
      // we'll have to test this
      if (map.find(match[2]) == map.end()) {
        // engine
        auto n = std::make_shared<ExertnNode>(match[2]); 
        map[n->id] = n;
        std::cout << "created ExertnNode " << n->id << " from " << match[0] << std::endl;
        answer = std::regex_replace(answer, exert_regex, n->id, std::regex_constants::format_first_only);
        const std::sregex_token_iterator End;
        std::string modelets = match[3];
        for (std::sregex_token_iterator it(modelets.begin(), modelets.end(), modelet_regex, 1); it != End; ++it) {
          std::string modelet = *it;
          if (map.find(modelet) == map.end()) {
            // this has to be a modelet because it hasn't been added to the map yet, as an engine-id would have been
            map[modelet] = std::make_shared<ConceptNode>(modelet);
            std::cout << "Created ConceptNode " << modelet << std::endl;
          }
          n->add_node(map[modelet]);
        }
      }
      return true;
    } else {
      return false;
    }
  }

  void consume_answer_set(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map, std::string & answer) {
    if (consume_exert(map, answer)) {
      consume_answer_set(map, answer);
    } else if (consume_subset(map, answer)) {
      consume_answer_set(map, answer);
    }
    return;
  }

  void consume_answer(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map, std::string & answer) {
    if (consume_exertn(map, answer)) {
      consume_answer(map, answer);
    }
    return;
  }

  std::vector<std::string> parse_reasoner_output(std::string output) {
    std::vector<std::string> trees;
    const std::regex answer_line_regex = std::regex("^% SZS answers Tuple \\[\\((.*)\\)\\|_\\] for [\\-\\w]+$");
    const std::regex answer_regex = std::regex("\\[\\w->(.*?)\\]\\|?");
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
        std::string answers = answer_match[1];
        const std::sregex_token_iterator End;
        for (std::sregex_token_iterator it(answers.begin(), answers.end(), answer_regex, 1); it != End; ++it) {
          std::string answer = *it;
          consume_answer(map, answer);
          trees.push_back(build_behavior_tree(map, answer));
        }
      } else {
        std::cout << "no match" << std::endl;
      }
    }
    return trees;
  }

  std::string build_behavior_tree(std::unordered_map<std::string, std::shared_ptr<GraphNode>> map, std::string root_id) {
    return "<root BTCPP_format='4'>\n<BehaviorTree ID='Understanding Solution XYZ'>\n<Sequence>\n" + map[root_id]->print(map) + "\n</Sequence>\n</BehaviorTree>\n</root>";
  }

  void execute(const std::shared_ptr<UnderstandActionGoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();
    const std::string modelet = goal->target_modelet;
    // TODO check that this works 
    std::stringstream ss;
    ss << "tff(query, question," << std::endl;
    ss << modelet << std::endl;
    ss << ").";
    std::string query = ss.str();
    // manage understanding process
    // - manage used theories
    // TODO WAITING PRIORITY create new reasoner session
    // load reasoner with context theories
    add_knowledge("agent_model", agent_model);
    //auto result = std::make_shared<UnderstandAction::Result>();
    std::string config = "";
    send_goal_from_action(query, config, goal_handle);
    // return result
  }


  void send_goal_from_action(std::string query, std::string config, const std::shared_ptr<UnderstandActionGoalHandle> understanding_goal_handle) {
    auto future = send_goal(query, config);
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

  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<coresense_msgs::action::QueryReasoner>>> send_goal(std::string query, std::string config) {

    std::cout << "waiting for reasoner" << std::endl;
    if (!query_reasoner_action_client_ptr->wait_for_action_server()) {
      //need to throw some exeception here to denote the failing wait
      return std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<coresense_msgs::action::QueryReasoner>>>();
    }
    std::cout << "have reasoner" << std::endl;

    auto goal_msg = QueryReasonerAction::Goal();
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
      std::stringstream ss;
      ss << "Current state of reasoning is " << feedback->status;
      RCLCPP_INFO(get_logger(), ss.str().c_str());
      //TODO forward feedback in some form?
    };

    send_goal_options.result_callback = [this](const QueryReasonerActionGoalHandle::WrappedResult & wrapped_result) {
      auto understanding_result = std::make_shared<UnderstandAction::Result>();
      switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: {
          RCLCPP_INFO(get_logger(), "Goal succeeded with code %d", wrapped_result.result->code);
          RCLCPP_INFO(get_logger(), "Reasoner output is:\n%s", wrapped_result.result->std_output.c_str());
          std::vector<std::string> trees = parse_reasoner_output(wrapped_result.result->std_output);
          //TODO PRIORITY: check that this works: how to get the result from this callback into the calling callback? used an object attribute?
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
  rclcpp::spin(std::make_shared<coresense_understanding_cpp::UnderstandingSystemNode>());
  rclcpp::shutdown();
  return 0;
}
