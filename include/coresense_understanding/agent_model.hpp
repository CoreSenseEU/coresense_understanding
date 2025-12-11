#ifndef agent_model_hpp
#define agent_model_hpp

#include <list>
#include <nlohmann/json.hpp>

#include <coresense_understanding/model.hpp>


namespace coresense::understanding::agent_model {
class AgentModel {
private:
  std::vector<coresense::understanding::model::Engine> engines;
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
    {"", std::set<std::string>()}
  };
public:
  std::string model;
  
  AgentModel() {};
  void create_engine_model(std::string node_name, std::string engine_annotation);
  void create_engine_relations();
  std::string create_inter_engine_relations(std::set<int> sizes);
  std::string create_distinct_axiom(std::string label, std::set<std::string> items);
  std::string create_existence_declarations(std::string klass, std::set<std::string> items);
};

}
#endif
