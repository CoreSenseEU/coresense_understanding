#ifndef knowledge_model_hpp
#define knowledge_model_hpp

#include <chrono>
#include <string>
#include <list>
#include <nlohmann/json.hpp>

#include <coresense_understanding/model.hpp>

namespace coresense::understanding::knowledge_model {
class KnowledgeModel {
private:
  std::map<std::string, coresense::understanding::model::Modelet> modelets;
  std::map<std::string, std::set<std::string>> distinct_instances {
    {"formalism", std::set<std::string>()},
    {"representation_class", std::set<std::string>()},
    {"property", std::set<std::string>()},
    {"resource", std::set<std::string>()},
    {"concept", std::set<std::string>()},
    {"modelet", std::set<std::string>()},
    {"", std::set<std::string>()}
  };
public:
  std::string model;
  bool dirty;
  std::chrono::time_point<std::chrono::system_clock> last_update;
  std::map<std::string, bool> updated;
  std::list<std::string> klasses = {"property", "concept", "formalism", "representation_class"};
  
  KnowledgeModel() {};
  void add_klass(std::string klass, std::string kb_response);
  void add_formalisms(std::string kb_response);
  void add_concepts(std::string kb_response);
  void add_representation_classes(std::string kb_response);
  void add_properties(std::string kb_response);
  void add_requirements(std::string kb_response);
  void create_knowledge_model(std::string kb_response);
};

}
#endif
