#ifndef knowledge_model_hpp
#define knowledge_model_hpp

#include <chrono>
#include <string>
#include <nlohmann/json.hpp>

#include <coresense_understanding/model.hpp>

namespace coresense::understanding::knowledge_model {
class KnowledgeModel {
private:
  std::map<std::string, coresense::understanding::model::Modelet> modelets;
public:
  std::string model;
  bool dirty;
  std::chrono::time_point<std::chrono::system_clock> last_update;
  
  KnowledgeModel() {};
  void create_knowledge_model(std::string kb_response);
};

}
#endif
