#include <chrono>
#include <string>

namespace coresense::understanding::session {
class Session {
public:
  Session();
  Session(std::string &id);
  std::string id;
  std::chrono::time_point<std::chrono::system_clock>  start_time;
  std::chrono::time_point<std::chrono::system_clock>  last_agent_update;
  std::chrono::seconds agent_model_max_age;
  std::chrono::time_point<std::chrono::system_clock>  last_knowledge_update;
  std::chrono::seconds knowledge_model_max_age;
  //TODO create servite to mark models as dirty so they can be updated when needed
};
}
