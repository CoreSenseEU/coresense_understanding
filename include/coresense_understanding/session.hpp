#include <ctime>
#include <string>

namespace coresense::understanding::session {
struct Session {
  std::string id;
  time_t start_time;
  time_t last_agent_update;
  time_t last_knowledge_update;
};
}
