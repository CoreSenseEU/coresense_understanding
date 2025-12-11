#ifndef model_hpp
#define model_hpp

#include <set>
#include <list>

#include <nlohmann/json.hpp>

namespace coresense::understanding::model {

std::string create_relation_limit1(std::string relation, std::string individual, std::string set_klass,  std::list<std::string> set);

std::string create_relation_exist1(std::string relation, std::string individual, std::string set_klass,  std::list<std::string> set);

std::string create_relation_exist2(std::string relation, std::string individual, std::string set_klass,  std::set<std::string> set);

std::string create_has_no_relation1(std::string relation, std::string individual, std::string set_klass);
std::string create_has_no_relation2(std::string relation, std::string individual, std::string set_klass);

struct Requirement {
  std::string name;
  std::string datatype;
  std::string value_range;
  std::string to_tff();
};
void from_json(const nlohmann::json& j, Requirement& r);

struct Template {
  std::string name;
  std::string formalism;
  std::string creator;
  std::list<std::string> representation_classes;
  std::list<std::string> concepts;
  //std::list<std::string> locations;
  //std::list<std::string> extents;
  std::list<Requirement> requirements;
  std::string to_tff();
};
void from_json(const nlohmann::json& j, Template& t);

struct Property {
  std::string name;
  std::string datatype;
  std::string value;
};
void from_json(const nlohmann::json& j, Property& p);

struct Modelet {
  std::string name;
  std::string formalism;
  std::list<std::string> representation_classes;
  std::list<std::string> concepts;
  std::list<Property> properties;
};
void from_json(const nlohmann::json& j, Modelet& m);

struct Resource {
  std::string name;
  int percentage;
};
void from_json(const nlohmann::json& j, Resource& r);

struct Engine {
  std::string name;
  std::list<Template> inputs;
  Modelet engine_output;
  int time_delay;
  int energy_cost;
  std::list<std::string> transit_properties;
  std::list<Resource> resources_consumed;
  std::list<Resource> resources_blocked;
  std::string to_tff();
};
void from_json(const nlohmann::json& j, Engine& e);

} // namespace 'coresense::understanding::model' ends

#endif
