#include "coresense_understanding/model.hpp"

namespace coresense::understanding::model {

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


void from_json(const nlohmann::json& j, Requirement& r) {
  j.at("name").get_to(r.name);
  j.at("datatype").get_to(r.datatype);
  j.at("value_range").get_to(r.value_range);
}

void from_json(const nlohmann::json& j, Template& t) {
  j.at("name").get_to(t.name);
  j.at("creator").get_to(t.creator);
  j.at("formalism").get_to(t.formalism);
  j.at("requirements").get_to(t.requirements);
  j.at("concepts").get_to(t.concepts);
  j.at("representation_classes").get_to(t.representation_classes);
//  j.at("extents").get_to(t.extents);
//  j.at("locations").get_to(t.locations);
}

void from_json(const nlohmann::json& j, Property& p) {
  j.at("name").get_to(p.name);
  j.at("datatype").get_to(p.datatype);
  j.at("value").get_to(p.value);
}

void from_json(const nlohmann::json& j, Modelet& m) {
  j.at("name").get_to(m.name);
  j.at("formalism").get_to(m.formalism);
  j.at("properties").get_to(m.properties);
  j.at("representation_classes").get_to(m.representation_classes);
  j.at("concepts").get_to(m.concepts);
//  j.at("extents").get_to(m.extents);
//  j.at("locations").get_to(m.locations);
}

void from_json(const nlohmann::json& j, Resource& r) {
  j.at("name").get_to(r.name);
  j.at("percentage").get_to(r.percentage);
}

void from_json(const nlohmann::json& j, Engine& e) {
  j.at("name").get_to(e.name);
  j.at("inputs").get_to(e.inputs);
  j.at("time_delay").get_to(e.time_delay);
  j.at("energy_cost").get_to(e.energy_cost);
  j.at("engine_output").get_to(e.engine_output);
  j.at("resources_consumed").get_to(e.resources_consumed);
  j.at("resources_blocked").get_to(e.resources_blocked);
  j.at("transit_properties").get_to(e.transit_properties);
}



std::string Requirement::to_tff() {
  std::stringstream req_decl, req_type, req_value_range, output;
  req_type << "tff(" << name << "_has_type, axiom, type_of_requirement(" << name << ") = " << datatype << ").\n";
  req_value_range << "tff(" << name << "_is_permissable, axiom, is_permissible(" << name << ", " << value_range << ")).\n";
  output << req_decl.str() << req_type.str() << req_value_range.str();
  return output.str();
}

std::string Template::to_tff() {
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
    for (Requirement req : requirements) {
      output << req.to_tff();
      requirement_names.insert(req.name);
    }
    output << create_relation_exist2("is_part_of", name, "requirement", requirement_names);
  }
  return output.str();
}

std::string Engine::to_tff() {
  std::stringstream output, templates;
  output << "tff(" << name << "_imparts_formalism, axiom,\n  output_modelet_formalism(" << name << ") = " << engine_output.formalism << "\n).\n";
  if (!inputs.empty()) {
    templates << "tff(" << name << "_input_definition_axiom, axiom,\n  defines_input" << inputs.size() << "(";
    for (Template templ : inputs) {
      //TODO PRIORITY this can lead to duplicate template definitions when engines define identically named templates
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
    for (Property property : engine_output.properties) {
      properties.push_back(property.name);
    }
    output << create_relation_exist1("engine_imparts_property", name, "property", properties);
  } else { // engine imparts no propertys
    output << create_has_no_relation1("engine_imparts_property", name, "property");
  }
  return output.str();
}

} // namespace 'coresense::understanding::model' ends
