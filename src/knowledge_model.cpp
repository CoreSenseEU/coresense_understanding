#include "coresense_understanding/knowledge_model.hpp"

namespace coresense::understanding::knowledge_model {

std::string split(std::string iri) {
  auto const pos = iri.find_last_of('#');
  return iri.substr(pos + 1);
}

// Adds existing instances to the knowledge_model's distinct_instances sets of the respective klass.
void KnowledgeModel::add_klass(std::string klass, std::string kb_response) {
  std::stringstream ss;
  ss << kb_response;
  nlohmann::json result = nlohmann::json::parse(ss);
  for (nlohmann::json binding : result["results"]["bindings"]) {
    std::string id = split(binding[klass]["value"].get<std::string>());
    distinct_instances[klass].insert(id);
  }
}
// This is not used
void KnowledgeModel::add_properties(std::string kb_response) {
  std::stringstream ss;
  ss << kb_response;
  nlohmann::json result = nlohmann::json::parse(ss);
  for (nlohmann::json binding : result["results"]["bindings"]) {
    coresense::understanding::model::Property property;
    property.klass = split(binding["property"]["value"].get<std::string>());
    property.value = split(binding["propertyValue"]["value"].get<std::string>());
    distinct_instances["property"].insert(property.klass);
    //TODO were to put the property object other than inside the modelet if so?
  }
}

// This is not used
void KnowledgeModel::add_requirements(std::string kb_response) {
  std::stringstream ss;
  ss << kb_response;
  nlohmann::json result = nlohmann::json::parse(ss);
  for (nlohmann::json binding : result["results"]["bindings"]) {
    coresense::understanding::model::Requirement requirement;
    requirement.klass = split(binding["requirement"]["value"].get<std::string>());
    requirement.value_range = split(binding["requirementValueRange"]["value"].get<std::string>());
    distinct_instances["property"].insert(requirement.klass);
    //TODO were to put the requirement object other than the template if so?
  }
}

void KnowledgeModel::create_knowledge_model(std::string kb_response) {
  std::stringstream ss;
  ss << kb_response;
  nlohmann::json result = nlohmann::json::parse(ss);
  for (nlohmann::json binding : result["results"]["bindings"]) {
    std::string id = split(binding["modelet"]["value"].get<std::string>());
    if (modelets.find(id) == modelets.end()) {
      coresense::understanding::model::Modelet m;
      m.name = id;
      m.formalism = split(binding["formalism"]["value"].get<std::string>());
      distinct_instances["formalism"].insert(m.formalism);
      modelets[id] = m;
      distinct_instances["modelet"].insert(id);
    }
    if (binding.find("concept") != binding.end()) {
      std::string concept = split(binding["concept"]["value"].get<std::string>());
      if  (!concept.empty()) {
        modelets[id].concepts.insert(concept);
        distinct_instances["concept"].insert(concept);
      }
    }
    if (binding.find("representationClass") != binding.end()) {
      std::string representation_class = split(binding["representationClass"]["value"].get<std::string>());
      if (!representation_class.empty()) {
        modelets[id].representation_classes.insert(representation_class);
        distinct_instances["representation_class"].insert(representation_class);
      }
    }
    if (binding.find("property") != binding.end()) {
      coresense::understanding::model::Property property;
      //TODO remove the http paths/namespaces?
      property.klass = split(binding["propertyClass"]["value"].get<std::string>());
      property.value = "\"" + split(binding["propertyValue"]["value"].get<std::string>()) + "\"^^xsd:" + split(binding["propertyValue"]["datatype"].get<std::string>());
      modelets[id].properties.insert(property);
      distinct_instances["property"].insert(property.klass);
    }
  }
  std::stringstream ss2;
  for (std::string klass : klasses) {
    for (std::string instance : distinct_instances[klass]) {
      ss2 << "tff(decl_" << instance << "_" << klass << ", type, " << klass << "_" << instance << " : " << klass << ").\n";
    }
  }
  for (std::string klass : {"property", "requirement"}) {
    for (std::string instance : distinct_instances[klass]) {
      ss2 << "tff(decl_" << instance << "_" << klass << ", type, " << klass << "_" << instance << " : " << klass << ").\n";
    }
  }
  //for (std::string instance : distinct_instances["value"]) {
  //  ss2 << "tff(decl_" << instance.substr(instance.rfind("^^xsd:")+6) << "_" << klass << ", type, '" << instance << "' : " << klass << ").\n";
  //}
  for (auto& [id, modelet] : modelets) {
    ss2 << modelet.to_tff() << std::endl;
  }
  model = ss2.str(); 
  last_update = std::chrono::system_clock::now();
  dirty = false;
  }
}
