#include "coresense_understanding/knowledge_model.hpp"

namespace coresense::understanding::knowledge_model {
  std::string split(std::string iri) {
    auto const pos = iri.find_last_of('#');
    return iri.substr(pos + 1);
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
      modelets[id] = m;
       //RCLCPP_INFO(get_logger(), "Adding modelet %s, using formalism %s", m.name.c_str(), m.formalism.c_str());
     }

     if (binding.find("concept") != binding.end()) {
       std::string concept = split(binding["concept"]["value"].get<std::string>());
       if  (!concept.empty()) {
         modelets[id].concepts.insert(concept);
         //RCLCPP_INFO(get_logger(), "Adding concept %s to modelet %s", concept.c_str(), modelet.c_str());
       }
     }
     if (binding.find("representationClass") != binding.end()) {
       std::string representation_class = split(binding["representationClass"]["value"].get<std::string>());
       if (!representation_class.empty()) {
         modelets[id].representation_classes.insert(representation_class);
         //RCLCPP_INFO(get_logger(), "Adding representation class %s to modelet %s", representation_class.c_str(), modelet.c_str());
       }
     }
     if (binding.find("property") != binding.end()) {
       coresense::understanding::model::Property property;
       property.name = split(binding["propertyName"]["value"].get<std::string>());
       property.datatype = split(binding["propertyType"]["value"].get<std::string>());
       property.value = split(binding["propertyValue"]["value"].get<std::string>());
       modelets[id].properties.insert(property);
       //RCLCPP_INFO(get_logger(), "Adding property %s to modelet %s", property.name.c_str(), modelet.c_str());
     }
     
       
     //RCLCPP_INFO(get_logger(), "Got modelet %s, in formalism %s, modelling concept %s", modelet.c_str(), formalism.c_str(), concept.c_str());

   }
   //TODO: insert modelet information into vampire state
   std::stringstream ss2;

   for (auto & [id, modelet] : modelets) {
     //RCLCPP_INFO(get_logger(), "Adding modelet %s from knowledge base to modelet model:\n%s", name.c_str(), modelet.to_tff().c_str());
     ss2 << modelet.to_tff() << std::endl;
   }
   model = ss2.str(); 
   last_update = std::chrono::system_clock::now();
   dirty = false;
  }
}
