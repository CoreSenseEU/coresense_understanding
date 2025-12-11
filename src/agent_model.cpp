
#include "coresense_understanding/agent_model.hpp"

namespace coresense::understanding::agent_model {

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


void AgentModel::create_engine_model(std::string node_name, std::string engine_annotation) {
    nlohmann::json j = nlohmann::json::parse(engine_annotation);
    auto engine = j.get<coresense::understanding::model::Engine>();
    engines.push_back(engine);
//    } catch (const json::parse_error& e) {
//      RCLCPP_ERROR(get_logger(), "Could not read Node %s annotation. Caught error: %s", node_name.c_str(), e.what());
//    } catch (const json::type_error& e) {
//      RCLCPP_ERROR(get_logger(), "Could not read Node %s annotation. Caught error: %s", node_name.c_str(), e.what());
//    } catch (const json::invalid_iterator& e) {
//      RCLCPP_ERROR(get_logger(), "Could not read Node %s annotation. Caught error: %s", node_name.c_str(), e.what());
//    } catch (const json::out_of_range& e) {
//      RCLCPP_ERROR(get_logger(), "Could not read Node %s annotation. Caught error: %s", node_name.c_str(), e.what());
//    } catch (const json::other_error& e) {
//      RCLCPP_ERROR(get_logger(), "Could not read Node %s annotation. Caught error: %s", node_name.c_str(), e.what());
//    } catch (...) {
//      std::cout << "caught some unknown error from json parsing" << std::endl;
    
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
  


void AgentModel::create_engine_relations() {
  std::stringstream part_relations;
  std::stringstream all_relations;
  std::stringstream engine_relations;
  std::set<int> engine_input_sizes;
  engine_input_sizes.insert(1);
  engine_input_sizes.insert(2);
  engine_input_sizes.insert(3);
   
  //collect data per needed relation from all the engines
  std::map<std::string, std::set<std::string>> imparted_rcs;
  std::map<std::string, std::set<std::string>> imparted_concepts;
  std::map<std::string, std::set<std::string>> imparted_properties;
  std::map<std::string, std::set<std::string>> templates_rc;
  std::map<std::string, std::set<std::string>> templates_concept;
  std::map<std::string, std::set<std::string>> templates_requirement;

  for (coresense::understanding::model::Engine engine : engines) {
    //add engine ttf to output
    engine_relations << engine.to_tff();
    // collect data for inter-engine relations
    distinct_instances["engine"].insert(engine.name);
    // Inputs
    engine_input_sizes.insert(engine.inputs.size());
    for (auto input : engine.inputs) {
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
    distinct_instances["formalism"].insert(engine.engine_output.formalism);
    for (std::string representation_class : engine.engine_output.representation_classes) {
      distinct_instances["representation_class"].insert(representation_class);
      if (imparted_rcs.find(representation_class) == imparted_rcs.end()) {
        imparted_rcs[representation_class] = std::set<std::string>();
      }
      imparted_rcs[representation_class].insert(engine.name);
    }
    for (std::string concept : engine.engine_output.concepts) {
      distinct_instances["concept"].insert(concept);
      if (imparted_concepts.find(concept) == imparted_concepts.end()) {
       imparted_concepts[concept] = std::set<std::string>();
      }
      imparted_concepts[concept].insert(engine.name);
    }
    for (auto property : engine.engine_output.properties) {
      distinct_instances["property"].insert(property.name);
      distinct_instances["proptype"].insert(property.datatype);
      if (imparted_properties.find(property.name) == imparted_properties.end()) {
       imparted_properties[property.name] = std::set<std::string>();
      }
      imparted_properties[property.name].insert(engine.name);
    } 
    for (auto resource : engine.resources_consumed) {
      distinct_instances["resource"].insert(resource.name);
    } 
    for (auto resource : engine.resources_blocked) {
      distinct_instances["resource"].insert(resource.name);
    } 
  }
  // create instance declarations and distinctness axioms
  for (auto& [key, set] : distinct_instances) {
    part_relations << create_existence_declarations(key, set);
    part_relations << create_distinct_axiom(key + "_distinctness", set);
  }

  //std::string inter_relations = create_inter_engine_relations(engine_input_sizes);
  //std::ofstream out1("/home/alex/plansys2_ws/inter_relations.tff");
  //out1 << inter_relations;
  //out1.close();
  //add_knowledge_internal("inter_relations", inter_relations);
  //add_knowledge("inter_relations", inter_relations);
  //all_relations << create_inter_engine_relations(engine_input_sizes);
  //std::string part_rels = part_relations.str();
  //std::ofstream out2("/home/alex/plansys2_ws/part_relations.tff");
  //out2 << part_rels;
  //out2.close();
  //add_knowledge_internal("parts", part_rels);
  //add_knowledge("parts", part_relations.str());
  //all_relations << part_relations.str();
  //std::string engine_rels = engine_relations.str();
  //std::ofstream out3("/home/alex/plansys2_ws/engine_relations.tff");
  //out3 << engine_rels;
  //out3.close();
  //add_knowledge_internal("engine_relations", engine_rels);
  //add_knowledge("engine_relations", engine_relations.str());
  all_relations << create_inter_engine_relations(engine_input_sizes);
  all_relations << part_relations.rdbuf();
  all_relations << engine_relations.rdbuf();
  
  // These encode the property->engine direction 1->n
  for (const auto& [requirement, templates] : templates_requirement) {
    if (!templates.empty()) {
      std::list<std::string> li(templates.begin(), templates.end());
      all_relations << coresense::understanding::model::create_relation_limit1("is_part_of", requirement, "template", li);
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
  all_relations << std::endl;
  //std::string rest_rels = all_relations.str();
  //std::ofstream out4("/home/alex/plansys2_ws/rest_relations.tff");
  //out4 << rest_rels;
  //out4.close();
  //add_knowledge("rest_engine_relations", engine_relations.str());
  model = all_relations.str();
}



std::string AgentModel::create_inter_engine_relations(std::set<int> sizes) {
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

 std::string AgentModel::create_distinct_axiom(std::string label, std::set<std::string> items) {
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

std::string AgentModel::create_existence_declarations(std::string klass, std::set<std::string> items) {
  std::stringstream ss;
  for (std::string item: items) {
    ss << "tff(decl_"<< item << ", type, " << item << ": " << klass << ")." << std::endl;
  }
  return ss.str();
}


}
