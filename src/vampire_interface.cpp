#include "coresense_understanding/vampire_interface.hpp"

namespace coresense::understanding::interfaces::vampire {

std::vector<std::string> VampireInterface::parse_output(std::string output) {
  std::vector<std::string> trees;
  //with label after 'for', with optional capturing group
  //const std::regex answer_line_regex = std::regex("^% SZS answers Tuple \\[\\(?:(.*)\\)?\\|_\\] for [\\-\\w]+$");
  const std::regex answer_line_regex = std::regex("^% SZS answers Tuple \\[(.*)\\|_\\] for");
  //with label after 'for', with necessary capturing group
  //const std::regex answer_line_regex = std::regex("^% SZS answers Tuple \\[(.*)\\|_\\] for [\\-\\w]+$");
  const std::regex answer_regex = std::regex("\\[\\w->(.*?)\\]\\|?");
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* rootXML = doc.NewElement("root");
  rootXML->SetAttribute("BTCPP_format", 4);
  doc.InsertFirstChild(rootXML);
  std::string line;
  std::stringstream string_handle;

  string_handle << output;
  while (std::getline(string_handle, line, '\n') ) {
    //extract includes
    std::smatch answer_match;
    std::unordered_map<std::string, std::shared_ptr<ns_graph::GraphNode>> map;
    if (std::regex_search(line, answer_match, answer_line_regex)) {
      std::string answers = answer_match[1];
      const std::sregex_token_iterator End;
      for (std::sregex_token_iterator it(answers.begin(), answers.end(), answer_regex, 1); it != End; ++it) {
        std::string answer = *it;
        consume_answer(map, answer);
        trees.push_back(build_behavior_tree(map, answer));
      }
    } else {
      // silently handle no match
    }
  }
  return trees;
}

std::string VampireInterface::build_behavior_tree(std::unordered_map<std::string, std::shared_ptr<ns_graph::GraphNode>> map, std::string root_id) {
  return "<root BTCPP_format=\"4\">\n<BehaviorTree ID=\"Understanding Solution XYZ\">\n<Sequence>\n" + map[root_id]->print(map) + "\n</Sequence>\n</BehaviorTree>\n</root>";
}

void VampireInterface::consume_answer(std::unordered_map<std::string, std::shared_ptr<ns_graph::GraphNode>> & map, std::string & answer) {
  if (consume_exertn(map, answer)) {
    consume_answer(map, answer);
  }
  return;
}

void VampireInterface::consume_answer_set(std::unordered_map<std::string, std::shared_ptr<ns_graph::GraphNode>> & map, std::string & answer) {
  if (consume_exert(map, answer)) {
    consume_answer_set(map, answer);
  } else if (consume_subset(map, answer)) {
    consume_answer_set(map, answer);
  }
  return;
}

bool VampireInterface::consume_exert(std::unordered_map<std::string, std::shared_ptr<ns_graph::GraphNode>> & map, std::string & answer) {
  // This reads an exert(engine, modelet_set) string and turns it into an object representation, recursively. This is the no-longer used set variant
  const std::regex exert_regex = std::regex("exert\\(([\\w-]+)\\,([\\w-]+)\\)");
  std::smatch match;
  // this does not yet handle parallel execution of nodes, which is why it was replaced by the set-less version below.
  if (std::regex_search(answer, match, exert_regex)) {
    if (map.find(match[1]) == map.end()) {
      map[match[1]] = std::make_shared<ns_graph::ConceptNode>(match[1]);
      //std::cout << "Created ConceptNode " << match[1] << std::endl;
    }
    if (map.find(match[2]) == map.end()) {
      map[match[2]] = std::make_shared<ns_graph::ConceptNode>(match[2]);
      //std::cout << "Created ConceptNode " << match[2] << std::endl;
    }
    auto n = std::make_shared<ns_graph::ExertNode>(match); 
    map[n->id] = n;
    //std::cout << "Created ExertNode " << n->id << " from exert(" << match[1] << "," << match[2] << ")" << std::endl;
    answer = std::regex_replace(answer, exert_regex, n->id, std::regex_constants::format_first_only);
    return true;
  } else {
    return false;
  }
}

bool VampireInterface::consume_exertn(std::unordered_map<std::string, std::shared_ptr<ns_graph::GraphNode>> & map, std::string & answer) {
  // This reads exert[0-9]+(engine, modelet1, modelet2, ..., modeletN) string and turns it into an object representation, recursively. This is the new set-less variant
  const std::regex exert_regex = std::regex("exert(\\d+)\\(([\\w-]+)([,\\w-]+)\\)");
  const std::regex modelet_regex = std::regex("([\\w-]+)");
  std::smatch match;
  if (std::regex_search(answer, match, exert_regex)) {
    //int modelet_count = stoi(match[1]);
    //TODO SECONDARY PRIORITY: how do we decide whether an engine was run yet
    // idea: create an id based on used inputs
    // not sure this scales to complex cases though (ones where the same engine is run with different configurations)
    // we'll have to test this
    if (map.find(match[2]) == map.end()) {
      // engine
      auto n = std::make_shared<ns_graph::ExertnNode>(match[2]); 
      map[n->id] = n;
      //std::cout << "created ExertnNode " << n->id << " from " << match[0] << std::endl;
      answer = std::regex_replace(answer, exert_regex, n->id, std::regex_constants::format_first_only);
      const std::sregex_token_iterator End;
      std::string modelets = match[3];
      for (std::sregex_token_iterator it(modelets.begin(), modelets.end(), modelet_regex, 1); it != End; ++it) {
        std::string modelet = *it;
        if (map.find(modelet) == map.end()) {
          // this has to be a modelet because it hasn't been added to the map yet, as an engine-id would have been
          map[modelet] = std::make_shared<ns_graph::ConceptNode>(modelet);
          //std::cout << "Created ConceptNode " << modelet << std::endl;
        }
        n->add_node(map[modelet]);
      }
    }
    return true;
  } else {
    return false;
  }
}

bool VampireInterface::consume_subset(std::unordered_map<std::string, std::shared_ptr<ns_graph::GraphNode>> & map, std::string & answer) {
  const std::regex subset_regex = std::regex("s\\(([\\w-]+)\\,([\\w-]+)\\)");
  std::smatch match;
  if (std::regex_search(answer, match, subset_regex)) {
    if (map.find(match[1]) == map.end()) {
      map[match[1]] = std::make_shared<ns_graph::ConceptNode>(match[1]);
      //std::cout << "created ConceptNode " << match[1] << std::endl;
    }
    if (map.find(match[2]) == map.end()) {
      map[match[2]] = std::make_shared<ns_graph::ConceptNode>(match[2]);
      //std::cout << "Created ConceptNode " << match[2] << std::endl;
    }
    auto n = std::make_shared<ns_graph::SubsetNode>(match);
    map[n->id] = n;
    //std::cout << "Created SubsetNode " << n->id << " from s(" << match[1] << "," << match[2] << ")" << std::endl;
    answer = std::regex_replace(answer, subset_regex, n->id, std::regex_constants::format_first_only);
    return true;
  } else {
    return false;
  }
}



tinyxml2::XMLElement * VampireInterface::add_parallel_node(tinyxml2::XMLDocument doc, tinyxml2::XMLElement* target_node) {
  tinyxml2::XMLElement * parallel_node = doc.NewElement("Parallel");
  target_node->InsertEndChild(parallel_node);
  return parallel_node;
}

tinyxml2::XMLElement * VampireInterface::add_sequence_node(tinyxml2::XMLDocument doc, tinyxml2::XMLElement* target_node) {
  tinyxml2::XMLElement * sequence_node = doc.NewElement("Sequence");
  target_node->InsertEndChild(sequence_node);
  return sequence_node;
}

void VampireInterface::add_engine_node(tinyxml2::XMLDocument doc, tinyxml2::XMLElement* root, tinyxml2::XMLElement* target_node, std::string engine_name, std::string ros_pkg, std::string path) {
  tinyxml2::XMLElement * subtree_node = doc.NewElement("Subtree");
  subtree_node->SetAttribute("ID", engine_name.c_str());
  target_node->InsertEndChild(subtree_node);
  tinyxml2::XMLElement * include_node = doc.NewElement("include");
  include_node->SetAttribute("ros_pkg", ros_pkg.c_str());
  include_node->SetAttribute("path", path.c_str());
  root->InsertFirstChild(include_node);
}

} // namespace 'coresense::understanding::interfaces::vampire' ends
