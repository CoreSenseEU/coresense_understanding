
#include "uuid/uuid.h"

#include "coresense_understanding/understanding_graph.hpp"

using namespace coresense::understanding::graph;

GraphNode::GraphNode() {
  uuid_t uuid;
  char tmp[37];
  uuid_generate(uuid);
  uuid_unparse(uuid, tmp);
  id = std::string(tmp);
}


SubsetNode::SubsetNode(std::smatch match)
  : GraphNode()
  , element{ match[1] }
  , set{ match[2] }  {
}

std::string SubsetNode::print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map) {
  std::string tree = map[set]->print(map) + map[element]->print(map);
  return tree;
}

std::string SubsetNode::get_id() {
  return name + "_" + id;
}


ExertNode::ExertNode(std::smatch match)
  : GraphNode()
  , modelet_set { match[2] } {
  name = match[1];
}

std::string ExertNode::print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map) {
  std::string tree = map[modelet_set]->print(map) + "\n" +"      <SubTree ID=\"" + name +"\"/>";
  return tree;
}

std::string ExertNode::get_id() {
  return "EXERT_" + name + "_" + id;
}


ConceptNode::ConceptNode(std::string modelet_name)
  : GraphNode() {
  name = modelet_name;
}

std::string ConceptNode::print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map) {
  return name;
}

std::string ConceptNode::get_id() {
  return "MODELET_" + name + "_" + id;
}


ExertnNode::ExertnNode(std::string engine_name)
  : GraphNode() {
  name = engine_name;
}

void ExertnNode::add_node(std::shared_ptr<GraphNode> & node) {
  if (node->get_id().rfind("MODELET_", 0) == 0) {
    modelets.push_back(node->name);
  } else if (node->get_id().rfind("EXERT_", 0) == 0) {
    exerts.push_back(node->id);
    modelets.push_back(node->id);
  }
}

std::string ExertnNode::print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map) {
  std::ostringstream tree;
  //TODO think about where sequence nodes are put
  //  top level sequence
  //  moving in levels down is in fact putting things into the sequence at an earlier spot
  //  so there should only be one sequence? no. if a subtree (in a parallel node) has another subtree, that should be new sequence
  // first: add required subtree exertions, either in parallel or just on a newline in the parent sequence
  if (exerts.size() > 1) {
    // this should only happen if modelets are in fact exerts
    tree << "<Parallel failure_count=\"1\" success_count=\"" << exerts.size() << "\">" << std::endl;
    for (auto exert : exerts) {
      tree << "<Sequence>" << std::endl;
      tree << map[exert]->print(map) << std::endl;
      tree << "</Sequence>" << std::endl;
    }
    tree << "</Parallel>" << std::endl;
  } else if (!exerts.empty()) {
    tree << map[exerts[0]]->print(map) << std::endl;
  } 
  tree << "<SubTree ID=\"" << name << "\" output=\"" << get_id() << "_port\" ";
  //tree << map[modelets[0]]->print(map) << std::endl << "<SubTree ID='" << map[modelets[0]]->get_id()  << "' ";
  int count = 1;
  for (auto modelet : modelets) {
    tree << "port_" << count++ << "=\"{" << map[modelet]->get_id() << "_port}\" ";
  }

  //for (auto exert : exerts) {
  //  tree << map[exert]->get_id() << "_port='{" << map[exert]->get_id() << "_port}' ";
  //}
  tree << "/>" << std::endl;
  //std::cout << tree.str() << std::endl;
  //tree << subtree_port_name << "='{" << parent_tree_output_port_name << "}'" << subtree_output_port_name << "='{" << id << "}'/>\n";
  return tree.str();
}

std::string ExertnNode::get_id() {
  return "EXERT_" + name + "_" + id;
}
