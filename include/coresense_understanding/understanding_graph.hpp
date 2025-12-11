#ifndef understanding_graph_hpp
#define understanding_graph_hpp

#include <string>
#include <unordered_map>
#include <memory>
#include <vector>
#include <regex>

#include "uuid/uuid.h"

namespace coresense::understanding::graph {

  // Classes
class GraphNode {
public:
  std::string id;
  std::string name;
  
  GraphNode();
  ~GraphNode() {};
  virtual std::string print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map)=0;
  virtual std::string get_id()=0;
};

class SubsetNode : public GraphNode {
public:
  std::string element;
  std::string set;
  
  SubsetNode(std::smatch match);
  ~SubsetNode() {};
  std::string print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map) override;
  std::string get_id() override;
};

class ExertNode : public GraphNode {
public:
  std::string modelet_set;
  
  ExertNode(std::smatch match);
  ~ExertNode() {};
  std::string print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map) override;
  std::string get_id() override;
};

class ConceptNode : public GraphNode {
public:
  ConceptNode(std::string modelet_name);
  ~ConceptNode() {};
  std::string print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map) override;
  std::string get_id() override;
};

class ExertnNode : public GraphNode {
public:
  std::string engine;
  std::vector<std::string> modelets;
  std::vector<std::string> exerts;
  
  ExertnNode(std::string engine_name);
  ~ExertnNode() {};
  std::string print(std::unordered_map<std::string, std::shared_ptr<GraphNode>> & map) override;
  std::string get_id() override;
  void add_node(std::shared_ptr<GraphNode> & node);
};

}
#endif
