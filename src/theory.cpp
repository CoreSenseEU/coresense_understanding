#include <iostream>
#include <fstream>
#include <string>
#include <regex>

#include <coresense_understanding/theory.hpp>
#include <coresense_understanding/model.hpp>

using namespace coresense::understanding::model;

namespace coresense::understanding::theory {


TheoryReader::TheoryReader(std::filesystem::path path)
  : package_path { path } {
}



void TheoryReader::parse_tptp_string(Theory t, std::stringstream& string_handle) {
  std::string content = string_handle.str();
  std::string line;
  // TODO find these declarations and collect them
  // TODO find the respective distinct axioms and remove them
  // engines
  // concepts
  // requirements
  // properties
  // modelets
  // representation_class
  // datatypes
  // formalisms
  // locations
  // extents
  // template sets
  while (std::getline(string_handle, line, '\n') ) {
  //extract includes
    std::smatch include_match;
    if (std::regex_search(line, include_match, include_regex)) {
      // TODO SECONDARY PRIORITY: this implies includes live on single lines, rather than spread over multiple lines.
      for (std::size_t i = 1; i < include_match.size(); ++i) { // find includes
        // handle includes
        // TODO SECONDARY PRIORITY: this assumes all the includes are magically also being added to the pile of theories...but there is no guarantee of that.
        // we need to parse them and make sure they are read
        // TODO SECONDARY PRIORITY includes are relative to some TPTP_HOME. how can we make sure it's the same home when they come from different packages?
        // TODO SECONDARY PRIORITY the include path strings are likely used as ids, to find corresponding theories, make sure this correspondence still holds (it likely does not)
        t.includes.push_back(include_match[i]);
      } 
    } else {
      //add line to cleaned string
      t.content += line + '\n';
    }
  } // TODO: SECONDARY PRIORITY t.content now still potentially includes queries and conjectures
  // idea: use regex match-ranges to remove the offending parts
  std::smatch query_match;
  if (std::regex_search(content, query_match, query_regex)) { // find queries
    for (std::size_t i = 0; i < query_match.size(); ++i) {
      std::cout << "found query:" << std::endl << query_match[i] << std::endl;
      t.queries.push_back(query_match[i]);
    }
  }
  theories[t.id.string()] = t;
}


Theory TheoryReader::import_tptp_file(std::filesystem::path path) {
  std::ifstream file_handle;
  std::stringstream string_handle;
  file_handle.open(path);
  string_handle << file_handle.rdbuf();
  Theory t = get_theory1(path);
  parse_tptp_string(t, string_handle);
  file_handle.close();
  return t;
}

void TheoryReader::read_package_logic(std::string package_path, std::string logic_directory) {
  // read tptp files and sort them by their includes
  std::filesystem::path path = std::filesystem::path(package_path) / std::filesystem::path(logic_directory);
  for (const auto & entry : std::filesystem::recursive_directory_iterator(path)) { // for files in path
    if (entry.is_regular_file()) {
      import_tptp_file(entry.path());
    }
  }
}


std::string TheoryReader::get_theories() {
  std::string body = "";
  for (auto& [key, t] : theories) {
    get_theory2(t, body);
  }
  return body;
}

Theory TheoryReader::get_theory1(std::filesystem::path path) {
  Theory t;
  // find or create theory
  auto it = theories.find(path.string());
  if (it == theories.end()) { // create a new theory
    t.id = path;
    theories[t.id.string()] = t;
  } else { // set current theory to found theory
    t = it->second;
  }
  return t;
}

void TheoryReader::get_theory2(Theory &t, std::string &body) {
  if (!t.added) {
    for (std::string key : t.includes) {
      std::filesystem::path file_path = package_path / std::filesystem::path(key);
      if (std::filesystem::exists(file_path)) {
        get_theory2(theories[file_path.string()], body);
      } else {
        //RCLCPP_WARN(get_logger(), "get_theory: Path does not exist.\n Missing: %s,\n Imported by: %s", file_path.string().c_str(), t.id.string().c_str());
      }
    }
    //RCLCPP_INFO(get_logger(), "get_theory: Adding Theory:\n%s", t.id.string().c_str());
    body += t.content;
    t.added = true;
  }
}
} // end namespace coresense::understanding::theory
