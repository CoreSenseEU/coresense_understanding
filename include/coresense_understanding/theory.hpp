#ifndef theory_hpp
#define theory_hpp

#include <map>
#include <string>
#include <vector>
#include <filesystem>

namespace coresense::understanding::theory {

class Theory {
public:
  std::filesystem::path id;
  std::string content{""};
  std::vector<std::string> includes{};
  std::vector<std::string> queries{};
  bool added{false};
};


class TheoryReader {
private:
  std::map<std::string, Theory> theories;
  const std::regex include_regex = std::regex("^include\\s*\\('(.*)'\\)\\.$");
  const std::regex query_regex = std::regex("tff\\s*\\([\\s\\w-]+\\, question\\s*\\,[^\\.]+\\)\\.");
  std::filesystem::path package_path;

  
  void parse_tptp_string(Theory t, std::stringstream& string_handle);
  Theory import_tptp_file(std::filesystem::path path);
  
  Theory get_theory1(std::filesystem::path path);
  void get_theory2(Theory &t, std::string &body);
  
public:
  TheoryReader(std::filesystem::path path);
  
  void read_package_logic(std::string package_path, std::string logic_directory);
  std::string get_theories();
};



} // namespace 'coresense::understanding::theory' ends
#endif
