#include "coresense_understanding/session.hpp"

namespace coresense::understanding::session {

Session::Session() {}
Session::Session(std::string &id): id(id), start_time(std::chrono::system_clock::now()) {}
}

