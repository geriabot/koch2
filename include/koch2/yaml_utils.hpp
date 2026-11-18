#ifndef KOCH2__YAML_UTILS_HPP_
#define KOCH2__YAML_UTILS_HPP_

#include <string>
#include <vector>

namespace koch2 {

std::string append_yaml_multiline_block(const std::string& base, const std::string& block_name, const std::string& block_content);
std::string append_yaml_string_list(const std::string & base, const std::string & key, const std::vector<std::string> & values);

} // namespace koch2

#endif // KOCH2__YAML_UTILS_HPP_
