// Copyright 2026 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "expression.hpp"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::failure_notification
{

std::vector<std::string> tokenize(const std::string & str)
{
  std::vector<std::string> tokens;
  std::string token;

  const auto flush_token = [&]() {
    if (!token.empty()) {
      tokens.push_back(token);
      token.clear();
    }
  };

  for (const char ch : str) {
    if (std::isalnum(ch)) {
      token += ch;
    } else if (ch == ' ') {
      flush_token();
    } else if (ch == '(' || ch == ')' || ch == ',') {
      flush_token();
      tokens.push_back(std::string(1, ch));
    } else {
      throw std::runtime_error("Invalid token character: " + std::string(1, ch));
    }
  }
  flush_token();
  return tokens;
}

bool check_index(const std::vector<std::string> & tokens, size_t index, const std::string & str)
{
  return index < tokens.size() && tokens.at(index) == str;
}

std::pair<size_t, Expression> parse_token(const std::vector<std::string> & tokens, size_t index)
{
  Expression expression;
  expression.data = tokens.at(index);
  index += 1;

  if (!check_index(tokens, index, "(")) {
    return {index, expression};
  }
  index += 1;

  expression.args = std::vector<Expression>();
  while (index < tokens.size()) {
    const auto result = parse_token(tokens, index);
    index = result.first;
    expression.args->push_back(result.second);

    if (check_index(tokens, index, ")")) {
      index += 1;
      break;
    }
    if (check_index(tokens, index, ",")) {
      index += 1;
      continue;
    }
    throw std::runtime_error("expect delimiter or arguments after " + tokens.at(index));
  }
  return {index, expression};
}

Expression Expression::parse(const std::string & str)
{
  const auto tokens = tokenize(str);
  const auto result = parse_token(tokens, 0);
  if (tokens.size() != result.first) {
    throw std::runtime_error("extra tokens");
  }
  return result.second;
}

}  // namespace autoware::failure_notification
