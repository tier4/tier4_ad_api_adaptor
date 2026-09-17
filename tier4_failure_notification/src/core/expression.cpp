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

#include <cctype>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::failure_notification
{

enum class TokenType { kText, kOpen, kClose, kComma, kEnd, kBegin };

struct Token
{
  TokenType type;
  std::string text;
};

std::pair<size_t, Expression> parse_expr(const std::vector<Token> & tokens, size_t index);
std::pair<size_t, std::vector<Expression>> parse_args(
  const std::vector<Token> & tokens, size_t index);

std::pair<size_t, Token> next_token(const std::string & str, size_t index)
{
  // Skip whitespace.
  while (index < str.size() && std::isspace(str.at(index))) {
    ++index;
  }
  if (index >= str.size()) {
    return {index, Token{TokenType::kEnd, "EOF"}};
  }

  // Handle identifier.
  const auto is_character = [](char ch) {
    return std::isalnum(static_cast<unsigned char>(ch)) || ch == '_';
  };
  if (is_character(str.at(index))) {
    size_t start = index;
    while (index < str.size() && is_character(str.at(index))) {
      ++index;
    }
    return {index, Token{TokenType::kText, str.substr(start, index - start)}};
  }

  // Handle delimiters.
  switch (str.at(index)) {
    case '(':
      return {index + 1, Token{TokenType::kOpen, "("}};
    case ')':
      return {index + 1, Token{TokenType::kClose, ")"}};
    case ',':
      return {index + 1, Token{TokenType::kComma, ","}};
    default:
      throw std::runtime_error("unexpected character: " + str.substr(index));
  }
}

std::vector<Token> tokenize(const std::string & str)
{
  std::vector<Token> tokens;
  size_t index = 0;
  TokenType type = TokenType::kBegin;

  while (type != TokenType::kEnd) {
    const auto [next, token] = next_token(str, index);
    index = next;
    tokens.push_back(token);
    type = token.type;
  }
  return tokens;
}

// args ::= expr ( ',' expr )*
std::pair<size_t, std::vector<Expression>> parse_args(
  const std::vector<Token> & tokens, size_t index)
{
  std::vector<Expression> args;
  {
    const auto result = parse_expr(tokens, index);
    index = result.first;
    args.push_back(result.second);
  }
  while (tokens.at(index).type == TokenType::kComma) {
    ++index;  // Skip comma token.
    const auto result = parse_expr(tokens, index);
    index = result.first;
    args.push_back(result.second);
  }
  return {index, args};
}

// expr = NAME | NAME() | NAME(args)
std::pair<size_t, Expression> parse_expr(const std::vector<Token> & tokens, size_t index)
{
  const auto & curr = tokens.at(index);
  if (curr.type != TokenType::kText) {
    throw std::runtime_error("expected text token: " + curr.text);
  }
  ++index;  // Skip text token.

  if (tokens.at(index).type != TokenType::kOpen) {
    return {index, Expression{curr.text, std::nullopt}};
  }
  ++index;  // Skip open token.

  std::vector<Expression> args;
  if (tokens.at(index).type != TokenType::kClose) {
    const auto result = parse_args(tokens, index);
    index = result.first;
    args = result.second;
  }
  if (tokens.at(index).type != TokenType::kClose) {
    throw std::runtime_error("expected close token: " + tokens.at(index).text);
  }
  ++index;  // Skip close token.

  return {index, Expression{curr.text, args}};
}

Expression Expression::parse(const std::string & str)
{
  const auto tokens = tokenize(str);
  const auto result = parse_expr(tokens, 0);
  if (tokens.at(result.first).type != TokenType::kEnd) {
    throw std::runtime_error("extra tokens");
  }
  return result.second;
}

}  // namespace autoware::failure_notification
