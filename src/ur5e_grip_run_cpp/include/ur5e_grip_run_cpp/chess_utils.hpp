#pragma once

#include <string>
#include <cctype>
#include <stdexcept>

struct ChessMove
{
  std::string from;
  std::string to;
};

inline bool squareToRowCol(const std::string& square, int& row, int& col)
{
  if (square.size() != 2)
    return false;

  const char file = static_cast<char>(
    std::toupper(static_cast<unsigned char>(square[0])));

  const char rank = square[1];

  if (file < 'A' || file > 'H')
    return false;

  if (rank < '1' || rank > '8')
    return false;

  col = file - 'A';
  row = rank - '1';

  return true;
}

inline ChessMove parseMoveString(const std::string& raw_input)
{
  std::string s;
  s.reserve(raw_input.size());

  for (char c : raw_input)
  {
    if (!std::isspace(static_cast<unsigned char>(c)))
    {
      s.push_back(static_cast<char>(
        std::tolower(static_cast<unsigned char>(c))));
    }
  }

  /*
    Accepts:
    e2e4
    E2E4
    "e2e4"
    e2e4q also becomes e2 -> e4, promotion ignored for robot motion.
  */
  if (s.size() < 4)
    throw std::runtime_error("Invalid move format. Use format like e2e4.");

  ChessMove move;
  move.from = s.substr(0, 2);
  move.to = s.substr(2, 2);

  int row = 0;
  int col = 0;

  if (!squareToRowCol(move.from, row, col))
    throw std::runtime_error("Invalid source square: " + move.from);

  if (!squareToRowCol(move.to, row, col))
    throw std::runtime_error("Invalid target square: " + move.to);

  return move;
}