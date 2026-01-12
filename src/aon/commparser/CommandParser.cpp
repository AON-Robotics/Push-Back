#include "CommandParser.hpp"
#include <cctype>
#include <string>

// trim in-place
static void trim_inplace(std::string& s) {
  size_t a = 0;
  while (a < s.size() && std::isspace((unsigned char)s[a])) a++;

  size_t b = s.size();
  while (b > a && std::isspace((unsigned char)s[b - 1])) b--;

  s = s.substr(a, b - a);
}

static bool parse_int(const std::string& s, int& out) {
  if (s.empty()) return false;

  size_t i = 0;
  int sign = 1;

  if (s[i] == '+' || s[i] == '-') {
    if (s[i] == '-') sign = -1;
    i++;
    if (i >= s.size()) return false;
  }

  if (!std::isdigit((unsigned char)s[i])) return false;

  long long v = 0;
  while (i < s.size() && std::isdigit((unsigned char)s[i])) {
    v = v * 10 + (s[i] - '0');
    i++;
  }

  while (i < s.size() && std::isspace((unsigned char)s[i])) i++;
  if (i != s.size()) return false;

  out = (int)(sign * v);
  return true;
}

Command parseCommandKV(std::string line) {
  Command cmd; // defaults 0

  // strip trailing \r\n
  while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();

  size_t start = 0;
  while (start < line.size()) {
    size_t end = line.find(';', start);
    if (end == std::string::npos) end = line.size();

    std::string token = line.substr(start, end - start);
    trim_inplace(token);

    if (!token.empty()) {
      size_t eq = token.find('=');
      if (eq != std::string::npos) {
        std::string key = token.substr(0, eq);
        std::string val = token.substr(eq + 1);
        trim_inplace(key);
        trim_inplace(val);

        int n = 0;
        if (parse_int(val, n)) {
          if (key == "Move") cmd.Move = (float)n;
          else if (key == "Turn") cmd.Turn = (float)n;
        }
      }
    }

    start = end + 1;
  }

  return cmd;
}
