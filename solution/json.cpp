#include "json.h"

#include <iomanip>

using namespace std;

namespace Json {

  const std::vector<Node>& Node::AsArray() const {
    return std::get<std::vector<Node>>(*this);
  }
  const std::map<std::string, Node>& Node::AsMap() const {
    return std::get<std::map<std::string, Node>>(*this);
  }
  int32_t Node::AsInt() const {
    return std::get<int32_t>(*this);
  }
  double Node::AsDouble() const {
    return std::get<double>(*this);
  }
  bool Node::AsBool() const {
    return std::get<bool>(*this);
  }
  const std::string& Node::AsString() const {
    return std::get<std::string>(*this);
  }

  Document::Document(Node root) : root(move(root)) {
  }

  const Node& Document::GetRoot() const {
    return root;
  }

  Node LoadNode(istream& input);

  Node LoadArray(istream& input) {
    vector<Node> result;

    for (char c; input >> c && c != ']'; ) {
      if (c != ',') {
        input.putback(c);
      }
      result.push_back(LoadNode(input));
    }

    return Node(move(result));
  }

  Node LoadInt(istream& input) {
    int32_t result = 0;
    while (isdigit(input.peek())) {
      result *= 10;
      result += input.get() - '0';
    }
    return Node(result);
  }
  
  Node LoadDouble(istream& input) {
    double result = 0.0;
    input >> result;
    return Node(result);
  }
  
  Node LoadBool(istream& input) {
    bool result = false;
    input >> boolalpha >> result;
    return Node(result);
  }

  Node LoadString(istream& input) {
    string line;
    getline(input, line, '"');
    return Node(move(line));
  }

  Node LoadDict(istream& input) {
    map<string, Node> result;

    for (char c; input >> c && c != '}'; ) {
      if (c == ',') {
        input >> c;
      }

      string key = LoadString(input).AsString();
      input >> c;
      Node node = 0;
      if (key == "latitude"s || key == "longitude"s || key == "bus_velocity"s) {
        node = LoadDouble(input);
      }
      else {
        node = LoadNode(input);
      }
      result.emplace(move(key), std::move(node));
    }

    return Node(move(result));
  }

  Node LoadNode(istream& input) {
    char c;
    input >> c;

    if (c == '[') {
      return LoadArray(input);
    } else if (c == '{') {
      return LoadDict(input);
    } else if (c == '"') {
      return LoadString(input);
    } else if (c == 't' || c == 'f') {
      input.putback(c);
      return LoadBool(input);
    } else {
      input.putback(c);
      return LoadInt(input);
    }
  }

  Document Load(istream& input) {
    return Document{LoadNode(input)};
  }

  ostream& PrintNode(ostream& output, const Node& node, size_t tab_count = 0, bool first_tabs = true);

  ostream& Print(ostream& output, const Document& document) {
    return PrintNode(output, document.GetRoot());
  }

  void PrintTabs(ostream& output, size_t tab_count) {
    for (size_t i = 0; i < tab_count; ++i) {
      output << ' ';
    }
  }

  void PrintString(ostream& output, const string& node, size_t tab_count, bool first_tabs = true) {
    if (first_tabs) {
      PrintTabs(output, tab_count);
    }

    output << '\"' << node << '\"';
  }

  void PrintArray(ostream& output, const vector<Node>& node, size_t tab_count, bool first_tabs = true) {
    if (first_tabs) {
      PrintTabs(output, tab_count);
    }
    output << '[';

    bool first = true;
    for (const auto& n : node) {
      if (!first) {
        output << ',';
      }
      output << '\n';
      first = false;

      PrintNode(output, n, tab_count + 1);
    }
    if (!node.empty()) {
      output << '\n';
      PrintTabs(output, tab_count);
    }
    output << ']';
  }
  
  void PrintMap(ostream& output, const map<string, Node>& node, size_t tab_count, bool first_tabs = true) {
    if (first_tabs) {
      PrintTabs(output, tab_count);
    }
    output << '{';

    bool first = true;
    for (const auto& [key, node] : node) {
      if (!first) {
        output << ',';
      }
      output << '\n';
      first = false;

      PrintString(output, key, tab_count + 1);
      output << ": ";
      PrintNode(output, node, tab_count + 2, false);
    }
    if (!node.empty()) {
      output << '\n';
      PrintTabs(output, tab_count);
    }
    output << '}';
  }

  void PrintInt(ostream& output, const int node, size_t tab_count, bool first_tabs = true) {
    if (first_tabs) {
      PrintTabs(output, tab_count);
    }

    output << node;
  }

  void PrintDouble(ostream& output, const double node, size_t tab_count, bool first_tabs = true) {
    if (first_tabs) {
      PrintTabs(output, tab_count);
    }

    output << setprecision(6) << node;
  }

  void PrintBool(ostream& output, const bool node, size_t tab_count, bool first_tabs = true) {
    if (first_tabs) {
      PrintTabs(output, tab_count);
    }

    output << boolalpha << node;
  }

  ostream& PrintNode(ostream& output, const Node& node, size_t tab_count, bool first_tabs) {
    if (holds_alternative<vector<Node>>(node)) {
      PrintArray(output, node.AsArray(), tab_count, first_tabs);
    }

    else if (holds_alternative<map<string, Node>>(node)) {
      PrintMap(output, node.AsMap(), tab_count, first_tabs);
    }
    
    else if (holds_alternative<int>(node)) {
      PrintInt(output, node.AsInt(), tab_count, first_tabs);
    }
    
    else if (holds_alternative<double>(node)) {
      PrintDouble(output, node.AsDouble(), tab_count, first_tabs);
    }
    
    else if (holds_alternative<bool>(node)) {
      PrintBool(output, node.AsBool(), tab_count, first_tabs);
    }
    
    else if (holds_alternative<string>(node)) {
      PrintString(output, node.AsString(), tab_count, first_tabs);
    }

    return output;
  }

}
