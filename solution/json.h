#pragma once

#include <iostream>
#include <map>
#include <string>
#include <variant>
#include <vector>

namespace Json {

  class Node : public std::variant<std::vector<Node>,
                            std::map<std::string, Node>,
                            int32_t,
                            double,
                            bool,
                            std::string> {
  public:
    using variant::variant;

    const std::vector<Node>& AsArray() const;
    const std::map<std::string, Node>& AsMap() const;
    int AsInt() const;
    double AsDouble() const;
    bool AsBool() const;
    const std::string& AsString() const;
  };

  class Document {
  public:
    explicit Document(Node root);

    const Node& GetRoot() const;

  private:
    Node root;
  };

  Document Load(std::istream& input);

  std::ostream& Print(std::ostream& output, const Document& document);
}
