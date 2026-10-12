// Tiny example: reading a URDF with the vendored pugixml parser
// (third_party/pugixml, compiled in by the Makefile).
//
// This only shows the XML plumbing. Deciding what the elements *mean*, which
// descriptions are valid, and the forward kinematics are your job
// (project3/spec/ROBOT_DESCRIPTION.md).
//
//     make build && ./build/urdf_example some_robot.urdf
#include "pugixml.hpp"

#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: urdf_example FILE.urdf\n";
        return 2;
    }
    std::ifstream file(argv[1], std::ios::binary);
    std::stringstream contents;
    contents << file.rdbuf();
    const std::string text = contents.str();  // a description arrives as one string like this

    // parse_default decodes &amp; and character references and skips comments,
    // processing instructions, and a <!DOCTYPE robot>. A leading UTF-8
    // byte-order mark is also skipped. Be aware that pugixml is more lenient
    // than XML 1.0 in a few places (for example, it silently skips stray text
    // outside the root element).
    pugi::xml_document doc;
    const pugi::xml_parse_result result =
        doc.load_buffer(text.data(), text.size(), pugi::parse_default, pugi::encoding_utf8);
    if (!result) {
        std::cerr << "not well-formed XML: " << result.description() << " at offset " << result.offset << "\n";
        return 1;
    }
    const pugi::xml_node robot = doc.document_element();
    std::cout << "root element: " << robot.name() << " name: " << robot.attribute("name").value() << "\n";
    for (const pugi::xml_node child : robot.children()) {  // direct children only
        if (child.type() == pugi::node_element && std::strcmp(child.name(), "joint") == 0) {
            const pugi::xml_node origin = child.child("origin");  // child() also looks only at direct children
            std::cout << "joint " << child.attribute("name").value() << " " << child.attribute("type").value()
                      << " origin xyz: " << (origin.attribute("xyz") ? origin.attribute("xyz").value() : "(none)")
                      << "\n";
        }
    }
    return 0;
}
