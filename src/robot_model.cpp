#include "robot_model.hpp"
#include "pugixml.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <string_view>
#include <unordered_set>
#include <utility>

namespace robot {
namespace {

bool xml_space(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

bool skip_document_misc(std::string_view text, std::size_t& i) {
    while (i < text.size()) {
        if (xml_space(text[i])) {
            ++i;
        } else if (text.substr(i, 4) == "<!--") {
            std::size_t end = text.find("-->", i + 4);
            if (end == std::string_view::npos) return false;
            i = end + 3;
        } else if (text.substr(i, 2) == "<?") {
            std::size_t end = text.find("?>", i + 2);
            if (end == std::string_view::npos) return false;
            i = end + 2;
        } else {
            break;
        }
    }
    return true;
}

bool skip_markup_tag(std::string_view text, std::size_t& i, bool& self_closing) {
    char quote = 0;
    for (std::size_t j = i + 1; j < text.size(); ++j) {
        char c = text[j];
        if (quote) {
            if (c == quote) quote = 0;
        } else if (c == '\'' || c == '"') {
            quote = c;
        } else if (c == '>') {
            std::size_t k = j;
            while (k > i && xml_space(text[k - 1])) --k;
            self_closing = k > i && text[k - 1] == '/';
            i = j + 1;
            return true;
        }
    }
    return false;
}

bool has_one_document_element(std::string_view text) {
    std::size_t i = 0;
    if (!skip_document_misc(text, i) || i == text.size() || text[i] != '<' ||
        i + 1 == text.size() || text[i + 1] == '!' || text[i + 1] == '?' ||
        text[i + 1] == '/') return false;

    int depth = 0;
    while (i < text.size()) {
        if (text[i] != '<') {
            ++i;
            continue;
        }
        if (text.substr(i, 4) == "<!--") {
            std::size_t end = text.find("-->", i + 4);
            if (end == std::string_view::npos) return false;
            i = end + 3;
            continue;
        }
        if (text.substr(i, 9) == "<![CDATA[") {
            std::size_t end = text.find("]]>", i + 9);
            if (end == std::string_view::npos) return false;
            i = end + 3;
            continue;
        }
        if (text.substr(i, 2) == "<?") {
            std::size_t end = text.find("?>", i + 2);
            if (end == std::string_view::npos) return false;
            i = end + 2;
            continue;
        }
        if (text.substr(i, 2) == "<!") return false;
        bool closing = text.substr(i, 2) == "</";
        bool self_closing = false;
        if (!skip_markup_tag(text, i, self_closing)) return false;
        if (closing) --depth;
        else if (!self_closing) ++depth;
        if (depth < 0) return false;
        if (depth == 0) {
            return skip_document_misc(text, i) && i == text.size();
        }
    }
    return false;
}

std::string_view trim(std::string_view text) {
    while (!text.empty() && xml_space(text.front())) text.remove_prefix(1);
    while (!text.empty() && xml_space(text.back())) text.remove_suffix(1);
    return text;
}

bool decimal(std::string_view text, double& value) {
    text = trim(text);
    if (text.empty()) return false;
    std::size_t i = 0;
    if (text[i] == '+' || text[i] == '-') ++i;
    std::size_t digits_before = 0;
    while (i < text.size() && text[i] >= '0' && text[i] <= '9') {
        ++digits_before;
        ++i;
    }
    std::size_t digits_after = 0;
    if (i < text.size() && text[i] == '.') {
        ++i;
        while (i < text.size() && text[i] >= '0' && text[i] <= '9') {
            ++digits_after;
            ++i;
        }
    }
    if (digits_before == 0 && digits_after == 0) return false;
    if (i < text.size() && (text[i] == 'e' || text[i] == 'E')) {
        ++i;
        if (i < text.size() && (text[i] == '+' || text[i] == '-')) ++i;
        std::size_t exponent_digits = 0;
        while (i < text.size() && text[i] >= '0' && text[i] <= '9') {
            ++exponent_digits;
            ++i;
        }
        if (exponent_digits == 0) return false;
    }
    if (i != text.size()) return false;
    std::string copy(text);
    char* end = nullptr;
    errno = 0;
    value = std::strtod(copy.c_str(), &end);
    return end == copy.c_str() + copy.size() && std::isfinite(value);
}

bool vector3(std::string_view text, Vector3& out) {
    std::size_t i = 0;
    for (double& number : out) {
        while (i < text.size() && xml_space(text[i])) ++i;
        if (i == text.size()) return false;
        std::size_t start = i;
        while (i < text.size() && !xml_space(text[i])) ++i;
        if (!decimal(text.substr(start, i - start), number)) return false;
    }
    while (i < text.size() && xml_space(text[i])) ++i;
    return i == text.size();
}

bool read_vector(pugi::xml_node node, const char* attribute, Vector3& out, std::string& error) {
    pugi::xml_attribute value = node.attribute(attribute);
    if (!value) return true;
    if (vector3(value.value(), out)) return true;
    error = std::string("invalid ") + node.name() + " " + attribute + " vector";
    return false;
}

bool read_scalar(pugi::xml_node node, const char* attribute, double& out, std::string& error) {
    pugi::xml_attribute value = node.attribute(attribute);
    if (!value) {
        error = std::string("missing ") + node.name() + " " + attribute;
        return false;
    }
    if (decimal(value.value(), out)) return true;
    error = std::string("invalid ") + node.name() + " " + attribute + " number";
    return false;
}

bool read_origin(pugi::xml_node owner, Origin& out, std::string& error) {
    pugi::xml_node origin = owner.child("origin");
    return !origin || (read_vector(origin, "xyz", out.xyz, error) &&
                       read_vector(origin, "rpy", out.rpy, error));
}

bool read_visual(pugi::xml_node node, Visual& out, std::string& error) {
    if (!read_origin(node, out.origin, error)) return false;
    pugi::xml_node geometry = node.child("geometry");
    if (!geometry) {
        error = "visual has no geometry";
        return false;
    }
    pugi::xml_node shape;
    for (pugi::xml_node child : geometry.children()) {
        if (child.type() == pugi::node_element) {
            shape = child;
            break;
        }
    }
    if (!shape) {
        error = "visual has empty geometry";
        return false;
    }
    std::string_view kind = shape.name();
    if (kind == "box") {
        out.type = GeometryType::Box;
        if (!shape.attribute("size")) {
            error = "box is missing size";
            return false;
        }
        return read_vector(shape, "size", out.box_size, error);
    }
    if (kind == "cylinder") {
        out.type = GeometryType::Cylinder;
        return read_scalar(shape, "radius", out.radius, error) &&
               read_scalar(shape, "length", out.length, error);
    }
    if (kind == "sphere") {
        out.type = GeometryType::Sphere;
        return read_scalar(shape, "radius", out.radius, error);
    }
    if (kind == "mesh") {
        out.type = GeometryType::Mesh;
        pugi::xml_attribute filename = shape.attribute("filename");
        if (!filename) {
            error = "mesh is missing filename";
            return false;
        }
        out.mesh_filename = filename.value();
        return read_vector(shape, "scale", out.mesh_scale, error);
    }
    error = "unsupported visual geometry";
    return false;
}

bool read_joint(pugi::xml_node node, Joint& out, std::string& error) {
    pugi::xml_attribute name = node.attribute("name");
    pugi::xml_attribute type = node.attribute("type");
    if (!name || !type) {
        error = "joint is missing name or type";
        return false;
    }
    out.name = name.value();
    std::string_view kind = type.value();
    if (kind == "revolute") out.type = JointType::Revolute;
    else if (kind == "continuous") out.type = JointType::Continuous;
    else if (kind == "prismatic") out.type = JointType::Prismatic;
    else if (kind == "fixed") out.type = JointType::Fixed;
    else {
        error = "unsupported joint type: " + std::string(kind);
        return false;
    }

    pugi::xml_node parent = node.child("parent");
    pugi::xml_node child = node.child("child");
    if (!parent || !child || !parent.attribute("link") || !child.attribute("link")) {
        error = "joint is missing parent or child link";
        return false;
    }
    out.parent_link = parent.attribute("link").value();
    out.child_link = child.attribute("link").value();
    if (!read_origin(node, out.origin, error)) return false;
    pugi::xml_node axis = node.child("axis");
    if (axis && !read_vector(axis, "xyz", out.axis, error)) return false;

    pugi::xml_node limit = node.child("limit");
    if (out.type == JointType::Revolute || out.type == JointType::Prismatic) {
        if (!limit) {
            error = "movable joint is missing limit";
            return false;
        }
        out.has_limit = true;
        if (limit.attribute("lower") &&
            !read_scalar(limit, "lower", out.lower_limit, error)) return false;
        if (limit.attribute("upper") &&
            !read_scalar(limit, "upper", out.upper_limit, error)) return false;
    }
    return true;
}

} // namespace

ParseResult parse_urdf(const std::string& description) {
    if (trim(description).empty()) return {std::nullopt, "empty robot description"};
    if (!has_one_document_element(description)) {
        return {std::nullopt, "description must contain one XML root element"};
    }

    pugi::xml_document document;
    pugi::xml_parse_result xml = document.load_buffer(
        description.data(), description.size(), pugi::parse_default, pugi::encoding_utf8);
    if (!xml) {
        return {std::nullopt, std::string("invalid XML: ") + xml.description()};
    }
    pugi::xml_node root = document.document_element();
    if (!root || std::string_view(root.name()) != "robot") {
        return {std::nullopt, "root element must be robot"};
    }

    RobotModel model;
    if (root.attribute("name")) model.name = root.attribute("name").value();
    std::unordered_set<std::string> joint_names;
    for (pugi::xml_node node : root.children()) {
        if (node.type() != pugi::node_element) continue;
        std::string_view kind = node.name();
        if (kind == "link") {
            pugi::xml_attribute name = node.attribute("name");
            if (!name) return {std::nullopt, "link is missing name"};
            Link link;
            link.name = name.value();
            if (trim(link.name).empty()) return {std::nullopt, "link name is empty"};
            if (!model.link_index.emplace(link.name, model.links.size()).second) {
                return {std::nullopt, "duplicate link name"};
            }
            pugi::xml_node visual = node.child("visual");
            if (visual) {
                Visual value;
                std::string error;
                if (!read_visual(visual, value, error)) return {std::nullopt, error};
                link.visual = std::move(value);
            }
            model.links.push_back(std::move(link));
        } else if (kind == "joint") {
            Joint joint;
            std::string error;
            if (!read_joint(node, joint, error)) return {std::nullopt, error};
            if (trim(joint.name).empty() || !joint_names.insert(joint.name).second) {
                return {std::nullopt, "empty or duplicate joint name"};
            }
            model.joints.push_back(std::move(joint));
        }
    }

    if (model.links.empty()) return {std::nullopt, "robot has no direct-child links"};
    model.child_joints.resize(model.links.size());
    std::vector<bool> has_parent(model.links.size(), false);
    for (std::size_t i = 0; i < model.joints.size(); ++i) {
        Joint& joint = model.joints[i];
        auto parent = model.link_index.find(joint.parent_link);
        auto child = model.link_index.find(joint.child_link);
        if (parent == model.link_index.end() || child == model.link_index.end()) {
            return {std::nullopt, "joint references an unknown link"};
        }
        joint.parent_index = parent->second;
        joint.child_index = child->second;
        if (joint.parent_index == joint.child_index || has_parent[joint.child_index]) {
            return {std::nullopt, "joint tree has a self-edge or multiple parents"};
        }
        has_parent[joint.child_index] = true;
        model.child_joints[joint.parent_index].push_back(i);
    }

    std::size_t roots = 0;
    for (std::size_t i = 0; i < has_parent.size(); ++i) {
        if (!has_parent[i]) {
            model.root_index = i;
            ++roots;
        }
    }
    if (roots != 1) return {std::nullopt, "robot must have exactly one root link"};
    model.root_link = model.links[model.root_index].name;

    std::vector<bool> reached(model.links.size(), false);
    std::vector<std::size_t> pending{model.root_index};
    std::size_t count = 0;
    while (!pending.empty()) {
        std::size_t link = pending.back();
        pending.pop_back();
        if (reached[link]) return {std::nullopt, "joint tree contains a cycle"};
        reached[link] = true;
        ++count;
        for (std::size_t joint : model.child_joints[link]) {
            pending.push_back(model.joints[joint].child_index);
        }
    }
    if (count != model.links.size()) return {std::nullopt, "joint tree is disconnected"};
    return {std::move(model), ""};
}

} // namespace robot
