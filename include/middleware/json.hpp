#ifndef MIDDLEWARE_JSON_HPP
#define MIDDLEWARE_JSON_HPP

#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <stdexcept>
#include <cctype>
#include <iomanip>
#include <cmath>
#include <limits>

namespace middleware {

enum class JsonType {
    Null,
    Boolean,
    Number,
    String,
    Array,
    Object
};

class JsonValue {
public:
    JsonType type{JsonType::Null};
    bool bool_val{false};
    double num_val{0.0};
    std::string str_val;
    std::vector<JsonValue> arr_val;
    std::map<std::string, JsonValue> obj_val;

    /*
    1. Initialize JsonValue instance with type Null.
    2. Set default values for primitive fields.
    */
    JsonValue() : type(JsonType::Null) {
        // Associated with Step 1 & 2: Set type to Null and initialize fields
    }

    /*
    1. Initialize JsonValue instance with type Boolean.
    2. Assign the boolean parameter to the internal boolean field.
    */
    JsonValue(bool b) : type(JsonType::Boolean), bool_val(b) {
        // Associated with Step 1 & 2: Set type to Boolean and store value
    }

    /*
    1. Initialize JsonValue instance with type Number.
    2. Store the numeric double parameter.
    */
    JsonValue(double d) : type(JsonType::Number), num_val(d) {
        // Associated with Step 1 & 2: Set type to Number and store value
    }

    /*
    1. Initialize JsonValue instance with type Number.
    2. Convert integer to double and store.
    */
    JsonValue(int i) : type(JsonType::Number), num_val(static_cast<double>(i)) {
        // Associated with Step 1 & 2: Convert integer to double and store as Number
    }

    /*
    1. Initialize JsonValue instance with type Number.
    2. Convert size_t to double and store.
    */
    JsonValue(size_t s) : type(JsonType::Number), num_val(static_cast<double>(s)) {
        // Associated with Step 1 & 2: Convert size_t to double and store as Number
    }

    /*
    1. Initialize JsonValue instance with type String.
    2. Assign the string parameter to the internal string field.
    */
    JsonValue(const std::string& s) : type(JsonType::String), str_val(s) {
        // Associated with Step 1 & 2: Set type to String and store value
    }

    /*
    1. Initialize JsonValue instance with type String.
    2. Construct string from C-style string and store.
    */
    JsonValue(const char* s) : type(JsonType::String), str_val(s ? s : "") {
        // Associated with Step 1 & 2: Set type to String and store C-string
    }

    /*
    1. Initialize JsonValue instance with type Array.
    2. Store the provided vector of JsonValues.
    */
    JsonValue(const std::vector<JsonValue>& a) : type(JsonType::Array), arr_val(a) {
        // Associated with Step 1 & 2: Set type to Array and store elements
    }

    /*
    1. Initialize JsonValue instance with type Object.
    2. Store the provided map of key-value pairs.
    */
    JsonValue(const std::map<std::string, JsonValue>& o) : type(JsonType::Object), obj_val(o) {
        // Associated with Step 1 & 2: Set type to Object and store key-value pairs
    }

    /*
    1. Create and return an empty JsonValue of type Array.
    */
    static JsonValue make_array() {
        // Associated with Step 1: Create JsonValue and set type to Array
        JsonValue v;
        v.type = JsonType::Array;
        return v;
    }

    /*
    1. Create and return an empty JsonValue of type Object.
    */
    static JsonValue make_object() {
        // Associated with Step 1: Create JsonValue and set type to Object
        JsonValue v;
        v.type = JsonType::Object;
        return v;
    }

    /*
    1. Query whether the current value is Null.
    */
    bool is_null() const noexcept {
        // Associated with Step 1: Return true if type is Null
        return type == JsonType::Null;
    }

    /*
    1. Query whether the current value is Boolean.
    */
    bool is_bool() const noexcept {
        // Associated with Step 1: Return true if type is Boolean
        return type == JsonType::Boolean;
    }

    /*
    1. Query whether the current value is Number.
    */
    bool is_number() const noexcept {
        // Associated with Step 1: Return true if type is Number
        return type == JsonType::Number;
    }

    /*
    1. Query whether the current value is String.
    */
    bool is_string() const noexcept {
        // Associated with Step 1: Return true if type is String
        return type == JsonType::String;
    }

    /*
    1. Query whether the current value is Array.
    */
    bool is_array() const noexcept {
        // Associated with Step 1: Return true if type is Array
        return type == JsonType::Array;
    }

    /*
    1. Query whether the current value is Object.
    */
    bool is_object() const noexcept {
        // Associated with Step 1: Return true if type is Object
        return type == JsonType::Object;
    }

    /*
    1. Retrieve the boolean value if type is Boolean.
    2. Throw std::runtime_error if type is not Boolean.
    */
    bool as_bool() const {
        // Associated with Step 2: Validate type
        if (type != JsonType::Boolean) {
            throw std::runtime_error("JsonValue is not a boolean");
        }
        // Associated with Step 1: Return boolean value
        return bool_val;
    }

    /*
    1. Retrieve the numeric value as double if type is Number.
    2. Throw std::runtime_error if type is not Number.
    */
    double as_number() const {
        // Associated with Step 2: Validate type
        if (type != JsonType::Number) {
            throw std::runtime_error("JsonValue is not a number");
        }
        // Associated with Step 1: Return number value
        return num_val;
    }

    /*
    1. Retrieve the numeric value cast to int if type is Number.
    2. Throw std::runtime_error if type is not Number.
    */
    int as_int() const {
        // Associated with Step 2: Validate type
        if (type != JsonType::Number) {
            throw std::runtime_error("JsonValue is not a number");
        }
        // Associated with Step 1: Return int value
        return static_cast<int>(num_val);
    }

    /*
    1. Retrieve the string value if type is String.
    2. Throw std::runtime_error if type is not String.
    */
    const std::string& as_string() const {
        // Associated with Step 2: Validate type
        if (type != JsonType::String) {
            throw std::runtime_error("JsonValue is not a string");
        }
        // Associated with Step 1: Return string reference
        return str_val;
    }

    /*
    1. Retrieve the array elements vector if type is Array.
    2. Throw std::runtime_error if type is not Array.
    */
    const std::vector<JsonValue>& as_array() const {
        // Associated with Step 2: Validate type
        if (type != JsonType::Array) {
            throw std::runtime_error("JsonValue is not an array");
        }
        // Associated with Step 1: Return array vector reference
        return arr_val;
    }

    /*
    1. Retrieve mutable array elements vector if type is Array.
    2. Throw std::runtime_error if type is not Array.
    */
    std::vector<JsonValue>& as_array() {
        // Associated with Step 2: Validate type
        if (type != JsonType::Array) {
            throw std::runtime_error("JsonValue is not an array");
        }
        // Associated with Step 1: Return mutable array vector reference
        return arr_val;
    }

    /*
    1. Retrieve the object map if type is Object.
    2. Throw std::runtime_error if type is not Object.
    */
    const std::map<std::string, JsonValue>& as_object() const {
        // Associated with Step 2: Validate type
        if (type != JsonType::Object) {
            throw std::runtime_error("JsonValue is not an object");
        }
        // Associated with Step 1: Return object map reference
        return obj_val;
    }

    /*
    1. Retrieve mutable object map if type is Object.
    2. Throw std::runtime_error if type is not Object.
    */
    std::map<std::string, JsonValue>& as_object() {
        // Associated with Step 2: Validate type
        if (type != JsonType::Object) {
            throw std::runtime_error("JsonValue is not an object");
        }
        // Associated with Step 1: Return mutable object map reference
        return obj_val;
    }

    /*
    1. Access or insert element by key in an Object JsonValue.
    2. Ensure type is Object, changing to Object if previously Null.
    3. Return reference to the value associated with the key.
    */
    JsonValue& operator[](const std::string& key) {
        // Associated with Step 2: Auto-convert Null to Object
        if (type == JsonType::Null) {
            type = JsonType::Object;
        }
        // Associated with Step 1 & 3: Access map entry by key and return reference
        return obj_val[key];
    }

    /*
    1. Access element by key in a const Object JsonValue.
    2. Throw std::out_of_range if key is not found.
    */
    const JsonValue& operator[](const std::string& key) const {
        // Associated with Step 1: Look up key in map
        auto it = obj_val.find(key);
        // Associated with Step 2: Check existence
        if (it == obj_val.end()) {
            throw std::out_of_range("Key not found in Json object: " + key);
        }
        return it->second;
    }

    /*
    1. Check if the given key exists in an Object JsonValue.
    2. Return true if key exists, false otherwise.
    */
    bool contains(const std::string& key) const noexcept {
        // Associated with Step 1 & 2: Query object map for key presence
        if (type != JsonType::Object) return false;
        return obj_val.find(key) != obj_val.end();
    }

    /*
    1. Append a value to an Array JsonValue.
    2. Ensure type is Array, converting Null to Array if needed.
    */
    void push_back(const JsonValue& val) {
        // Associated with Step 2: Convert Null to Array if needed
        if (type == JsonType::Null) {
            type = JsonType::Array;
        }
        // Associated with Step 1: Append value to vector
        arr_val.push_back(val);
    }

    /*
    1. Serialize the JsonValue to a valid JSON string.
    2. Handle proper escaping for strings and formatting for numbers, arrays, and objects.
    3. Return the serialized JSON string.
    */
    std::string dump() const {
        std::ostringstream ss;
        // Associated with Step 1 & 2: Format based on JsonType
        switch (type) {
            case JsonType::Null:
                ss << "null";
                break;
            case JsonType::Boolean:
                ss << (bool_val ? "true" : "false");
                break;
            case JsonType::Number: {
                // *CHANGES*: Avoid undefined integer casts and invalid JSON for nonfinite values.
                if (!std::isfinite(num_val)) {
                    throw std::runtime_error("Cannot serialize a nonfinite JSON number");
                }
                if (num_val >= static_cast<double>(std::numeric_limits<long long>::min()) &&
                    num_val < static_cast<double>(std::numeric_limits<long long>::max()) &&
                    num_val == static_cast<double>(static_cast<long long>(num_val))) {
                    ss << static_cast<long long>(num_val);
                } else {
                    ss << std::setprecision(std::numeric_limits<double>::max_digits10) << num_val;
                }
                break;
            }
            case JsonType::String:
                escape_string(ss, str_val);
                break;
            case JsonType::Array: {
                ss << "[";
                for (size_t i = 0; i < arr_val.size(); ++i) {
                    if (i > 0) ss << ",";
                    ss << arr_val[i].dump();
                }
                ss << "]";
                break;
            }
            case JsonType::Object: {
                ss << "{";
                size_t count = 0;
                for (const auto& pair : obj_val) {
                    if (count > 0) ss << ",";
                    escape_string(ss, pair.first);
                    ss << ":" << pair.second.dump();
                    count++;
                }
                ss << "}";
                break;
            }
        }
        // Associated with Step 3: Return formatted JSON string
        return ss.str();
    }

    /*
    1. Parse a JSON string into a JsonValue object.
    2. Skip leading whitespace and invoke recursive descent parsing.
    3. Verify all input has been consumed.
    4. Return the parsed JsonValue.
    */
    static JsonValue parse(const std::string& input) {
        size_t index = 0;
        // Associated with Step 2: Parse value starting at index 0
        JsonValue val = parse_value(input, index);
        skip_whitespace(input, index);
        // Associated with Step 3: Check that trailing characters are consumed
        if (index < input.size()) {
            throw std::runtime_error("Unexpected trailing characters after JSON value");
        }
        // Associated with Step 4: Return parsed value
        return val;
    }

private:
    /*
    1. Advance index past any whitespace characters (' ', '\t', '\n', '\r').
    */
    static void skip_whitespace(const std::string& s, size_t& i) noexcept {
        // Associated with Step 1: Skip whitespace characters
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) {
            ++i;
        }
    }

    /*
    1. Output properly escaped string with surrounding double quotes to stream.
    */
    static void escape_string(std::ostream& os, const std::string& s) {
        os << '"';
        for (char c : s) {
            switch (c) {
                case '"':  os << "\\\""; break;
                case '\\': os << "\\\\"; break;
                case '\b': os << "\\b"; break;
                case '\f': os << "\\f"; break;
                case '\n': os << "\\n"; break;
                case '\r': os << "\\r"; break;
                case '\t': os << "\\t"; break;
                default:
                    if (static_cast<unsigned char>(c) < 0x20) {
                        os << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                           << static_cast<int>(static_cast<unsigned char>(c)) << std::dec;
                    } else {
                        os << c;
                    }
                    break;
            }
        }
        os << '"';
    }

    /*
    1. Determine the type of the next JSON token by inspecting the current character.
    2. Delegate parsing to the appropriate helper function (string, number, object, array, bool, null).
    3. Return the parsed JsonValue.
    */
    static JsonValue parse_value(const std::string& s, size_t& i) {
        skip_whitespace(s, i);
        if (i >= s.size()) {
            throw std::runtime_error("Unexpected end of input while parsing JSON");
        }
        char c = s[i];
        // Associated with Step 1 & 2: Delegate based on character
        if (c == '"') {
            return JsonValue(parse_string(s, i));
        } else if (c == '{') {
            return parse_object(s, i);
        } else if (c == '[') {
            return parse_array(s, i);
        } else if (c == 't' || c == 'f') {
            return parse_bool(s, i);
        } else if (c == 'n') {
            return parse_null(s, i);
        } else if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
            return parse_number(s, i);
        }
        throw std::runtime_error(std::string("Unexpected character in JSON: '") + c + "'");
    }

    /*
    1. Expect opening quote and parse characters until matching unescaped closing quote.
    2. Handle escape sequences (\\, \", \n, \r, \t, etc.).
    3. Return the decoded string.
    */
    static std::string parse_string(const std::string& s, size_t& i) {
        if (i >= s.size() || s[i] != '"') {
            throw std::runtime_error("Expected '\"' at start of string");
        }
        ++i; // skip opening quote
        std::string res;
        while (i < s.size() && s[i] != '"') {
            char c = s[i++];
            if (c == '\\') {
                if (i >= s.size()) throw std::runtime_error("Unterminated escape sequence");
                char esc = s[i++];
                switch (esc) {
                    case '"':  res += '"'; break;
                    case '\\': res += '\\'; break;
                    case '/':  res += '/'; break;
                    case 'b':  res += '\b'; break;
                    case 'f':  res += '\f'; break;
                    case 'n':  res += '\n'; break;
                    case 'r':  res += '\r'; break;
                    case 't':  res += '\t'; break;
                    case 'u': {
                        // *CHANGES*: Decode JSON Unicode escapes as UTF-8, including surrogate pairs.
                        unsigned int code = parse_hex_quad(s, i);
                        if (code >= 0xD800 && code <= 0xDBFF) {
                            if (i + 2 > s.size() || s[i] != '\\' || s[i + 1] != 'u') {
                                throw std::runtime_error("Missing low surrogate in JSON string");
                            }
                            i += 2;
                            unsigned int low = parse_hex_quad(s, i);
                            if (low < 0xDC00 || low > 0xDFFF) {
                                throw std::runtime_error("Invalid low surrogate in JSON string");
                            }
                            code = 0x10000 + ((code - 0xD800) << 10) + (low - 0xDC00);
                        } else if (code >= 0xDC00 && code <= 0xDFFF) {
                            throw std::runtime_error("Unexpected low surrogate in JSON string");
                        }
                        append_utf8(res, code);
                        break;
                    }
                    default:
                        throw std::runtime_error("Invalid escape in JSON string");
                }
            } else {
                if (static_cast<unsigned char>(c) < 0x20) {
                    throw std::runtime_error("Unescaped control character in JSON string");
                }
                res += c;
            }
        }
        if (i >= s.size() || s[i] != '"') {
            throw std::runtime_error("Unterminated string in JSON");
        }
        ++i; // skip closing quote
        return res;
    }

    // *CHANGES*: Keep the Unicode conversion local to the JSON parser.
    static unsigned int parse_hex_quad(const std::string& s, size_t& i) {
        if (i + 4 > s.size()) throw std::runtime_error("Invalid unicode escape sequence");
        unsigned int value = 0;
        for (int n = 0; n < 4; ++n) {
            char c = s[i++];
            value <<= 4;
            if (c >= '0' && c <= '9') value += c - '0';
            else if (c >= 'a' && c <= 'f') value += c - 'a' + 10;
            else if (c >= 'A' && c <= 'F') value += c - 'A' + 10;
            else throw std::runtime_error("Invalid unicode escape sequence");
        }
        return value;
    }

    static void append_utf8(std::string& out, unsigned int code) {
        if (code < 0x80) out += static_cast<char>(code);
        else if (code < 0x800) {
            out += static_cast<char>(0xC0 | (code >> 6));
            out += static_cast<char>(0x80 | (code & 0x3F));
        } else if (code < 0x10000) {
            out += static_cast<char>(0xE0 | (code >> 12));
            out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (code & 0x3F));
        } else {
            out += static_cast<char>(0xF0 | (code >> 18));
            out += static_cast<char>(0x80 | ((code >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (code & 0x3F));
        }
    }

    /*
    1. Parse numeric literal matching optional '-', digits, optional fractional '.', and optional exponent.
    2. Convert parsed substring to double using std::stod.
    3. Return JsonValue with type Number.
    */
    static JsonValue parse_number(const std::string& s, size_t& i) {
        size_t start = i;
        if (i < s.size() && s[i] == '-') ++i;
        while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) ++i;
        if (i < s.size() && s[i] == '.') {
            ++i;
            while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) ++i;
        }
        if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
            ++i;
            if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
            while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) ++i;
        }
        std::string num_str = s.substr(start, i - start);
        double val = std::stod(num_str);
        return JsonValue(val);
    }

    /*
    1. Expect opening '{', then parse key-value pairs separated by commas until '}'.
    2. For each entry, parse key string, colon separator ':', and value.
    3. Return JsonValue with type Object.
    */
    static JsonValue parse_object(const std::string& s, size_t& i) {
        ++i; // skip '{'
        skip_whitespace(s, i);
        JsonValue obj = JsonValue::make_object();
        if (i < s.size() && s[i] == '}') {
            ++i;
            return obj;
        }
        while (i < s.size()) {
            skip_whitespace(s, i);
            std::string key = parse_string(s, i);
            skip_whitespace(s, i);
            if (i >= s.size() || s[i] != ':') {
                throw std::runtime_error("Expected ':' after object key");
            }
            ++i; // skip ':'
            JsonValue val = parse_value(s, i);
            obj[key] = val;
            skip_whitespace(s, i);
            if (i < s.size() && s[i] == '}') {
                ++i;
                return obj;
            }
            if (i >= s.size() || s[i] != ',') {
                throw std::runtime_error("Expected ',' or '}' in object");
            }
            ++i; // skip ','
        }
        throw std::runtime_error("Unterminated object in JSON");
    }

    /*
    1. Expect opening '[', then parse comma-separated values until ']'.
    2. Append each parsed value to internal array.
    3. Return JsonValue with type Array.
    */
    static JsonValue parse_array(const std::string& s, size_t& i) {
        ++i; // skip '['
        skip_whitespace(s, i);
        JsonValue arr = JsonValue::make_array();
        if (i < s.size() && s[i] == ']') {
            ++i;
            return arr;
        }
        while (i < s.size()) {
            JsonValue val = parse_value(s, i);
            arr.push_back(val);
            skip_whitespace(s, i);
            if (i < s.size() && s[i] == ']') {
                ++i;
                return arr;
            }
            if (i >= s.size() || s[i] != ',') {
                throw std::runtime_error("Expected ',' or ']' in array");
            }
            ++i; // skip ','
        }
        throw std::runtime_error("Unterminated array in JSON");
    }

    /*
    1. Check for literal 'true' or 'false'.
    2. Advance index and return JsonValue with type Boolean.
    */
    static JsonValue parse_bool(const std::string& s, size_t& i) {
        if (s.compare(i, 4, "true") == 0) {
            i += 4;
            return JsonValue(true);
        } else if (s.compare(i, 5, "false") == 0) {
            i += 5;
            return JsonValue(false);
        }
        throw std::runtime_error("Invalid boolean value in JSON");
    }

    /*
    1. Check for literal 'null'.
    2. Advance index and return JsonValue with type Null.
    */
    static JsonValue parse_null(const std::string& s, size_t& i) {
        if (s.compare(i, 4, "null") == 0) {
            i += 4;
            return JsonValue();
        }
        throw std::runtime_error("Invalid null value in JSON");
    }
};

} // namespace middleware

#endif // MIDDLEWARE_JSON_HPP
