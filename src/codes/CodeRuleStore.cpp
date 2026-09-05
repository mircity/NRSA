#include "codes/CodeRuleStore.h"

#include <fstream>
#include <stdexcept>

namespace nrsa::codes {

namespace {

std::vector<std::string> splitPipe(const std::string& s) {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (true) {
        std::size_t pos = s.find('|', start);
        if (pos == std::string::npos) {
            parts.push_back(s.substr(start));
            break;
        }
        parts.push_back(s.substr(start, pos - start));
        start = pos + 1;
    }
    return parts;
}

std::string trimCr(std::string s) {
    if (!s.empty() && s.back() == '\r') s.pop_back();
    return s;
}

}  // namespace

std::vector<CodeRule> loadCodeRules(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("loadCodeRules: cannot open '" + path + "' for reading");

    std::vector<CodeRule> rules;
    std::string line;
    while (std::getline(f, line)) {
        line = trimCr(line);
        if (line.empty() || line[0] == '#') continue;
        auto parts = splitPipe(line);
        if (parts.size() != 7) {
            throw std::invalid_argument("loadCodeRules: malformed rule line (expected 7 fields): " + line);
        }
        CodeRule rule;
        rule.code = parts[0];
        rule.version = parts[1];
        rule.chapter = parts[2];
        rule.clause = parts[3];
        rule.description = parts[4];
        try {
            rule.value = std::stod(parts[5]);
        } catch (const std::exception&) {
            throw std::invalid_argument("loadCodeRules: non-numeric value field in line: " + line);
        }
        rule.unit = parts[6];
        rules.push_back(rule);
    }
    return rules;
}

std::optional<CodeRule> findRule(const std::vector<CodeRule>& rules, const std::string& code,
                                  const std::string& version, const std::string& clause) {
    for (const auto& r : rules) {
        if (r.code == code && r.version == version && r.clause == clause) return r;
    }
    return std::nullopt;
}

void saveCodeRules(const std::vector<CodeRule>& rules, const std::string& path) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("saveCodeRules: cannot open '" + path + "' for writing");
    f << "# code|version|chapter|clause|description|value|unit\n";
    for (const auto& r : rules) {
        f << r.code << '|' << r.version << '|' << r.chapter << '|' << r.clause << '|' << r.description
          << '|' << r.value << '|' << r.unit << '\n';
    }
}

}  // namespace nrsa::codes
