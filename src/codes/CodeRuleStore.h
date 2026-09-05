#pragma once

#include <optional>
#include <string>
#include <vector>

namespace nrsa::codes {

// Roadmap Section 7 ("Code Update System"): the roadmap's own stated
// goal is that a code value should live in a STRUCTURED, DATA-DRIVEN
// rule system --
//   Code -> Version -> Chapter -> Clause -> Formula -> Limit -> Unit
// -- specifically so that when BNBC (or any code) gets a new edition,
// the SOFTWARE doesn't need rewriting, just the rule DATA does.
//
// SCOPE, STATED PLAINLY: this is a proof-of-concept of that idea, not
// a migration of every hardcoded constant in this project onto it.
// Doing that migration for real would mean reworking
// SeismicAnalysis/WindAnalysis/Settlement's own tested code, which
// risks the exact kind of regression this project's testing culture
// exists to catch -- not something to do as a drive-by inside adding
// a NEW module. What's built here: a simple, genuinely external
// (loaded from a plain text file at runtime, not compiled in) rule
// store following exactly the Code/Version/Chapter/Clause/Limit/Unit
// shape the roadmap describes, with a FEW example rules -- proving the
// mechanism, not replacing the existing modules' internal constants.
//
// FILE FORMAT: one rule per line, pipe-delimited:
//   code|version|chapter|clause|description|value|unit
// A line starting with # is a comment and is skipped. This is
// deliberately as simple as a format can be while still being humanly
// editable in a plain text editor by someone updating a rule when a
// new code edition comes out -- exactly the "don't need to rewrite
// the software" scenario the roadmap describes.
struct CodeRule {
    std::string code;         // e.g. "BNBC"
    std::string version;      // e.g. "2020"
    std::string chapter;      // e.g. "6.2"
    std::string clause;       // e.g. "6.2.13"
    std::string description;  // human-readable
    double value = 0.0;
    std::string unit;         // e.g. "dimensionless", "kPa", "m/s"
};

// Loads every rule from a pipe-delimited text file at path (see class
// doc comment for the format). Throws std::runtime_error if the file
// cannot be opened, and std::invalid_argument if any non-comment,
// non-blank line doesn't have exactly 7 pipe-delimited fields or has a
// non-numeric value field.
std::vector<CodeRule> loadCodeRules(const std::string& path);

// Finds the rule matching code+version+clause exactly (case-sensitive),
// or std::nullopt if no such rule exists in `rules`.
std::optional<CodeRule> findRule(const std::vector<CodeRule>& rules, const std::string& code,
                                  const std::string& version, const std::string& clause);

// Writes rules back to path in the same pipe-delimited format
// loadCodeRules reads -- the "update" half of "Code Update System":
// an engineer (or an automated import script, not built here) edits
// or appends rules and this persists them, without any recompilation.
void saveCodeRules(const std::vector<CodeRule>& rules, const std::string& path);

}  // namespace nrsa::codes
