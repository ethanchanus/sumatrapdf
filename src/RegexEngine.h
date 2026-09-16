/* Copyright 2026 the SumatraPDF project authors (see AUTHORS file).
   License: GPLv3 */

// Compact regex engine operating on Unicode codepoints, used by the Find UI's
// "regex" toggle (see TextSearch.cpp). Implements Thompson NFA construction +
// Pike's VM simulation (https://swtch.com/~rsc/regexp/regexp2.html): matching
// is O(pattern * text) with no backtracking, so it can't suffer the
// catastrophic-backtracking (ReDoS) blowup a naive backtracking matcher would
// be exposed to on untrusted, user-typed patterns. It also never throws:
// SumatraPDF is built with C++ exceptions off, so an invalid pattern must be
// reported through a return value, not std::regex_error.
//
// Supported syntax: literal characters, '.', character classes [abc] [^abc]
// [a-z], \d \D \w \W \s \S (standalone or inside a class), the quantifiers
// * + ? (each optionally non-greedy via a trailing '?'), grouping (...),
// alternation |, and the whole-text anchors ^ / $ (matching only the very
// start/end of the searched text, not per line). Not supported: {m,n}
// counted repetition (its braces are matched literally), capture
// groups/backreferences, and lookaround.

struct RegexProgram;

// Compiles `pattern` (UTF-8). Returns nullptr if the syntax is invalid.
RegexProgram* RegexCompile(Str pattern);
void RegexFree(RegexProgram* prog);

// Finds the leftmost match of `prog` in the codepoint array `cps[0..cpsLen)`,
// considering only matches that start at or after `from` and end at or before
// `limit`. `matchCase` selects case-sensitive matching (mirrors
// TextSearch::matchCase). On success returns true and sets *startOut /
// *endOut to the matched [start, end) codepoint range.
bool RegexSearch(const RegexProgram* prog, const int* cps, int cpsLen, int from, int limit, bool matchCase,
                  int* startOut, int* endOut);
