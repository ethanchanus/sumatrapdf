/* Copyright 2026 the SumatraPDF project authors (see AUTHORS file).
   License: GPLv3 */

#include "base/Base.h"

#include "RegexEngine.h"

// see RegexEngine.h for the supported syntax and the design rationale
// (Thompson NFA / Pike's VM instead of backtracking or std::regex).

// defined in TextSearch.cpp; not pulling in TextSearch.h here avoids dragging
// in its own (heavier) include-order requirements for a single free function
int FoldCaseForSearch(int c);

enum class RxOp : u8 {
    Char,  // match a single literal codepoint (x: next pc)
    Dot,   // match any codepoint except '\n' (x: next pc)
    Any,   // match any codepoint, incl. '\n' -- only used by the unanchored-search wrapper
    Class, // match against classes[classIdx] (x: next pc)
    Match, // accept
    Jmp,   // goto x
    Split, // fork: x (higher priority), then y (lower priority)
    Bol,   // zero-width: only at codepoint 0 (x: next pc)
    Eol,   // zero-width: only at the end of text (x: next pc)
};

struct RxInst {
    RxOp op = RxOp::Match;
    int c = 0;         // Char
    int classIdx = -1; // Class
    int x = -1;
    int y = -1;
};

struct RxRange {
    int lo, hi;
};

struct RxClass {
    Vec<RxRange> ranges; // positive membership (predefined \D \W \S are pre-complemented into this)
    bool negate = false; // the class's own leading '^' (or none for a standalone escape class)
};

struct RegexProgram {
    Vec<RxInst> prog;
    Vec<RxClass> classes;
    int start = -1;     // entry point: the unanchored-search wrapper
    int realStart = -1; // where the user's pattern itself begins
};

static int RxEmit(RegexProgram* rp, RxOp op) {
    RxInst inst;
    inst.op = op;
    VecAppend(rp->prog, inst);
    return len(rp->prog) - 1;
}

// a "dangling" (pc, which-field) pair still waiting to be pointed at whatever
// comes next once it's known (Thompson's construction "patch list")
struct RxPatch {
    int pc;
    bool isY;
};
using RxPatchList = Vec<RxPatch>;

static void RxPatchAll(RegexProgram* rp, const RxPatchList& list, int target) {
    for (const RxPatch& p : list) {
        if (p.isY) {
            rp->prog[p.pc].y = target;
        } else {
            rp->prog[p.pc].x = target;
        }
    }
}

static void RxAppendPatchList(RxPatchList& a, const RxPatchList& b) {
    for (const RxPatch& p : b) {
        VecAppend(a, p);
    }
}

struct RxFrag {
    // a user-declared (if defaulted) ctor keeps RxFrag from being an aggregate,
    // so `return {};` value-initializes via this ctor -- Vec's *explicit*
    // default ctor would otherwise make plain aggregate-style `{}` ill-formed
    RxFrag() = default;
    int start = -1;
    RxPatchList out;
};

// predefined \d \w \s member ranges (sorted, non-overlapping)
static const RxRange kDigitRanges[] = {{'0', '9'}};
static const RxRange kWordRanges[] = {{'0', '9'}, {'A', 'Z'}, {'_', '_'}, {'a', 'z'}};
static const RxRange kSpaceRanges[] = {{'\t', '\r'}, {' ', ' '}};
constexpr int kMaxCodepoint = 0x10FFFF;

static void RxAppendRanges(Vec<RxRange>& out, const RxRange* ranges, int count) {
    for (int i = 0; i < count; i++) {
        VecAppend(out, ranges[i]);
    }
}

// appends the complement of `ranges` (assumed sorted, non-overlapping) over
// [0, kMaxCodepoint], so e.g. \D can be unioned into a bracket expression
// alongside other members without needing per-class negation there
static void RxAppendComplementRanges(Vec<RxRange>& out, const RxRange* ranges, int count) {
    int start = 0;
    for (int i = 0; i < count; i++) {
        if (ranges[i].lo > start) {
            VecAppend(out, RxRange{start, ranges[i].lo - 1});
        }
        start = ranges[i].hi + 1;
    }
    if (start <= kMaxCodepoint) {
        VecAppend(out, RxRange{start, kMaxCodepoint});
    }
}

struct RxParser {
    Str pat;
    int pos = 0;
    int n = 0;
    RegexProgram* rp = nullptr;
    bool ok = true;

    bool Eof() const { return pos >= n; }
    int PeekByte() const { return pos < n ? (u8)pat.s[pos] : -1; }
    void Fail() { ok = false; }

    RxFrag ParseAlt();
    RxFrag ParseCat();
    RxFrag ParseRep();
    RxFrag ParseAtom();
    RxFrag ParseClassAtom();
    RxFrag ParseEscapeAtom();
    RxFrag EmitLiteral(int cp);
    RxFrag EmitClassFrag(const Vec<RxRange>& ranges, bool negate = false);
    // decodes one class-range endpoint (a literal char or \n \t \r \\ \] \^ \-
    // or any other over-escaped char); returns -1 and sets ok=false at EOF
    int ParseClassRangeEndpoint();
};

RxFrag RxParser::EmitLiteral(int cp) {
    int pc = RxEmit(rp, RxOp::Char);
    rp->prog[pc].c = cp;
    RxFrag f;
    f.start = pc;
    VecAppend(f.out, RxPatch{pc, false});
    return f;
}

RxFrag RxParser::EmitClassFrag(const Vec<RxRange>& ranges, bool negate) {
    RxClass rc;
    rc.ranges = ranges;
    rc.negate = negate;
    int idx = len(rp->classes);
    VecAppend(rp->classes, rc);
    int pc = RxEmit(rp, RxOp::Class);
    rp->prog[pc].classIdx = idx;
    RxFrag f;
    f.start = pc;
    VecAppend(f.out, RxPatch{pc, false});
    return f;
}

int RxParser::ParseClassRangeEndpoint() {
    if (Eof()) {
        Fail();
        return -1;
    }
    if (PeekByte() == '\\') {
        pos++;
        if (Eof()) {
            Fail();
            return -1;
        }
        int e = PeekByte();
        pos++;
        switch (e) {
            case 'n':
                return '\n';
            case 't':
                return '\t';
            case 'r':
                return '\r';
            default:
                return e; // lenient: \] \^ \- \\ or any other over-escaped char
        }
    }
    int byteIdx = pos;
    int cp = Utf8CodepointNext(pat, byteIdx);
    pos = byteIdx;
    return cp;
}

RxFrag RxParser::ParseClassAtom() {
    pos++; // consume '['
    bool negate = false;
    if (!Eof() && PeekByte() == '^') {
        negate = true;
        pos++;
    }
    Vec<RxRange> ranges;
    bool first = true;
    for (;;) {
        if (Eof()) {
            Fail();
            return {};
        }
        int c = PeekByte();
        if (c == ']' && !first) {
            pos++;
            break;
        }
        first = false;
        if (c == ']') {
            // ']' right after '[' or '[^' is a literal per common convention
            pos++;
            VecAppend(ranges, RxRange{']', ']'});
            continue;
        }
        if (c == '\\' && pos + 1 < n) {
            int e = (u8)pat.s[pos + 1];
            bool isPredefined = e == 'd' || e == 'D' || e == 'w' || e == 'W' || e == 's' || e == 'S';
            if (isPredefined) {
                pos += 2;
                switch (e) {
                    case 'd':
                        RxAppendRanges(ranges, kDigitRanges, dimofi(kDigitRanges));
                        break;
                    case 'D':
                        RxAppendComplementRanges(ranges, kDigitRanges, dimofi(kDigitRanges));
                        break;
                    case 'w':
                        RxAppendRanges(ranges, kWordRanges, dimofi(kWordRanges));
                        break;
                    case 'W':
                        RxAppendComplementRanges(ranges, kWordRanges, dimofi(kWordRanges));
                        break;
                    case 's':
                        RxAppendRanges(ranges, kSpaceRanges, dimofi(kSpaceRanges));
                        break;
                    case 'S':
                        RxAppendComplementRanges(ranges, kSpaceRanges, dimofi(kSpaceRanges));
                        break;
                }
                continue;
            }
        }
        int cp = ParseClassRangeEndpoint();
        if (!ok) {
            return {};
        }
        if (!Eof() && PeekByte() == '-' && pos + 1 < n && pat.s[pos + 1] != ']') {
            pos++; // consume '-'
            int cp2 = ParseClassRangeEndpoint();
            if (!ok) {
                return {};
            }
            if (cp2 < cp) {
                Fail();
                return {};
            }
            VecAppend(ranges, RxRange{cp, cp2});
        } else {
            VecAppend(ranges, RxRange{cp, cp});
        }
    }
    return EmitClassFrag(ranges, negate);
}

RxFrag RxParser::ParseEscapeAtom() {
    pos++; // consume '\'
    if (Eof()) {
        Fail();
        return {};
    }
    int e = PeekByte();
    pos++;
    Vec<RxRange> ranges;
    switch (e) {
        case 'd':
            RxAppendRanges(ranges, kDigitRanges, dimofi(kDigitRanges));
            return EmitClassFrag(ranges);
        case 'D':
            RxAppendComplementRanges(ranges, kDigitRanges, dimofi(kDigitRanges));
            return EmitClassFrag(ranges);
        case 'w':
            RxAppendRanges(ranges, kWordRanges, dimofi(kWordRanges));
            return EmitClassFrag(ranges);
        case 'W':
            RxAppendComplementRanges(ranges, kWordRanges, dimofi(kWordRanges));
            return EmitClassFrag(ranges);
        case 's':
            RxAppendRanges(ranges, kSpaceRanges, dimofi(kSpaceRanges));
            return EmitClassFrag(ranges);
        case 'S':
            RxAppendComplementRanges(ranges, kSpaceRanges, dimofi(kSpaceRanges));
            return EmitClassFrag(ranges);
        case 'n':
            return EmitLiteral('\n');
        case 't':
            return EmitLiteral('\t');
        case 'r':
            return EmitLiteral('\r');
        default: {
            // over-escaped literal (\. \* \+ \? \( \) \[ \] \{ \} \| \\ \^ \$
            // or any other char); un-consume and decode as a full codepoint
            pos--;
            int byteIdx = pos;
            int cp = Utf8CodepointNext(pat, byteIdx);
            pos = byteIdx;
            return EmitLiteral(cp);
        }
    }
}

RxFrag RxParser::ParseAtom() {
    if (Eof()) {
        Fail();
        return {};
    }
    int c = PeekByte();
    if (c == '(') {
        pos++;
        if (pos + 1 < n && pat.s[pos] == '?' && pat.s[pos + 1] == ':') {
            pos += 2; // "(?:" behaves the same as a plain "(" here (no captures anyway)
        }
        RxFrag f = ParseAlt();
        if (!ok) {
            return {};
        }
        if (Eof() || PeekByte() != ')') {
            Fail();
            return {};
        }
        pos++;
        return f;
    }
    if (c == '.') {
        pos++;
        int pc = RxEmit(rp, RxOp::Dot);
        RxFrag f;
        f.start = pc;
        VecAppend(f.out, RxPatch{pc, false});
        return f;
    }
    if (c == '^') {
        pos++;
        int pc = RxEmit(rp, RxOp::Bol);
        RxFrag f;
        f.start = pc;
        VecAppend(f.out, RxPatch{pc, false});
        return f;
    }
    if (c == '$') {
        pos++;
        int pc = RxEmit(rp, RxOp::Eol);
        RxFrag f;
        f.start = pc;
        VecAppend(f.out, RxPatch{pc, false});
        return f;
    }
    if (c == '[') {
        return ParseClassAtom();
    }
    if (c == '\\') {
        return ParseEscapeAtom();
    }
    if (c == '*' || c == '+' || c == '?' || c == ')' || c == '|') {
        Fail(); // quantifier / close-paren with nothing to apply to
        return {};
    }
    int byteIdx = pos;
    int cp = Utf8CodepointNext(pat, byteIdx);
    pos = byteIdx;
    return EmitLiteral(cp);
}

RxFrag RxParser::ParseRep() {
    RxFrag a = ParseAtom();
    if (!ok) {
        return {};
    }
    for (;;) {
        int c = PeekByte();
        if (c != '*' && c != '+' && c != '?') {
            break;
        }
        pos++;
        bool lazy = false;
        if (!Eof() && PeekByte() == '?') {
            lazy = true;
            pos++;
        }
        if (c == '*') {
            int splitPc = RxEmit(rp, RxOp::Split);
            RxPatchAll(rp, a.out, splitPc);
            RxFrag f;
            f.start = splitPc;
            if (!lazy) {
                rp->prog[splitPc].x = a.start;
                VecAppend(f.out, RxPatch{splitPc, true});
            } else {
                rp->prog[splitPc].y = a.start;
                VecAppend(f.out, RxPatch{splitPc, false});
            }
            a = f;
        } else if (c == '+') {
            int splitPc = RxEmit(rp, RxOp::Split);
            RxPatchAll(rp, a.out, splitPc);
            RxFrag f;
            f.start = a.start;
            if (!lazy) {
                rp->prog[splitPc].x = a.start;
                VecAppend(f.out, RxPatch{splitPc, true});
            } else {
                rp->prog[splitPc].y = a.start;
                VecAppend(f.out, RxPatch{splitPc, false});
            }
            a = f;
        } else { // '?'
            int splitPc = RxEmit(rp, RxOp::Split);
            RxFrag f;
            f.start = splitPc;
            f.out = a.out;
            if (!lazy) {
                rp->prog[splitPc].x = a.start;
                VecAppend(f.out, RxPatch{splitPc, true});
            } else {
                rp->prog[splitPc].y = a.start;
                VecAppend(f.out, RxPatch{splitPc, false});
            }
            a = f;
        }
    }
    return a;
}

RxFrag RxParser::ParseCat() {
    if (Eof() || PeekByte() == '|' || PeekByte() == ')') {
        // empty concatenation (e.g. "()" or the branch after a trailing '|')
        // matches the empty string: a passthrough Jmp with a dangling target
        int pc = RxEmit(rp, RxOp::Jmp);
        RxFrag f;
        f.start = pc;
        VecAppend(f.out, RxPatch{pc, false});
        return f;
    }
    RxFrag a = ParseRep();
    if (!ok) {
        return {};
    }
    while (!Eof() && PeekByte() != '|' && PeekByte() != ')') {
        RxFrag b = ParseRep();
        if (!ok) {
            return {};
        }
        RxPatchAll(rp, a.out, b.start);
        a.out = b.out;
    }
    return a;
}

RxFrag RxParser::ParseAlt() {
    RxFrag a = ParseCat();
    if (!ok) {
        return {};
    }
    while (!Eof() && PeekByte() == '|') {
        pos++;
        RxFrag b = ParseCat();
        if (!ok) {
            return {};
        }
        int splitPc = RxEmit(rp, RxOp::Split);
        rp->prog[splitPc].x = a.start;
        rp->prog[splitPc].y = b.start;
        RxFrag merged;
        merged.start = splitPc;
        merged.out = a.out;
        RxAppendPatchList(merged.out, b.out);
        a = merged;
    }
    return a;
}

RegexProgram* RegexCompile(Str pattern) {
    if (len(pattern) == 0) {
        return nullptr;
    }
    auto* rp = new RegexProgram();
    RxParser parser;
    parser.pat = pattern;
    parser.n = len(pattern);
    parser.rp = rp;
    RxFrag body = parser.ParseAlt();
    if (!parser.ok || parser.pos != parser.n) {
        delete rp;
        return nullptr;
    }
    int matchPc = RxEmit(rp, RxOp::Match);
    RxPatchAll(rp, body.out, matchPc);
    rp->realStart = body.start;

    // wrap with a non-greedy ".*?" prefix so the pattern can be found
    // starting anywhere, not just at codepoint 0 (the classic technique for
    // unanchored search with Pike's VM, see RegexEngine.h)
    int splitPc = RxEmit(rp, RxOp::Split); // L0
    int anyPc = RxEmit(rp, RxOp::Any);      // L1
    int jmpPc = RxEmit(rp, RxOp::Jmp);      // L2
    rp->prog[splitPc].x = rp->realStart;    // prefer entering the real pattern
    rp->prog[splitPc].y = anyPc;
    rp->prog[anyPc].x = jmpPc;
    rp->prog[jmpPc].x = splitPc;
    rp->start = splitPc;
    return rp;
}

void RegexFree(RegexProgram* prog) {
    delete prog;
}

// ASCII-only case swap for bracket-expression ranges like [A-Z]: predefined
// \d \w \s already include both cases explicitly, and literal characters
// elsewhere use FoldCaseForSearch's full Unicode folding, so this only needs
// to cover the common case of a user-typed ASCII range.
static int RxAsciiSwapCase(int c) {
    if (c >= 'a' && c <= 'z') {
        return c - 32;
    }
    if (c >= 'A' && c <= 'Z') {
        return c + 32;
    }
    return c;
}

static bool RxInRanges(const Vec<RxRange>& ranges, int cp) {
    for (const RxRange& r : ranges) {
        if (cp >= r.lo && cp <= r.hi) {
            return true;
        }
    }
    return false;
}

static bool RxClassMatches(const RegexProgram* rp, int classIdx, int cp, bool matchCase) {
    const RxClass& rc = rp->classes[classIdx];
    bool m = RxInRanges(rc.ranges, cp);
    if (!m && !matchCase) {
        int alt = RxAsciiSwapCase(cp);
        if (alt != cp) {
            m = RxInRanges(rc.ranges, alt);
        }
    }
    return m != rc.negate;
}

static bool RxCharMatches(int inputCp, int patCp, bool matchCase) {
    if (matchCase) {
        return inputCp == patCp;
    }
    return FoldCaseForSearch(inputCp) == FoldCaseForSearch(patCp);
}

struct RxThread {
    int pc;
    int start;
};

// a thread list for one VM step: threads plus a per-pc "last generation added"
// stamp so a pc is added at most once per step (this dedup is what keeps the
// simulation linear instead of exponential)
struct RxThreadList {
    Vec<RxThread> threads;
    Vec<int> visitedGen;
    int gen = 0;

    void Init(int progLen) {
        VecResize(visitedGen, progLen);
        for (int i = 0; i < progLen; i++) {
            visitedGen[i] = -1;
        }
        gen = 0;
    }
    void StartStep() {
        VecClear(threads);
        gen++;
    }
};

// epsilon-closure: follows Jmp/Split/Bol/Eol until a consuming instruction
// (Char/Dot/Any/Class) or Match is reached, adding it to `list`. `start` is
// the position this thread's overall match began at; it's overridden to
// `pos` the moment execution enters the user pattern itself (realStart),
// which is what makes multiple simultaneously-live threads correctly report
// the earliest (leftmost) start once one of them matches.
static void RxAddThread(const RegexProgram* rp, RxThreadList& list, int pc, int start, int pos, int cpsLen) {
    if (list.visitedGen[pc] == list.gen) {
        return;
    }
    list.visitedGen[pc] = list.gen;
    if (pc == rp->realStart) {
        start = pos;
    }
    const RxInst& inst = rp->prog[pc];
    switch (inst.op) {
        case RxOp::Jmp:
            RxAddThread(rp, list, inst.x, start, pos, cpsLen);
            return;
        case RxOp::Split:
            RxAddThread(rp, list, inst.x, start, pos, cpsLen);
            RxAddThread(rp, list, inst.y, start, pos, cpsLen);
            return;
        case RxOp::Bol:
            if (pos == 0) {
                RxAddThread(rp, list, inst.x, start, pos, cpsLen);
            }
            return;
        case RxOp::Eol:
            if (pos == cpsLen) {
                RxAddThread(rp, list, inst.x, start, pos, cpsLen);
            }
            return;
        default:
            break;
    }
    VecAppend(list.threads, RxThread{pc, start});
}

bool RegexSearch(const RegexProgram* prog, const int* cps, int cpsLen, int from, int limit, bool matchCase,
                  int* startOut, int* endOut) {
    if (!prog || !cps || from < 0 || from > cpsLen) {
        return false;
    }
    if (limit > cpsLen) {
        limit = cpsLen;
    }
    if (from > limit) {
        return false;
    }

    int progLen = len(prog->prog);
    RxThreadList listA, listB;
    listA.Init(progLen);
    listB.Init(progLen);
    RxThreadList* clist = &listA;
    RxThreadList* nlist = &listB;

    clist->StartStep();
    RxAddThread(prog, *clist, prog->start, -1, from, cpsLen);

    bool matched = false;
    int bestStart = 0, bestEnd = 0;
    int pos = from;
    for (;;) {
        if (len(clist->threads) == 0) {
            break;
        }
        int cp = (pos < limit) ? cps[pos] : -1;
        nlist->StartStep();
        for (int i = 0; i < len(clist->threads); i++) {
            const RxThread& t = clist->threads[i];
            const RxInst& inst = prog->prog[t.pc];
            bool advance = false;
            switch (inst.op) {
                case RxOp::Char:
                    advance = pos < limit && RxCharMatches(cp, inst.c, matchCase);
                    break;
                case RxOp::Dot:
                    advance = pos < limit && cp != '\n';
                    break;
                case RxOp::Any:
                    advance = pos < limit;
                    break;
                case RxOp::Class:
                    advance = pos < limit && RxClassMatches(prog, inst.classIdx, cp, matchCase);
                    break;
                case RxOp::Match:
                    bestStart = t.start;
                    bestEnd = pos;
                    matched = true;
                    break;
                default:
                    break; // epsilon ops never appear in a resolved thread list
            }
            if (advance) {
                RxAddThread(prog, *nlist, inst.x, t.start, pos + 1, cpsLen);
            }
            if (inst.op == RxOp::Match) {
                break; // lower-priority threads this step can't beat this match
            }
        }
        if (pos >= limit) {
            break;
        }
        pos++;
        RxThreadList* tmp = clist;
        clist = nlist;
        nlist = tmp;
    }

    if (!matched) {
        return false;
    }
    *startOut = bestStart;
    *endOut = bestEnd;
    return true;
}
