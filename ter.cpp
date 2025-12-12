#include <bits/stdc++.h>
using namespace std;

/*
 Usage:
  ./ter 1A2B3C4D5E6    (11-hex digits)
  or provide hex on stdin.
 
 Interpretation:
  - Input hex -> binary. Use low 42 bits as 21 trits, 2 bits per trit.
  - 2-bit encoding -> trit value:
      00 -> -1
      01 ->  0
      10 ->  1
      11 -> treated as 0 (unused)
  - Value = sum_{i=0..20} trit[i] * 3^i
  - Output decimal (signed 64-bit)
*/

int main(int argc, char **argv) {
    string s;
    if (argc >= 2) s = argv[1];
    else {
        if (!getline(cin, s)) return 1;
    }
    // allow optional 0x/0X prefix
    if (s.size() >= 2 && s[0]=='0' && (s[1]=='x' || s[1]=='X')) s = s.substr(2);
    // trim whitespace
    while (!s.empty() && isspace((unsigned char)s.back())) s.pop_back();
    while (!s.empty() && isspace((unsigned char)s.front())) s.erase(s.begin());

    if (s.empty()) {
        cerr << "no input\n";
        return 1;
    }

    // parse hex to unsigned long long
    unsigned long long v = 0;
    for (char c : s) {
        int d = -1;
        if (c >= '0' && c <= '9') d = c - '0';
        else if (c >= 'a' && c <= 'f') d = 10 + (c - 'a');
        else if (c >= 'A' && c <= 'F') d = 10 + (c - 'A');
        else continue;
        v = (v << 4) | (unsigned long long)(d & 0xF);
    }

    // compute balanced ternary value from low 42 bits (21 trits)
    long long result = 0;
    long long place = 1;
    for (int i = 0; i < 21; ++i) {
        unsigned int two = (v >> (2*i)) & 0x3u;
        int trit = 0;
        if (two == 0u) trit = -1; // 00
        else if (two == 1u) trit = 0; // 01
        else if (two == 2u) trit = 1; // 10
        else /*3*/ trit = 0; // treat 11 as 0
        result += (long long)trit * place;
        place *= 3LL;
    }

    cout << result << "\n";
    return 0;
}