#pragma once
#include <vector>
#include <iostream>
#include <set>

// Return indices of the sets which can cover the universal set in minimum number.
// The universal set is {0, 1, 2, ..., total - 1}
// The overlapThreshold means that essential overlap between adjacent sets, in [0, 1).
// The overlap is calculate by (size of the intersection / size of the union)
std::vector<int> calcSetCoverGreedy(std::vector<std::set<int>> &sets, int total, double overlapThreshold);