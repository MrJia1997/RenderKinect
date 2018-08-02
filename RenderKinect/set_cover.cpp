#include "set_cover.h"
#include <algorithm>
#include <iterator>

bool canCover(std::vector<std::set<int>>& sets, int total) {
    std::vector<bool> total_flag(total, false);
    for (auto set : sets) {
        for (auto ele : set) {
            if (ele >= 0 && ele < total) {
                total_flag[ele] = true;
            }
            else {
                std::cerr << "Element out of range." << std::endl;
                exit(-1);
            }
        }
    }
    for (auto flag : total_flag) {
        if (!flag)
            return false;
    }
    return true;
}

double calcOverlap(std::set<int> &A, std::set<int> &B) {
    std::vector<int> i, u;
    std::set_intersection(A.begin(), A.end(), B.begin(), B.end(), std::back_inserter(i));
    std::set_union(A.begin(), A.end(), B.begin(), B.end(), std::back_inserter(u));
    //std::cout << "I size: " << i.size() << ", U size: " << u.size();
    return (double)i.size() / (double)u.size();
}

std::vector<int> calcSetCoverGreedy(std::vector<std::set<int>>& sets, int total, double overlapThreshold)
{
    if (canCover(sets, total)) {
        std::cerr << "Cannot cover the universal set." << std::endl;
        exit(-1);
    }

    int size = sets.size();
    std::vector<bool> isValid(size, true);
    std::set<int> I;
    std::vector<int> result;
    
    while (I.size() != total) {
        int maxValue = 0, maxIndex = -1;
        std::vector<int> tempI;
        if (result.empty()) {
            for (int i = 0; i < size; i++) {
                if (isValid[i]) {
                    if (sets[i].size() > maxValue) {
                        maxValue = sets[i].size();
                        maxIndex = i;
                    }
                }
            }
        }
        else {
            for (int i = 0; i < size; i++) {
                if (isValid[i]) {
                    bool enoughOverlap = false;
                    for (int id : result) {
                        if (calcOverlap(sets[id], sets[i]) >= overlapThreshold) {
                            enoughOverlap = true;
                            break;
                        };
                    }
                    if (!enoughOverlap)
                        continue;


                    std::vector<int> diff;
                    std::set_difference(sets[i].begin(), sets[i].end(), I.begin(), I.end(), std::back_inserter(diff));
                    if (diff.size() > maxValue) {
                        maxValue = diff.size();
                        maxIndex = i;
                    }
                    else if (diff.size() == 0) {
                        isValid[i] = false;
                    }
                }
            }
        }

        // combine I and sets[maxIndex];
        std::set_union(sets[maxIndex].begin(), sets[maxIndex].end(), I.begin(), I.end(), std::back_inserter(tempI));
        std::copy(tempI.begin(), tempI.end(), std::inserter(I, I.end()));
        isValid[maxIndex] = false;
        result.push_back(maxIndex);
    }
    return result;
}
