#include "set_cover.h"
#include <algorithm>
#include <iterator>

namespace set_cover {
    int pointsCover(std::vector<std::set<int>>& sets, int total) {
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
        int cnt = 0;
        for (auto flag : total_flag) {
            if (flag)
                cnt++;
        }
        return cnt;
    }

    double calcOverlap(std::set<int> &A, std::set<int> &B) {
        std::vector<int> i, u;
        std::set_intersection(A.begin(), A.end(), B.begin(), B.end(), std::back_inserter(i));
        std::set_union(A.begin(), A.end(), B.begin(), B.end(), std::back_inserter(u));
        //std::cout << "I size: " << i.size() << ", U size: " << u.size();
        return (double)i.size() / (double)u.size();
    }

    std::vector<std::pair<int, int>> calcSetCoverGreedy(std::vector<std::set<int>>& sets, 
        int total, 
        double overlapThreshold,
        double coverThreshold)
    {
        int points_cover = pointsCover(sets, total);
        if (points_cover < total) {
            std::cout << "Cannot cover the universal set." << std::endl;
            std::cout << "Left " << total - points_cover << " points." << std::endl;
        }
        if (coverThreshold > (double)points_cover / (double)total) {
            std::cout << "Cover threshold cannot achieve. Only " << (double)100 * points_cover / (double)total << "% points can get covered" << std::endl;
        }

        int size = sets.size();
        std::vector<bool> isValid(size, true);
        std::set<int> I;
        std::vector<std::pair<int, int>> result;
        int bound = std::min(points_cover, (int)(total * coverThreshold));

        while (I.size() < bound) {
            int maxValue = 0, maxIndex = -1, matchIndex = -1;
            std::vector<int> tempI;
            if (result.empty()) {
                for (int i = 0; i < size; i++) {
                    if (!isValid[i])
                        continue; 
                    
                    if (sets[i].size() > maxValue) {
                        maxValue = sets[i].size();
                        maxIndex = i;
                    }
                    
                }
            }
            else {
                for (int i = 0; i < size; i++) {
                    if (!isValid[i])
                        continue; 
                    
                    bool enoughOverlap = false;
                    int overlapId = -1;
                    for (auto id : result) {
                        if (calcOverlap(sets[id.first], sets[i]) >= overlapThreshold) {
                            enoughOverlap = true;
                            overlapId = id.first;
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
                        matchIndex = overlapId;
                    }
                    else if (diff.size() == 0) {
                        isValid[i] = false;
                    }
                    
                }
            }

            if (maxIndex < 0) {
                // Try again
                for (int i = 0; i < size; i++) {
                    if (!isValid[i])
                        continue;
                    
                    double maxOverlap = 0, maxOverlapId = -1;
                    for (auto id : result) {
                        maxOverlap = std::max(maxOverlap, calcOverlap(sets[id.first], sets[i]));
                        maxOverlapId = id.first;
                    }

                    std::vector<int> diff;
                    std::set_difference(sets[i].begin(), sets[i].end(), I.begin(), I.end(), std::back_inserter(diff));
                    if (diff.size() > maxValue) {
                        maxValue = diff.size();
                        maxIndex = i;
                        matchIndex = maxOverlapId;
                    }
                    else if (diff.size() == 0) {
                        isValid[i] = false;
                    }
                }
            }
                
            // combine I and sets[maxIndex];
            std::set_union(sets[maxIndex].begin(), sets[maxIndex].end(), I.begin(), I.end(), std::back_inserter(tempI));
            std::copy(tempI.begin(), tempI.end(), std::inserter(I, I.end()));
            isValid[maxIndex] = false;
            result.push_back(std::make_pair(maxIndex, matchIndex));
            //std::cout << "Put in " << result.size() << " pose (No." << maxIndex << ")" << std::endl;
            //std::cout << "Current cover points: " << I.size() << std::endl;
            std::cout << result.size() << " " << I.size() << std::endl;
        }
        return result;
    }
}