#ifndef RANDOM_COMBINATIONS_H
#define RANDOM_COMBINATIONS_H
#include <iostream>
#include <vector>
#include <algorithm>
void generateCombinations(std::vector<int> &nums, int start, std::vector<int> &current, std::vector<std::vector<int>> &result);
std::vector<std::vector<int>> generateAllCombinations(std::vector<int> &nums);
#endif // RANDOM_COMBINATIONS_H