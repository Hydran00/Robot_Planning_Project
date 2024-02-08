#include "planner/voronoi/utils/random_combinations.h"
// Function to generate combinations recursively
// Function to generate combinations recursively
void generateCombinations(std::vector<int>& nums, int start, std::vector<int>& current, std::vector<std::vector<int>>& result) {
    result.push_back(current); // Add current combination to result

    // Iterate over the rest of the elements
    for (size_t i = start; i < nums.size(); ++i) {
        current.push_back(nums[i]); // Include current element

        // Recur for remaining elements starting from i + 1
        generateCombinations(nums, i + 1, current, result);

        current.pop_back(); // Backtrack, remove current element to try other combinations
    }
}

// Function to generate all combinations of a list of integers
std::vector<std::vector<int>> generateAllCombinations(std::vector<int>& nums) {
    std::vector<std::vector<int>> result;
    std::vector<int> current;
    generateCombinations(nums, 0, current, result);
    return result;
}