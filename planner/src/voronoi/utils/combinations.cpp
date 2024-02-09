#include "planner/voronoi/utils/combinations.h"
using namespace std;
vector<vector<int>> generate_subsets(vector<int>& nums) {
    sort(nums.begin(), nums.end()); // Sort the input vector
    int n = nums.size();
    vector<vector<int>> result;
    do {
        for (int i = 0; i < (1 << n); ++i) {
            vector<int> subset;
            for (int j = 0; j < n; ++j) {
                if (i & (1 << j)) {
                    subset.push_back(nums[j]);
                }
            }
            result.push_back(subset);
        }
    } while (next_permutation(nums.begin(), nums.end())); // Generate next permutation
    return result;
}