// #ifndef COMBINATIONS_H
// #define COMBINATIONS_H
// #include <iostream>
// #include <vector>
// #include <algorithm>
// template <typename T> std::vector<std::vector<T>> combinations(const std::vector<T>& arr, int k);
// template <typename T> std::vector<std::vector<T>> permutations(std::vector<T>& arr);
// template <typename T> std::vector<std::vector<T>> all_permutations_with_subsets(const std::vector<T>& arr);
// #endif // COMBINATIONS_H

#ifndef PERMUTATIONS_H
#define PERMUTATIONS_H

#include <vector>
#include <algorithm>

template <typename T>
std::vector<std::vector<T>> combinations(const std::vector<T>& arr, int k) {
    std::vector<std::vector<T>> result;
    std::vector<bool> bitmask(k, true);
    bitmask.resize(arr.size(), false);

    do {
        std::vector<T> combination;
        for (size_t i = 0; i < arr.size(); ++i) {
            if (bitmask[i]) {
                combination.push_back(arr[i]);
            }
        }
        result.push_back(combination);
    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

    return result;
}

template <typename T>
std::vector<std::vector<T>> permutations(std::vector<T>& arr) {
    std::vector<std::vector<T>> result;
    do {
        result.push_back(arr);
    } while (std::next_permutation(arr.begin(), arr.end()));
    return result;
}

template <typename T>
std::vector<std::vector<T>> all_permutations_with_subsets(const std::vector<T>& arr) {
    std::vector<std::vector<T>> all_perms;
    for (size_t k = 0; k <= arr.size(); ++k) {
        auto subsets = combinations(arr, k);
        for (const auto& subset : subsets) {
            auto mutable_subset = subset;  // Make a copy to make it mutable
            auto perms = permutations(mutable_subset);
            all_perms.insert(all_perms.end(), perms.begin(), perms.end());
        }
    }
    return all_perms;
}

#endif // PERMUTATIONS_H
