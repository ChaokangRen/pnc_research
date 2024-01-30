#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
    // 80. 删除有序数组中的重复项 II
    int removeDuplicates(vector<int> &nums) {
        int len = 0, k = 2;
        for (int &num : nums) {
            if (len < k || nums[len - k] != num) {
                nums[len++] = num;
            }
        }
        return len;
    }
    // 14. 最长公共前缀
    string find_same_str(const string &str1, const string &str2) {
        int len = std::min(str1.size(), str2.size());
        string str;
        for (int i = 0; i < len; ++i) {
            if (str1[i] != str2[i]) {
                break;
            }
            str += str1[i];
        }
        return str;
    }
    string longestCommonPrefix(vector<string> &strs) {
        if (strs.size() <= 1) return strs.front();
        string str = strs[0];
        for (int j = 1; j < strs.size(); ++j) {
            str = find_same_str(str, strs[j]);
            if (str.size() == 0) break;
        }

        return str;
    }
    // 373. 查找和最小的 K 对数字
    vector<vector<int>> kSmallestPairs(vector<int> &nums1, vector<int> &nums2,
                                       int k) {
        vector<vector<int>> smallest_pairs;
        for (int i = 0; i < nums2.size(); ++i) {
            for (int j = 0; j < nums1.size(); ++j) {
                if (i * nums1.size() + j >= k) return smallest_pairs;
            }
        }
        return smallest_pairs;
    }
};

int main(void) {
    std::vector<int> nums{0, 0, 1, 1, 1, 1, 2, 3, 3};
    Solution su;
    // int cnt = su.removeDuplicates(nums);
    std::vector<string> strs{"flower", "flow", "flight"};

    string str = su.longestCommonPrefix(strs);
    std::cout << str << std::endl;
    return 0;
}