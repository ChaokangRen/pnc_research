#include <iostream>
#include <queue>
#include <thread>
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
    bool flag = true;
    vector<vector<int>> kSmallestPairs(vector<int> &nums1, vector<int> &nums2,
                                       int k) {
        vector<vector<int>> ans;
        int n = nums1.size(), m = nums2.size();
        if (n > m) {  // 始终确保nums1为两数组中长度较少的那个
            swap(nums1, nums2);
            swap(m, n);
            flag = false;
        }
        // 定义比较规则
        auto cmp = [&](const auto &a, const auto &b) {
            return nums1[a.first] + nums2[a.second] >
                   nums1[b.first] + nums2[b.second];
        };
        priority_queue<pair<int, int>, vector<pair<int, int>>, decltype(cmp)> q(
            cmp);
        for (int i = 0; i < min(n, k); i++) {
            q.push({i, 0});
        }
        while (ans.size() < k and q.size()) {
            auto [a, b] = q.top();
            q.pop();
            flag ? ans.push_back({nums1[a], nums2[b]})
                 : ans.push_back({nums2[b], nums1[a]});
            if (b + 1 < m) q.push({a, b + 1});
        }
        return ans;
    }

    /**
     * Definition for singly-linked list.
     * struct ListNode {
     *     int val;
     *     ListNode *next;
     *     ListNode() : val(0), next(nullptr) {}
     *     ListNode(int x) : val(x), next(nullptr) {}
     *     ListNode(int x, ListNode *next) : val(x), next(next) {}
     * };
     */
    struct ListNode {
        int val;
        ListNode *next;
        ListNode() : val(0), next(nullptr) {}
        ListNode(int x) : val(x), next(nullptr) {}
        ListNode(int x, ListNode *next) : val(x), next(next) {}
    };
    ListNode *merge(ListNode *left, ListNode *right) {
        if (left == nullptr && right != nullptr) return right;
        if (left != nullptr && right == nullptr) return left;
        if (left == nullptr && right == nullptr) return nullptr;

        ListNode *head = nullptr;
        if (left->val < right->val) {
            head = left;
            left = left->next;
            head->next = nullptr;
        } else {
            head = right;
            right = right->next;
            head->next = nullptr;
        }
        head->next = nullptr;
        ListNode *tmp = head;

        while (left != nullptr && right != nullptr) {
            if (left->val < right->val) {
                tmp->next = left;
                left = left->next;
            } else {
                tmp->next = right;
                right = right->next;
            }
            tmp = tmp->next;
            tmp->next = nullptr;
        }
        if (left != nullptr) tmp->next = left;
        if (right != nullptr) tmp->next = right;

        return head;
    }

    // std::pair<ListNode * left, ListNode * right>
    ListNode *cut(ListNode *head) {
        ListNode *slow = head, *fast = head;
        ListNode *slow_pre = nullptr;
        while (fast != nullptr) {
            if (fast->next == nullptr) break;
            fast = fast->next->next;
            slow_pre = slow;
            slow = slow->next;
        }
        slow_pre->next = nullptr;

        if (head != nullptr && head->next != nullptr) head = cut(head);
        if (slow != nullptr && slow->next != nullptr) slow = cut(slow);

        return merge(head, slow);
    }
    ListNode *sortList(ListNode *head) {
        if (head == nullptr || head->next == nullptr) return head;
        return cut(head);
    }

    void test() {
        // ListNode *a = new ListNode(0);
        // ListNode *b = new ListNode(1);
        // ListNode *aa = a, *bb = b;
        // for (int i = 2; i < 12; ++i) {
        //     if (i % 2 == 0) {
        //         aa->next = new ListNode(i);
        //         aa = aa->next;
        //     } else {
        //         bb->next = new ListNode(i);
        //         bb = bb->next;
        //     }
        // }
        // aa = a, bb = b;
        // while (aa != nullptr) {
        //     std::cout << aa->val << "  ";
        //     aa = aa->next;
        // }
        // std::cout << endl;
        // while (bb != nullptr) {
        //     std::cout << bb->val << "  ";
        //     bb = bb->next;
        // }
        // std::cout << endl;

        // auto ans = merge(a, b);
        // while (ans != nullptr) {
        //     std::cout << ans->val << "  ";
        //     ans = ans->next;
        // }

        vector<int> tmp{-1, 5, 3, 4, 0};
        ListNode *a = new ListNode(6);
        ListNode *aa = a;
        for (int it : tmp) {
            aa->next = new ListNode(it);
            aa = aa->next;
        }
        aa = a;
        while (aa != nullptr) {
            std::cout << aa->val << "  ";
            aa = aa->next;
        }
        cout << endl;

        aa = sortList(a);
        if (aa == nullptr) cout << std::endl << "nullptr";
        while (aa != nullptr) {
            std::cout << aa->val << "  ";
            aa = aa->next;
        }
    }
};

int main(void) {
    std::vector<int> nums{0, 0, 1, 1, 1, 1, 2, 3, 3};
    Solution su;
    // 80

    //  int cnt = su.removeDuplicates(nums);
    //  std::vector<string> strs{"flower", "flow", "flight"};
    // 14
    //  string str = su.longestCommonPrefix(strs);
    //  std::cout << str << std::endl;

    // 373
    // vector<int> nums1{1, 2, 3, 4, 5, 6};
    // vector<int> nums2{3, 5, 7, 9};
    // vector<vector<int>> ans = su.kSmallestPairs(nums1, nums2, 20);
    // for (auto &it : ans) {
    //     std::cout << '[' << it[0] << ',' << it[1] << "],";
    // }
    su.test();
    return 0;
}
