#include <bits/stdc++.h>
#define pb push_back
using namespace std;

void solve()
{
    int n;
    cin >> n;
    vector<int> a(n);
    for (int i = 0; i < n; i++)
    {
        cin >> a[i];
    }
    int ans = 0;
    int sum = 0;
    if (a[0] > 0)
    {
        ans = a[0];
    }
    sum = max(ans, sum);
    for (int i = 0; i < n - 1; i++)
    {

        if (a[i + 1] % 2 != a[i] % 2)
        {
            ans += a[i + 1];
        }
        else
        {
            sum = max(sum, ans);
            if (a[i] > 0)
            {
                ans = a[i];
            }
            else
            {
                ans = 0;
            }
        }
    }
    cout << sum << "\n";
}

int main()
{
    // Fast I/O
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);

    int t = 1;
    cin >> t;
    while (t--)
    {
        solve();
    }
    return 0;
}