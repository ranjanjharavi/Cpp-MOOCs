// C++ Program to find the integer sum in vectors.

#include <iostream>
#include <vector> //required to use vector
using namespace std;

// stores the length of vector
const int N = 40;

// inline function to calculate sum and changes the value using call by reference
// n is length of the vector
inline void sum(int &p, int n, const vector<int> d)
{
    for (int i = 0; i < n; ++i)
        p += d.at(i); 
}

int main(void)
{
    int accum = 0;
    vector<int> data;

    // Process values to vector using predfined pushback method
    for (int i = 0; i < N; ++i)
        data.push_back(i);

    // passing the variable to be used as call by reference
    sum(accum, N, data);

    cout << "sum is " << accum << endl;

    return 0;
}
