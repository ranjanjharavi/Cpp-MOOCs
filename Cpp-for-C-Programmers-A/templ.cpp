#include <iostream>
using namespace std;

template <class summable>
summable sum(summable arr[], int length, summable s = 0) { 
    for (int i = 0; i < length; i++)
        s += arr[i];
    return s;
}

int main(void) {
    int arr[4] = {4, 5, 6, 7};
    int length = sizeof(arr)/sizeof(arr[0]);

    cout << sum(arr, length, 2) << endl;
    
    return 0;
}