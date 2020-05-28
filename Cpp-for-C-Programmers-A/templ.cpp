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
    double doubleArr[] = {1.1, 1.2, 1.3, 1.4, 1.5 };

    int length = sizeof(arr)/sizeof(arr[0]);
    int len = sizeof(doubleArr)/sizeof(arr[0]);
    cout << sum(arr, length, 2) << endl;    
    cout << sum(doubleArr, len) << endl;
    
    return 0;
}