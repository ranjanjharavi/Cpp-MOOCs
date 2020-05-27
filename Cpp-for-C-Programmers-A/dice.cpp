#include <iostream>
#include <cstdlib>
#include <ctime>

using namespace std;

const int sides = 6;
inline int r_sides() { return rand() % sides + 1; }

int main(void)
{
    const int n_dice = 2;
    srand(clock());

    int trials;
    cout << "Enter Number of trials : ";
    cin >> trials;
    cout << "Trials = " << trials << endl;

    int size = n_dice * sides + 1;
    int *outcomes = new int[size];

    for (int i = 0; i < trials; ++i)
    {
        int roll = 0;
        for (int j = 1; j <= n_dice; ++j)
            roll += r_sides();
        cout << "Roll " << roll << endl;
        outcomes[roll]++;
    }

    cout << "Data of the outcomes :" << endl;
    for (int i = 0; i < size; ++i)
        cout << outcomes[i] << " , ";

    cout << endl
         << "probability\n";
    for (int j = 2; j < n_dice * sides + 1; ++j)
        cout << "j = " << j << " p = "
             << static_cast<double>(outcomes[j]) / trials
             << endl;

    return 0;
}
