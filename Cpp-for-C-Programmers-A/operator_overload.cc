#include <iostream>

using namespace std;

typedef enum Days{SUN, MON, TUE, WED, THU, FRI, SAT} days;

inline Days operator++(Days& d, int) { return d  = static_cast<Days>(d + 1 % 7); }

int main(void)
{
    days day = MON;
    
    cout << "Operator overloading : ";
    cout << day++ << " " << day++ << " " << day++ << endl; 

    return 0;
}