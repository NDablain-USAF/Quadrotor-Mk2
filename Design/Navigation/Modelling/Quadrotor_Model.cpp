#include <iostream>
#include <cstdio>
#include <math.h>
#include "main.h"

using namespace std;

int main(){
    freopen("output.csv","w",stdout);
    struct Quadrotor Drone;
    // Initial conditions
    Drone.Euler[0] = 5*D2R;
    Drone.Euler[1] = -15*D2R; 
    unsigned long long i = 0;
    unsigned char j = 0;
    while (i++<(T_F/D_T)){
        if ((i*D_T)>15){Reference[0] = 0.0;}
        if (++j>9){
            Control(&Drone);
            j = 0;
        }
        Forces_Moments(&Drone);
        Conservation_Of_Momentum(&Drone);
        Rotation_Update(&Drone);
        Inertial_Tracking(&Drone);
        Print(&Drone);
    }

    return 0;
}
