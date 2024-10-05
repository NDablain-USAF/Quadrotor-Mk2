#ifndef MAIN_H
#define MAIN_H

#define I_XX_EST 0.013787f
#define I_XX_REAL 0.015f
#define I_YY_EST 0.013795f
#define I_YY_REAL 0.013f
#define I_ZZ_EST 0.026414f // kg-m^2
#define I_ZZ_REAL 0.028f
#define M_EST 1.409f // Estimated mass of drone in kg
#define M_REAL 1.50f // Real mass of drone in kg
#define L_EST 0.1555f // Distance from center of drone to propeller
#define L_REAL 0.16f
#define G_EST 9.81f // Acceleration due to gravity m/s^2
#define G_REAL 9.79f
#define D_T 0.001f // Integration time
#define T_F 30.0f // Final time of simulation
#define K_F 0.000003125f // Relates thrust (N) to motor speed in (rad/s)^2
#define K_T 0.000000037f // Relates torque (N-m) to motor speed in (rad/s)^2 

#define PI 3.1415f
#define D2R (PI/180.0)

double Reference[4] = {5.0,0.0,0.0,0.0}; // Height, Roll, Pitch, Yaw
const double K_Roll[3] = {4.6749,3.2607,1.0440};
const double K_Pitch[3] = {4.6750,3.2607,1.0440};
const double K_Yaw[3] = {4.7325,3.3183,1.0841};
const double K_Thrust[4] = {0.3,0.03,1.1,0.001};

struct Motor {
    double e;
    double e_int;
    double Speed;
};

struct Quadrotor{
    double Displacement[3]; // Inertial Position, x-y-z, m
    double Euler[3]; // Euler angles, roll-pitch-yaw, rad
    double Angular_Rate[3]; // Body Angular Rate, x-y-z, rad/s
    double Velocity[3]; // Body velocity, x-y-z, m/s
    double Force_Output[3];
    double Moment_Output[3];
    double Thrust_Requested;
    double Moment_Requested[3];
    struct Motor Motor1; // Front
    struct Motor Motor2; // Right 
    struct Motor Motor3; // Rear
    struct Motor Motor4; // Left
};

double Low_Pass_Filter(double Speed_Ref, double Speed_Current){
    // Motor is modeled as a low pass filter, 20/(s+20) = O_p/I_p -> O_p*(s+20) = 20*I_p...
    // O_p*s = 20*(I_p-O_p);
    double Motor_Accel = 20.0*(Speed_Ref-Speed_Current);
    return ((Motor_Accel*D_T) + Speed_Current);
}

void Control(struct Quadrotor* Drone){
    double D_T2 = 0.01;
    static double e_int[4];
    static double e_int2 = 0;
    double e[4];
    static double e_last = 0;
    // Thrust, uses PID feedback combined with feedforward control to counter gravity
    e[0] = Reference[0]-Drone->Displacement[2];
    e_int[0] += (e[0]*D_T2);
    e_int2 += (e_int[0]*D_T2);
    double Feed_Forward = M_EST*G_EST*(1/cos(Drone->Euler[0]))*(1/cos(Drone->Euler[1]));
    double Feed_Back = (K_Thrust[0]*e[0]) + (K_Thrust[1]*e_int[0]) + K_Thrust[2]*((e[0]-e_last)/D_T2) + (K_Thrust[3]*e_int2);
    Drone->Thrust_Requested = Feed_Forward + Feed_Back;
    e_last = e[0];
    // Torques, uses tracking LQR
    for (int i=1;i<4;i++){
        e[i] = Reference[i]-Drone->Euler[i-1];
        e_int[i] += (e[i]*D_T2);
    }
    Drone->Moment_Requested[0] = K_Roll[0]*e_int[1] - K_Roll[1]*Drone->Euler[0] - K_Roll[2]*Drone->Angular_Rate[0];
    Drone->Moment_Requested[1] = K_Pitch[0]*e_int[2] - K_Pitch[1]*Drone->Euler[1] - K_Pitch[2]*Drone->Angular_Rate[1];
    Drone->Moment_Requested[2] = K_Yaw[0]*e_int[3] - K_Yaw[1]*Drone->Euler[2] - K_Yaw[2]*Drone->Angular_Rate[2];
}

void Forces_Moments(struct Quadrotor* Drone){
    // Turn requested thrust and moments into motor speeds
    double Speed_ref[4];
    Speed_ref[0] = sqrt(abs((Drone->Thrust_Requested/(4*K_F)) + (Drone->Moment_Requested[2]/(4*K_T)) + (Drone->Moment_Requested[1]/(2*K_F))));
    Speed_ref[1] = sqrt(abs((Drone->Thrust_Requested/(4*K_F)) - (Drone->Moment_Requested[2]/(4*K_T)) - (Drone->Moment_Requested[0]/(2*K_F))));
    Speed_ref[2] = sqrt(abs((Drone->Thrust_Requested/(4*K_F)) + (Drone->Moment_Requested[2]/(4*K_T)) - (Drone->Moment_Requested[1]/(2*K_F))));
    Speed_ref[3] = sqrt(abs((Drone->Thrust_Requested/(4*K_F)) - (Drone->Moment_Requested[2]/(4*K_T)) + (Drone->Moment_Requested[0]/(2*K_F))));
    // Run requested motor speeds through motor speed control model
    Drone->Motor1.Speed = Low_Pass_Filter(Speed_ref[0],Drone->Motor1.Speed);
    Drone->Motor2.Speed = Low_Pass_Filter(Speed_ref[1],Drone->Motor2.Speed);
    Drone->Motor3.Speed = Low_Pass_Filter(Speed_ref[2],Drone->Motor3.Speed);
    Drone->Motor4.Speed = Low_Pass_Filter(Speed_ref[3],Drone->Motor4.Speed);
    // Output actual motor speeds as thrust and moments
    Drone->Force_Output[0] = -M_REAL*G_REAL*sin(Drone->Euler[1]); 
    Drone->Force_Output[1] = M_REAL*G_REAL*sin(Drone->Euler[0])*cos(Drone->Euler[1]);
    Drone->Force_Output[2] = (M_REAL*G_REAL*cos(Drone->Euler[0])*cos(Drone->Euler[1])) - K_F*(pow(Drone->Motor1.Speed,2)+pow(Drone->Motor2.Speed,2)+pow(Drone->Motor3.Speed,2)+pow(Drone->Motor4.Speed,2));
    Drone->Moment_Output[0] = K_F*L_REAL*(pow(Drone->Motor4.Speed,2)-pow( Drone->Motor2.Speed,2));
    Drone->Moment_Output[1] = K_F*L_REAL*(pow(Drone->Motor1.Speed,2)-pow(Drone->Motor3.Speed,2));
    Drone->Moment_Output[2] = K_T*(pow(Drone->Motor1.Speed,2)+pow(Drone->Motor3.Speed,2)-pow( Drone->Motor2.Speed,2)-pow(Drone->Motor4.Speed,2)); // In this coordinate system...
    // A clockwise rotation is positive, so we are assuming the front and back motors are turning CCW to produce...
    // A CW torque on the drone body
}

void Conservation_Of_Momentum(struct Quadrotor* Drone){
    // H = I*w -> H_dot = I*w_dot = M -> I*w_dot = M - wxI*w
    // w_dot = I^-1(M-wxI*w)
    // w: body angular rates, w_dot: body angular acceleration, I: Moment of inertia matrix, M: Applied Moment
    // P = m*v -> P_dot = m*v_dot = F -> m*v_dot = F+vxw
    // v_dot = (1/m)*(F - vxw)
    // v: body linear velocity, v_dot: body linear acceleration, m: mass, F: Applied force
    //     cross(x(7:9),x(10:12)) + 9.81.*[-sin(x(5));cos(x(5))*sin(x(4));cos(x(5))*cos(x(4))] - [0;0;u(1)/m];...
    double w_dot[3];
    w_dot[0] = (Drone->Moment_Output[0] + ((I_YY_REAL-I_ZZ_REAL)*Drone->Angular_Rate[1]*Drone->Angular_Rate[2]));
    w_dot[0] /= I_XX_REAL;
    w_dot[1] = (Drone->Moment_Output[1] + ((I_ZZ_REAL-I_XX_REAL)*Drone->Angular_Rate[0]*Drone->Angular_Rate[2]));
    w_dot[1] /= I_YY_REAL; 
    w_dot[2] = (Drone->Moment_Output[2] + ((I_XX_REAL-I_YY_REAL)*Drone->Angular_Rate[0]*Drone->Angular_Rate[1]));
    w_dot[2] /= I_ZZ_REAL;
    double v_dot[3];
    v_dot[0] = (Drone->Force_Output[0] + (Drone->Velocity[1]*Drone->Angular_Rate[2]) - (Drone->Velocity[2]*Drone->Angular_Rate[1]));
    v_dot[0] /= M_REAL;
    v_dot[1] = (Drone->Force_Output[1] - (Drone->Velocity[0]*Drone->Angular_Rate[2]) + (Drone->Velocity[2]*Drone->Angular_Rate[0]));
    v_dot[1] /= M_REAL;
    v_dot[2] = (Drone->Force_Output[2] + (Drone->Velocity[0]*Drone->Angular_Rate[1]) - (Drone->Velocity[1]*Drone->Angular_Rate[0]));
    v_dot[2] /= M_REAL;
    for (int i = 0; i<3; i++){
        Drone->Angular_Rate[i] += (w_dot[i]*D_T);
        Drone->Velocity[i] += (v_dot[i]*D_T);
    }
}

void Rotation_Update(struct Quadrotor* Drone){
    double Euler_Dot[3];
    Euler_Dot[0] = Drone->Angular_Rate[0] + (Drone->Angular_Rate[1]*sin(Drone->Euler[0])*tan(Drone->Euler[1])) + (Drone->Angular_Rate[2]*cos(Drone->Euler[0])*tan(Drone->Euler[1]));
    Euler_Dot[1] = (Drone->Angular_Rate[1]*cos(Drone->Euler[0])) - (Drone->Angular_Rate[2]*sin(Drone->Euler[0]));
    Euler_Dot[2] = (Drone->Angular_Rate[1]*sin(Drone->Euler[0])*(1/cos(Drone->Euler[1]))) + (Drone->Angular_Rate[2]*cos(Drone->Euler[0])*(1/cos(Drone->Euler[1])));
    for (int i = 0;i<3;i++){
        Drone->Euler[i] += (Euler_Dot[i]*D_T);
    }
}

void Inertial_Tracking(struct Quadrotor* Drone){
//    cos(x(5))*cos(x(6))                                                               sin(x(4))*sin(x(5))*cos(x(6))-cos(x(4))*sin(x(6))                          cos(x(4))*sin(x(5))*cos(x(6))+sin(x(4))*sin(x(6));... 
//    cos(x(5))*sin(x(6))                                                               sin(x(4))*sin(x(5))*sin(x(6))+cos(x(4))*cos(x(6))                           cos(x(4))*sin(x(5))*sin(x(6))-sin(x(4))*cos(x(6));...
//    -sin(x(5))                                                          sin(x(4))*cos(x(5))                       cos(x(4))*cos(x(5))
    double Displacement_Dot[3];
    double phi = Drone->Euler[0];
    double theta = Drone->Euler[1];
    double psi = Drone->Euler[2];
    Displacement_Dot[0] = (Drone->Velocity[0]*cos(theta)*cos(psi)) + (Drone->Velocity[1]*((sin(phi)*cos(theta)*cos(psi))-(cos(phi)*sin(psi)))) + (Drone->Velocity[2]*((cos(phi)*sin(theta)*cos(psi))+(sin(phi)*sin(psi))));
    Displacement_Dot[1] = (Drone->Velocity[0]*cos(theta)*sin(psi)) + (Drone->Velocity[1]*((sin(phi)*cos(theta)*sin(psi))+(cos(phi)*cos(psi)))) + (Drone->Velocity[2]*((cos(phi)*sin(theta)*sin(psi))-(sin(phi)*cos(psi))));
    Displacement_Dot[2] = (Drone->Velocity[0]*-sin(theta)) + (Drone->Velocity[1]*sin(phi)*cos(theta)) + (Drone->Velocity[2]*cos(phi)*cos(theta));
    for (int i = 0;i<3;i++){
        if (i==2){
            if (Drone->Displacement[2]<0){
                Drone->Displacement[2] = 0;
                Drone->Velocity[2] = 0;
            }
            else {Drone->Displacement[2] += (-Displacement_Dot[2]*D_T);}
        } // Flip sign so that height is positive
        else {Drone->Displacement[i] += (Displacement_Dot[i]*D_T);}
    }
}

void Print(struct Quadrotor* Drone){
    using namespace std;
    //cout << "Roll:" << Drone->Euler[0] << "  Pitch:" << Drone->Euler[1] << "  Yaw:" << Drone->Euler[2];
    //cout << "  X_n:" << Drone->Displacement[0] << "  X_e:" << Drone->Displacement[1] << "  Height:" << Drone->Displacement[2];
    //cout << "  Front Motor Speed:" << Drone->Motor1.Speed << "  Right Motor Speed:" << Drone->Motor2.Speed  << "  Rear Motor Speed:" << Drone->Motor3.Speed << "  Left Motor Speed:" << Drone->Motor4.Speed  << "\n";
    cout << Drone->Euler[0] << "," << Drone->Euler[1] << "," << Drone->Euler[2] << "," << Drone->Displacement[0] << "," << Drone->Displacement[1] << "," << Drone->Displacement[2];
    cout << "," << Drone->Motor1.Speed << "," << Drone->Motor2.Speed  << "," << Drone->Motor3.Speed << "," << Drone->Motor4.Speed  << "\n";
}

#endif
