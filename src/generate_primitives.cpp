/*
    Example code for motion primitive generation using ACADO toolkit.

    References:
        1. Ship model:
           Equation (4.6), Optimization-based Solutions to Constrained
           Trajectory-tracking and Path-following Problems
        2. Objective function:
           Equation (4), Lattice-based Motion Planning for a General 2-trailer
           system
*/

#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <acado_math/acado_math_functions.h>


int main( ){

    USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // -------------------------

    DifferentialState x1, x2  ; // x, y positions
    DifferentialState x3, x4  ; // yaw angle, yaw rate
    DifferentialState w       ; // ship speed
    Control           u       ; // rudder angle
    Control           a       ; // ship acceleration

    const double L    = 1.0   ; // minimum turning radius (?)
    const double K    = 1.0   ; // control gain
    const double tau  = 1.0   ; // time constant (?)
    const double umax = M_PI/6; // max rudder deflection
    const double wmax = 15.0  ; // max ship speed

    double T         = 20.0   ; // total time
    int step         = 20     ; // number of time steps

    DifferentialEquation  f;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    f << dot(x1) == w*cos(x3) - L*w*x4*sin(x3);
    f << dot(x2) == w*sin(x3) + L*w*x4*cos(x3);
    f << dot(x3) == x4                        ;
    f << dot(x4) == 1/tau*(-x4 + K*u)         ;
    f << dot(w)  == a                         ;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------

    OCP ocp( 0, T, step );

    ocp.minimizeLagrangeTerm( 10.0*x4*x4 + a*a );
    ocp.subjectTo( f );

    ocp.subjectTo( AT_START, x1 ==  0.0 ); // start x position
    ocp.subjectTo( AT_START, x2 ==  0.0 ); // start y position
    ocp.subjectTo( AT_START, x3 ==  0.0 );
    ocp.subjectTo( AT_START, x4 ==  0.0 );
    ocp.subjectTo( AT_START, w  ==  3.0 ); // start ship speed
    ocp.subjectTo( AT_START, u  ==  0.0 );
    ocp.subjectTo( AT_START, a  ==  0.0 );

    ocp.subjectTo( AT_END  , x1 == 50.0 ); // goal x position
    ocp.subjectTo( AT_END  , x2 == 30.0 ); // goal y position
    // ocp.subjectTo( AT_END  , x3 == 0.0  );
    ocp.subjectTo( AT_END  , x4 == 0.0  );
    ocp.subjectTo( AT_END  , w  == 3.0  ); // goal ship speed
    ocp.subjectTo( AT_END  , u  == 0.0  );
    ocp.subjectTo( AT_END  , a  == 0.0  );

    ocp.subjectTo( -umax <= u <= umax )  ; // rudder constraint
    ocp.subjectTo(  0.0  <= w <= wmax )  ; // speed constraint
    ocp.subjectTo( a >= 0.0 )            ; // acceleration constraint


    // VISUALIZE THE RESULTS IN A GNUPLOT WINDOW:
    // ------------------------------------------

    GnuplotWindow window;
        window.addSubplot( x2, x1, "POSITION", "y", "x"  ); // NED
        window.addSubplot( x3,     "YAW ANGLE"           );
        window.addSubplot( x4,     "YAW RATE"            );
        window.addSubplot( w ,     "SHIP SPEED"          );
        window.addSubplot( u ,     "RUDDER CONTROL"      );
        window.addSubplot( a ,     "ACCELERATION CONTROL");


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------

    OptimizationAlgorithm algorithm(ocp);

    algorithm.set( MAX_NUM_ITERATIONS, 20 );
    algorithm << window;
    algorithm.solve();


    // PRINT THE RESULTS TO TEXT FILES:
    // ---------------------------------------------------

    algorithm.getDifferentialStates("primitive_states.txt");
//     algorithm.getParameters("primitive_pars.txt");
    algorithm.getControls("primitive_controls.txt");

    return 0;
}
