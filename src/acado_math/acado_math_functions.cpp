#include <acado_math/acado_math_functions.h>

Expression atan2( Expression y, Expression x) {
    Expression result;
    result = 2*atan(y/(sqrt(x*x + y*y)+x));
    // result = 2*atan((sqrt(x*x + y*y)-x)/y);
    return result;
}

Expression tanh( Expression x ) {
    Expression y;
    y = (exp(2*x) - 1)/(exp(2*x) + 1);
    return y;
}

// Calculates the difference between the angles x and y
// This function subtracts x from y with the result wrapped on the interval [-pi,pi]
Expression angdiff( Expression x, Expression y ) {
    Expression result;
    result = atan2(sin(x-y), cos(x-y));
    return result;
}

// Wrap angle in radians to [−pi pi]
Expression wrapToPi( Expression x ) {
    Expression result;
    result = atan2(sin(x),cos(x));
    return result;
}

// Known as the logistic function
// where k is the sharpness parameter
// approaches the unit step function when k tends to infinity
Expression sigmoid( Expression x , double k ) {
    Expression y;
    y = 1/(1 + exp(-k*x));
    return y;
}

// Known as the analytic function, or SmoothReLU function
// where k is the sharpness parameter
// approaches the unit ramp function when k tends to infinity
Expression softplus( Expression x, double k ) {
    Expression y;
    y = log(1 + exp(k*x))/k;
    return y;
}

// Computes the absolute value of x using a smooth approximation and a smoothing parameter a
// the smaller the value of smoothing parameter a, the better the approximation
// Uses square root smoothing by default
Expression abs( Expression x, double alpha, SmoothApproximation f ) {
    Expression y;
    switch(f)
    {
        case SQRT:
            y = sqrt(pow(x,2)+alpha);
            break;
        case TANH:
            y = x*tanh(x/alpha);
            break;
        case ATAN:
            y = 2/M_PI*x*atan(x/alpha);
            break;
    }
    return y;
}

// Compute the log of the sum of exponentials of x and y
// Smooth approximation of the maximum function
// the larger the value of the sharpness parameter k, the better the approximation
Expression logsumexp( Expression x, Expression y, double k ) {
    Expression result;
    result = log(exp(k*x) + exp(k*y))/k;
    return result;
}

// Smooth approximation of the maximum function
// the smaller the value of smoothing parameter a, the better the approximation
Expression max( Expression x, Expression y, double alpha ) {
    Expression result;
    result = 0.5*(x + y + abs(x-y,alpha,SQRT));
    return result;
}

// Smooth approximation of the minimum function
// the smaller the value of smoothing parameter a, the better the approximation
Expression min( Expression x, Expression y, double alpha ) {
    Expression result;
    result = 0.5*(x + y - abs(x-y,alpha,SQRT));
    return result;
}
