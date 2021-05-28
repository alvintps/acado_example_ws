#pragma once

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>

USING_NAMESPACE_ACADO

enum SmoothApproximation{
    SQRT,  // smooth approximation result will be greater than |x|
    TANH,  // smooth approximation result will be less than |x|
    ATAN   // smooth approximation result will be less than |x|
};

Expression atan2( Expression y, Expression x);
Expression tanh( Expression x );
Expression angdiff( Expression x, Expression y );
Expression wrapToPi( Expression x );
Expression sigmoid( Expression x , double k = 1);
Expression softplus( Expression x, double k = 1);
Expression abs( Expression x, double alpha = 1.0, SmoothApproximation f = SQRT );
Expression logsumexp( Expression x, Expression y, double k = 1.0 );
Expression max( Expression x, Expression y, double alpha = 1.0e-6 );
Expression min( Expression x, Expression y, double alpha = 1.0e-6 );
