#include "epigraph.hpp"

#include <iostream>

// This example solves the portfolio optimization problem in QP form

using namespace cvx;

int main()
{
    size_t n = 3; // Assets
    size_t m = 3; // Factors

    // Set up problem data.
    double gamma = 0.5;          // risk aversion parameter
    Eigen::VectorXd q(n);
    q << -22, -14.5, 13;    // vector of expected returns
    double r = 15; 
    Eigen::MatrixXd P(n, n);
    P << 13, 12, -2, 12, 17, 6, -2, 6, 12; // asset return covariance

    //Eigen::MatrixXd P(n, m);     // factor-loading matrix
    //Eigen::VectorXd D(n);        // diagonal of idiosyncratic risk


    //q.setRandom();
    //P.setRandom();
    //D.setRandom();

    //q = q.cwiseAbs();
    //P = P.cwiseAbs();

    OptimizationProblem qp;

    VectorX x = qp.addVariable("x", n);

    //qp.addConstraint(greaterThan(x, -1.));

    //qp.addConstraint(lessThan(x, 2.));

    qp.addConstraint(box(-10, x, 10.));

    qp.addCostTerm( x.transpose() * par( gamma * P) * x + dynpar(q).dot(x) + r);

    std::cout << qp << "\n";

    // Create and initialize the solver instance.
    osqp::OSQPSolver solver(qp);

    // Print the canonical problem formulation for inspection
    std::cout << solver << "\n";

    // Solve problem and show solver output
    const bool verbose = true;
    solver.solve(verbose);

    std::cout << "Solver message:  " << solver.getResultString() << "\n";
    std::cout << "Solver exitcode: " << solver.getExitCode() << "\n";

    // Call eval() to get the variable values
    std::cout << "Solution:\n" << eval(x) << "\n";

    // Update data
    // q.setRandom();
    // q = q.cwiseAbs();

    // // Solve again
    // // OSQP will warm start automatically
    // solver.solve(verbose);

    // std::cout << "Solution after changing the cost function:\n" << eval(x) << "\n";


    //qp.addConstraint(equalTo(x.sum(), 1.));
    //D = D.cwiseAbs();

    //Sigma = F * F.transpose();
    //Sigma.diagonal() += D;

    // Formulate QP.
    

    // Declare variables with...
    // addVariable(name) for scalars,
    // addVariable(name, rows) for vectors and
    // addVariable(name, rows, cols) for matrices.
    

    // Available constraint types are equalTo(), lessThan(), greaterThan() and box()
    
    

    // Make q dynamic in the cost function so we can change it later
    

    // Print the problem formulation for inspection
}