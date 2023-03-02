#include "epigraph.hpp"

#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

// This example solves the portfolio optimization problem in QP form

using namespace cvx;

int main()
{
    size_t n_0 = 2; // Assets
    size_t m = 5; // Factors

    //Eigen::MatrixXd kroneq(n_0 * m, n_0);
    //kroneq << 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1;

    double gamma = 0.5;
    double g = 0.;
    double l = 1.3;
    double C = 10.;
    double epsilon = 0.01;

    Eigen::VectorXd Y(n_0 * m);
    Y << 2, 3,  8,  6,  0,  1, 5, 6,  0, 2;

    Eigen::VectorXd epsilons(m);
    epsilons << 0.01, 0.01, 0.01, 0.01, 0.01;

    Eigen::MatrixXd K(n_0 * m, n_0 * m);

    K << 1,0,0.00486619098275701,0,0.169458379837583,0,2.36798146806657e-05,0,0.0518925584448117,0,0,1,0,0.00486619098275701,0,0.169458379837583,0,2.36798146806657e-05,0,0.0518925584448117,0.00486619098275701,0,1,0,0.000252519099976334,0,1.31038621539531e-05,0,7.25137445817965e-06,0,0,0.00486619098275701,0,1,0,0.000252519099976334,0,1.31038621539531e-05,0,7.25137445817965e-06,0.169458379837583,0,0.000252519099976334,0,1,0,3.76292372876303e-07,0,0.0287161424971784,0,0,0.169458379837583,0,0.000252519099976334,0,1,0,3.76292372876303e-07,0,0.0287161424971784,2.36798146806657e-05,0,1.31038621539531e-05,0,3.76292372876303e-07,0,1,0,0.00149015410284437,0,0,2.36798146806657e-05,0,1.31038621539531e-05,0,3.76292372876303e-07,0,1,0,0.00149015410284437,0.0518925584448117,0,7.25137445817965e-06,0,0.0287161424971784,0,0.00149015410284437,0,1,0,0,0.0518925584448117,0,7.25137445817965e-06,0,0.0287161424971784,0,0.00149015410284437,0,1;
    //q.setRandom();
    //P.setRandom();
    //D.setRandom();

    //q = q.cwiseAbs();
    //P = P.cwiseAbs();
    //std::cout << "chegou";
    OptimizationProblem qp;


    VectorX a = qp.addVariable("a", m);
    VectorX u = qp.addVariable("u", n_0 * m);
    Scalar t = qp.addVariable("t");

    //qp.addConstraint(greaterThan(x, -1.));

    //qp.addConstraint(lessThan(x, 2.));

    //qp.addConstraint(box(par(-C), u, par(C) ));

    //qp.addConstraint(lessThan( u.norm(), par(g)));
    //qp.addConstraint(equalTo(u.norm(), par(g)) );
    //qp.addConstraint(equalTo(u, 1.));
    //qp.addConstraint(equalTo(u.dot(kroneq), par(g)))

    //qp.addConstraint(lessThan( (u.reshaped(5, 2).rowwise().norm() - alpha), par(g)));

    //qp.addConstraint(lessThan( (u.reshaped(5, 2).rowwise().norm() - a), par(g)));

    //qp.addConstraint(lessThan( u.reshaped(5, 2).rowwise().norm() - a ,par(g)));



    //for (size_t i = 0; i < m; i++)
    //{
        //qp.addConstraint(lessThan(u.reshaped(5, 2).rowwise().norm()[i] - a[i], 0.));

        //qp.addConstraint(lessThan((u.reshaped(5, 2).rowwise()).norm()[i],
        //                             a[i]));

        //qp.addConstraint(lessThan(u(i,i+n_0) - a[i], 0. ));
    //}

    //qp.addCostTerm( u.transpose() * par(gamma * K) * u - par(Y).dot(u)  + par(epsilon) * a.sum() );
    //qp.addConstraint(lessThan( (u.transpose() * par(gamma * K) * u - par(Y).dot(u)  + par(epsilon) * a.sum()) - t, par(g)   ));

    //qp.addConstraint(lessThan( u.transpose() * par(gamma * K) * u - par(Y).dot(u)  + par(epsilon) * a.sum(), 1. - t )  );
    //qp.addConstraint(lessThan( - par(Y).dot(u) + par(epsilon) * a.sum() ,  t )  );

    //qp.addConstraint(greaterThan(t, u.transpose() * par(gamma * K) * u - par(Y).dot(u) + par(epsilon) * a.sum() )  );
    //qp.addCostTerm(t);

    //qp.addCostTerm(u.transpose() * par(gamma * K) * u - par(Y).dot(u) + par(epsilon) * a.sum());


    //qp.addCostTerm((par(K).transpose() * u).norm() - par(Y).dot(u) + par(epsilon) * a.sum());

    //qp.addConstraint(lessThan((par(K).transpose() * u).norm() - par(Y).dot(u) + par(epsilon) * a.sum(), t));


    //tenta escrever como lista
    //qp.addConstraint(lessThan(( par(K).transpose() * u).norm() - par(Y).dot(u) + par(epsilon) * a.sum(), t));


    // for (size_t i = 0; i < m; i++)
    // {
    //     socp.addConstraint(lessThan((par(A[i]) * x + par(b[i])).norm(),
    //                                 par(c[i]).dot(x) + par(d[i])));
    // }

    //qp.addConstraint(lessThan((par(K.cwiseSqrt()).cwiseProduct(u)).norm() - par(Y).dot(u) + par(epsilon) * a.sum(), t));

    //qp.addCostTerm(t);

    //qp.addCostTerm((par((1/gamma) * K).transpose() * u).norm() - par(Y).dot(u) + par(epsilon) * a.sum());

    //qp.addConstraint(lessThan(u.reshaped(2, 5).transpose().rowwise().norm() - a, par(g)));
    //qp.addConstraint(lessThan(u.reshaped(2, 5).transpose().rowwise().norm(), a ));

    qp.addConstraint(box(par(g), a, par(C) ));
    qp.addConstraint( equalTo(u.reshaped(2, 5).rowwise().sum(), par(g)));
    qp.addConstraint( lessThan(u.reshaped(2, 5).transpose().rowwise().norm(), a ));


    //qp.addCostTerm( u.transpose() * par(gamma * K ) * u - par(Y).dot(u) + par(epsilon) * a.sum());

    //qp.addConstraint( lessThan(  u.transpose() * par(gamma * K ) * u - par(Y).dot(u) + par(epsilon) * a.sum(), t) );
    // u.transpose() * par(gamma * K ) * u

    qp.addConstraint( lessThan( - par(Y).dot(u) + par(epsilon) * a.sum(), t)); 
    //qp.addConstraint( lessThan( (K) * u).norm(), 2 )); 
    //qp.addConstraint( lessThan( (par(K.sqrt()) * u - par(K.sqrt() * Y) ).norm() + par(epsilon) * a.sum(), t ));
    qp.addConstraint( lessThan( (par((gamma * K).sqrt()) * u - par((gamma * K).sqrt() * Y) ).norm() + par(epsilon) * a.sum(), t ));

    qp.addCostTerm(t);
    std::cout << qp << "\n";

    // Create and initialize the solver instance.

    //osqp::OSQPSolver solver(qp);
    cvx::ecos::ECOSSolver solver(qp);

    //osqp::OSQPSolver solver(qp);

    // Print the canonical problem formulation for inspection
    std::cout << solver << "\n";

    // Solve problem and show solver output
    const bool verbose = true;

    
    solver.solve(verbose);
    //solver.setMaxIter(1000);


    // void setAlpha(c_float alpha);
    // void setDelta(c_float delta);
    // void setEpsAbs(c_float eps);
    // void setEpsPinf(c_float eps);
    // void setEpsDinf(c_float eps);
    // void setEpsRel(c_float eps);
    // void setMaxIter(c_int iter);
    // void setPolish(bool polish);
    // void setPolishRefine(c_int iter);
    // void setRho(c_float rho);
    // void setScaledTermination(bool scaled_termination);
    // void setTimeLimit(c_float t);
    // void setCheckTermination(c_int interval);
    // void setWarmStart(bool warm_start);

    Eigen::VectorXd u_sol = eval(u);
    Eigen::VectorXd a_sol = eval(a);

    std::cout << "U:\n" << eval(u_sol) << std::endl;

    std::cout << "A:\n" << eval(a_sol) << std::endl;

    std::cout << "Solver message:  " << solver.getResultString() << "\n";
    std::cout << "Solver exitcode: " << solver.getExitCode() << "\n";
    std::cout << "Solver info : " << &solver.getInfo() << "\n";

    // Call eval() to get the variable values
    std::cout << "Solution:\n" << eval(u) << std::endl;
    std::cout <<  "u_reshaped:\n" << eval(u.reshaped(5, 2)) << std::endl;
    //std::cout <<  "u_reshaped:\n" << eval(u.reshaped(5, 2).rowwise().norm() - a) << std::endl;
    //std::cout <<  "norm:\n" << eval(u.reshaped(5, 2)(0)) << std::endl;
    std::cout <<  "norm equal to zero:\n" << eval(u.reshaped(2, 5).rowwise().sum()) << std::endl;
    std::cout << "Alpha:\n" << eval(a) <<  std::endl;
    //std::cout << "less than zero:\n" << eval(u.reshaped(5, 2).rowwise().norm() - a) <<  std::endl;
    //std::cout << "less than zero:\n" << eval(u.reshaped(2, 5).colwise().norm() ) <<  std::endl;

    std::cout << "u_reshaped:\n" << eval(u.reshaped(2, 5).transpose().rowwise().norm() ) <<  std::endl;

    std::cout << "u_reshaped - alpha:\n" << eval(u.reshaped(2, 5).transpose().rowwise().norm() - a ) <<  std::endl;
    std::cout << "u_reshaped - alpha:\n" << eval(u.reshaped(2, 5).transpose().rowwise().norm() ) <<  std::endl;
    std::cout << "k^{1/2}:\n" << K.sqrt() <<  std::endl;
    //std::cout << "k^{1/2} * u :\n" << u.transpose() * K.sqrt()  <<  std::endl;
    //std::cout << "|| k^{1/2} * u || :\n" << (K.sqrt() * u).norm() <<  std::endl;

    const double pi = std::acos(-1.0);
    
    Eigen::MatrixXd A(2,2);
    A << cos(pi/3), -sin(pi/3), 
        sin(pi/3),  cos(pi/3);

    Eigen::VectorXd x(2);
    x << cos(pi/3), -sin(pi/3);

    std::cout << "The matrix A is:\n" << A << "\n\n";
    std::cout << "The matrix square root of A is:\n" << A.sqrt() << "\n\n";
    std::cout << "The square of the last matrix is:\n" << A.sqrt() * A.sqrt() << "\n";
    std::cout << "The square of the last matrix is x:\n" << A.sqrt() * x << "\n";

    std::cout << "The square of the last matrix is x:\n" << (K.sqrt() * u_sol).norm() << "\n";
    std::cout << "Objective funcion is:\n" << (K.sqrt() * u_sol).norm()  + epsilon * a_sol.sum() << "\n";
   // std::cout << "||k^{1/2} * u ||:\n" << eval(K.sqrt()) <<  std::endl;
    //std::cout << "less than zero:\n" << eval(u(2) ) <<  std::endl;
}


//C++ = 0.002433 s
// real    0m0.078s
// user    0m0.029s
// sys     0m0.049s
//MATLAB = 0.49 s