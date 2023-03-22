#include "epigraph.hpp"

#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/KroneckerProduct>

using namespace cvx;

// double rbf_kernel(Eigen::VectorXd x1, Eigen::VectorXd x2, double l){
//     return std::exp(- (std::pow((x1-x2).norm(), 2) / (std::pow(l,2))));
// }

// Eigen::MatrixXd create_K_RBF(Eigen::MatrixXd X, int m, int n_0, double l, Eigen::MatrixXd K, std::string kron){
//     //Eigen::MatrixXd K(n_0 * m, n_0 * m);
//     for (int i=0; i < m; i++){
//         for (int j=0; j < m; j++){
//             //std::cout << "X:\n" <<  X.row(i) << "|" << X.row(j) <<  std::endl;
//             //std::cout << "X:\n" <<  rbf_kernel(X.row(i), X.row(j), l) <<  std::endl;
//             K(i, j) = rbf_kernel(X.row(i), X.row(j), l);
//         }
//     }
    
//     //std::cout << "k:\n" <<  K <<  std::endl;
//     //std::cout << "k:\n" << Eigen::KroneckerProduct(K, Eigen::MatrixXd::Identity(n_0, n_0))<<  std::endl;
//     if (kron == "yes"){
//         return Eigen::KroneckerProduct(K, Eigen::MatrixXd::Identity(n_0, n_0)) ;
//     }
//     else {
//         return K;
//     }
    
// }
int main(){    
    size_t n_0 = 2; 
    size_t m = 5;


    Eigen::MatrixXd Y(m, n_0);
    Y << 2, 3,  
        8, 6,  
        0, 1, 
        5, 6,  
        0, 2;

    Eigen::MatrixXd K(m, n_0);
    //K << -0.162214,  0.899715,   -0.7394,  0.696136, -0.902747,
    //  -0.12489,  0.436021,  -0.71049,   0.71595, -0.487018;


    K << -0.162214, -0.12489, 
          0.899715,  0.436021,
          -0.7394, -0.71049,
          0.696136,  0.71595,
          -0.902747, -0.487018;
    //K = K.transpose();
    //K = K.reshaped(m, n_0);
    std::cout << "Y:\n" << Y << "\n";
    std::cout << "K:\n" << K << "\n";
    std::cout << "Y - K:\n" << Y - K << "\n";

    OptimizationProblem solveb;

    // Scalar t1 = solveb.addVariable("t1");
    // Scalar t2 = solveb.addVariable("t2");
    // Scalar t3 = solveb.addVariable("t3");
    // Scalar t4 = solveb.addVariable("t4");
    // Scalar t5 = solveb.addVariable("t5");

    Eigen::VectorXd q_ones = Eigen::VectorXd::Ones(m);

    VectorX t = solveb.addVariable("t", m); 

    VectorX b = solveb.addVariable("b", n_0); 


    // solveb.addConstraint( lessThan(  (-b + par(Y.row(0).transpose())).norm(), t1) );
    // solveb.addConstraint( lessThan(  (-b + par(Y.row(1).transpose())).norm(), t2) );
    // solveb.addConstraint( lessThan(  (-b + par(Y.row(2).transpose())).norm(), t3) );
    // solveb.addConstraint( lessThan(  (-b + par(Y.row(3).transpose())).norm(), t4) );
    // solveb.addConstraint( lessThan(  (-b + par(Y.row(4).transpose())).norm(), t5) );
    // solveb.addCostTerm( t1+ t2+t3+t4+t5);

    for (int i=0; i < m; i++){
        solveb.addConstraint( lessThan(  (-b + par(Y.row(i).transpose() - K.row(i).transpose())).norm(), t(i)) );
    }
    solveb.addCostTerm(t.dot(par(q_ones)));

    cvx::ecos::ECOSSolver solverb(solveb);
    std::cout << solverb << "\n";

    solverb.solve(true);

    std::cout << "Solver message: " << solverb.getResultString() << "\n";

    std::cout << "b:\n" <<  eval(b) << std::endl;
    // double gamma = 0.5;
    // double g = 0.;
    // double l = 1.3;
    // double C = 1.;
    // double epsilon = 0.01;

    // size_t n_1 = 3; 
    // size_t n_0 = 2; 
    // size_t m = 5;

    // Eigen::VectorXd Y(n_0 * m);
    // Y << 2, 3,  8,  6,  0,  1, 5, 6,  0, 2;
    
    // Eigen::MatrixXd X(m, n_1);
    // X << 1,2, 3, 
    //     2,4,1,
    //     0,1,2,
    //     5,1,2,
    //     2,0,3;

    // std::cout << "X:\n" << X << "\n";

    // Eigen::MatrixXd K(m, m);
    // K = create_K_RBF(X, m, n_0, l, K, "yes");

    // Eigen::MatrixXd K_pad = Eigen::MatrixXd::Zero(n_0 * m + m, n_0 * m + m);
    // // K_pad(1, 1) = 3;
    

    // Eigen::MatrixXd eps_diag = Eigen::MatrixXd::Zero( m, m);
    // Eigen::VectorXd epsilons = Eigen::VectorXd::Ones(m) * epsilon;
    // //epsilons << epsilon;
    // std::cout << "epsilons:\n" <<  epsilons <<  std::endl;
    // eps_diag.diagonal() << epsilons;

    
    // K_pad.bottomRightCorner(m,m) = eps_diag;

    
    // std::cout << "k:\n" <<  K.row(0) <<  std::endl;
    
    // K_pad.topLeftCorner(n_0 * m, n_0 * m) = K;

    // std::cout << "k_pad:\n" <<  K_pad <<  std::endl;

    // Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(m);

    // Eigen::VectorXd q(Y.size() + q_zero.size());
    // q << -Y, q_zero;

    // OptimizationProblem qp;
    // VectorX x = qp.addVariable("x", n_0 * m + m);
    // Scalar t = qp.addVariable("t");

    // qp.addConstraint( box(par(g), x.tail(m), par(C) ));
    // qp.addConstraint( equalTo(x.head(m*n_0).reshaped(n_0, m).rowwise().sum(), par(g)));
    // qp.addConstraint( lessThan(x.head(m*n_0).reshaped(n_0, m).transpose().rowwise().norm(), x.tail(m) ));
    // qp.addConstraint( lessThan(  (par((K_pad).sqrt()) * x + par(gamma * (K_pad.sqrt().inverse()) * q)).norm() , t ));
    // qp.addCostTerm(t);

    // cvx::ecos::ECOSSolver solver(qp);
    // std::cout << solver << "\n";
    // const bool verbose = true;

    // solver.solve(verbose);

    // Eigen::VectorXd x_sol = eval(x);

    // std::cout << "X:\n" << eval(x_sol) << std::endl;
    // std::cout << "Objective funcion is:\n" <<  x_sol.head(m*n_0).transpose() * gamma * K  * x_sol.head(m*n_0) - Y.dot(x_sol.head(m*n_0)) + epsilon * x_sol.tail(m).sum() << "\n";
    // std::cout << "Solution:\n" << eval(x_sol) << std::endl;
    // std::cout <<  "u_reshaped:\n" << eval(x_sol.head(m*n_0).reshaped(m, n_0)) << std::endl;
    // std::cout <<  "norm equal to zero:\n" << eval(x_sol.head(m*n_0).reshaped(n_0, m).rowwise().sum()) << std::endl;
    // std::cout << "Alpha:\n" << x_sol.tail(m) <<  std::endl;
    // std::cout << "u_reshaped - alpha:\n" << eval(x_sol.head(m*n_0).reshaped(n_0, m).transpose().rowwise().norm() - x_sol.tail(m) ) <<  std::endl;
    // std::cout << "u_reshaped rowise norm:\n" << eval(x_sol.head(m*n_0).reshaped(n_0, m).transpose().rowwise().norm() ) <<  std::endl;

    // Eigen::MatrixXd K_new(m, m);
    // K_new = create_K_RBF(X, m, n_0, l, K_new, "no");

    // std::cout << "K_new:\n" << K_new << std::endl;
    // std::cout << "u * K_new:\n" << x_sol.head(m*n_0).reshaped(n_0, m) * K_new << std::endl;
    // std::cout << "y transpose:\n" << Y.reshaped(n_0, m).transpose()   << std::endl;
    // std::cout << "y transpose - u * K_new:\n" << Y.reshaped(n_0, m).transpose() -  (x_sol.head(m*n_0).reshaped(n_0, m) * K_new).transpose()   << std::endl;
    // std::cout << "norm(y.T - u * K_new) :\n" << (Y.reshaped(n_0, m).transpose() -  (x_sol.head(m*n_0).reshaped(n_0, m) * K_new).transpose()).rowwise().norm()   << std::endl;

    // std::cout << "sum(norm(y.T - u * K_new)) :\n" << ((Y.reshaped(n_0, m).transpose() -  (x_sol.head(m*n_0).reshaped(n_0, m) * K_new).transpose()).rowwise().norm()).sum()   << std::endl;

    

    // Eigen::VectorXd Y2(n_0);
    // Y2 << 2, 3;
    // std::cout << "Y2 :\n" << Y2.replicate(1, 5)  << std::endl;




    // // std::cout << "b0:\n" << eval(b.replicate(1, m)) << std::endl;
    // // std::cout << "b1:\n" << Y2.replicate(1, 5)  - eval(b.replicate(1, m)) << std::endl;
    // // std::cout << "b2:\n" << Y2.replicate(1, 5)  - eval(b.replicate(1, m)) -  (x_sol.head(m*n_0).reshaped(n_0, m) * K_new) << std::endl;
    // // std::cout << "b3:\n" << Y.reshaped(n_0, m) - eval(b.replicate(1, m))   << std::endl;
    // // std::cout << "b4:\n" <<  (Y.reshaped(n_0, m) - eval(b.replicate(1, m)) -  (x_sol.head(m*n_0).reshaped(n_0, m) * K_new)  ).rowwise().norm().sum() << std::endl;
    // // std::cout << "b5:\n" <<  Y.reshaped(n_0, m).transpose().row(0)  << std::endl;
    // // std::cout << "b6:\n" <<  Y.reshaped(n_0, m).transpose().row(0).transpose() - eval(b)  << std::endl;
    // // std::cout << "b7:\n" <<  eval(b)  << std::endl;

    // //solveb.addConstraint( lessThan( ( par(Y.reshaped(n_0, m)) - b.replicate(1, m) -  par(x_sol.head(m*n_0).reshaped(n_0, m) * K_new)  ).rowwise().norm().sum() , t2 ));
    
    // //solveb.addCostTerm(   ( par(Y.reshaped(n_0, m)) - b.replicate(1, m)).rowwise().norm().sum());
    
    // //solveb.addCostTerm(   (par(Y.reshaped(n_0, m)) - b.replicate(1, m) - par(x_sol.head(m*n_0).reshaped(n_0, m) * K_new)).rowwise().norm().sum());
    
    // OptimizationProblem solveb;

    // // Scalar t2 = solveb.addVariable("t2");
    // // Scalar t3 = solveb.addVariable("t3");
    // // Scalar t4 = solveb.addVariable("t4");
    // // Scalar t5 = solveb.addVariable("t5");
    // // Scalar t6 = solveb.addVariable("t6");

    // VectorX t1 = solveb.addVariable("t1", m); 
    // VectorX b = solveb.addVariable("b", n_0);  
    // Eigen::VectorXd q_ones = Eigen::VectorXd::Ones(m);
    // //solveb.addCostTerm(   par(Y.reshaped(n_0, m).transpose().row(0).transpose()) - b );

    // std::cout << "b7:\n" <<  Y2 - eval(b) << std::endl;
    // std::cout << "b7:\n" << (x_sol.head(m*n_0).reshaped(n_0, m) * K_new).transpose().row(0).transpose()  << std::endl;
    // //std::cout << "b7:\n" <<  Y.reshaped(n_0, m).transpose().row(0).transpose() - eval(b) << std::endl;
    // std::cout << "b8:\n" <<  Y.reshaped(n_0, m).transpose() << std::endl;

    // // solveb.addCostTerm(  ( par(Y.reshaped(n_0, m).transpose().row(0).transpose() - (x_sol.head(m*n_0).reshaped(n_0, m) * K_new).transpose().row(0).transpose() ) - b).norm() +
    // //                      ( par(Y.reshaped(n_0, m).transpose().row(1).transpose() - (x_sol.head(m*n_0).reshaped(n_0, m) * K_new).transpose().row(1).transpose() ) - b).norm() +
    // //                       ( par(Y.reshaped(n_0, m).transpose().row(2).transpose() - (x_sol.head(m*n_0).reshaped(n_0, m) * K_new).transpose().row(2).transpose() ) - b).norm() +
    // //                        ( par(Y.reshaped(n_0, m).transpose().row(3).transpose() - (x_sol.head(m*n_0).reshaped(n_0, m) * K_new).transpose().row(3).transpose() ) - b).norm() +
    // //                         ( par(Y.reshaped(n_0, m).transpose().row(4).transpose() - (x_sol.head(m*n_0).reshaped(n_0, m) * K_new).transpose().row(4).transpose() ) - b).norm() );


    
    // solveb.addConstraint( lessThan( (par(Y.reshaped(n_0, m).transpose().row(0).transpose() - (x_sol.head(m*n_0).reshaped(n_0, m) * K_new).transpose().row(0).transpose() ) - b).norm() ,t1(0) ));
    // //solveb.addConstraint( lessThan( (par(Y.reshaped(n_0, m).transpose().row(1).transpose() - (x_sol.head(m*n_0).reshaped(n_0, m) * K_new).transpose().row(1).transpose() ) - b).norm() ,t1(1) ));
    // //solveb.addConstraint( lessThan( (par(Y.reshaped(n_0, m).transpose().row(2).transpose() - (x_sol.head(m*n_0).reshaped(n_0, m) * K_new).transpose().row(2).transpose() ) - b).norm() ,t1(2) ));
    // //solveb.addConstraint( lessThan( (par(Y.reshaped(n_0, m).transpose().row(3).transpose() - (x_sol.head(m*n_0).reshaped(n_0, m) * K_new).transpose().row(3).transpose() ) - b).norm() ,t1(3) ));
    // //solveb.addConstraint( lessThan( (par(Y.reshaped(n_0, m).transpose().row(4).transpose() - (x_sol.head(m*n_0).reshaped(n_0, m) * K_new).transpose().row(4).transpose() ) - b).norm() ,t1(4) ));

    // //solveb.addCostTerm(par(q_ones).dot(t1));
    // solveb.addCostTerm(t1.dot(par(q_ones)));
    // //not working
    // //osqp::OSQPSolver solverb(solveb);
    // cvx::ecos::ECOSSolver solverb(solveb);
    // std::cout << solverb << "\n";



    // std::cout << "Solver message: " << solverb.getResultString() << "\n";
    // //std::cout << "Solver info : " << aaa  << "\n";
    // //std::cout << "Solver info : " << solverb.getInfo() << "\n";
    // std::cout << "b7:\n" <<  eval(b) << std::endl;
    // OptimizationProblem qp2;

    // VectorX xa = qp2.addVariable("xa", 3);
    // qp2.addConstraint(equalTo(xa.sum(), 1.));
    // qp2.addConstraint(box(-1., xa, 1.));
    // qp2.addCostTerm((2. + xa(1)) * xa(1) + (1. + xa(0)) * xa(0) + (1. + xa(0)) * xa(1) + xa(2) * (2. + xa(2)) + xa(2) * xa(2));

    // osqp::OSQPSolver solver2(qp2);

    // solver2.solve(false);

    // std::cout << qp2 << "\n";
    // std::cout << solver2 << "\n";



    //solverb.solve(verbose);
}