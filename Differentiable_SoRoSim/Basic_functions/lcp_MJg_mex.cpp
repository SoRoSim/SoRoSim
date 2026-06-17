#include "mex.h"
#include "matrix.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

// ---------- helpers ----------
static Eigen::MatrixXd mx_to_mat(const mxArray* A) {
    const mwSize m = mxGetM(A), n = mxGetN(A);
    double* pr = mxGetPr(A);
    Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor>> M(pr, m, n);
    return Eigen::MatrixXd(M);
}
static Eigen::VectorXd mx_to_vec(const mxArray* a) {
    const mwSize m = mxGetM(a), n = mxGetN(a);
    double* pr = mxGetPr(a);
    Eigen::Map<Eigen::VectorXd> v(pr, m*n);
    return Eigen::VectorXd(v);
}
static double comp_res_inf(const Eigen::VectorXd& lambda, const Eigen::VectorXd& w) {
    double r = 0.0;
    for (int i = 0; i < lambda.size(); ++i) {
        const double ri = std::min(lambda[i], w[i]);
        r = std::max(r, std::abs(ri));
    }
    return r;
}

// Apply Delassus: y = J * (M^{-1} * (J^T * x))
template <class Solver>
static inline Eigen::VectorXd apply_G(const Eigen::MatrixXd& J,
                                      const Solver& Msolver,
                                      const Eigen::VectorXd& x) {
    Eigen::VectorXd u = J.transpose() * x;      // n_v
    Eigen::VectorXd v = Msolver.solve(u);       // n_v
    Eigen::VectorXd y = J * v;                  // m
    return y;
}

// Estimate Lipschitz constant L ≈ lambda_max(G) via power iteration
template <class Solver>
static double estimate_L(const Eigen::MatrixXd& J,
                         const Solver& Msolver,
                         int iters = 8) {
    const int m = (int)J.rows();
    Eigen::VectorXd x = Eigen::VectorXd::Random(m);
    x.normalize();

    double L = 1.0;
    for (int k = 0; k < iters; ++k) {
        Eigen::VectorXd y = apply_G(J, Msolver, x);
        const double n = y.norm();
        if (n > 0) x = y / n;
        // Rayleigh quotient
        Eigen::VectorXd Gx = apply_G(J, Msolver, x);
        L = x.dot(Gx);
    }
    // safety margin
    if (!(L > 0.0)) L = 1.0;
    return 1.10 * L;
}

// ---------- main MEX ----------
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Usage:
    //   [lambda, info] = mex_lcp_MJg(M, J, g, lambda0, maxIters, tol)
    //
    // M: (n_v x n_v) SPD mass matrix
    // J: (m x n_v) normal contact Jacobian
    // g: (m x 1) bias/free normal velocity term (so w = G*lambda + g)
    // lambda0: optional warm start (m x 1), default zeros
    // maxIters: optional, default 100
    // tol: optional, default 1e-8 (complementarity residual inf-norm)

    if (nrhs < 3) mexErrMsgTxt("Need at least M, J, g.");

    Eigen::MatrixXd M = mx_to_mat(prhs[0]);
    Eigen::MatrixXd J = mx_to_mat(prhs[1]);
    Eigen::VectorXd g = mx_to_vec(prhs[2]);

    const int nv = (int)M.rows();
    if (M.cols() != nv) mexErrMsgTxt("M must be square.");
    if ((int)J.cols() != nv) mexErrMsgTxt("Size mismatch: J must have nv columns.");
    const int m = (int)J.rows();
    if (g.size() != m) mexErrMsgTxt("Size mismatch: g must have length m = rows(J).");

    Eigen::VectorXd lambda0 = Eigen::VectorXd::Zero(m);
    if (nrhs >= 4 && !mxIsEmpty(prhs[3])) {
        Eigen::VectorXd tmp = mx_to_vec(prhs[3]);
        if (tmp.size() != m) mexErrMsgTxt("lambda0 must be length m.");
        lambda0 = tmp;
        // project warm start to >=0
        for (int i = 0; i < m; ++i) lambda0[i] = std::max(0.0, lambda0[i]);
    }

    int maxIters = 100;
    double tol = 1e-8;
    if (nrhs >= 5) maxIters = (int)mxGetScalar(prhs[4]);
    if (nrhs >= 6) tol = mxGetScalar(prhs[5]);

    // Factorize M once
    Eigen::LLT<Eigen::MatrixXd> llt(M);
    if (llt.info() != Eigen::Success) {
        // fallback if M not SPD due to numerical issues
        Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
        if (ldlt.info() != Eigen::Success) mexErrMsgTxt("Factorization failed: M not SPD/LDLT failed.");

        // Solve with LDLT
        const double L = estimate_L(J, ldlt, 8);

        Eigen::VectorXd x = lambda0;
        Eigen::VectorXd y = x;
        double t = 1.0;

        Eigen::VectorXd w(m);
        bool conv = false;
        int it_used = 0;
        double res = 0.0;

        for (int k = 0; k < maxIters; ++k) {
            Eigen::VectorXd Gy = apply_G(J, ldlt, y);
            Eigen::VectorXd grad = Gy + g;

            Eigen::VectorXd xnew = y - (1.0 / L) * grad;
            for (int i = 0; i < m; ++i) xnew[i] = std::max(0.0, xnew[i]);

            const double tnew = 0.5 * (1.0 + std::sqrt(1.0 + 4.0 * t * t));
            y = xnew + ((t - 1.0) / tnew) * (xnew - x);
            x = xnew;
            t = tnew;

            w = apply_G(J, ldlt, x) + g;
            res = comp_res_inf(x, w);
            it_used = k + 1;
            if (res <= tol) { conv = true; break; }
        }

        // output lambda
        plhs[0] = mxCreateDoubleMatrix(m, 1, mxREAL);
        double* outp = mxGetPr(plhs[0]);
        for (int i = 0; i < m; ++i) outp[i] = x[i];

        // info
        if (nlhs >= 2) {
            const char* fields[] = {"iters","converged","res_inf","L_est"};
            plhs[1] = mxCreateStructMatrix(1,1,4,fields);
            mxSetField(plhs[1],0,"iters", mxCreateDoubleScalar((double)it_used));
            mxSetField(plhs[1],0,"converged", mxCreateDoubleScalar(conv ? 1.0 : 0.0));
            mxSetField(plhs[1],0,"res_inf", mxCreateDoubleScalar(res));
            mxSetField(plhs[1],0,"L_est", mxCreateDoubleScalar(L));
        }
        return;
    }

    // Solve with LLT
    const double L = estimate_L(J, llt, 8);

    Eigen::VectorXd x = lambda0;
    Eigen::VectorXd y = x;
    double t = 1.0;

    Eigen::VectorXd w(m);
    bool conv = false;
    int it_used = 0;
    double res = 0.0;

    for (int k = 0; k < maxIters; ++k) {
        Eigen::VectorXd Gy = apply_G(J, llt, y);
        Eigen::VectorXd grad = Gy + g;

        Eigen::VectorXd xnew = y - (1.0 / L) * grad;
        for (int i = 0; i < m; ++i) xnew[i] = std::max(0.0, xnew[i]);

        const double tnew = 0.5 * (1.0 + std::sqrt(1.0 + 4.0 * t * t));
        y = xnew + ((t - 1.0) / tnew) * (xnew - x);
        x = xnew;
        t = tnew;

        w = apply_G(J, llt, x) + g;
        res = comp_res_inf(x, w);
        it_used = k + 1;
        if (res <= tol) { conv = true; break; }
    }

    // output lambda
    plhs[0] = mxCreateDoubleMatrix(m, 1, mxREAL);
    double* outp = mxGetPr(plhs[0]);
    for (int i = 0; i < m; ++i) outp[i] = x[i];

    // info
    if (nlhs >= 2) {
        const char* fields[] = {"iters","converged","res_inf","L_est"};
        plhs[1] = mxCreateStructMatrix(1,1,4,fields);
        mxSetField(plhs[1],0,"iters", mxCreateDoubleScalar((double)it_used));
        mxSetField(plhs[1],0,"converged", mxCreateDoubleScalar(conv ? 1.0 : 0.0));
        mxSetField(plhs[1],0,"res_inf", mxCreateDoubleScalar(res));
        mxSetField(plhs[1],0,"L_est", mxCreateDoubleScalar(L));
    }
}
