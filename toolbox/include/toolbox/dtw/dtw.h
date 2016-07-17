#ifndef DTW_H_
#define DTW_H_

#include <eigen3/Eigen/Core>
/*
namespace dtw
{

double dist(const Eigen::MatrixXd& sig1, const Eigen::MatrixXd& sig2, int i, int j);

double dtw(const Eigen::MatrixXd& sig1, const Eigen::MatrixXd& sig2, Eigen::MatrixXd& D, int w = -1);

double dtw(const Eigen::MatrixXd& sig1, const Eigen::MatrixXd& sig2, int w = -1);

void align_idx(const Eigen::MatrixXd& sig1, const Eigen::MatrixXd& sig2, Eigen::VectorXi& idx, int w = -1);

void align_phase(Eigen::VectorXd& phase1, const Eigen::VectorXd& phase2, const Eigen::MatrixXd& sig1, const Eigen::MatrixXd& sig2, int w = -1);

void align_phase(Eigen::MatrixXd& phase1, const Eigen::MatrixXd& phase2, const Eigen::MatrixXd& sig1, const Eigen::MatrixXd& sig2, int w = -1);

} // namespace
*/

namespace{

namespace dtw{

double dist(const Eigen::MatrixXd& sig1, const Eigen::MatrixXd& sig2, int i, int j)
{
    return (sig1.row(i) - sig2.row(j)).norm();
}

double dtw(const Eigen::MatrixXd& sig1, const Eigen::MatrixXd& sig2, Eigen::MatrixXd& D, int w = -1)
{
    assert(sig1.cols() == sig2.cols());
    double d=0;
    int l1 = sig1.rows();
    int l2 = sig2.rows();
    int sizediff = l1 - l2;
    int i,j;
    int j1,j2;
    double cost, temp;

    if(w!=-1 && w<sizediff) w = sizediff; // Adapt the window size, use sizediff

    // Initialize the cost of D
    D = Eigen::MatrixXd::Constant(l1+1,l2+1,-1);
    D(0,0) = 0;

    // Dynamic programming algorithm
    for(i = 1; i <= l1; i++)
    {
        if(w==-1)
        {
            j1 = 1;
            j2 = l2;
        }
        else
        {
            j1 = i-w>1 ? i-w : 1;
            j2 = i+w<l2 ? i+w : l2;
        }
        for(j = j1; j <= j2; j++)
        {
            cost = dist(sig1,sig2,i-1,j-1);
            temp = D(i-1,j);
            if(D(i,j-1)!=-1)
            {
                if(temp ==-1 || D(i,j-1) < temp) temp = D(i,j-1);
            }
            if(D(i-1,j-1) != -1)
            {
                if(temp == -1 || D(i-1,j-1) < temp) temp = D(i-1,j-1);
            }
            D(i,j) = cost + temp;
        }
    }

    d = D(l1,l2);

    return d;
}

double dtw(const Eigen::MatrixXd& sig1, const Eigen::MatrixXd& sig2, int w = -1)
{
    Eigen::MatrixXd D;
    return dtw(sig1,sig2,D,w);
}

void align_idx(const Eigen::MatrixXd& sig1, const Eigen::MatrixXd& sig2, Eigen::VectorXi& idx, int w = -1)
{
    Eigen::MatrixXd D_tmp;
    Eigen::MatrixXd::Index first_min_idx;
    dtw(sig1,sig2,D_tmp,w);

    Eigen::MatrixXd D = D_tmp.block(1,1,D_tmp.rows()-1,D_tmp.cols()-1);

    idx.resize(D.rows());
    for (int i_row = 0; i_row<D.rows(); i_row++)
    {
        D.row(i_row).minCoeff(&first_min_idx);
        idx(i_row) = static_cast<int>(first_min_idx);
    }
}

void align_phase(Eigen::VectorXd& phase1, const Eigen::VectorXd& phase2, const Eigen::MatrixXd& sig1, const Eigen::MatrixXd& sig2, int w = -1)
{
    assert(phase1.size() == sig1.rows());
    assert(phase2.size() == sig2.rows());

    Eigen::VectorXi idx;
    align_idx(sig1,sig2,idx,w);

    for (int i = 0; i<idx.size(); i++)
        phase1(i) = phase2(idx(i));
}

void align_phase(Eigen::MatrixXd& phase1, const Eigen::MatrixXd& phase2, const Eigen::MatrixXd& sig1, const Eigen::MatrixXd& sig2, int w = -1)
{
    assert(phase1.rows() == sig1.rows());
    assert(phase2.rows() == sig2.rows());
    assert(phase1.cols() == 1);
    assert(phase2.cols() == 1);

    Eigen::VectorXi idx;
    align_idx(sig1,sig2,idx,w);

    for (int i = 0; i<idx.size(); i++)
        phase1(i,0) = phase2(idx(i),0);
}

} // dtw namespace

} // anonym namespace


#endif /* DTW_H_ */

