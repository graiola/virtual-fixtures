#ifndef MATH_H
#define MATH_H

////////// STD
#include <cmath>

////////// Eigen
#include <eigen3/Eigen/Core>

////////// BOOST
#include <boost/math/distributions/chi_squared.hpp>

namespace tool_box
{

inline double Intersection2Width(double center1, double center2, double ratio)
{
    assert(ratio > 0.0 && ratio <= 1);
    return std::abs(center2-center1)/(2.0*std::sqrt(2.0*std::log(1.0/ratio)));
}

inline double GaussMf(double x, double c, double sigma)
{
    assert(sigma>0);
    return std::exp(-(x-c)*(x-c)/(2.0*sigma*sigma));
}

inline void ComputeAbscisse(const Eigen::MatrixXd& xyz, Eigen::MatrixXd& abscisse)
{
    // Compute the abscisse curviligne
    abscisse.resize(xyz.rows(),1);
    abscisse.setZero();
    for(int i=0;i<xyz.rows()-1;i++)
        abscisse(i+1,0) = (xyz.row(i+1) - xyz.row(i)).norm() + abscisse(i,0);
    // Normalize
    double tot_length = abscisse(xyz.rows()-1,0);
    for(int i=0;i<abscisse.size();i++)
        abscisse(i,0) = abscisse(i,0)/tot_length;
}

inline int lratiotest(double loglik_unrestricted, double loglik_restricted, int dofs)
{
    boost::math::chi_squared chidist(dofs);
    double alpha = 0.05;
    double stat = 2*(loglik_unrestricted-loglik_restricted);

    if(stat < 0) // The restricted is "better"
        return 0;

    double p = 1 - boost::math::cdf(chidist,stat);
    if(p<=alpha)
        return 1;
    else
        return 0;
}

class DynSystemFirstOrder
{

public:
    DynSystemFirstOrder()
    {
        state_ = 0.0;
        gain_ = 0.0;
    }

    DynSystemFirstOrder(double gain, double dt = 0.001,  double ref = 1.0): DynSystemFirstOrder()
    {
        assert(dt > 0.0);
        dt_ = dt;
        assert(gain >= 0.0);
        gain_ = gain;
        ref_ = ref;
    }

    inline double IntegrateForward(double dt)
    {
        return state_ = gain_ * (ref_ - state_) * dt + state_;
    }

    inline double IntegrateBackward(double dt)
    {
        return state_ = gain_ * (-state_) * dt + state_;
    }

    inline double IntegrateForward()
    {
        return state_ = gain_ * (ref_ - state_) * dt_ + state_;
    }

    inline double IntegrateBackward()
    {
        return state_ = gain_ * (-state_) * dt_ + state_;
    }

    inline void Reset()
    {
        state_ = 0.0;
    }

    inline double GetState() const
    {
        return state_;
    }

    inline void SetGain(double gain)
    {
         assert(gain >= 0.0);
         gain_ = gain;
    }

    inline void SetRef(double ref)
    {
         ref_ = ref;
    }

private:

    double dt_;
    double gain_;
    double ref_;
    double state_;

};

class MinJerk
{
    public:
        MinJerk():x(0),v(0),p(0),j(0){}
        inline double GetX() const {return x;}
        inline double GetXDot() const {return v;}
        inline double GetXDotDot() const {return p;}
        inline double GetXDotDotDot() const {return j;}

        inline void Compute(double& t)
        {
            tau=(t-t0)/D;

            if (tau>1.0)
            {
                x=xf;v=0.0;p=0.0;j=0.0;
                return;
            }
            x=	a0 +
                a1*tau +
                a2*pow(tau,2)+
                a3*pow(tau,3)+
                a4*pow(tau,4)+
                a5*pow(tau,5);
            v=	a1/D +
                2*a2*tau/D+
                3*a3*pow(tau,2)/D+
                4*a4*pow(tau,3)/D+
                5*a5*pow(tau,4)/D;

            p=	2*a2/pow(D,2)+
                6*a3*tau/pow(D,2)+
                12*a4*pow(tau,2)/pow(D,2)+
                20*a5*pow(tau,3)/pow(D,2);

            j=	6*a3/pow(D,3)+
                24*a4*tau/pow(D,3)+
                60*a5*pow(tau,2)/pow(D,3);
        }

        inline void Create(double mxi,double mvi,double mpi,double mxf,double mt0,double mtf)
        {
            t0=mt0;
            tf=mtf;
            xf=mxf;
            D=mtf-mt0;
            a0=mxi;
            a1=D*mvi;
            a2=pow(D,2)*mpi/2.0;
            a3=-1.5*mpi*pow(D,2) - 6*mvi*D + 10*(mxf-mxi);
            a4= 1.5*mpi*pow(D,2) + 8*mvi*D - 15*(mxf-mxi);
            a5=-0.5*mpi*pow(D,2) - 3*mvi*D +  6*(mxf-mxi);
        }

    private:
        double t0,tf,xf,D,a0,a1,a2,a3,a4,a5;
        double x,v,p,j;
        double tau;
};

class AdaptiveGain
{
    public:

        AdaptiveGain(std::vector<double> gains_params)
        {
            assert(gains_params.size()==3);
            AdaptiveGain(gains_params[0],gains_params[1],gains_params[2]);
        }

        AdaptiveGain(double gain_at_zero, double gain_at_inf = 0.0, double zero_slope_error_value = 0.0)
        {
            if(gain_at_inf==0.0) // Constant gain
                b_ = 0.0;
            else // Adaptive gain
            {
                // Checks
                assert(gain_at_inf > 0.0);
                assert(gain_at_zero > gain_at_inf);
                assert(zero_slope_error_value > 0.0);
                b_ = 6/zero_slope_error_value;
            }
            c_ = gain_at_inf;
            a_ = gain_at_zero - gain_at_inf;
        }

        double ComputeGain(const double& error)
        {
            return a_ * std::exp(-b_ * std::abs(error)) + c_;
        }

    private:
        double a_;
        double b_;
        double c_;
};




} // namespace

#endif
