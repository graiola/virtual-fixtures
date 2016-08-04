#ifndef FILTERS_H
#define FILTERS_H

////////// Toolbox
#include "toolbox/toolbox.h"

#include <math.h>

namespace filters
{

	using namespace std;
	using namespace Eigen;
	
/* 
	Implements a IIR digital filter calculation.
  
	y[T] = b[n-1]*x[T]+b[n-2]*x[T-1]+...+b[0]*x[T-n]
	- (a[n-2]*y[T-1]+a[n-3]*y[T-2]+...+a[0]*y[T-n])
  
	Where T is the present time interval; n is the number of terms; x & y are sampled at descrete points
	in time with a constant period. Note that b[0],a[0] are coefficients for the 
	oldest sample, which may be counter-intuitive to book algorithms.
  
	Coefficients b[0..n] and a[1..n] must be predivided by a[0] if it does not equal 1.0
	The algorithm does no use a[0] for calculation efficiency. The value stored at a[0] 
	is not affected by the algorithm.

  Usage: 
	For the butterworth filters, you will need to choose a cutoff frequency. The
	lower the cutoff_freq, the smoother the estimated velocity, at the cost of delay.
    
	For a starting value, select the cutoff frequency to be equal to the lowest 
	response frequency you need. Ex: 1kHz sample freq & 30Hz cutoff Freq for
	a haptic device. (Humans can affect input up to about 8Hz).
  
*/

#define MAXFILTERTERMS 3000
    class Filter
	{
		public:
            Filter(const int type_id):_a(MAXFILTERTERMS,0.0),_b(MAXFILTERTERMS,0.0),_x(MAXFILTERTERMS,0.0),_y(MAXFILTERTERMS,0.0),Nterms(MAXFILTERTERMS),order(2),buffer_idx(0),T(1.0/1000) // FIXME 1000 is the rt frequency
			{
			  
                switch (type_id) // FIXME port the others
                {
                    case (1):
                        type = BUTTERWORTH;
                        Butterworth_Filter();
                        break;
                    case(2):
                        type = DIFF_BUTTERWORTH;
                        Diff_Butterworth_Filter();
                        break;
                    case(3):
                        type = AVERAGE;
                        Average_Filter();
                        break;
                }
			}
            inline void Dump(){ //Dump the contents of the class to cout
                std::cout << "terms:" << Nterms << "\n";
                std::cout << "Index:" << buffer_idx << "\n";
                for ( int cnt = 0; cnt < Nterms; cnt++ ) {
                    std::cout << "a: " << _a[cnt] << " b: " << _b[cnt] << " x: " << _x[cnt] << " y: " << _y[cnt] << "\n" ;
                    }
                }
            inline void Clear(){ //Clear the history of the filter
                buffer_idx=0;
                for ( int cnt = 0; cnt < MAXFILTERTERMS; cnt++ ) {
                    _x[cnt]=0.0;
                    _y[cnt]=0.0;
                    _a[cnt]=0.0;
                    _b[cnt]=0.0;
                    }
                }
            /*inline int Coefficients(int N,std::vector<double> A,std::vector<double> B){ //Set the coefficients of the filter.
                    if (N > MAXFILTERTERMS)
                        return -1; //Filter is too small to take this many coefficients
                    Nterms=N;
                    for (int cnt = 0;cnt < N;cnt++){
                        _a[cnt]=A[cnt];
                        _b[cnt]=B[cnt];
                    }
                    return 0; //success
            }*/
            double Step(double x_0){ //evaluate the filter
                //double yt=0.0;
                switch ( type ) {
                    case BUTTERWORTH:
                        return Step ( x_0,order+1 );
                    case DIFF_BUTTERWORTH:
                        return Step ( x_0,order+1 );
                    case AVERAGE:
                        return Step ( x_0,Nterms );
                    /*case LEAST_SQUARES_ESTIMATE:
                        return Step ( x_0,Nterms );
                    case IDENTITY:
                    return x_0;*/
                    default:
                        //ROS_ERR ( "Filter Step error: Not a valid type.\n" );
                        break;
                    }
                return 0.0;
                }
            inline double Step(double x_0,int N){ //evaluate the filter
               double retval=0.0;
               int i=0, start_idx=0;
               _x[buffer_idx] = x_0;
               _y[buffer_idx] = 0.0;
               start_idx = buffer_idx - N + MAXFILTERTERMS + 1; //Precalc the index where the history data starts in the buffer.
               for ( int n = 0; n < N; n++ ) {
                   i = ( start_idx + n ) % ( MAXFILTERTERMS );
                   _y[buffer_idx] +=  _b[n]*_x[i] - _a[n]*_y[i];
                   //}
                   }
               retval = _y[buffer_idx];
               buffer_idx = ( buffer_idx+1 ) % ( MAXFILTERTERMS );
               return retval;
               }


            void UpdateFilter ( ) {
                switch ( type ) {
                    /*case NONE:
                        Identity_Filter();
                        break;*/
                    case BUTTERWORTH:
                        Butterworth_Filter();
                        break;
                    case DIFF_BUTTERWORTH:
                        Diff_Butterworth_Filter();
                        break;
                    case AVERAGE:
                        Average_Filter();
                        break;
                    /*case LEAST_SQUARES_ESTIMATE:
                        Least_Squares_Estimate();
                        break;
                    case IDENTITY:
                        Identity_Filter();
                        break;*/
                    default:
                        //ROS_ERR ( "Filter UpdateParams error: Not a valid type.\n" );
                        break;
                    }
                return;
                }


			int GetN()const{return Nterms;}
			int GetOrder()const{return order;}
			//int GetType()const{return type;}
			double GetCutOffFreq(){return cutoff_freq;}
			void SetN(int N){
			  if(N>MAXFILTERTERMS){
			    N = MAXFILTERTERMS;
                //ROS_INFO("N set to max (%d)",MAXFILTERTERMS);
			  }
			  Nterms = N;
			  UpdateFilter();}
			void SetCutoff_freq(double cutoff_freq){this->cutoff_freq = cutoff_freq;UpdateFilter();} 
			void SetOrder(int order){this->order = order;UpdateFilter();}
            //void SetType(string t);
            //const char * GetType();
			
		protected:
			enum FILTER_TYPE{NONE, BUTTERWORTH, DIFF_BUTTERWORTH, LEAST_SQUARES_ESTIMATE, IDENTITY, AVERAGE}; // is inside actuator.pb.h
			int Nterms; //Number of terms in the calculation
			int buffer_idx; //Present start of the x & y history circular buffers
            void Butterworth_Filter(){
                //double T = sample_period; //Shorthand for clearer code.
                double scale=1.0; //Scaling factor (really the a[1] value which everything gets divided by)

                double pi = 3.14159625;

                //std::vector<double> a(4,0.0);
                //std::vector<double> b(4,0.0);

                double wo = cutoff_freq*2.0*pi; //Omega naught (rad/sec)
                double w = ( 2./T ) *atan2 ( wo*T,2. ); //Omega cutoff (frequency warping from analog to digital domain)
                if ( order== 1 ) {
                    double st = 2/T/w;
                    scale = 1./ ( 1.+st );

                    _a[1] = 0.0;
                    _a[0] = ( 1.-st ) *scale;

                    _b[1] = scale;
                    _b[0] = scale;
                    }
                else if ( order == 2 ) {
                    double st = 2/T/w;
                    scale = 1/ ( 1+sqrt ( 2.0 ) *st+st );

                    _a[2] = 0.0;
                    _a[1] = ( 2.-2.*st ) * scale;
                    _a[0] = ( 1.-static_cast<double> ( sqrt ( 2.0 ) ) *st+st ) * scale;

                    _b[2] = scale;
                    _b[1] = 2.*scale;
                    _b[0] = scale;
                    }
                else if ( order == 3 ) {
                    double A = 2./ ( w*T );
                    double p2 = 2.*A;
                    double p3 = 2.*A*A;
                    double p4 = A*A*A;

                    scale = 1./ ( 1.+p2+p3+p4 );

                    _a[3] = 0.;
                    _a[2] = ( 3.+p2-p3-3.*p4 ) *scale;
                    _a[1] = ( 3.-p2-p3+3.*p4 ) * scale;
                    _a[0] = ( 1.-p2+p3-p4 ) * scale;

                    _b[3] = scale;
                    _b[2] = 3.*scale;
                    _b[1] = 3.*scale;
                    _b[0] = scale;

                    }
                else {
                    //ROS_INFO ( "Invalid order %d for Butterworth filter\n",order );
                    return ; //
                    }
                //Coefficients(order+1,a,b);
                }
            /*
                Creates a digitized butterworth filter plus differentiator using the
                bilinear transform. Oppenheim & Schafer pp450 (2nd ed)
                order: the order of the filter. 1, 2 or 3.
                cutoff_freq: cutoff frequency of the lowpass filter
                sample_period: time between samples.
            */
            void Diff_Butterworth_Filter() {
                double scale=1.0; //Scaling factor (really the a[1] value which everything gets divided by)

                double pi = 3.14159625;

                double wo = cutoff_freq*2.0*pi; //Omega naught (rad/sec)
                double w = 2./T*atan2 ( wo*T,2. ); //Omega cutoff (frequency warping from analog to digital domain)

                if ( order == 1 ) {
                    double st = 1/w;
                    scale = 1/ ( T/2+st );

                    _a[1] = 0.0;
                    _a[0] = ( T/2-st ) *scale;

                    _b[1] = scale;
                    _b[0] = -scale;
                    }
                else if ( order == 2 ) {
                    double st = sqrt ( 2.0 ) /w;
                    scale = 1/ ( st*st/T + st + T/2.0 );

                    _a[2] = 0.0;
                    _a[1] = ( T-2*st*st/T ) * scale;
                    _a[0] = ( T/2 - st + st*st/T ) * scale;

                    _b[2] = scale;
                    _b[1] = 0.0;
                    _b[0] = -scale;
                    }
                else if ( order == 3 ) {
                    double st = 2/w;
                    double nd = st*st/T;

                    scale = 1/ ( T+st+nd );

                    _a[3] = 0.0;
                    _a[2] = ( st-nd ) *scale;
                    _a[1] = ( 3*T-st-nd ) * scale;
                    _a[0] = ( nd-st ) * scale;

                    _b[3] = scale;
                    _b[2] = scale;
                    _b[1] = -scale;
                    _b[0] = -scale;
                    }
                else {
                    //ROS_INFO ( "Incorrect Filter configuration for Diff_Butterworth_Filter\n" );
                    }
                //Coefficients(order+1,a,b);
                }

            //void Least_Squares_Estimate();
            //void Identity_Filter();
            void Average_Filter() {
                for ( int i=0; i<Nterms; i++ ){
                    _b[i]=1.0/ ( double ) Nterms;
                    _a[i]=0.0;
                    }
                //Coefficients(N,a,b);
                }

			int type;
			int order;
			double cutoff_freq;
			double T; //Period
		private:
			std::vector<double> _a; //y filter coefficients in reverse order.
			std::vector<double> _b; //x filter coefficients in reverse order. 
			std::vector<double> _x; //independent value history (circular buffer)
			std::vector<double> _y; //dependent value history (circular buffer)
			
			
	};
	
}//namespace

#endif
