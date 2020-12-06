/*(utf8)
1€ Filter, template-compliant version
Jonathan Aceituno <join@oin.name>

25/04/14: fixed bug with last_time_ never updated on line 40

For details, see http://www.lifl.fr/~casiez/1euro
*/

#include <cmath>

template <typename T = double>
struct low_pass_filter {
	low_pass_filter() : hatxprev(0), xprev(0), hadprev(false) {}
	T operator()(T x, T alpha) {
		T hatx;
		if(hadprev) {
			hatx = alpha * x + (1-alpha) * hatxprev;
		} else {
			hatx = x;
			hadprev = true;
		}
		hatxprev = hatx;
		xprev = x;
		return hatx;
	}
	T hatxprev;
	T xprev;
	bool hadprev;
};

template <typename T = double, typename timestamp_t = double>
struct one_euro_filter {
	one_euro_filter(double _freq, T _mincutoff, T _beta, T _dcutoff) : freq(_freq), mincutoff(_mincutoff), beta(_beta), dcutoff(_dcutoff), last_time_(-1) {}
	T operator()(T x, timestamp_t t = -1) {
		T dx = 0;
		
		if(last_time_ != -1 && t != -1 && t != last_time_) {
			freq = 1.0 / (t - last_time_);
		}
		last_time_ = t;
		
		if(xfilt_.hadprev)
			dx = (x - xfilt_.xprev) * freq;
		
		T edx = dxfilt_(dx, alpha(dcutoff));
		T cutoff = mincutoff + beta * std::abs(static_cast<double>(edx));
		return xfilt_(x, alpha(cutoff));
	}
	
	double freq;
	T mincutoff, beta, dcutoff;
private:
	T alpha(T cutoff) {
		T tau = 1.0 / (2 * M_PI * cutoff);
		T te = 1.0 / freq;
		return 1.0 / (1.0 + tau / te);
	}
	
	timestamp_t last_time_;
	low_pass_filter<T> xfilt_, dxfilt_;
};

// =================================================================
// = ============================================================= =
// =================================================================

// An example 

#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <cstdlib>

struct randomizer {
	randomizer(int a, int b) : start_(std::min(a, b)), range_(std::max(a,b) - start_) {}
	int operator()() {
		return start_ + rand() % range_;
	}
private:
	int start_, range_;
};

template <typename T>
T rand01() {
	return rand() / static_cast<T>(RAND_MAX);
}

int main() {
	srand((unsigned)time(0));
	
	static const double duration = 10;
	static const double frequency = 120;
	static const double mincutoff = 1;
	static const double beta = 1;
	static const double dcutoff = 1;
	
	one_euro_filter<> filter(frequency, mincutoff, beta, dcutoff);
	
	std::cout << "#SRC 1efilter.cc" << std::endl;
	std::cout << "#CFG {'beta':" << beta << ", 'freq':" << frequency << ", 'dcutoff':" << dcutoff << ", 'mincutoff':" << mincutoff << "}" << std::endl;
	std::cout << "#LOG timestamp, signal, noisy, filtered" << std::endl;
	
	for(double t=0; t<duration; t += 1/frequency) {
		double signal = std::sin(t);
		double noisy = signal + (rand01<double>() - 0.5) / 5;
		double filtered = filter(noisy, t);
		std::cout << t << ", " << signal << ", " << noisy << ", " << filtered << std::endl;
	}
	
	return 0;
}