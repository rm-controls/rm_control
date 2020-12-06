/*(utf8)
1€ Filter, template-compliant version
Jonathan Aceituno <join@oin.name>

25/04/14: fixed bug with last_time_ never updated on line 40

For details, see http://www.lifl.fr/~casiez/1euro
*/

#include "oneEuro_filter.h"

#define DURATION 10
#define FREQUENCY 120
#define MINCUTOFF 1
#define BETA 1
#define DCUTOFF 1

int main() {
	srand((unsigned)time(0));

    one_euro_filter<> filter(FREQUENCY, MINCUTOFF, BETA, DCUTOFF);

	std::cout << "#SRC 1efilter.cc" << std::endl;
	std::cout << "#CFG {'beta':" << BETA << ", 'freq':" << FREQUENCY << ", 'dcutoff':" << DCUTOFF << ", 'mincutoff':" << MINCUTOFF << "}" << std::endl;
	std::cout << "#LOG timestamp, signal, noisy, filtered" << std::endl;

	for(double t=0; t < DURATION; t += 1/FREQUENCY) {
		double signal = std::sin(t);
		double noisy = signal + (rand01<double>() - 0.5) / 5;
		double filtered = filter(noisy, t);
		std::cout << t << ", " << signal << ", " << noisy << ", " << filtered << std::endl;
	}

	return 0;
}
