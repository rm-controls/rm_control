/*(utf8)
1€ Filter, template-compliant version
Jonathan Aceituno <join@oin.name>

25/04/14: fixed bug with last_time_ never updated on line 40

For details, see http://www.lifl.fr/~casiez/1euro
*/

//#include "one_euro_filter.h"
#include "filters.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


//if high speed lag is a problem, increase beta; if slow speed jitter is a problem, decrease mincutoff.
#define FREQUENCY 120.
#define MINCUTOFF 2.543785
#define BETA 0.000001
#define DCUTOFF 1.

template <typename T>
T rand01() {
    return rand() / static_cast<T>(RAND_MAX);
}

int main(int argc, char **argv) {
    srand((unsigned)time(0));
    OneEuroFilter<double> filter(FREQUENCY, MINCUTOFF, BETA, DCUTOFF);

    std::cout << "#CFG {'beta':" << BETA << ", 'freq':" << FREQUENCY << ", 'dcutoff':" << DCUTOFF << ", 'mincutoff':" << MINCUTOFF << "}" << std::endl;

    ros::init(argc, argv, "traj_test");
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<geometry_msgs::Vector3>("test_cmd", 100);
    geometry_msgs::Vector3 data{};
    ros::Rate loop_rate(FREQUENCY);
    double t=0;
    while(ros::ok()){

        t += 1/FREQUENCY;
        double signal = 10 * std::sin(t);
        double noisy = signal + 10 * ((rand01<double>() - 0.5) / 5);

        filter.input(noisy);
        double filtered = filter.output();

        data.x = signal;
        data.y = filtered;
        data.z = 0;
        
        pub.publish(data);
        loop_rate.sleep();
    }

    return 0;
}

