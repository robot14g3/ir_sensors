#include <ros/ros.h>
#include <ras_arduino_msgs/ADConverter.h>

class IrCalibrator {

public:
    IrCalibrator() : number(50) {}
    ~IrCalibrator() {}

    void update(const ras_arduino_msgs::ADConverter& msg) {
        average_1 += msg->ch1;
        average_4 += msg->ch4;
    }

    void calculateAverage() {
        average_1 = average_1/number;
        average_4 = average_4/number;
    }

    void printAverage() {
        std::cout << "Average sensor 1: " << average_1 << std::endl;
        std::cout << "Average sensor 4: " << average_4 << std::endl;
     }

private:
    int average_1;
    int average_4;
    int number;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "ir_calibration");
    ros::NodeHandle nh;

    ros::Subscriber adc_sub = nh.subscribe("/arduino/adc", 1, &IrCalibrator::update, this);

    ros::Rate loop_rate(10);

    IrCalibrator ir = IrCalibrator();

    int number = 50;
    int iter = 0;

    while (nh.ok()) {
        if (iter >= number) break;
        iter++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    ir.calculateAverage();
    ir.printAverage();

    return 0;
}
