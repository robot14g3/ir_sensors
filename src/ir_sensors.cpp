#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <math.h>

using namespace std;

class IrSensorValues {

public:
    ros::NodeHandle nh;
    ros::Subscriber adc_sub;

    IrSensorValues() {
        nh = ros::NodeHandle();
        adc_sub = nh.subscribe("/arduino/adc", 1, &IrSensorValues::adcCallback, this);
    }
    ~IrSensorValues() {}

    void adcCallback(const ras_arduino_msgs::ADConverterConstPtr& msg) {

        //Calculation for sensor 1, front left
        sensor_cm[0] = 58.13 * exp(-0.0155*msg->ch1) + 17.27 * exp(-0.002833*msg->ch1);

        //Calculation for sensor 2, front right
        sensor_cm[1] = 59.23 * exp(-0.01451*msg->ch2) + 16.28 * exp(-0.002646*msg->ch2);

        //Calculation for sensor 3, back left
        sensor_cm[2] = 96.08 * exp(-0.01784*msg->ch3) + 16.27 * exp(-0.002609*msg->ch3); /*-0.2036 * pow(msg->ch3, 5) + 1.59 * pow(msg->ch3, 4) +
                -4.644 * pow(msg->ch3, 3) + 6.961 * pow(msg->ch3, 2) + -8.387 * pow(msg->ch3, 1)
                + 12.43;*/

        //Calculation for sensor 4, back right
        sensor_cm[3] = 106.5 * exp(-0.01905*msg->ch4) + 17.5 * exp(-0.002805*msg->ch4);

        //Calculation for sensor 5, front
        sensor_cm[4] = 366.8 * exp(-0.02793*msg->ch5) + 58.15 * exp(-0.003915*msg->ch5);
    }

    void printValues() {
        cout << "Front left: " << sensor_cm[0] << endl;
        cout << "Front right: " << sensor_cm[1] << endl;
        cout << "Back left: " << sensor_cm[2] << endl;
        cout << "Back right: " << sensor_cm[3] << endl;
        cout << "Front: " << sensor_cm[4] << endl;
    }

private:
    double sensor_cm[5];

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "ir_sensors");

    IrSensorValues ir = IrSensorValues();

    ros::Rate loop_rate(10.0);

    while (ir.nh.ok()) {
        ros::spinOnce();
        ir.printValues();
        loop_rate.sleep();
    }

    return 0;
}
