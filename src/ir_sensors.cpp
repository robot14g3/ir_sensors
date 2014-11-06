#include <ros/ros.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <math.h>

using namespace std;

class IrSensorValues {

public:
    ros::NodeHandle nh;
    ros::Subscriber adc_sub;
    ros::Publisher cm_pub;

    IrSensorValues() {
        nh = ros::NodeHandle();
        adc_sub = nh.subscribe("/arduino/adc", 1, &IrSensorValues::adcCallback, this);
        cm_pub = nh.advertise<ras_arduino_msgs::ADConverter>("/ir_sensor_cm", 1);
    }
    ~IrSensorValues() {}

    void adcCallback(const ras_arduino_msgs::ADConverterConstPtr& msg) {

        //Calculation for sensor 1, front left
        sensor_cm.ch1 = nearbyint(58.13 * exp(-0.0155*msg->ch1) + 17.27 * exp(-0.002833*msg->ch1));

        //Calculation for sensor 2, front right
        sensor_cm.ch2 = nearbyint(59.23 * exp(-0.01451*msg->ch2) + 16.28 * exp(-0.002646*msg->ch2));

        //Calculation for sensor 3, back left
        sensor_cm.ch3 = nearbyint(96.08 * exp(-0.01784*msg->ch3) + 16.27 * exp(-0.002609*msg->ch3));

        //Calculation for sensor 4, back right
        sensor_cm.ch4 = nearbyint(106.5 * exp(-0.01905*msg->ch4) + 17.5 * exp(-0.002805*msg->ch4));

        //Calculation for sensor 5, front
        sensor_cm.ch5 = nearbyint(366.8 * exp(-0.02793*msg->ch5) + 58.15 * exp(-0.003915*msg->ch5));

        /*Check if any of the sensor values are out of range. In range according to datasheet are
        4-30 cm for sensors 1-4 and 10-80 cm for sensor 5.
        */
        if (sensor_cm.ch1 < 4 || sensor_cm.ch1 > 30) sensor_cm.ch1 = -1;
        if (sensor_cm.ch2 < 4 || sensor_cm.ch1 > 30) sensor_cm.ch2 = -1;
        if (sensor_cm.ch3 < 4 || sensor_cm.ch1 > 30) sensor_cm.ch3 = -1;
        if (sensor_cm.ch4 < 4 || sensor_cm.ch1 > 30) sensor_cm.ch4 = -1;
        if (sensor_cm.ch5 < 10 || sensor_cm.ch1 > 80) sensor_cm.ch5 = -1;

    }

    void publishValues() {
        cm_pub.publish(sensor_cm);
    }

private:
    ras_arduino_msgs::ADConverter sensor_cm;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "ir_sensors");

    IrSensorValues ir = IrSensorValues();

    ros::Rate loop_rate(10.0);

    while (ir.nh.ok()) {
        ros::spinOnce();
        ir.publishValues();
        loop_rate.sleep();
    }

    return 0;
}
