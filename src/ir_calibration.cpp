#include <ros/ros.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <iostream>
#include <fstream>

using namespace std;


class IrCalibrator {

public:
    ros::Subscriber adc_sub;
    ros::NodeHandle nh;

    IrCalibrator() : number(100) {
        nh = ros::NodeHandle();
        adc_sub = nh.subscribe("/arduino/adc", 1, &IrCalibrator::update, this);
        memset(average_1, 0, sizeof(average_1));
        memset(average_4, 0, sizeof(average_4));
        index = 0;
    }

    ~IrCalibrator() {}

    void update(const ras_arduino_msgs::ADConverterConstPtr& msg) {
        average_1[index] += msg->ch5;
        //average_4[index] += msg->ch4;
    }

    void calculateAverage() {
        average_1[index] = average_1[index]/number;
        //average_4[index] = average_4[index]/number;
    }

    void printAverage(ofstream &file) {
        cout << "Average sensor 2: " << average_1[index] << endl;
        cout << "Average sensor 4: " << average_4[index] << endl;
        file << average_1[index] << endl; //<< ", " << average_4[index] << endl;
     }

    void incrementIndex() {
        index++;
    }

    int getIndex() {
        return index;
    }

private:
  int average_1[92]; //28 for short range
    int average_4[92];
    int number;
    int index;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "ir_calibration");

    IrCalibrator ir = IrCalibrator();

    ros::Rate loop_rate(10);

    int number = 100;
    int iter = 0;
    int numberOfTests = 92;//28;

    ofstream file("ir_values_calibration.txt");
    file << "Sensor front: " << endl;//  Sensor 4:" << endl;

    while (ir.nh.ok()) {
        if (ir.getIndex() >= numberOfTests) break;
        if (iter >= number) {
            iter = 0;
            ir.calculateAverage();
            ir.printAverage(file);
            ir.incrementIndex();
            cout << "Press key to start next set" << endl;
            cin.ignore();
        }
        iter++;

        ros::spinOnce();
        loop_rate.sleep();
    }

    file.close();

    return 0;
}
