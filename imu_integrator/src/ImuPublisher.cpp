#include "stdio.h"
#include <chrono>
#include <stdlib.h>
#include "sbgCom/sbgCom.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>


//----------------------------------------------------------------------//
//  Main program                                                        //
//----------------------------------------------------------------------//

struct vector {
    double x;
    double y;
    double z;

    vector(double inp_x, double inp_y, double inp_z) :
        x(inp_x), y(inp_y), z(inp_z) {
    };

    vector() :
        x(0.0), y(0.0), z(0.0) {
    };

    vector& operator+=(const vector& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    vector& operator-=(const vector& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    friend vector operator+(vector lhs, const vector& rhs) {
        lhs += rhs;
        return lhs;
    }

    friend vector operator-(vector lhs, const vector& rhs) {
        lhs -= rhs;
        return lhs;
    }

    friend vector operator/(vector lhs, double scalar) {
        lhs.x = lhs.x / scalar;
        lhs.y = lhs.y / scalar;
        lhs.z = lhs.z / scalar;
        return lhs;
    }

    friend vector operator*(vector lhs, double scalar) {
        lhs.x = lhs.x * scalar;
        lhs.y = lhs.y * scalar;
        lhs.z = lhs.z * scalar;
        return lhs;
    }

    friend vector operator*(double scalar, vector lhs) {
        return lhs * scalar;
    }

    friend std::ostream& operator<<(std::ostream& os, const vector& rhs) {
        os << rhs.x << " " << rhs.y << " " << rhs.z;
        return os;
    }
};


class Integrator {

public:
    Integrator() : num_calibrate_steps(20) {
        calls_to_update = 0;
    };

    void Update(vector inp_accelerations) {
        calls_to_update++;

        if (calls_to_update <= num_calibrate_steps) {
            //Sum up stationary accelerations in order to calculate the average offset
            calibrate_accelerations_sum += inp_accelerations;

        }
        else if (calls_to_update == num_calibrate_steps + 1) {
            //Find the average offset and make the first acceleration data point
            accelerations_offset = calibrate_accelerations_sum / double(num_calibrate_steps);      
            prev_update_call_time = std::chrono::high_resolution_clock::now(); 
            prev_accelerations = inp_accelerations;
            std::cout << "Finished calibration" << std::endl;
        }
        else if (calls_to_update == num_calibrate_steps + 2) {
            //Start the velocity integration chain and make the first position data point

            std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
            double time_step = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - prev_update_call_time).count();

            velocities += 0.5 * (inp_accelerations + prev_accelerations - 2.0 * accelerations_offset) * time_step;

            prev_accelerations = inp_accelerations;
            prev_velocities = velocities;
            prev_update_call_time = current_time;

        } else {
            //Carry on the velocity integrations as well as the position integrations
            std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
            double time_step = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - prev_update_call_time).count();

            velocities += 0.5 * (inp_accelerations + prev_accelerations - 2.0 * accelerations_offset) * time_step;
            positions += 0.5 * (prev_velocities + velocities) * time_step;

            prev_accelerations = inp_accelerations;
            prev_velocities = velocities;
            prev_update_call_time = current_time;

            std::cout << positions << std::endl;

        }

    }
private:

    //Number of calls to update function that will just be used to find the offset
    //for each acceleration direction and not actually be integrated
    const int num_calibrate_steps;

    int calls_to_update;

    //Used to calculate the average acceleration offset during calibration
    vector calibrate_accelerations_sum;
    vector accelerations_offset;

    //Used for integration
    std::chrono::high_resolution_clock::time_point  prev_update_call_time;
    vector prev_accelerations;
    vector velocities;

    vector prev_velocities;
    vector positions;


};




int main(int argc, char** argv)
{

    ros::init(argc, argv, "ImuPublisher");
    ros::NodeHandle n;
    //The ros topic for the imu data is called 'imu_data'
    //ros::Publisher imu_pub = n.advertise<msg_definitions::imu_data>("imu_data", 1000);


    SbgProtocolHandle protocolHandle;
    SbgErrorCode error;
    SbgOutput output;
    //msg_definitions::imu_data msg_packet; 

    Integrator integrator;






    //
    // Init our communications with the device (Please change here the com port and baud rate)
    //
    if (sbgComInit("/dev/ttyUSB0", 115200, &protocolHandle) == SBG_NO_ERROR)
    {
        //
        // Wait until the device has been initialised
        //
        sbgSleep(50);
        
        if(sbgRestoreDefaultSettings(protocolHandle, false) != SBG_NO_ERROR) {
            std::cout << "Error restoring defaults" << std::endl;
        }


        //
        // Main loop
        //
        
        for(int i = 0; i < 200; ++i)
        {
            error = sbgGetSpecificOutput(protocolHandle, SBG_OUTPUT_ACCELEROMETERS, &output);
            if (error == SBG_NO_ERROR)
            {
               vector accelerations(output.accelerometers[0], output.accelerometers[1], output.accelerometers[2]);
               integrator.Update(accelerations); 



                //imu_pub.publish(msg_packet);
                ros::spinOnce();
            }

            //
            // Small pause to unload CPU
            //
            sbgSleep(10);
        }

        char error_log[SBG_ERROR_LOG_SIZE_BYTES];
        if (sbgGetErrorLog(protocolHandle, &error_log, 0) != SBG_NO_ERROR) {
            std::cout << "Getting error log had an error" << std::endl;
        }
        std::cout << (int)error_log[0] << std::endl;

        //
        // Close our protocol system
        //
        sbgProtocolClose(protocolHandle);

        return 0;
    }
    else
    {
            fprintf(stderr, "Unable to open IG-500 device\n");
            return -1;
    }
}
