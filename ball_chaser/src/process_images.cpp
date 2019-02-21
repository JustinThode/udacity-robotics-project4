#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call sevice drive_bot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;

    int count = 0;
    int sum_x = 0;

    // Loop through each pixel and find mean x of all bright white ones
    // Then, turn towards the mean x
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    for (size_t i = 0; i < img.height; ++i)
    {
        for (size_t j = 0; j < img.width; ++j)
        {
            size_t index = i * img.step + 3 * j;
            int val = img.data[index] + img.data[index + 1] + img.data[index + 2];
            if (val == 3 * white_pixel)
            {
                //add the current column index to the sum of detected x values
                sum_x += j;
                ++count;
            }
        }
    }

    if (count > 0)
    {
        float meanX = sum_x / (float)count;
        float turn = -0.4 * (meanX - 400) / 400.0F;
        float speed = 1.5655 * pow(count, -0.233F);
        speed = fmin(speed, 0.4F);
        speed -= 0.5F * fabs(turn);
        speed = fmax(speed, -0.1F);
        ROS_INFO("meanX %1.2f count %d v %1.2f t %1.2f", meanX, count, speed, turn);
        drive_robot(speed, turn);
    }
    else
    {
        drive_robot(0.0f, 0.0f);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

