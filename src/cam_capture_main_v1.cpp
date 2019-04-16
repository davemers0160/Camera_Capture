//=============================================================================
// 
// 
// 
//=============================================================================

// FTDI Driver Includes
//#include "ftd2xx.h"
#include "ftdi_functions.h"

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#define _CRT_SECURE_NO_WARNINGS					

#ifndef _WIN32_WINNT		// Allow use of features specific to Windows XP or later.                   
#define _WIN32_WINNT 0x0501	// Change this to the appropriate value to target other versions of Windows.
#endif		

// windows Includes
//#include <windows.h> 
//#include <tchar.h>
#endif

// C++ Includes
#include <cstdio>
#include <cstdint>
#include <ctime>
#include <map>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>

// OPENCV Includes
#include <opencv2/core/core.hpp>           
#include <opencv2/highgui/highgui.hpp>     
#include <opencv2/imgproc/imgproc.hpp>  

// Point Grey Includes
#include "FlyCapture2.h"
#include "chameleon_utilities.h"
#include "image_capture.h"
namespace FC2 = FlyCapture2;

// Lens Driver Includes
#include "lens_driver.h"

// Custom Includes
//#include "path_check.h"
#include "num2string.h"
#include "get_current_time.h"
#include "file_parser.h"
#include "make_dir.h"

int main(int argc, char** argv)
{
    uint32_t idx, jdx, kdx;
    uint8_t status;
    std::string console_input;
    std::vector<double> range;
    std::ofstream DataLogStream;

    std::vector<ftdiDeviceDetails> ftdi_devices;

    // Lens Driver Variables
    uint32_t ftdi_device_count = 0;
    ftdiDeviceDetails lens_driver_details;
    FT_HANDLE lens_driver_handle = NULL;
    uint32_t lens_driver_dev_num = 0;
    uint32_t connect_count = 0;
    lens_driver ld;
    std::vector<lens_packet_struct> focus_packets;
    std::vector<uint8_t> lens_step(1);

    // Camera Specific Variables
    FC2::Error error;
    FC2::Camera cam;
    FC2::FC2Config camera_config;
    FC2::CameraInfo cam_info;
    FC2::PixelFormat pixelFormat = FC2::PIXEL_FORMAT_RGB8;  //  FC2::PIXEL_FORMAT_422YUV8, FC2::PIXEL_FORMAT_444YUV8;
    FC2::Property Shutter;
    cam_properties_struct cam_properties;
    uint64_t cam_serial_number;
    uint32_t cam_index;
    uint32_t cam_number = 0;
    uint32_t x_offset, y_offset, width, height;
    bool camera_on = true;
    uint32_t avg_count = 19;
    std::vector<double> shutter;
    std::string shutter_str;
    double cam_temp = 0.0;

    // OpenCV Variables
    char key;
    cv::Mat image;
    cv::Size img_size;
    std::string image_window = "Image";
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(1);

    std::string sdate, stime;
    std::string log_filename = "camera_capture_log_";
    std::string image_capture_name = "image_";
    std::string output_save_location = "";

    const std::string params =
        "{help h ?  |  | Display Usage message }"
        "{cfg_file  |  | Alternate input method to supply all parameters, all parameters must be included in the file }"
        "{v_step    | 127:1:145 | voltage step range}"
        "{x_off     | 8 | X offset for camera }"
        "{y_off     | 4 | Y offset for camera }"
        "{width     | 1264 | Width of the captured image }"
        "{height    | 1020 | Height of the captured image }"
        "{sharpness | 3072 | Sharpness setting for the camera }"
        "{fps       | 10.0 | Frames per second setting for the camera }"
        "{shutter   | 60:-10:10 | Shutter speed range settings for the camera }"
        "{gain      | 5.0 | Inital gain setting before letting the camera find one }"
        "{avg       | 11 | Number of images to capture for an average }"
        "{output    | ../results/       | Output directory to save lidar images }"
        ;

    // use opencv's command line parser
    cv::CommandLineParser parser(argc, argv, params);

    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }

    focus_packets.clear();
    range.clear();

    // if the input is a config file use this over all other input parameters
    if (parser.has("cfg_file"))
    {
        // input config file should contain all required inputs
        std::string cfg_filename = parser.get<std::string>("cfg_file");
        std::vector<std::vector<std::string>> cfg_params;
        parseCSVFile(cfg_filename, cfg_params);

        // config file should be in the following format
        // Line 1: colon separated values of the lens driver voltage step range (start:inc:stop)
        // Line 2: comma separated values for the x offset, y offset, width, height of the camera
        // Line 3: comma separated values for the camera properties: sharpness, fps, shutter range
        // Line 4: base directory where the results will be saved
        if (cfg_params.size() == 5)
        {
            // setup the focus level packets in a vector - later this will be an input configurable option
            parse_input_range(cfg_params[0][0], range);

            x_offset = std::stoi(cfg_params[1][0]);
            y_offset = std::stoi(cfg_params[1][1]);
            width = std::stoi(cfg_params[1][2]);
            height = std::stoi(cfg_params[1][3]);

            // camera properties settings
            cam_properties.sharpness = std::stoi(cfg_params[2][0]);
            cam_properties.fps = std::stof(cfg_params[2][1]);
            parse_input_range(cfg_params[2][2], shutter);
            cam_properties.shutter = shutter[0];
            cam_properties.gain = std::stof(cfg_params[2][3]);

            avg_count = std::stoi(cfg_params[3][0]);

            // output save location
            output_save_location = cfg_params[4][0];
        }
        else
        {
            std::cout << "The number of supplied parameters in the file does not meet the required criteria: N = " << cfg_params.size() << std::endl;
            std::cin.ignore();
            return -1;
        }
    }
    else
    {
        parse_input_range(parser.get<string>("v_step"), range);
        x_offset = parser.get<uint32_t>("x_off");		// 40
        y_offset = parser.get<uint32_t>("y_off");		// 228;
        width = parser.get<uint32_t>("width");		    // 1200;
        height = parser.get<uint32_t>("height");		// 720;
        
        // camera properties settings
        cam_properties.sharpness = parser.get<uint32_t>("sharpness");
        cam_properties.fps = parser.get<float>("fps");
        parse_input_range(parser.get<string>("shutter"), shutter);
        cam_properties.shutter = shutter[0];
        cam_properties.gain = parser.get<float>("gain");

        avg_count = parser.get<uint32_t>("avg");

        output_save_location = parser.get<std::string>("output");
    }

    // convert the voltage step into a series of packets for the lens driver
    for (idx = 0; idx < range.size(); ++idx)
    {
        focus_packets.push_back(lens_packet_struct(FAST_SET_VOLT, 1, { (uint8_t)range[idx] }));
    }

    path_check(output_save_location);

    // default camera properties
    cam_properties.auto_exp = 0.0;
    cam_properties.brightness = 4.0;

    get_current_time(sdate, stime);
    log_filename = log_filename + sdate + "_" + stime + ".txt";

    std::cout << "Log File: " << (output_save_location + log_filename) << std::endl << std::endl;
    DataLogStream.open((output_save_location + log_filename), ios::out | ios::app);

    // Add the date and time to the start of the log file
    DataLogStream << "------------------------------------------------------------------" << std::endl;
    DataLogStream << "Version: 1.0    Date: " << sdate << "    Time: " << stime << std::endl;
    DataLogStream << "------------------------------------------------------------------" << std::endl;

	try
    {

        ftdi_device_count = get_device_list(ftdi_devices);
        if (ftdi_device_count == 0)
        {
            std::cout << "No ftdi devices found... Exiting!" << std::endl;
            DataLogStream << "No ftdi devices found... Exiting!" << std::endl;
            std::cin.ignore();
            DataLogStream.close();
            return -1;
        }

        for (idx = 0; idx < ftdi_devices.size(); ++idx)
        {
            std::cout << ftdi_devices[idx];
        }

        std::cout << "Select Lens Driver: ";
        std::getline(std::cin, console_input);
        lens_driver_dev_num = stoi(console_input);

        std::cout << std::endl << "Connecting to Lens Driver..." << std::endl;
        ftdi_devices[lens_driver_dev_num].baud_rate = 250000;
        while ((lens_driver_handle == NULL) && (connect_count < 10))
        {
            lens_driver_handle = open_com_port(ftdi_devices[lens_driver_dev_num]);
            ++connect_count;
        }

        if (lens_driver_handle == NULL)
        {
            std::cout << "No Lens Driver found... Exiting!" << std::endl;
            DataLogStream << "No Lens Driver found... Exiting!" << std::endl;
            std::cin.ignore();
            DataLogStream.close();
            return -1;
        }
      
        ld.lens_tx = lens_packet_struct(CON, 0);

        // send connection request packet and get response back
        ld.send_lens_packet(ld.lens_tx, lens_driver_handle);
        status = ld.receive_lens_packet(ld.lens_rx, lens_driver_handle, 9);

        if (status == false)
        {
            std::cout << "Error communicating with lens driver... Exiting!" << std::endl;
            DataLogStream << "Error communicating with lens driver... Exiting!" << std::endl;
            std::cin.ignore();
            DataLogStream.close();
            return -1;
        }

        ld.set_lens_driver_info(ld.lens_rx);
        std::cout << ld << std::endl;
        DataLogStream << ld << std::endl;
        DataLogStream << "------------------------------------------------------------------" << std::endl;

        ld.send_lens_packet(focus_packets[(focus_packets.size()>>1)], lens_driver_handle);

        error = get_camera_selection(cam_index);
        if (error != FC2::PGRERROR_OK)
        {
            print_error(error);
        }

        // Initialize the camera
        error = init_camera(cam, cam_index, camera_config, cam_info);
        if (error == FC2::PGRERROR_OK)
        {
            std::cout << "------------------------------------------------------------------" << std::endl;
            std::cout << cam_info;
            DataLogStream << cam_info;
            cam_serial_number = (uint64_t)cam_info.serialNumber;
        }
        else
        {
            print_error(error);
            std::cin.ignore();
            return -1;
        }

        get_camera_temperature(cam, cam_temp);
        std::cout << "  Camera Temperature:  " << cam_temp << std::endl;
        DataLogStream << "  Camera Temperature:  " << cam_temp << std::endl << std::endl;

        error = config_imager_format(cam, x_offset, y_offset, width, height, pixelFormat);
        if (error != FC2::PGRERROR_OK)
        {
            print_error(error);
        }

        error = cam.StartCapture();
        if (error != FC2::PGRERROR_OK)
        {
            print_error(error);
        }

        error = config_camera_propeties(cam, cam_properties);
        if (error != FC2::PGRERROR_OK)
        {
            print_error(error);
        }

        img_size = cv::Size(width, height);
        std::cout << "------------------------------------------------------------------" << std::endl;
        std::cout << "X, Y, Width, Height: " << x_offset << ", " << y_offset << ", " << width << ", " << height << std::endl;
        std::cout << cam_properties;
        std::cout << "Average Capture Number:  " << avg_count << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl;

        DataLogStream << "------------------------------------------------------------------" << std::endl;
        DataLogStream << "X, Y, Width, Height: " << x_offset << ", " << y_offset << ", " << width << ", " << height << std::endl;
        DataLogStream << cam_properties << std::endl;
        DataLogStream << "Average Capture Number:  " << avg_count << std::endl;
        DataLogStream << "------------------------------------------------------------------" << std::endl;

        std::cout << "Root save location: " << output_save_location << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl;

        std::cout << std::endl << "Press the following keys to perform actions:" << std::endl;
        std::cout << "  s - Save an image" << std::endl;
        std::cout << "  q - Quit" << std::endl;
        std::cout << std::endl;

        // set the camera to software trigger to get images
        error = set_software_trigger(cam, true);
        if (error != FC2::PGRERROR_OK)
        {
            print_error(error);
        }

        image_window = image_window + num2str<uint64_t>(cam_serial_number, "_%d");

        do
        {
            // wait for the camera to be ready for a software trigger
            poll_trigger_ready(cam);      

            error = fire_software_trigger(cam);
            if (error != FC2::PGRERROR_OK)
            {
                print_error(error);
                std::cout << "Error firing software trigger" << std::endl;
                DataLogStream << "Error firing software trigger" << std::endl;
            }

            error = get_image(cam, image);
            if (error != FC2::PGRERROR_OK)
            {
                print_error(error);
            }
            
            cv::namedWindow(image_window, cv::WindowFlags::WINDOW_NORMAL);
            cv::imshow(image_window, image);

            key = cv::waitKey(20);

            // check to save the image
            if (key == 's')
            {
                get_current_time(sdate, stime);
                ld.send_lens_packet(focus_packets[0], lens_driver_handle);

                sleep_ms(100);

                for (kdx = 0; kdx < shutter.size(); ++kdx)
                {
                    cam_properties.shutter = shutter[kdx];
                    // config Shutter to initial value and set to auto
                    config_property(cam, Shutter, FC2::SHUTTER, false, true, true);
                    error = set_abs_property(cam, Shutter, cam_properties.shutter);
                    if (error != FC2::PGRERROR_OK)
                    {
                        print_error(error);
                    }

                    std::string combined_save_location = "";
                    std::string sub_dir = "exp_" + num2str(shutter[kdx], "%02.0f");
                    int32_t stat = make_dir(output_save_location, sub_dir);
                    if (stat != 1 && stat != (int32_t)ERROR_ALREADY_EXISTS)
                    {
                        std::cout << "Error creating directory: " << stat << std::endl;
                        DataLogStream << "Error creating directory: " << stat << std::endl;
                        combined_save_location = output_save_location;
                    }
                    else
                    {
                        combined_save_location = output_save_location + sub_dir + "/";
                    }

                    // sleep for a little to let the camera settle down
                    sleep_ms(100);

                    shutter_str = num2str(cam_properties.shutter, "%2.2f_");

                    for (idx = 0; idx < focus_packets.size(); ++idx)
                    {
                        cv::Mat sum_image = cv::Mat(img_size, CV_64FC3, cv::Scalar::all(0));

                        ld.send_lens_packet(focus_packets[idx], lens_driver_handle);
                        sleep_ms(100);   // sleep for a little for the lens/camera to settle down

                        std::string voltage_step = num2str(focus_packets[idx].data[0], "%03d_");
                        std::string save_name = combined_save_location + image_capture_name + voltage_step + shutter_str + num2str<uint64_t>(cam_serial_number, "%d_") + sdate + "_" + stime + ".png";
                        std::cout << "Saving image to: " << save_name << std::endl;
                        DataLogStream << save_name << std::endl;

                        std::vector<cv::Mat> cap_img;

                        std::cout << ":";
                        for (jdx = 0; jdx < avg_count; ++jdx)
                        {
                            std::cout << ".";
                            // wait for the camera to be ready for a software trigger
                            poll_trigger_ready(cam);

                            error = fire_software_trigger(cam);
                            error = get_image(cam, image);
                            if (error != FC2::PGRERROR_OK)
                            {
                                print_error(error);
                                std::cout << "Error firing software trigger and receiving image" << std::endl;
                                DataLogStream << "Error firing software trigger and receiving image" << std::endl;
                            }

                            error = fire_software_trigger(cam);
                            error = get_image(cam, image);      // double grab just to be sure the buffer has been flushed
                            if (error != FC2::PGRERROR_OK)
                            {
                                print_error(error);
                                std::cout << "Error firing software trigger and receiving image" << std::endl;
                                DataLogStream << "Error firing software trigger and receiving image" << std::endl;
                            }

                            cv::imshow(image_window, image);
                            cap_img.push_back(image.clone());
                            cv::waitKey(20);

                            cv::add(cap_img[jdx], sum_image, sum_image, cv::Mat(), CV_64FC3);

                        }   // end of jdx - time averaging loop

                        std::cout << ":" << std::endl;

                        sum_image.convertTo(sum_image, CV_8UC3, (1.0 / (double)avg_count));

                        cv::imwrite(save_name, sum_image, compression_params);

                    }   // end of idx - voltage step loop

                    DataLogStream << "------------------------------------------------------------------" << std::endl;
                    std::cout << "------------------------------------------------------------------" << std::endl;

                }   // end of kdx - shutter loop

                std::cout << "Saving Complete!" << std::endl << std::endl;

                // reset the lens driver and camera back to thier initial values
                ld.send_lens_packet(focus_packets[(focus_packets.size()>>1)], lens_driver_handle);

                cam_properties.shutter = shutter[0];
                // config Shutter to initial value and set to auto
                config_property(cam, Shutter, FC2::SHUTTER, false, true, true);
                error = set_abs_property(cam, Shutter, cam_properties.shutter);
                if (error != FC2::PGRERROR_OK)
                {
                    print_error(error);
                }

            }   // end of save 

        } while (key != 'q');
    
        // turn off the software trigger
        error = set_software_trigger(cam, false);
        if (error != FC2::PGRERROR_OK)
        {
            print_error(error);
        }

        // Stop capturing images
        error = cam.StopCapture();
        if (error != FC2::PGRERROR_OK)
        {
            print_error(error);
        }

        // Disconnect the camera
        error = cam.Disconnect();
        if (error != FC2::PGRERROR_OK)
        {
            print_error(error);
        }

        close_com_port(lens_driver_handle);

    }
    catch(std::exception e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        DataLogStream << "Error: " << e.what() << std::endl;
    }

    DataLogStream.close();
    cv::destroyAllWindows();
    std::cout << "Program Compete!" << std::endl;
    //std::cin.ignore();
    return 0;
    
}   // end of main
    
    