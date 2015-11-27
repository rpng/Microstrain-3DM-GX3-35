#include <termios.h> // terminal io (serial port) interface
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <assert.h>
#include <signal.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"

#include <glib.h>
#include <gio/gio.h>

#include "micro/microstrain_comm.h"
#include "micro/util.h"

#include "libbot/timestamp.h"
#include "libbot/rotations.h"
#include "libbot/small_linalg.h"
#include "libbot/ringbuf.h"

using namespace std;

// Global reference to Glib main loop
static GMainLoop* mainloop = NULL;

// Global ROS publisher
ros::Publisher imu_data_pub_;

// Core app self structure
class app_t {
public:
    //com port communication
    int  comm;  //com port fd
    char comm_port_name[255];
    unsigned int baud_rate;
    unsigned int data_rate;

    //buffers: com port buffer -> read_buffer -> input_buffer
    Byte input_buffer[INPUT_BUFFER_SIZE];
    BotRingBuf* read_buffer;

    char current_segment;         //header or payload
    int  expected_segment_length; //packet length

    //state flags (not currently be used, but may be in future)
    bool changed_baud_rate;
    bool changed_data_rate;
    bool in_continuous_mode;

    //boolean setting flags
    bool verbose; //for normal info print
    bool debug;   //for error info print
    bool little_endian;

    int64_t utime;
    int64_t utime_prev;

    string channal;

    //Our imu message
    sensor_msgs::Imu reading;

    bot_timestamp_sync_state* sync;
    bool do_sync; //sync
};

//sig_action()--------------------------------------------------------------------------------
//callback function that will exit the glib loop.
//--------------------------------------------------------------------------------------------
static void sig_action(int signal, siginfo_t* s, void* user)
{
    // kill the glib main loop...
    if (g_main_loop_is_running(mainloop)) {
        g_main_loop_quit(mainloop);
    }
}

//install_signal_handler()--------------------------------------------------------------------
//set our signal callback, and have it called on SIGINT, SIGTERM, SIGKILL, SIGHUP.
//--------------------------------------------------------------------------------------------
void install_signal_handler()
{
    struct sigaction action;
    action.sa_sigaction = sig_action;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;

    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGKILL, &action, NULL);
    sigaction(SIGHUP, &action, NULL);
}

//API Functions-------------------------------------------------------------------------------
//they are for opening com port with proper settings and finding attached microstrain devices.
//--------------------------------------------------------------------------------------------
//scandev()-----------------------------------------------------------------------------------
bool scandev(char* comm_port_name) {
    FILE *instream;
    char dev_names[255][255]; //allows for up to 256 devices with path links up to 255 characters long each
    int dev_count = 0;
    int usr_choice = 0; //default
    int i, j;

    cout << "Searching for MicroStrain devices..." << endl;

    char command[] = "find /dev/serial -print | grep -i microstrain"; //search /dev/serial for microstrain devices
    instream = popen(command, "r"); //execute piped command in read mode

    if (!instream) { //SOMETHING WRONG WITH THE SYSTEM COMMAND PIPE...EXITING
        cout << "Error: failed in opening pipeline : " << command << endl;
        return false;
    }

    for (i = 0; i < 255 && (fgets(dev_names[i], sizeof(dev_names[i]), instream)); ++i) { //load char array of device addresses
        ++dev_count;
    }

    for (i = 0; i < dev_count; ++i) {
        for (j = 0; j < sizeof(dev_names[i]); ++j) {
            if (dev_names[i][j] == '\n') {
                dev_names[i][j] = '\0'; //replaces newline inserted by pipe reader with char array terminator character
                break; //breaks loop after replacement
            }
        }
        cout << "Device found: " << i << " : " << dev_names[i] << endl;
    }

    //CHOOSE DEVICE TO CONNECT TO AND CONNECT TO IT (IF THERE ARE CONNECTED DEVICES)
    if (dev_count > 0) {
        if (dev_count > 1) {
            fprintf(stderr, "Please choose a device to connect (0 to %i):\n", dev_count - 1);
            while (scanf("%i", &usr_choice) == 0 || usr_choice < 0 || usr_choice > dev_count - 1) { //check that there's input and in the correct range
                fprintf(stderr, "Invalid choice... Please choose one between 0 and %d:\n", dev_count - 1);
                getchar(); //clear carriage return from keyboard buffer after invalid choice
            }
        }

        strcpy(comm_port_name, dev_names[usr_choice]);
        return true;
    } else {
        fprintf(stderr, "No MicroStrain devices found\n");
        return false;
    }
}

//setup_com_port()----------------------------------------------------------------------------
int setup_com_port(ComPortHandle comPort, speed_t baudRate) {
    struct termios options;

    //get the current options
    tcgetattr(comPort, &options);

    //set the desired baud rate (default for MicroStrain is 115200)
    //int baudRate = B115200;
    cfsetospeed(&options, baudRate);
    cfsetispeed(&options, baudRate);

    //set the number of data bits.
    options.c_cflag &= ~CSIZE; //mask the character size bits
    options.c_cflag |= CS8;

    //set the number of stop bits to 1
    options.c_cflag &= ~CSTOPB;

    //set parity to None
    options.c_cflag &= ~PARENB;

    //set for non-canonical (raw processing, no echo, etc.)
    options.c_iflag = IGNPAR; //ignore parity check close_port(int
    options.c_oflag = 0;      //raw output
    options.c_lflag = 0;      //raw input

    //time-outs -- won't work with NDELAY option in the call to open
    options.c_cc[VMIN]  = 0;   //block reading until RX x characers. If x = 0, it is non-blocking.
    options.c_cc[VTIME] = 100; //Inter-Character Timer -- i.e. timeout= x*.1 s

    //set local mode and enable the receiver
    options.c_cflag |= (CLOCAL | CREAD);

    //Purge com port I\O buffers-----------------
    tcflush(comPort, TCIOFLUSH);

    //set the new options
    int status = tcsetattr(comPort, TCSANOW, &options);

    if (status != 0) {
        fprintf(stderr, "Error: failed in configuring com port\n");
        return status;
    }

    //Purge com port I\O buffers-----------------
    tcflush(comPort, TCIOFLUSH);

    return comPort;
}

//open_com_port()-----------------------------------------------------------------------------
//it opens a com port with the correct settings.
//tweaked OpenComPort()- added baud rate argument and split it into two steps.
//--------------------------------------------------------------------------------------------
int open_com_port(const char* comPortPath, speed_t baudRate) {
    int comPort = open(comPortPath, O_RDWR | O_NOCTTY);

    if (comPort == -1) {
        fprintf(stderr, "Error: could not open the com port %x : %i\n", comPortPath, errno);
        return -1;
    }

    return setup_com_port(comPort, baudRate);
}

//cksum()-------------------------------------------------------------------------------------
//it uses 2-byte Fletcher ckecksum algorithm.
//--------------------------------------------------------------------------------------------
unsigned short cksum(const Byte* packet_bytes, int packet_length) {
    uint8_t checksum_byte1 = 0;
    uint8_t checksum_byte2 = 0;

    for (int i = 0; i < packet_length - 2; ++i) {
        checksum_byte1 += packet_bytes[i];
        checksum_byte2 += checksum_byte1;
    }

    return ((uint16_t) checksum_byte1 << 8) + ((uint16_t) checksum_byte2);
}

//handle_message()----------------------------------------------------------------------------
//it parses the content in payload bytes.
//--------------------------------------------------------------------------------------------
bool handle_message(app_t* app) {
    bool success = false;

    uint8_t header_byte_set_desc;
    uint8_t header_byte_payload_length;

    uint8_t field_1_byte_length,   field_2_byte_length,   field_3_byte_length;   //field_4_...
    uint8_t field_1_byte_cmd_desc, field_2_byte_cmd_desc, field_3_byte_cmd_desc; //field_4_...
    //for command set reply
    uint8_t field_1_byte_cmd_echo, field_2_byte_cmd_echo, field_3_byte_cmd_echo; //field_4_...
    uint8_t field_1_byte_err_code, field_2_byte_err_code, field_3_byte_err_code; //field_4_...
    //for data set reply
    uint8_t field_1_byte_data,     field_2_byte_data,     field_3_byte_data;     //field_4_...

    uint8_t len_payload;
    uint8_t len_field_1, len_field_2, len_field_3; //len_field_4

    float float_vals[9] = {0}; //for the longest data format (36-byte) convert

    int ins_timer;
    int64_t utime = bot_timestamp_now(); //get current timestamp
    int64_t utime_nosyn = utime;

    if (app->verbose)
        print_array_char_hex((unsigned char*) app->input_buffer, app->expected_segment_length);

    //Byte indices(index) in MIP packet
    header_byte_set_desc = 2;
    header_byte_payload_length = 3;
    len_payload = (uint8_t) app->input_buffer[header_byte_payload_length];

    field_1_byte_length   = 4;
    field_1_byte_cmd_desc = 5;
    field_1_byte_cmd_echo = 6; //for command set reply
    field_1_byte_err_code = 7; //for command set reply
    field_1_byte_data     = 6; //for data set reply
    len_field_1 = (uint8_t) app->input_buffer[field_1_byte_length];

    if (len_payload - len_field_1 != 0) {
        field_2_byte_length   = field_1_byte_length + len_field_1;
        field_2_byte_cmd_desc = field_2_byte_length + 1;
        field_2_byte_cmd_echo = field_2_byte_length + 2;
        field_2_byte_err_code = field_2_byte_length + 3;
        field_2_byte_data     = field_2_byte_length + 2;
        len_field_2 = (uint8_t) app->input_buffer[field_2_byte_length];

        if (len_payload - len_field_1 - len_field_2 != 0) {
            field_3_byte_length   = field_2_byte_length + len_field_2;
            field_3_byte_cmd_desc = field_3_byte_length + 1;
            field_3_byte_cmd_echo = field_3_byte_length + 2;
            field_3_byte_err_code = field_3_byte_length + 3;
            field_3_byte_data     = field_3_byte_length + 2;
            len_field_3 = (uint8_t) app->input_buffer[field_3_byte_length];

            //if (len_payload - len_field_1 - len_field_2 - len_field_3 != 0){
                 //field_4_...
                 //field_4_...
                 //field_4_...
                 //field_4_...
                 //field_4_...
                 //len_field_4...
            //}
        }
    }

    //parsing message... ...
    switch (app->input_buffer[header_byte_set_desc]) {
        case BASE_COMMAND_SET: {
            if (app->input_buffer[field_1_byte_cmd_desc] == BASE_COMMAND_REPLY) {//for Reply Field 1: ACK/NACK
                switch (app->input_buffer[field_1_byte_cmd_echo]) {
                    case PING: {
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_NONE) {
                            fprintf(stderr, "Received [Ping] command echo : no error\n");
                            success = true;
                        }
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_UNKNOWN_COMMAND)
                            fprintf(stderr, "Received [Ping] command echo : unknown command\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_CHECKSUM_INVALID)
                            fprintf(stderr, "Received [Ping] command echo : checksum invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_PARAMETER_INVALID)
                            fprintf(stderr, "Received [Ping] command echo : parameter invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_FAILED)
                            fprintf(stderr, "Received [Ping] command echo : command failed\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_TIMEOUT)
                            fprintf(stderr, "Received [Ping] command echo : command timeout\n");

                        break;
                    }
                    case SET_TO_IDLE: {
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_NONE) {
                            fprintf(stderr, "Received [Set To Idle] command echo : no error\n");
                            success = true;
                        }
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_UNKNOWN_COMMAND)
                            fprintf(stderr, "Received [Set To Idle] command echo : unknown command\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_CHECKSUM_INVALID)
                            fprintf(stderr, "Received [Set To Idle] command echo : checksum invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_PARAMETER_INVALID)
                            fprintf(stderr, "Received [Set To Idle] command echo : parameter invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_FAILED)
                            fprintf(stderr, "Received [Set To Idle] command echo : command failed\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_TIMEOUT)
                            fprintf(stderr, "Received [Set To Idle] command echo : command timeout\n");

                        break;
                    }
                    case GET_DEV_INFO:
                    case GET_DEV_DESC:
                    case DEV_BUILT_IN_TEST:
                    case RESUME: {
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_NONE) {
                            fprintf(stderr, "Received [Resume] command echo : no error\n");
                            success = true;
                        }
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_UNKNOWN_COMMAND)
                            fprintf(stderr, "Received [Resume] command echo : unknown command\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_CHECKSUM_INVALID)
                            fprintf(stderr, "Received [Resume] command echo : checksum invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_PARAMETER_INVALID)
                            fprintf(stderr, "Received [Resume] command echo : parameter invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_FAILED)
                            fprintf(stderr, "Received [Resume] command echo : command failed\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_TIMEOUT)
                            fprintf(stderr, "Received [Resume] command echo : command timeout\n");

                        break;
                    }
                    case DEV_RESET: {
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_NONE) {
                            fprintf(stderr, "Received [Device Reset] command echo : no error\n");
                            success = true;
                        }
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_UNKNOWN_COMMAND)
                            fprintf(stderr, "Received [Device Reset] command echo : unknown command\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_CHECKSUM_INVALID)
                            fprintf(stderr, "Received [Device Reset] command echo : checksum invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_PARAMETER_INVALID)
                            fprintf(stderr, "Received [Device Reset] command echo : parameter invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_FAILED)
                            fprintf(stderr, "Received [Device Reset] command echo : command failed\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_TIMEOUT)
                            fprintf(stderr, "Received [Device Reset] command echo : command timeout\n");

                        break;
                    }
                    default: {
                        fprintf(stderr, "Base Command Reply : nothing\n");
                        break;

                    }
                }//switch
            }//if

            //if (app->input_buffer[3 + field_1_length + 2] == 0x81/0x83/...) {//for Reply Field 2: ... ...
            //
            //}

            break;
        }

        case DM_COMMAND_SET: {
            if (app->input_buffer[field_1_byte_cmd_desc] == BASE_COMMAND_REPLY) {//for Reply Field 1: ACK/NACK
                switch (app->input_buffer[field_1_byte_cmd_echo]) {
                    case AHRS_MESSAGE_FORMAT: {
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_NONE) {
                            fprintf(stderr, "Received [AHRS Message Format] command echo : no error\n");
                            success = true;
                        }
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_UNKNOWN_COMMAND)
                            fprintf(stderr, "Received [AHRS Message Format] command echo : unknown command\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_CHECKSUM_INVALID)
                            fprintf(stderr, "Received [AHRS Message Format] command echo : checksum invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_PARAMETER_INVALID)
                            fprintf(stderr, "Received [AHRS Message Format] command echo : parameter invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_FAILED)
                            fprintf(stderr, "Received [AHRS Message Format] command echo : command failed\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_TIMEOUT)
                            fprintf(stderr, "Received [AHRS Message Format] command echo : command timeout\n");

                        break;
                    }
                    case GPS_MESSAGE_FORMAT: {
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_NONE) {
                            fprintf(stderr, "Received [GPS Message Format] command echo : no error\n");
                            success = true;
                        }
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_UNKNOWN_COMMAND)
                            fprintf(stderr, "Received [GPS Message Format] command echo : unknown command\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_CHECKSUM_INVALID)
                            fprintf(stderr, "Received [GPS Message Format] command echo : checksum invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_PARAMETER_INVALID)
                            fprintf(stderr, "Received [GPS Message Format] command echo : parameter invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_FAILED)
                            fprintf(stderr, "Received [GPS Message Format] command echo : command failed\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_TIMEOUT)
                            fprintf(stderr, "Received [GPS Message Format] command echo : command timeout\n");

                        break;
                    }
                    case EN_DEV_CONT_DATA_STREAM: {
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_NONE) {
                            fprintf(stderr, "Received [Enable Device Continuous Data Stream] command echo : no error\n");
                            success = true;
                        }
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_UNKNOWN_COMMAND)
                            fprintf(stderr, "Received [Enable Device Continuous Data Stream] command echo : unknown command\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_CHECKSUM_INVALID)
                            fprintf(stderr, "Received [Enable Device Continuous Data Stream] command echo : checksum invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_PARAMETER_INVALID)
                            fprintf(stderr, "Received [Enable Device Continuous Data Stream] command echo : parameter invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_FAILED)
                            fprintf(stderr, "Received [Enable Device Continuous Data Stream] command echo : command failed\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_TIMEOUT)
                            fprintf(stderr, "Received [Enable Device Continuous Data Stream] command echo : command timeout\n");

                        break;
                    }
                    case SAVE_DEV_STARTUP_SETTING: {
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_NONE) {
                            fprintf(stderr, "Received [Save Device Startup Setting] command echo : no error\n");
                            success = true;
                        }
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_UNKNOWN_COMMAND)
                            fprintf(stderr, "Received [Save Device Startup Setting] command echo : unknown command\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_CHECKSUM_INVALID)
                            fprintf(stderr, "Received [Save Device Startup Setting] command echo : checksum invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_PARAMETER_INVALID)
                            fprintf(stderr, "Received [Save Device Startup Setting] command echo : parameter invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_FAILED)
                            fprintf(stderr, "Received [Save Device Startup Setting] command echo : command failed\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_TIMEOUT)
                            fprintf(stderr, "Received [Save Device Startup Setting] command echo : command timeout\n");

                        break;
                    }
                    case UART_BAUD_RATE: {
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_NONE) {
                            fprintf(stderr, "Received [UART Baud Rate] command echo : no error\n");
                            success = true;
                        }
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_UNKNOWN_COMMAND)
                            fprintf(stderr, "Received [UART Baud Rate] command echo : unknown command\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_CHECKSUM_INVALID)
                            fprintf(stderr, "Received [UART Baud Rate] command echo : checksum invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_PARAMETER_INVALID)
                            fprintf(stderr, "Received [UART Baud Rate] command echo : parameter invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_FAILED)
                            fprintf(stderr, "Received [UART Baud Rate] command echo : command failed\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_TIMEOUT)
                            fprintf(stderr, "Received [UART Baud Rate] command echo : command timeout\n");

                        break;
                    }
                    default: {
                        fprintf(stderr, "DM Command Reply : nothing\n");
                        break;
                    }
                }//switch
            }//if
            break;
        }

        case SYS_COMMAND_SET: {
            if (app->input_buffer[field_1_byte_cmd_desc] == BASE_COMMAND_REPLY) {//for Reply Field 1: ACK/NACK
                switch (app->input_buffer[field_1_byte_cmd_echo]) {
                    case COMMUNICATION_MODE: {
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_NONE) {
                            fprintf(stderr, "Received [Communication Mode] command echo : no error\n");
                            success = true;
                        }
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_UNKNOWN_COMMAND)
                            fprintf(stderr, "Received [Communication Mode] command echo : unknown command\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_CHECKSUM_INVALID)
                            fprintf(stderr, "Received [Communication Mode] command echo : checksum invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_PARAMETER_INVALID)
                            fprintf(stderr, "Received [Communication Mode] command echo : parameter invalid\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_FAILED)
                            fprintf(stderr, "Received [Communication Mode] command echo : command failed\n");
                        if (app->input_buffer[field_1_byte_err_code] == MIP_ACK_NACK_ERROR_COMMAND_TIMEOUT)
                            fprintf(stderr, "Received [Communication Mode] command echo : command timeout\n");

                        break;
                    }
                    default: {
                        fprintf(stderr, "System Command Reply : nothing\n");
                        break;
                    }
                }//switch
            }//if
            break;
        }

        case AHRS_DATA_SET: {
            switch (app->input_buffer[field_1_byte_cmd_desc]) {
                case SCALED_ACC_VECTOR: {
                    pack32BitFloats(float_vals, &app->input_buffer[field_1_byte_data], 3, app->little_endian);
                    //cout << "acc: " << float_vals[0] * GRAVITY << ", " << float_vals[1] * GRAVITY << ", " << float_vals[2] * GRAVITY << endl;
                    app->reading.linear_acceleration.x = float_vals[0] * GRAVITY; //~g * 9.8m/s
                    app->reading.linear_acceleration.y = float_vals[1] * GRAVITY; //~g * 9.8m/s
                    app->reading.linear_acceleration.z = float_vals[2] * GRAVITY; //~g * 9.8m/s

                    break;
                }
                //case ...:
                    //... ...
                    //break;
                default:
                    break;
            }
            switch (app->input_buffer[field_2_byte_cmd_desc]) {
                case SCALED_GYRO_VECTOR: {
                    pack32BitFloats(float_vals, &app->input_buffer[field_2_byte_data], 3, app->little_endian);
                    //cout << "gyro: " << float_vals[0] << ", " << float_vals[1] << ", " << float_vals[2] << endl;
                    app->reading.angular_velocity.x = float_vals[0]; //rad/s
                    app->reading.angular_velocity.y = float_vals[1]; //rad/s
                    app->reading.angular_velocity.z = float_vals[2]; //rad/s

                    break;
                }
                //case ...:
                    //... ...
                    //break;
                default:
                    break;
            }
            switch (app->input_buffer[field_3_byte_cmd_desc]) {
                case INTERNAL_TIME_STAMP: {
                    ins_timer = make32UnsignedInt(&app->input_buffer[field_3_byte_data], app->little_endian);
                    //cout << "timestamp: " << ins_timer << endl;

                    if (app->do_sync) {
                        app->utime = bot_timestamp_sync(app->sync, ins_timer, utime);
                    } else {
                        app->utime = utime_nosyn;
                    }
                    //cout << "timestamp: " << app-utime << endl;

                    app->reading.header.stamp = ros::Time::now(); ////////////////////////////

                    break;
                }
                //case ...:
                    //... ...
                    //break;
                default:
                    break;
            }

            success = true;

            //publish to imu topic
            imu_data_pub_.publish(app->reading);

            break;
        }

        case GPS_DATA_SET: {
            switch (app->input_buffer[field_1_byte_cmd_desc]) {
                case ECEF_POSITION:
                    //... ...
                    break;
                default:
                    break;
            }
            switch (app->input_buffer[field_2_byte_cmd_desc]) {
                case ECEF_VELOCITY:
                    //... ...
                    break;
                default:
                    break;
            }

            success = true;
            break;
        }

        default: {
            if (app->debug)
                fprintf(stderr, "Error: unknown message with the starting byte : 0x%x\n", app->input_buffer[0]);
            break;
        }
    }//switch

    return success;
}

//unpack_packets()----------------------------------------------------------------------------
//it gets data packets out of ring buffer.
//it has two states, either parsing the header bytes or parsing payload and checksum bytes.
//it will be waiting until it has all expected bytes before taking appropriate action.
//--------------------------------------------------------------------------------------------
void unpack_packets(app_t* app) {

    while (bot_ringbuf_available(app->read_buffer) >= app->expected_segment_length) {

        switch (app->current_segment) {
        case 'h':
            bot_ringbuf_peek(app->read_buffer, 4, app->input_buffer); //peek packet header
            if (app->verbose)
                fprintf(stderr, "Received packet with the starting byte : 0x%x\n", app->input_buffer[0]);
            //parse header
            if ((app->input_buffer[0] == MIP_PACKET_SYNC1) &&
                (app->input_buffer[1] == MIP_PACKET_SYNC2)) {
                app->expected_segment_length = LEN_PACKET_HEADER + app->input_buffer[3] + LEN_PACKET_CHECKSUM;
                app->current_segment = 'p';
            } else {
                if (app->debug)
                    fprintf(stderr, "Error: unknown packet with the starting byte : 0x%x\n", app->input_buffer[0]);
                //read 1 byte and continue until we find a match one
                bot_ringbuf_read(app->read_buffer, 1, app->input_buffer); //read
                app->current_segment = 'h';
                break;
            }
            break;

        case 'p':
            bot_ringbuf_read(app->read_buffer, app->expected_segment_length, app->input_buffer); //read whole packet

            unsigned short inpacket_cksum = make16UnsignedInt(&app->input_buffer[app->expected_segment_length - 2], app->little_endian);
            unsigned short computed_cksum = cksum(app->input_buffer, app->expected_segment_length);

            if (computed_cksum != inpacket_cksum) {
                if (app->debug)
                    fprintf(stderr, "Error: failed in checksum! got : %d, expected : %d\n", inpacket_cksum, computed_cksum);
                app->current_segment = 'h';
                app->expected_segment_length = 4;
                break;
            }
            //parse message
            if (app->verbose)
                fprintf(stderr, "Passed checksum, handling message...\n");
            bool success = handle_message(app);

            if (!success && app->debug)
                fprintf(stderr, "Error: failed in handling data message\n");

            app->current_segment = 'h';
            app->expected_segment_length = 4;
            break;
        }
    }
}

//set_commands()------------------------------------------------------------------------------
//it generates the commands for setting AHRS and GPS.
//it uses std::string to operate the commands, so that user can modify the contents easily.
//--------------------------------------------------------------------------------------------
bool set_commands(app_t* app, const char* command_header, const char* command_field_1, const char* command_field_2) {
    cout << "in function set_commands()" << endl;

    string command_t;
    string command_field_1_t, command_field_2_t;
    string len_cmd_field_1_t, len_cmd_field_2_t;
    string checksum_byte_t;

    cout << "1===================================" << endl;
    for (int i = 0; command_header[i] != 'x'; ++i) {
        fprintf(stderr, "%x ", command_header[i]);
        command_t += command_header[i];
    }
    fprintf(stderr, "\n");
    cout << "length of command header: " << command_t.size() << endl;

    cout << "2===================================" << endl;
    if (command_field_1 != 0) {
        for (int j = 0; command_field_1[j] != 'x'; ++j) {
            fprintf(stderr, "%x ", command_field_1[j]);
            command_field_1_t += command_field_1[j];
        }
        fprintf(stderr, "\n");
        len_cmd_field_1_t += (unsigned char) command_field_1_t.size();
        cout << "length of 1st command field: " << command_field_1_t.size() << endl;
        command_t.replace(3, 1, len_cmd_field_1_t); //reset the payload length
        command_t += command_field_1_t; //combine this field to payload
    }

    cout << "3===================================" << endl;
    if (command_field_2 != 0) {
        for (int k = 0; command_field_2[k] != 'x'; ++k) {
            fprintf(stderr, "%x ", command_field_2[k]);
            command_field_2_t += command_field_2[k];
        }
        fprintf(stderr, "\n");
        len_cmd_field_2_t += (unsigned char) (command_field_1_t.size() + command_field_2_t.size());
        cout << "length of 2nd command field: " << command_field_2_t.size() << endl;
        command_t.replace(3, 1, len_cmd_field_2_t); //reset the payload length
        command_t += command_field_2_t; //combine this field to payload
    } else {
        cout << "length of 2nd command field: " << 0 << endl;
    }

    cout << "4===================================" << endl;
    uint8_t checksum_byte_1 = 0;
    uint8_t checksum_byte_2 = 0;
    for (string::iterator it = command_t.begin(); it != command_t.end(); ++it) {//2-byte Fletcher checksum
        checksum_byte_1 += *it;
        checksum_byte_2 += checksum_byte_1;
    }
    checksum_byte_t += (unsigned char) checksum_byte_1;
    checksum_byte_t += (unsigned char) checksum_byte_2;
    fprintf(stderr, "%x ", checksum_byte_1);
    fprintf(stderr, "%x\n", checksum_byte_2);
    cout << "length of checksum: " << checksum_byte_t.size() << endl;

    command_t += checksum_byte_t; //combine checksum bytes to command

    cout << "5===================================" << endl;
    //for (int m = 0; m < command_t.size(); ++m) {
    //    fprintf(stderr, "%x ", command_t[m]);
    //}
    //fprintf(stderr, "\n");

    char* command = new char [command_t.size() + 1]; //allocated on heap; +1 means add "\0" to the end
    memcpy(command, command_t.c_str(), command_t.size() + 1);
    for (int n = 0; n < command_t.size(); ++n) {
        fprintf(stderr, "%x ", command[n]);
    }
    fprintf(stderr,"\n");
    cout << "length of command: " << command_t.size() << endl;

    if (write(app->comm, command, command_t.size() + 1) != (command_t.size() + 1)) {
        cout << "Error: failed in sending the command" << endl;
        return false;
    }

    delete[] command;

    cout << "6===================================" << endl;
    if (read(app->comm, app->input_buffer, LEN_REPLY_HEADER) == -1) {
        cout << "Error: failed in receiving the header of command echo" << endl;
        return false;
    }

    uint8_t len_reply_payload = (uint8_t) app->input_buffer[3];
    app->expected_segment_length = LEN_REPLY_HEADER + len_reply_payload + LEN_REPLY_CHECKSUM;
    if (read(app->comm, &app->input_buffer[LEN_REPLY_HEADER], len_reply_payload + LEN_REPLY_CHECKSUM) == -1) {
        cout << "Error: failed in receiving the rest of command echo" << endl;
        return false;
    }

    for (int l = 0; l < app->expected_segment_length; ++l) {
        fprintf(stderr, "%x " , app->input_buffer[l]);
    }
    fprintf(stderr, "\n");
    cout << "length of reply: " << app->expected_segment_length << endl;

    cout << "7===================================" << endl;
    if (app->input_buffer[0] == MIP_PACKET_SYNC1 &&
        app->input_buffer[1] == MIP_PACKET_SYNC2) {
        unsigned short inpacket_cksum = make16UnsignedInt(&app->input_buffer[app->expected_segment_length - 2], app->little_endian);
        unsigned short computed_cksum = cksum(app->input_buffer, app->expected_segment_length);
        if (computed_cksum != inpacket_cksum) {
            fprintf(stderr, "Error: failed in checksum! got : %d, expected : %d\n", inpacket_cksum, computed_cksum);
            return false;
        }

        if (handle_message(app) == false) {
            cout << "Error: failed in handling echo message" << endl;
            return false;
        }
    } else {
        fprintf(stderr, "Error: unknown echo with the starting byte : 0x%x\n", app->input_buffer[0]);
        exit(1);
    }

    cout << "out of function set_commands()" << endl;
    return true;
}

/**
  * This is the callback function from the glib main loop
  * Reads serial bytes from ardu as they become available from g_io_watch
  * These bytes are then writen to a circular buffer and the `unpack_packets` method is called.
  */
static gboolean serial_read_handler(GIOChannel* source, GIOCondition condition, void* user)
{

    // Check to see if the user has requested a stop
    if (!ros::ok()) {
        g_main_loop_quit(mainloop);
        return true;
    }

    app_t* app = (app_t*) user;

    static uint8_t middle_buffer[INPUT_BUFFER_SIZE];

    // get number of bytes available
    int available = 0;

    if (ioctl(app->comm, FIONREAD, &available) != 0) {
        if (!app->debug)
            fprintf(stderr, "ioctl check for bytes available didn't return 0, breaking read\n");
        return true;
    }

    if (available > INPUT_BUFFER_SIZE) {
        if (!app->debug)
            fprintf(stderr, "too many bytes available: %d, flushing input buffer\n", available);
        tcflush(app->comm, TCIFLUSH);
        return true;
    }

    int num_read = read(app->comm, middle_buffer, available);

    if (num_read != available) {
        if (!app->debug)
            fprintf(stderr, "warning, read %d of %d available bytes\n", num_read, available);
    }

    if (num_read > 0) {
        bot_ringbuf_write(app->read_buffer, num_read, middle_buffer);
    }

    unpack_packets(app);

    return true;
}

/**
 * Our main method
 * This method first reads in all parameter information
 * After setting the configuration of the driver, the imu is set to continuous mode
 * This main glib loop is what waits for data from the imu untill the proccess is ended
 */
int main(int argc, char** argv)
{
    app_t* app = new app_t();
    app->little_endian = systemLittleEndianCheck();

    ros::init(argc, argv, "microstrain_comm");
    ros::NodeHandle nh("~");

    // Our publisher
    ros::NodeHandle imu_node_handle("imu");
    imu_data_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("data", 100);

    // Default settings
    string user_comm_port_name;
    string data_rate;
    string dev_init;

    // Get our params from the config file, or command line
    nh.param("verbose", app->verbose, false);
    nh.param("debug", app->debug, false);
    nh.param("com_port", user_comm_port_name, string(""));
    nh.param("rate", data_rate, string("low"));
    nh.param("time_sync", app->do_sync, true);
    nh.param("init", dev_init, string("yes"));

    // Data rate (which also determines baud rate)
    if (data_rate == "low") {
        app->data_rate = DATA_RATE_DEFAULT;
        app->baud_rate = BAUD_RATE_DEFAULT;
    } else if (data_rate == "medium") {
        app->data_rate = DATA_RATE_MED;
        app->baud_rate = BAUD_RATE_MED;
    } else if (data_rate == "high") {
        app->data_rate = DATA_RATE_HIGH;
        app->baud_rate = BAUD_RATE_HIGH;
    } else {
        cerr << "Unknown update rate flag - using default rate" << endl;
    }

    if (!app->debug)
        cout << "Setting data rate to " << app->data_rate << " Hz" << endl;

    if (!app->debug)
        fprintf(stderr, "Little endian = %d\n", (int)app->little_endian);

    mainloop = g_main_loop_new(NULL, FALSE);
    app->utime_prev = bot_timestamp_now();
    app->sync = bot_timestamp_sync_init(62500, (int64_t)68719 * 62500, 1.001);
    app->read_buffer = bot_ringbuf_create(INPUT_BUFFER_SIZE);

    // Use user specified port if there is one
    if (user_comm_port_name == "")
        scandev(app->comm_port_name);
    else
        strcpy(app->comm_port_name, user_comm_port_name.c_str());

    // Initialize comm port at default baud rate
    app->comm = open_com_port(app->comm_port_name, BAUD_RATE_DEFAULT);
    if (app->comm < 0) {
        exit(1);
    }

    // Install signal handler
    install_signal_handler();

    // Simple state machine
    if (dev_init == "yes") {
    cout << "initialization begin..." << endl;

        if (!set_commands(app, DM_Command_Header, Disable_AHRS_Data_Stream, Disable_GPS_Data_Stream)) {
            cout << "ERROR in initialization: disableDataStream failed" << endl;
            exit(1);
        }

        if (!set_commands(app, DM_Command_Header, Set_AHRS_Message_Format, 0)) {
            cout << "ERROR in initialization: setImuDataStreamFormat failed" << endl;
            exit(1);
        }

        if (!set_commands(app, DM_Command_Header, Set_GPS_Message_Format, 0)) {
            cout << "ERROR in initialization: setGpsDataStreamFormat failed" << endl;
            exit(1);
        }

        if (!set_commands(app, DM_Command_Header, Save_AHRS_Message_Format, Save_GPS_Message_Format)) {
            cout << "ERROR in initialization: saveDataStreamFormat failed" << endl;
            exit(1);
        }

        if (!set_commands(app, DM_Command_Header, Enable_AHRS_Data_Stream, Enable_GPS_Data_Stream)) {
            cout << "ERROR in initialization: enableDataStream failed" << endl;
            exit(1);
        }

        cout << "initialization is done" << endl;
    } else {//dev_init = "no"
        if (!set_commands(app, Base_Command_Header, Resume, 0)) {
            cout << "ERROR in initialization: resume device failed" << endl;
            exit(1);
        }
    }

    // Set our current segment for unpacking
    app->current_segment = 'h';
    app->expected_segment_length = 4;

    // Create a glib channel and main thread
    GIOChannel* ioc = g_io_channel_unix_new(app->comm);
    g_io_add_watch_full(ioc, G_PRIORITY_HIGH, G_IO_IN, (GIOFunc)serial_read_handler, (void*)app, NULL);
    g_main_loop_run(mainloop);

    set_commands(app, Base_Command_Header, Set_To_Idle, 0);

    // Close our imu port
    close(app->comm);

    delete app;

    return 0;
}

//Customized commands for MIP protocol--------------------------------------------------------
//it uses an 'x' as the end for each part.
//--------------------------------------------------------------------------------------------
//Headers-------------------------------------------------------------------------------------
char Base_Command_Header[] = { static_cast<char> (MIP_PACKET_SYNC1),
                               static_cast<char> (MIP_PACKET_SYNC2),
                               static_cast<char> (BASE_COMMAND_SET),
                               static_cast<char> (0x02), //Payload length (default)
                               'x'
                             };

char DM_Command_Header[] = { static_cast<char> (MIP_PACKET_SYNC1),
                             static_cast<char> (MIP_PACKET_SYNC2),
                             static_cast<char> (DM_COMMAND_SET),
                             static_cast<char> (0x00), //Payload length (mannual)
                             'x'
                           };

char Sys_Command_Header[] = { static_cast<char> (MIP_PACKET_SYNC1),
                              static_cast<char> (MIP_PACKET_SYNC2),
                              static_cast<char> (SYS_COMMAND_SET),
                              static_cast<char> (0x04), //Payload length (default)
                              'x'
                            };

//Fields--------------------------------------------------------------------------------------
char Set_To_Idle[] = { static_cast<char> (LEN_OF_FIELD(0x02)),
                       static_cast<char> (SET_TO_IDLE),
                       'x'
                     };

char Resume[] = { static_cast<char> (LEN_OF_FIELD(0x02)),
                  static_cast<char> (RESUME),
                  'x'
                };

char Device_Reset[] = { static_cast<char> (LEN_OF_FIELD(0x02)),
                        static_cast<char> (DEV_RESET),
                        'x'
                      };

//-----------------------------------------------
char Disable_AHRS_Data_Stream[] = { static_cast<char> (LEN_OF_FIELD(0x05)),      //Field length
                                    static_cast<char> (EN_DEV_CONT_DATA_STREAM), //Cmd Desc.
                                    static_cast<char> (APPLY_NEW_SETTINGS),      //Action (Apply)--
                                    static_cast<char> (INDEX_OF_DEVICE(0x01)),   //Device (AHRS)--
                                    static_cast<char> (DATA_STREAM_ON(0x00)),    //Stream (Off)--
                                    'x'
                                  };

char Disable_GPS_Data_Stream[] = { static_cast<char> (LEN_OF_FIELD(0x05)),      //Field length
                                   static_cast<char> (EN_DEV_CONT_DATA_STREAM), //Cmd Desc.
                                   static_cast<char> (APPLY_NEW_SETTINGS),      //Action (Apply)--
                                   static_cast<char> (INDEX_OF_DEVICE(0x02)),   //Device (GPS)--
                                   static_cast<char> (DATA_STREAM_ON(0x00)),    //Stream (Off)--
                                   'x'
                                 };

char Set_UART_BAUD_Rate[] = { static_cast<char> (LEN_OF_FIELD(0x07)),
                              static_cast<char> (UART_BAUD_RATE),
                              static_cast<char> (APPLY_NEW_SETTINGS),
                              static_cast<char> (BAUD_RATE_DEFAULT_BYTE_1), //Baud rate h
                              static_cast<char> (BAUD_RATE_DEFAULT_BYTE_2), //Baud rate h
                              static_cast<char> (BAUD_RATE_DEFAULT_BYTE_3), //Baud rate l
                              static_cast<char> (BAUD_RATE_DEFAULT_BYTE_4), //Baud rate l (115200)--
                              'x'
                            };

char Set_AHRS_Message_Format[] = { static_cast<char> (LEN_OF_FIELD(0x0D)),
                                   static_cast<char> (AHRS_MESSAGE_FORMAT),
                                   static_cast<char> (APPLY_NEW_SETTINGS),
                                   static_cast<char> (0x03),                //Desc. count--
                                   static_cast<char> (SCALED_ACC_VECTOR),   //1st Desc. (Acc)--
                                   static_cast<char> (0x00),                //Rate dec h
                                   static_cast<char> (AHRS_DATA_RATE(100)), //Rate dec l (100Hz)--
                                   static_cast<char> (SCALED_GYRO_VECTOR),  //2nd Desc. (Gyro)--
                                   static_cast<char> (0x00),                //Rate dec h
                                   static_cast<char> (AHRS_DATA_RATE(100)), //Rate dec l (100Hz)--
                                   static_cast<char> (INTERNAL_TIME_STAMP), //3rd Desc. (Timestamp)--
                                   static_cast<char> (0x00),                //Rate dec h
                                   static_cast<char> (AHRS_DATA_RATE(100)), //Rate dec l (100Hz)--
                                   'x'
                                 };

char Set_GPS_Message_Format[] = { static_cast<char> (LEN_OF_FIELD(0x0A)),
                                  static_cast<char> (GPS_MESSAGE_FORMAT),
                                  static_cast<char> (APPLY_NEW_SETTINGS),
                                  static_cast<char> (0x02),             //Desc. count--
                                  static_cast<char> (ECEF_POSITION),    //ECEF Pos Desc.--
                                  static_cast<char> (0x00),             //Rate dec h
                                  static_cast<char> (GPS_DATA_RATE(1)), //Rate dec l (1Hz)--
                                  static_cast<char> (ECEF_VELOCITY),    //ECEF Vel Desc.--
                                  static_cast<char> (0x00),             //Rate dec h
                                  static_cast<char> (GPS_DATA_RATE(1)), //Rate dec l (1Hz)--
                                  'x'
                                };

char Save_AHRS_Message_Format[] = { static_cast<char> (LEN_OF_FIELD(0x04)),
                                    static_cast<char> (AHRS_MESSAGE_FORMAT),
                                    static_cast<char> (SAVE_CURRENT_SETTINGS_AS_STARTUP_SETTINGS),
                                    static_cast<char> (0x00), //Desc. count--
                                    'x'
                                  };

char Save_GPS_Message_Format[] = { static_cast<char> (LEN_OF_FIELD(0x04)),
                                   static_cast<char> (GPS_MESSAGE_FORMAT),
                                   static_cast<char> (SAVE_CURRENT_SETTINGS_AS_STARTUP_SETTINGS),
                                   static_cast<char> (0x00), //Desc. count--
                                   'x'
                                 };

char Enable_AHRS_Data_Stream[] = { static_cast<char> (LEN_OF_FIELD(0x05)),
                                   static_cast<char> (EN_DEV_CONT_DATA_STREAM),
                                   static_cast<char> (APPLY_NEW_SETTINGS),
                                   static_cast<char> (INDEX_OF_DEVICE(0x01)), //Device (AHRS)--
                                   static_cast<char> (DATA_STREAM_ON(0x01)),  //Stream (On)--
                                   'x'
                                 };

char Enable_GPS_Data_Stream[] = { static_cast<char> (LEN_OF_FIELD(0x05)),
                                  static_cast<char> (EN_DEV_CONT_DATA_STREAM),
                                  static_cast<char> (APPLY_NEW_SETTINGS),
                                  static_cast<char> (INDEX_OF_DEVICE(0x02)), //Device (GPS)--
                                  static_cast<char> (DATA_STREAM_ON(0x01)),  //Stream (On)--
                                  'x'
                                };

