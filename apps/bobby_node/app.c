#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif


// Bobby Code Start //
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "portable.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "main.h"

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float64_multi_array.h>

#define STRING_BUFFER_LEN 100
//#define STRING_BUFFER_LEN 50
extern QueueHandle_t sensorQueueHandle;
extern QueueHandle_t encoderQueueHandle;
extern QueueHandle_t motorctrlQueueHandle;
// Bobby Code End //


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t ping_publisher;
rcl_publisher_t pong_publisher;
rcl_subscription_t ping_subscriber;
rcl_subscription_t pong_subscriber;

std_msgs__msg__Header incoming_ping;
std_msgs__msg__Header outcoming_ping;
std_msgs__msg__Header incoming_pong;

// Bobby Code Start //

rcl_publisher_t jointstate_publisher;
#define ARRAY_SIZE 1
//sensor_msgs__msg__JointState jointstate_data;
sensor_msgs__msg__JointState jointstate_data;
static double position_buffer[ARRAY_SIZE];
static double velocity_buffer[ARRAY_SIZE];

#define ARRAY_SIZE_2 2
rcl_subscription_t position_subscriber;
std_msgs__msg__Float64MultiArray motorcontrol_data;
static rosidl_runtime_c__double__Sequence target_buffer[ARRAY_SIZE_2];


//Work to receive following ROS2 Message
//std_msgs.msg.Float64MultiArray(layout=std_msgs.msg.MultiArrayLayout(dim=[], data_offset=0), data=[0.5, 0.5])




// Bobby Code End //

int device_id;
int seq_no;
int pong_count;

void ping_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);

	if (timer != NULL) {

		seq_no = rand();
		sprintf(outcoming_ping.frame_id.data, "%d_%d", seq_no, device_id);
		outcoming_ping.frame_id.size = strlen(outcoming_ping.frame_id.data);

		// Fill the message timestamp
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		outcoming_ping.stamp.sec = ts.tv_sec;
		outcoming_ping.stamp.nanosec = ts.tv_nsec;

		// Reset the pong count and publish the ping message
		pong_count = 0;
		rcl_publish(&ping_publisher, (const void*)&outcoming_ping, NULL);
		printf("Ping send seq %s\n", outcoming_ping.frame_id.data);


		if(xQueueReceive(encoderQueueHandle, &ENCODER_1, 90)){
						// Publish Joint State

			position_buffer[0] = ENCODER_1.position; // degrees in radians
			jointstate_data.position.data = position_buffer; // degrees in radians

			velocity_buffer[0] = ENCODER_1.radspsec; //radians per second
			jointstate_data.velocity.data = velocity_buffer; //radians per second

						//printf("IMU: [%.2f, %.2f, %.2f] m/s^2\n",  imu_data.x, imu_data.y, imu_data.z);
						rcl_publish(&jointstate_publisher, (const void*)&jointstate_data, NULL);
				}
	}
}

void ping_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;

	// Dont pong my own pings
	if(strcmp(outcoming_ping.frame_id.data, msg->frame_id.data) != 0){
		printf("Ping received with seq %s. Answering.\n", msg->frame_id.data);
		rcl_publish(&pong_publisher, (const void*)msg, NULL);
	}
}


void pong_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;

	if(strcmp(outcoming_ping.frame_id.data, msg->frame_id.data) == 0) {
		pong_count++;
		printf("Pong for seq %s (%d)\n", msg->frame_id.data, pong_count);
	}
}


void position_subscription_callback(const void * msgin)
{
float tempvalue;
		printf("Position.....Wow\n");
		target_buffer->data = motorcontrol_data.data.data;
		MOTORCTRL_1.target=(float)*target_buffer[0].data;

		  if (! xQueueSend(motorctrlQueueHandle, &MOTORCTRL_1,100)){
			  printf("Failed to write motor data to QueueHandle\n");
			}


}

void appMain(void *argument)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "pingpong_node", "", &support));

	// Create a reliable ping publisher
	RCCHECK(rclc_publisher_init_default(&ping_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping"));

	// Create a best effort pong publisher
	RCCHECK(rclc_publisher_init_best_effort(&pong_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong"));

	// Create a best effort ping subscriber
	RCCHECK(rclc_subscription_init_best_effort(&ping_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping"));

	// Create a best effort  pong subscriber
	RCCHECK(rclc_subscription_init_best_effort(&pong_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong"));

	// Bobby Code Start //
	// Create a best effort jointstate publisher

	RCCHECK(rclc_publisher_init_best_effort(&jointstate_publisher, &node,
					ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/sensor/jointstate"));

	// Create a best effort  pong subscriber
	RCCHECK(rclc_subscription_init_best_effort(&position_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/forward_command_controller_position/commands"));


	// Bobby Code End //

	// Create a 1000 milli seconds ping timer timer,
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), ping_timer_callback));

	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping,
		&ping_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &pong_subscriber, &incoming_pong,
		&pong_subscription_callback, ON_NEW_DATA));

	RCCHECK(rclc_executor_add_subscription(&executor, &position_subscriber, &motorcontrol_data,
		&position_subscription_callback, ON_NEW_DATA));

	// Create and allocate the pingpong messages
	char outcoming_ping_buffer[STRING_BUFFER_LEN];
	outcoming_ping.frame_id.data = outcoming_ping_buffer;
	outcoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

	char incoming_ping_buffer[STRING_BUFFER_LEN];
	incoming_ping.frame_id.data = incoming_ping_buffer;
	incoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

	char incoming_pong_buffer[STRING_BUFFER_LEN];
	incoming_pong.frame_id.data = incoming_pong_buffer;
	incoming_pong.frame_id.capacity = STRING_BUFFER_LEN;

	device_id = rand();



	// Bobby Code Start //
	// Using static memory

	jointstate_data.velocity.capacity = ARRAY_SIZE;
	jointstate_data.velocity.size = ARRAY_SIZE;
	jointstate_data.velocity.data = velocity_buffer;

	jointstate_data.position.capacity = ARRAY_SIZE;
	jointstate_data.position.size = ARRAY_SIZE;
	jointstate_data.position.data = position_buffer;



	double Joint1_Position = 0.34;
	target_buffer[0].data = &Joint1_Position;
	double Joint2_Position = 5.12;
	target_buffer[1].data = &Joint2_Position;
	char myString[7] = "Joint1 ";
	rosidl_runtime_c__String myNamestruct;

	myNamestruct.data = &myString;

	motorcontrol_data.layout.dim.data->label.data = &myString;
	motorcontrol_data.layout.dim.data->label.size = 7;
	motorcontrol_data.layout.dim.data->stride = 0;
	motorcontrol_data.layout.data_offset = 0;
	motorcontrol_data.layout.dim.size = 2;
	motorcontrol_data.layout.dim.capacity = 2;
	motorcontrol_data.data.capacity = 2;
	motorcontrol_data.data.size = 2;
	//motorcontrol_data.data = target_buffer[0];
	motorcontrol_data.data.data = target_buffer[0].data;

	// Bobby Code Stop //

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		usleep(10000);
	}

	// Free resources
	RCCHECK(rcl_publisher_fini(&ping_publisher, &node));
	RCCHECK(rcl_publisher_fini(&pong_publisher, &node));
	RCCHECK(rcl_subscription_fini(&ping_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&pong_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&position_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}
