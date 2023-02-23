#include <stdio.h>
#include <stdint.h>
#include <hidapi/hidapi.h>
#include <pthread.h>
#include "cglm/cglm.h"
#include "../../Fusion/Fusion/Fusion.h"

#define AIR_VID 0x3318
#define AIR_PID 0x0424

// this is a guess
// ticks are in nanoseconds, 1000 Hz packets
#define TICK_LEN (1.0f / 1E9f)

// based on 24bit signed int w/ FSR = +/-2000 dps, datasheet option
#define GYRO_SCALAR (1.0f / 8388608.0f * 2000.0f)
// based on 24bit signed int w/ FSR = +/-16 g, datasheet option
#define ACCEL_SCALAR (1.0f / 8388608.0f * 16.0f)

#define SAMPLE_RATE (1000) // replace this with actual sample rate


static hid_device* device = 0;
static pthread_t thread = 0;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static versor rotation = GLM_QUAT_IDENTITY_INIT;
static vec3 ang_vel = {}, accel_vec = {};
static FusionEuler euler;
static FusionVector earth;

// Initialise algorithms
static FusionOffset offset;
static FusionAhrs ahrs;


static bool running = false;

typedef struct {
	uint64_t tick;
	int32_t ang_vel[3];
	int32_t accel[3];
} air_sample;

static int
parse_report(const unsigned char* buffer, int size, air_sample* out_sample)
{
	if (size != 64) {
		printf("Invalid packet size");
		return -1;
	}
	// clock in nanoseconds
	buffer += 4;
	out_sample->tick = ((uint64_t)*(buffer++)) | (((uint64_t)*(buffer++))  << 8) | (((uint64_t)*(buffer++)) << 16) | (((uint64_t)*(buffer++)) << 24) 
						| (((uint64_t)*(buffer++)) << 32) | (((uint64_t)*(buffer++))  << 40) | (((uint64_t)*(buffer++))  << 48) | (((uint64_t)*(buffer++))  << 56);
	
	// gyroscope measurements
	buffer += 6;
	if( *(buffer+2) & 0x80 ) {
		out_sample->ang_vel[0] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	} else {
		out_sample->ang_vel[0] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}
	
	if( *(buffer+2) & 0x80 ) {
		out_sample->ang_vel[1] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	} else {
		out_sample->ang_vel[1] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}

	if( *(buffer+2) & 0x80 ) {
		out_sample->ang_vel[2] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	} else {
		out_sample->ang_vel[2] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}

	// accelerometer data
	buffer += 6;
	if( *(buffer+2) & 0x80 ) {
		out_sample->accel[0] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	} else {
		out_sample->accel[0] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}
	
	if( *(buffer+2) & 0x80 ) {
		out_sample->accel[1] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	} else {
		out_sample->accel[1] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}

	if( *(buffer+2) & 0x80 ) {
		out_sample->accel[2] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	} else {
		out_sample->accel[2] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}

	return 0;
}

static void
process_ang_vel(const int32_t in_ang_vel[3], vec3 out_vec)
{
	// these scale and bias corrections are all rough guesses
	out_vec[0] = (float)(in_ang_vel[0]) * -1.0f * GYRO_SCALAR;
	out_vec[1] = (float)(in_ang_vel[2]) * GYRO_SCALAR;
	out_vec[2] = (float)(in_ang_vel[1]) * GYRO_SCALAR;
}

static void
process_accel(const int32_t in_accel[3], vec3 out_vec)
{
	// these scale and bias corrections are all rough guesses
	out_vec[0] = (float)(in_accel[0]) * ACCEL_SCALAR;
	out_vec[1] = (float)(in_accel[2]) * ACCEL_SCALAR;
	out_vec[2] = (float)(in_accel[1]) * ACCEL_SCALAR;
}

static hid_device*
open_device()
{
	struct hid_device_info* devs = hid_enumerate(AIR_VID, AIR_PID);
	struct hid_device_info* cur_dev = devs;
	hid_device* dev = NULL;

	while (devs) {
		if (cur_dev->interface_number == 3) {
			dev = hid_open_path(cur_dev->path);
			break;
		}

		cur_dev = cur_dev->next;
	}

	hid_free_enumeration(devs);
	return dev;
}

static void
update_rotation(versor new_rotation)
{
	pthread_mutex_lock(&mutex);
	glm_quat_copy(new_rotation, rotation);
	
	glm_quat_normalize(rotation);
	pthread_mutex_unlock(&mutex);
}

static void*
track(void* arg)
{
	unsigned char buffer[64] = {};
	uint64_t last_sample_tick = 0;
	air_sample sample = {};
	vec3 ang_vel = {};

	// Define calibration (replace with actual calibration data if available)
    const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

	while (running) {
		int res = hid_read(device, (void*)&buffer, sizeof(buffer));
		if (res < 0) {
			printf("Unable to get feature report\n");
			break;
		}

		if (buffer[0] != 0x01 || buffer[1] != 0x02)
			continue;

		parse_report(buffer, sizeof(buffer), &sample);
		process_ang_vel(sample.ang_vel, ang_vel);
		process_accel(sample.accel, accel_vec);

		// Acquire latest sensor data
        const uint64_t timestamp = sample.tick; // replace this with actual gyroscope timestamp
        FusionVector gyroscope = {ang_vel[0], ang_vel[1], ang_vel[2]}; // replace this with actual gyroscope data in degrees/s
        FusionVector accelerometer = {accel_vec[0], accel_vec[1], accel_vec[2]}; // replace this with actual accelerometer data in g
       
        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
		
		// Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

        // Calculate delta time (in seconds) to account for gyroscope sample clock error
        static uint64_t previousTimestamp;
        const float deltaTime = (float) (timestamp - previousTimestamp) / (float) 1e9;
        previousTimestamp = timestamp;

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

		FusionQuaternion temp_quat = FusionAhrsGetQuaternion(&ahrs);
		versor new_rotation = {temp_quat.element.x, temp_quat.element.y, temp_quat.element.z, temp_quat.element.w};

		update_rotation(new_rotation);
	}

	return 0;
}

int
tracking_start()
{



    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .gain = 0.5f,
            .accelerationRejection = 10.0f,
            .magneticRejection = 20.0f,
            .rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);


	device = open_device();
	if (!device) {
		printf("Unable to open device\n");
		return -1;
	}

	uint8_t magic_payload[] = { 0x00, 0xaa, 0xc5, 0xd1, 0x21, 0x42, 0x04, 0x00, 0x19, 0x01 };
	int res = hid_write(device, magic_payload, sizeof(magic_payload));
	if (res < 0) {
		printf("Unable to write to device\n");
		return -2;
	}

	glm_quat_copy(GLM_QUAT_IDENTITY, rotation);

	running = true;
	int err = pthread_create(&thread, NULL, track, NULL);
	if (err) {
		printf("Unable to start tracking\n");
		return -3;
	}

	return 0;
}

void
tracking_get(versor out)
{
	pthread_mutex_lock(&mutex);
	glm_quat_copy(rotation, out);
	pthread_mutex_unlock(&mutex);
}

void
tracking_set(versor ref)
{
	pthread_mutex_lock(&mutex);
	glm_quat_copy(ref, rotation);
	pthread_mutex_unlock(&mutex);
}

void
tracking_stop()
{
	running = false;
	pthread_join(thread, NULL);
}
