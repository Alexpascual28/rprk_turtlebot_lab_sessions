#include <stdio.h>
#include <stdlib.h>
#include "simpleble_c/simpleble.h"

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

void msleep(int ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    usleep(ms * 1000);
#endif
}

/*
    These define the peripheral device we want to connect to
    SimpleBLE doesn't seem to be reading the names properly, so its
    best to rely on the address
*/
#define WANTED_IDENTIFIER "BLEPeripheralTest"
#define WANTED_ADDRESS "fa:46:2e:a5:95:72"

/*
	These define the UUIDs of our services and characteristics
    Even when using the 16-bit UUIDs, SimpleBLE requires the full 128-bit versions
    SimpleBLE also requires all the letters to be lower case
*/
#define HELLO_WORLD_SERVICE_UUID "0000180a-0000-1000-8000-00805f9b34fb" // 0x180A = "Device Information"
#define HELLO_WORLD_TEXT_UUID    "00002a25-0000-1000-8000-00805f9b34fb" // 0x2A25 = "Serial Number String"

#define IR_RANGE_SERVICE_UUID    "73150000-4179-4433-9f08-164aaa2f0de6"
#define IR_RANGE_CHARA1_UUID     "73150001-4179-4433-9f08-164aaa2f0de6"
#define IR_RANGE_CHARA2_UUID     "73150002-4179-4433-9f08-164aaa2f0de6"

#define USONIC_SERVICE_UUID      "5a060000-0027-4090-bbcf-ca3ff3a88f63"
#define USONIC_CHARA1_UUID       "5a060001-0027-4090-bbcf-ca3ff3a88f63"
#define USONIC_CHARA2_UUID       "5a060002-0027-4090-bbcf-ca3ff3a88f63"

#define LED_SERVICE_UUID         "7eca0000-67d9-4473-8015-5dbe1681ae1a"
#define LED_CHARA_R_UUID         "7eca0001-67d9-4473-8015-5dbe1681ae1a"
#define LED_CHARA_G_UUID         "7eca0002-67d9-4473-8015-5dbe1681ae1a"
#define LED_CHARA_B_UUID         "7eca0003-67d9-4473-8015-5dbe1681ae1a"
#define LED_CHARA_MONO_UUID      "7eca0004-67d9-4473-8015-5dbe1681ae1a" // Mono LED is the single colour status LED

// Fucntion prototypes
static void clean_on_exit(void);
static void adapter_on_scan_found(simpleble_adapter_t adapter, simpleble_peripheral_t peripheral, void* userdata);
static simpleble_uuid_t str_to_uuid(char* uuid_str);

static void IR1Notify(simpleble_uuid_t service, simpleble_uuid_t characteristic, const uint8_t* data,
                                 size_t data_length, void* userdata);
static void IR2Notify(simpleble_uuid_t service, simpleble_uuid_t characteristic, const uint8_t* data,
                                 size_t data_length, void* userdata);
								 
static void USonic1Notify(simpleble_uuid_t service, simpleble_uuid_t characteristic, const uint8_t* data,
                                 size_t data_length, void* userdata);
static void USonic2Notify(simpleble_uuid_t service, simpleble_uuid_t characteristic, const uint8_t* data,
                                 size_t data_length, void* userdata);

// Globals

// These hold pointers to the handles for our adapter and our peripheralonce connected
// They are initialised to NULL before we detetect our adapter and connect to our peripheral
static simpleble_adapter_t adapter = NULL;
static simpleble_peripheral_t our_peripheral = NULL;


int main(){
	// This variable will hold an error code if an error occurs
	simpleble_err_t err_code = SIMPLEBLE_SUCCESS;
	
	// Run the clean_on_exit function when the program ends
	atexit(clean_on_exit);
	
	// Get number of BT adapters. If none are found, exit
	size_t adapter_count = simpleble_adapter_get_count();
    if (adapter_count == 0) {
        printf("No adapter was found.\n");
        return 1;
    }

    // Select the first adapter. Most of the time this will be the only one
    adapter = simpleble_adapter_get_handle(0);
    if (adapter == NULL) {
        printf("No adapter was found.\n");
        return 1;
    }
	
	// Set our handler function for discovering peripherals
	simpleble_adapter_set_callback_on_scan_found(adapter, adapter_on_scan_found, NULL);
	
	// Scan for peripherals for 5 seconds
	simpleble_adapter_scan_for(adapter, 5000);
	
	// Tell the user we are connecting to their peripheral
	char* peripheral_identifier = simpleble_peripheral_identifier(our_peripheral);
    char* peripheral_address = simpleble_peripheral_address(our_peripheral);
    printf("Connecting to \"%s\" @ address [%s]\n", peripheral_identifier, peripheral_address);
	// Free memory used temporarily by strings
    simpleble_free(peripheral_identifier);
    simpleble_free(peripheral_address);
	
	// Connect to the peripheral, exit if error
	err_code = simpleble_peripheral_connect(our_peripheral);
    if (err_code != SIMPLEBLE_SUCCESS) {
        printf("Failed to connect.\n");
        return 1;
    }

    printf("Successfully connected.\n");
	
	printf("Data read test\n");
	
	// Create pointer that will point to the returned data array
	// Even if you are only reading a characteristic with a single byte, it will be put into an array of size 1
	uint8_t** readData = malloc(sizeof(uint8_t*));
	
	// Create a pointer that will point to the length of the data in the returned array
	size_t* dataSize = malloc(sizeof(size_t));
	
	// Read our characteristic from our peripheral
	simpleble_peripheral_read(our_peripheral, str_to_uuid(HELLO_WORLD_SERVICE_UUID), str_to_uuid(HELLO_WORLD_TEXT_UUID), readData, dataSize);
	
	// If you want to use the returned data as a string you need to add a null terminator to the end
	// readData and dataSize must be dereferenced before they are used
	(*readData)[*dataSize] ='\0';
	
	// Print the length of the returned data and the data as a string
	printf("Data length: %zd\n", *dataSize);
	printf("Read String: %s\n",*readData);
	
	// Free the memory once we are done with it
	free(readData);
	free(dataSize);
	
	// Sleep for 2 seconds
	msleep(2000);
	
	printf("Data write test\n");
	
	printf("Turning off mono LED\n");
	
	// Even if we only want to write a signle byte it must be packaged as an array of size 1
	uint8_t dataToWrite[1];
	dataToWrite[0] = 0;
	
	simpleble_peripheral_write_request(our_peripheral, str_to_uuid(LED_SERVICE_UUID), str_to_uuid(LED_CHARA_MONO_UUID), dataToWrite, sizeof(dataToWrite));
	
	msleep(2000);
	
	printf("Turning on R of RGB LED\n");
	
	// Going to reuse same dataToWrite value for neatness
	simpleble_peripheral_write_request(our_peripheral, str_to_uuid(LED_SERVICE_UUID), str_to_uuid(LED_CHARA_R_UUID), dataToWrite, sizeof(dataToWrite));
	
	msleep(2000);
	
	printf("Turning on B of RGB LED (purple)\n");
	
	simpleble_peripheral_write_request(our_peripheral, str_to_uuid(LED_SERVICE_UUID), str_to_uuid(LED_CHARA_B_UUID), dataToWrite, sizeof(dataToWrite));
	
	msleep(2000);
	
	printf("Turning on G, turning off RB\n");
	
	/*
	   The way these characteristics have been set up means this operation requires 3 separate writes, which shows
	   a clear delay between turning off the Red and Blue LEDs and the Green LED turning on.
	   This is a good example of why it can be better to pack multiple values into the same characteristic, as
	   a combined "RGB" characteristic would only require a single write.
	*/
	simpleble_peripheral_write_request(our_peripheral, str_to_uuid(LED_SERVICE_UUID), str_to_uuid(LED_CHARA_G_UUID), dataToWrite, sizeof(dataToWrite));
	dataToWrite[0] = 1;
	simpleble_peripheral_write_request(our_peripheral, str_to_uuid(LED_SERVICE_UUID), str_to_uuid(LED_CHARA_R_UUID), dataToWrite, sizeof(dataToWrite));
	simpleble_peripheral_write_request(our_peripheral, str_to_uuid(LED_SERVICE_UUID), str_to_uuid(LED_CHARA_B_UUID), dataToWrite, sizeof(dataToWrite));
	
	msleep(2000);
	
	printf("Turning off G, turning on mono\n");
	
	simpleble_peripheral_write_request(our_peripheral, str_to_uuid(LED_SERVICE_UUID), str_to_uuid(LED_CHARA_G_UUID), dataToWrite, sizeof(dataToWrite));
	simpleble_peripheral_write_request(our_peripheral, str_to_uuid(LED_SERVICE_UUID), str_to_uuid(LED_CHARA_MONO_UUID), dataToWrite, sizeof(dataToWrite));
	
	printf("Subscribing to notifications for sensor data for 10 seconds\n");
	
	// Subscribe to notifications for our characteristics and attach handler functions
	simpleble_peripheral_notify(our_peripheral, str_to_uuid(IR_RANGE_SERVICE_UUID),
                                str_to_uuid(IR_RANGE_CHARA1_UUID), IR1Notify, NULL);							
	simpleble_peripheral_notify(our_peripheral, str_to_uuid(IR_RANGE_SERVICE_UUID),
                                str_to_uuid(IR_RANGE_CHARA2_UUID), IR2Notify, NULL);
								
	simpleble_peripheral_notify(our_peripheral, str_to_uuid(USONIC_SERVICE_UUID),
                                str_to_uuid(USONIC_CHARA1_UUID), USonic1Notify, NULL);
	simpleble_peripheral_notify(our_peripheral, str_to_uuid(USONIC_SERVICE_UUID),
                                str_to_uuid(USONIC_CHARA2_UUID), USonic2Notify, NULL);
								
								
	// Sleep for 10 seconds - notifications will still come through while waiting
    msleep(10000000);
	
	printf("Unsubscribing from notifications\n");

	// Unsubscribe from our characteristic
    simpleble_peripheral_unsubscribe(our_peripheral, str_to_uuid(IR_RANGE_SERVICE_UUID),
                                str_to_uuid(IR_RANGE_CHARA1_UUID));
	simpleble_peripheral_unsubscribe(our_peripheral, str_to_uuid(IR_RANGE_SERVICE_UUID),
                                str_to_uuid(IR_RANGE_CHARA2_UUID));
									 
	simpleble_peripheral_unsubscribe(our_peripheral, str_to_uuid(USONIC_SERVICE_UUID),
                                str_to_uuid(USONIC_CHARA1_UUID));
	simpleble_peripheral_unsubscribe(our_peripheral, str_to_uuid(USONIC_SERVICE_UUID),
                                str_to_uuid(USONIC_CHARA2_UUID));

	// Disconnect from our peripheral
    simpleble_peripheral_disconnect(our_peripheral);

    return 0;
}

// Function to clean up memory on exit
static void clean_on_exit(void) {
    printf("Releasing allocated resources.\n");

	// Release peripheral
    simpleble_peripheral_release_handle(our_peripheral);
	
	// Release adapter
    simpleble_adapter_release_handle(adapter);
}


// This function handles what happens when a peripheral is found during a scan
static void adapter_on_scan_found(simpleble_adapter_t adapter, simpleble_peripheral_t peripheral, void* userdata) {
    char* peripheral_identifier = simpleble_peripheral_identifier(peripheral);
    char* peripheral_address = simpleble_peripheral_address(peripheral);
	
	// Print the identifier and address
	printf("Found: \"%s\" @ address: [%s]\n", peripheral_identifier, peripheral_address);

	// If this is our peripheral, save its handle
    if (!strcmp(peripheral_identifier, WANTED_IDENTIFIER) || !strcmp(peripheral_address, WANTED_ADDRESS)) {
        // Save the peripheral
        our_peripheral = peripheral;
    } else {
        // If its not the our peripheral, release its handle
        simpleble_peripheral_release_handle(peripheral);
    }

    // Let's not forget to release all allocated memory.
    simpleble_free(peripheral_identifier);
    simpleble_free(peripheral_address);
}

/*
    This helper function converts a string into a UUID as used by the SimpleBLE library
    This allow us to directly specify what services and characteristics we want to use
    rather than having to scan through all the characteristics to find the one we want
	as we already know what their UUIDs are going to be
*/
static simpleble_uuid_t str_to_uuid(char* uuid_str){
	simpleble_uuid_t uuid;
	strcpy(uuid.value, uuid_str);
	return uuid;
}

/*
    These are the handler functions that handle what happens when a notification
	comes in from the peripheral. Even if only a single byte is received it is
	presented as an array of size 1.
*/
static void IR1Notify(simpleble_uuid_t service, simpleble_uuid_t characteristic, const uint8_t* data,
                                 size_t data_length, void* userdata) {
    printf("IR1: %d\n", data[0]);
}
static void IR2Notify(simpleble_uuid_t service, simpleble_uuid_t characteristic, const uint8_t* data,
                                 size_t data_length, void* userdata) {
    printf("IR2: %d\n", data[0]);
								 }

static void USonic1Notify(simpleble_uuid_t service, simpleble_uuid_t characteristic, const uint8_t* data,
                                 size_t data_length, void* userdata) {
    printf("uSonic1: %d\n", data[0]);
}

static void USonic2Notify(simpleble_uuid_t service, simpleble_uuid_t characteristic, const uint8_t* data,
                                 size_t data_length, void* userdata) {
    printf("uSonic2: %d\n", data[0]);
}