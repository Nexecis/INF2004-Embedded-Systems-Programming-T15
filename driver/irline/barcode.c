#include <stdio.h>
#include "pico/stdlib.h"
#include "string.h"
#include "hardware/gpio.h"

#define IR_SENSOR_PIN1 14  // Pin for the first IR sensor
#define IR_SENSOR_PIN2 15  // Pin for the second IR sensor
#define IR_SENSOR_PIN3 16  // Pin for the third IR sensor
#define BARCODE_LENGTH 13  // Length of a Code 39 barcode (excluding start and stop characters *)

// Define the character map for Code 39
const char* code39_characters = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";

// Function to initialize the IR sensors
void initIRSensors() {
    gpio_init(IR_SENSOR_PIN1);
    gpio_init(IR_SENSOR_PIN2);
    gpio_init(IR_SENSOR_PIN3);
    gpio_set_dir(IR_SENSOR_PIN1, GPIO_IN);
    gpio_set_dir(IR_SENSOR_PIN2, GPIO_IN);
    gpio_set_dir(IR_SENSOR_PIN3, GPIO_IN);
}

// Function to measure pulse width
uint32_t measurePulseWidth(uint pin) {
    uint32_t start_time, end_time;

    // Wait for the falling edge of the pulse
    while (gpio_get(pin)) {}

    start_time = time_us_32();

    // Wait for the rising edge of the pulse
    while (!gpio_get(pin)) {}

    end_time = time_us_32();

    return end_time - start_time;
}

// Function to encode a character into a Code 39 barcode
void encodeCharacter(char character, char* barcode) {
    char* position = strchr(code39_characters, character);
    if (position != NULL) {
        int index = (int)(position - code39_characters);
        for (int i = 0; i < BARCODE_LENGTH; i++) {
            barcode[i] = (index & (1 << (BARCODE_LENGTH - 1 - i))) ? '1' : '0';
        }
    }
}

// Function to decode a Code 39 barcode into a character
char decodeBarcode(const char* barcode) {
    int index = 0;
    for (int i = 0; i < BARCODE_LENGTH; i++) {
        if (barcode[i] == '1') {
            index |= (1 << (BARCODE_LENGTH - 1 - i));
        }
    }
    return code39_characters[index];
}

int main() {
    stdio_init_all();
    initIRSensors();
    char input_char = 'D';  // Change this character to encode/decode different characters

    // Encode the input character into a Code 39 barcode
    char encodedBarcode[BARCODE_LENGTH + 1];  // +1 for null-terminator
    encodedBarcode[BARCODE_LENGTH] = '\0';  // Null-terminate the string
    encodeCharacter(input_char, encodedBarcode);

    printf("Character: %c\n", input_char);
    printf("Encoded Barcode: %s\n", encodedBarcode);

    // Decode a Code 39 barcode back into a character
    char decodedChar = decodeBarcode(encodedBarcode);
    printf("Decoded Character: %c\n", decodedChar);

    return 0;
}
