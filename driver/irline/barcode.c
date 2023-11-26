#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <string.h>

#define IR_SENSOR_PIN 28  // Pin for the IR sensor

// Function to initialize the IR sensor
void initIRSensor() {
    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
}

// Function to measure the pulse width
uint32_t measurePulseWidth(int black_line) {
    uint32_t start_time, end_time;

    if(black_line == 1) {
        // Wait for the rising edge of the pulse (transition from white to black)
        while (gpio_get(IR_SENSOR_PIN) == 0) {}

        start_time = time_us_32();

        // Wait for the falling edge of the pulse (transition from black to white)
        while (gpio_get(IR_SENSOR_PIN) == 1) {}

        end_time = time_us_32();
    }
    else {
        // Wait for the falling edge of the pulse (transition from black to white)
        while (gpio_get(IR_SENSOR_PIN) == 1) {}

        start_time = time_us_32();

        // Wait for the rising edge of the pulse (transition from white to black)
        while (gpio_get(IR_SENSOR_PIN) == 0) {}

        end_time = time_us_32();
    }

    return end_time - start_time;
}


// Function to perform barcode encoding
char encodeBarcode(const char *barcode) {
    // Check each pattern and return the corresponding character
    if (strcmp(barcode, "100010111011101010100011101110101000101110111010") == 0) {
        return '0';
    } else if (strcmp(barcode, "100010111011101011101000101011101000101110111010") == 0) {
        return '1';
    } else if (strcmp(barcode, "100010111011101010111000101011101000101110111010") == 0) {
        return '2';
    } else if (strcmp(barcode, "100010111011101011101110001010101000101110111010") == 0) {
        return '3';
    } else if (strcmp(barcode, "100010111011101010100011101011101000101110111010") == 0) {
        return '4';
    } else if (strcmp(barcode, "100010111011101011101000111010101000101110111010") == 0) {
        return '5';
    } else if (strcmp(barcode, "100010111011101010111000111010101000101110111010") == 0) {
        return '6';
    } else if (strcmp(barcode, "100010111011101010100010111011101000101110111010") == 0) {
        return '7';
    } else if (strcmp(barcode, "100010111011101011101000101110101000101110111010") == 0) {
        return '8';
    } else if (strcmp(barcode, "100010111011101010111000101110101000101110111010") == 0) {
        return '9';
    } else if (strcmp(barcode, "100010111011101011101010001011101000101110111010") == 0) {
        return 'A';
    } else if (strcmp(barcode, "100010111011101010111010001011101000101110111010") == 0) {
        return 'B';
    } else if (strcmp(barcode, "100010111011101011101110100010101000101110111010") == 0) {
        return 'C';
    } else if (strcmp(barcode, "100010111011101010101110001011101000101110111010") == 0) {
        return 'D';
    } else if (strcmp(barcode, "100010111011101011101011100010101000101110111010") == 0) {
        return 'E';
    } else if (strcmp(barcode, "100010111011101010111011100010101000101110111010") == 0) {
        return 'F';
    } else if (strcmp(barcode, "100010111011101010101000111011101000101110111010") == 0) {
        return 'G';
    } else if (strcmp(barcode, "100010111011101011101010001110101000101110111010") == 0) {
        return 'H';
    } else if (strcmp(barcode, "100010111011101010111010001110101000101110111010") == 0) {
        return 'I';
    } else if (strcmp(barcode, "100010111011101010101110001110101000101110111010") == 0) {
        return 'J';
    } else if (strcmp(barcode, "100010111011101011101010100011101000101110111010") == 0) {
        return 'K';
    } else if (strcmp(barcode, "100010111011101010111010100011101000101110111010") == 0) {
        return 'L';
    } else if (strcmp(barcode, "100010111011101011101110101000101000101110111010") == 0) {
        return 'M';
    } else if (strcmp(barcode, "100010111011101010101110100011101000101110111010") == 0) {
        return 'N';
    } else if (strcmp(barcode, "100010111011101011101011101000101000101110111010") == 0) {
        return 'O';
    } else if (strcmp(barcode, "100010111011101010111011101000101000101110111010") == 0) {
        return 'P';
    } else if (strcmp(barcode, "100010111011101010101011100011101000101110111010") == 0) {
        return 'Q';
    } else if (strcmp(barcode, "100010111011101011101010111000101000101110111010") == 0) {
        return 'R';
    } else if (strcmp(barcode, "100010111011101010111010111000101000101110111010") == 0) {
        return 'S';
    } else if (strcmp(barcode, "100010111011101010101110111000101000101110111010") == 0) {
        return 'T';
    } else if (strcmp(barcode, "100010111011101011100010101011101000101110111010") == 0) {
        return 'U';
    } else if (strcmp(barcode, "100010111011101010001110101011101000101110111010") == 0) {
        return 'V';
    } else if (strcmp(barcode, "100010111011101011100011101010101000101110111010") == 0) {
        return 'W';
    } else if (strcmp(barcode, "100010111011101010001011101011101000101110111010") == 0) {
        return 'X';
    } else if (strcmp(barcode, "100010111011101011100010111010101000101110111010") == 0) {
        return 'Y';
    } else if (strcmp(barcode, "100010111011101010001110111010101000101110111010") == 0) {
        return 'Z';
    } else if (strcmp(barcode, "100010111011101010001010111011101000101110111010") == 0) {
        return '-';
    } else if (strcmp(barcode, "100010111011101011100010101110101000101110111010") == 0) {
        return '.';
    } else if (strcmp(barcode, "100010111011101010001110101110101000101110111010") == 0) {
        return ' ';
    } else if (strcmp(barcode, "100010111011101010001000100010101000101110111010") == 0) {
        return '$';
    } else if (strcmp(barcode, "100010111011101010001000101000101000101110111010") == 0) {
        return '/';
    } else if (strcmp(barcode, "100010111011101010001010001000101000101110111010") == 0) {
        return '+';
    } else if (strcmp(barcode, "100010111011101010100010001000101000101110111010") == 0) {
        return '%';
    } else {
        return '?';  // Return '?' for unrecognized patterns
    }
}

// Function to reverse a string
void reverseString(char str[]) {
    int length = strlen(str);
    int start = 0;
    int end = length - 1;

    while (start < end) {
        // Swap characters at start and end
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;

        // Move the pointers towards each other
        start++;
        end--;
    }
}

int main() {
    stdio_init_all();
    initIRSensor();
    bool black_line = 1; // the start of barcode is white space, so black == false (0) 
    char bar_type[4];
    int counter = 0;
    char barcode[65];  // String to store the barcode
    barcode[0] = '\0';  // Initialize the string
    bool reverse = 0;
    
    uint32_t first_reading = measurePulseWidth(black_line);
    printf("first_reading: %u us\n", first_reading);
    printf("Detected bar type: 1 with pulse width: %u us\n", first_reading);
    strcpy(bar_type, "1");  // Thin black bar
    strcat(barcode, bar_type);
    counter++;
    printf("Counter: %d\n", counter);
    black_line = !black_line; // swap the colour to white line

    while (1) {
        // Measure the pulse width using the IR sensor
        uint32_t pulse_width = measurePulseWidth(black_line);
        
        // Determine the bar type based on the pulse width
        if (black_line == 1) {
            if (pulse_width > (2.1 * first_reading) ) {
                strcpy(bar_type, "111");  // Thick black bar
            } else {
                strcpy(bar_type, "1");  // Thin black bar
            }
        }
        else {
            if (pulse_width > (2.1 * first_reading) ) {
                strcpy(bar_type, "000");  // Thick white bar
            } else {
                strcpy(bar_type, "0");  // Thin white bar
            }
        }
        
        printf("Detected bar type: %s with pulse width: %u us\n", bar_type, pulse_width);

        counter++;
        printf("Counter: %d\n", counter);

        // Concatenate the result of bar_type to the barcode string
        strcat(barcode, bar_type);
        printf("Barcode Binaries: %s\n\n", barcode);
        if (counter == 2) {
            if (bar_type != "000") {
                printf("Reverse Detected!\n");
                reverse = 1;
            }
        }
        if (counter == 29)
        {
            if (reverse == 0) {
                strcat(barcode, "0"); // add a 0 at the back of the binary string, signalling the end of barcode (barcode ends with 0)
                printf("len(barcode): %d\n", strlen(barcode));
                printf("Final Barcode Binaries: %s\n", barcode);
                char encodedChar = encodeBarcode(barcode);
                printf("Encoded Character: %c\n", encodedChar);
                barcode[0] = '\0';
                break;
            }
            else { // reverse == 1, so do reversing
                reverseString(barcode); // reverse the string
                strcat(barcode, "0"); // add a 0 at the back of the binary string, signalling the end of barcode (barcode ends with 0)
                printf("len(barcode): %d\n", strlen(barcode));
                printf("Final Barcode Binaries: %s\n", barcode);
                char encodedChar = encodeBarcode(barcode);
                printf("Encoded Character: %c\n", encodedChar);
                barcode[0] = '\0';
                break;
            }
            
        }

        black_line = !black_line; // swap the colour of the barcode line
    }

    return 0;
}

