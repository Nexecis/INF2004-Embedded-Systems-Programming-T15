#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include <stdio.h>
#include "hardware/gpio.h"

#define IR_SENSOR_PIN 17  // Pin for the IR sensor

#define TCP_PORT 4242
#define BUF_SIZE 1

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    bool complete;
    uint8_t buffer_sent[BUF_SIZE];
    uint8_t buffer_recv[BUF_SIZE];
    int sent_len;
    int recv_len;
    int run_count;
} TCP_SERVER_T;

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

err_t IRbarcode(void *arg, struct tcp_pcb *tpcb){
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
            if (strcmp(bar_type, "000") != 0) {
                printf("Reverse Detected!\n");
                reverse = 1;
            }
        }
        if (counter == 29)
        {
            if (reverse == 1) {
                reverseString(barcode); // reverse the string   
            }
            strcat(barcode, "0"); // add a 0 at the back of the binary string, signalling the end of barcode (barcode ends with 0)
            printf("len(barcode): %d\n", strlen(barcode));
            printf("Final Barcode Binaries: %s\n", barcode);
            char encodedChar = encodeBarcode(barcode);
            printf("Encoded Character: %c\n", encodedChar);
            barcode[0] = '\0';

            TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
            state->buffer_sent[0] = encodedChar; // data to send
            state->sent_len = 0;
            tcp_write(tpcb, state->buffer_sent, BUF_SIZE, TCP_WRITE_FLAG_COPY);

            return ERR_OK;
        }

        black_line = !black_line; // swap the colour of the barcode line
    }
}




static TCP_SERVER_T* tcp_server_init(void) {
    TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
    if (!state) {
        printf("failed to allocate state\n");
        return NULL;
    }
    return state;
}

static err_t tcp_server_close(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    err_t err = ERR_OK;
    if (state->client_pcb != NULL) {
        tcp_arg(state->client_pcb, NULL);
        tcp_poll(state->client_pcb, NULL, 0);
        tcp_sent(state->client_pcb, NULL);
        tcp_recv(state->client_pcb, NULL);
        tcp_err(state->client_pcb, NULL);
        err = tcp_close(state->client_pcb);
        if (err != ERR_OK) {
            printf("close failed %d, calling abort\n", err);
            tcp_abort(state->client_pcb);
            err = ERR_ABRT;
        }
        state->client_pcb = NULL;
    }
    if (state->server_pcb) {
        tcp_arg(state->server_pcb, NULL);
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }
    return err;
}

static err_t tcp_server_result(void *arg, int status) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (status == 0) {
        printf("test success\n");
    } else {
        printf("test failed %d\n", status);
    }
    state->complete = true;
    return tcp_server_close(arg);
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (!p) {
        return tcp_server_result(arg, -1);
    }

    if (p->tot_len > 0) {
        printf("tcp_server_recv %d/%d err %d\n", p->tot_len, state->recv_len, err);

        // Receive the buffer
        const uint16_t buffer_left = BUF_SIZE - state->recv_len;
        state->recv_len += pbuf_copy_partial(p, state->buffer_recv + state->recv_len,
                                             p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);

    uint8_t received_byte;
    printf("Received data: ");
    for (int i = 0; i < state->recv_len; i++) {
        received_byte = state->buffer_recv[i];
        printf("%c", received_byte);
    }
    printf("\n");

    if(received_byte == 'X'){
        tcp_server_result(arg, 0);
        return ERR_OK;
    }

    state->recv_len = 0;

    return ERR_OK;
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        printf("Failure in accept\n");
        tcp_server_result(arg, err);
        return ERR_VAL;
    }
    printf("Client connected\n");

    state->client_pcb = client_pcb;
    tcp_arg(client_pcb, state);
    tcp_recv(client_pcb, tcp_server_recv);

    return IRbarcode(arg, state->client_pcb);
}

static bool tcp_server_open(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), TCP_PORT);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        printf("failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_PORT);
    if (err) {
        printf("failed to bind to port %u\n", TCP_PORT);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        printf("failed to listen\n");
        if (pcb) {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    return true;
}

void run_tcp_server_test(void) {
    TCP_SERVER_T *state = tcp_server_init();
    if (!state) {
        return;
    }
    if (!tcp_server_open(state)) {
        tcp_server_result(state, -1);
        return;
    }
    while(!state->complete) {
        // if you are not using pico_cyw43_arch_poll, then WiFI driver and lwIP work
        // is done via interrupt in the background. This sleep is just an example of some (blocking)
        // work you might be doing.
        printf("Waiting\n");
        sleep_ms(1000);
    }
    free(state);
}

int main() {
    stdio_init_all();

    gpio_init(16);
    gpio_set_dir(16, GPIO_OUT);
    gpio_put(16, 1);

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
    }

    initIRSensor();

    run_tcp_server_test();
    cyw43_arch_deinit();
    return 0;
}