#ifndef USB_HOST_H
#define USB_HOST_H


#include "kinetis.h"
#include "usb_dev.h"


#define GET_STATUS		0
#define CLEAR_FEATURE		1
#define SET_FEATURE		3
#define SET_ADDRESS		5
#define GET_DESCRIPTOR		6
#define SET_DESCRIPTOR		7
#define GET_CONFIGURATION	8
#define SET_CONFIGURATION	9
#define GET_INTERFACE		10
#define SET_INTERFACE		11
#define SYNCH_FRAME		12

#define BDT_OWN		0x80
#define BDT_DATA1	0x40
#define BDT_DATA0	0x00
#define BDT_DTS		0x08
#define BDT_STALL	0x04
#define BDT_PID(n)	(((n) >> 2) & 15)

#define BDT_DESC(count, data)	(BDT_OWN | BDT_DTS \
				| ((data) ? BDT_DATA1 : BDT_DATA0) \
				| ((count) << 16))

#define TX   1
#define RX   0
#define ODD  1
#define EVEN 0
#define DATA0 0
#define DATA1 1
#define index(endpoint, tx, odd) (((endpoint) << 2) | ((tx) << 1) | (odd))
#define stat2bufferdescriptor(stat) (table + ((stat) >> 2))

typedef struct {
    uint32_t desc;
    void * addr;
} bdt_t;


extern "C" {
void serial_print(const char *p);
void serial_phex(uint32_t n);
void serial_phex16(uint32_t n);
void serial_phex32(uint32_t n);

void delay(uint32_t msec);

void usb_host_isr(void);
void usb_init_host_mode(void);

extern bdt_t table[(NUM_ENDPOINTS+1)*4];

}


class USB_Host
{
    public:
        /** Default constructor */
        USB_Host();
        /** Default destructor */
        virtual ~USB_Host();

        void usb_host_mode(void);

        static void usb_host_isr(void);

        static void endpoint0_transmit(const void *data, uint32_t len);

        static void endpoint0_receive(const void *data, uint32_t len);

        static void resetDevice(uint8_t time);
        static void setConfiguration(uint8_t num);
        static void setAddress(uint8_t num);
        static uint8_t getDescriptor(uint8_t descType, uint8_t descIndex, uint16_t langID, uint8_t length);
        static void debug_tokdne(uint8_t stat);


        static uint8_t ep0_rx0_buf[EP0_SIZE] __attribute__ ((aligned (4)));
        static uint8_t ep0_rx1_buf[EP0_SIZE] __attribute__ ((aligned (4)));
        static constexpr uint8_t *ep0_tx_ptr = NULL;
        static uint16_t ep0_tx_len;
        static uint8_t ep0_tx_bdt_bank;
        static uint8_t ep0_tx_data_toggle;

        static uint8_t ep0_tx0_buf[EP0_SIZE] __attribute__ ((aligned (4)));
        static uint8_t ep0_tx1_buf[EP0_SIZE] __attribute__ ((aligned (4)));
        static constexpr uint8_t *ep0_rx_ptr = NULL;
        static uint16_t ep0_rx_len;
        static uint8_t ep0_rx_bdt_bank;
        static uint8_t ep0_rx_data_toggle;



        static uint8_t data_in_buf[255]; // to receive data in as host

        static uint8_t setup_command_buffer[8]; // for setup commands

        static uint8_t maxPacketSize; // maximum packet size we can ask for. Start with 8 until we know how much (get device descriptor)



    protected:
    private:


};

#endif // USB_HOST_H
