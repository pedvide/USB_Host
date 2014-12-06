#include "USB_Host.h"

extern uint8_t data_in_buf[64];

// initialize static members

uint8_t USB_Host::maxPacketSize = 8;
uint8_t USB_Host::ep0_tx_bdt_bank = 0;
uint8_t USB_Host::ep0_tx_data_toggle = 0;
uint8_t USB_Host::ep0_rx_bdt_bank = 0;
uint8_t USB_Host::ep0_rx_data_toggle = 0;

uint8_t USB_Host::ep0_rx0_buf[];
uint8_t USB_Host::ep0_rx1_buf[];
uint8_t USB_Host::ep0_tx0_buf[];
uint8_t USB_Host::ep0_tx1_buf[];

uint8_t USB_Host::data_in_buf[];
uint8_t USB_Host::setup_command_buffer[];

USB_Host::USB_Host()
{
    //ctor

    // set desc table base addr
    /*
	USB0_BDTPAGE1 = ((uint32_t)table) >> 8;
	USB0_BDTPAGE2 = ((uint32_t)table) >> 16;
	USB0_BDTPAGE3 = ((uint32_t)table) >> 24;
	*/

	//table_address = (USB0_BDTPAGE1 << 8) | (USB0_BDTPAGE2 << 16) | (USB0_BDTPAGE3 << 24);

	//uint32_t *pointer = (uint32_t *)0x1FFF8000;

	//table_Host = (bdt_t *)table_address;
	//table_Host = (bdt_t *)0x1FFF8000;

	//table_Host = (bdt_t *)pointer;

}

USB_Host::~USB_Host()
{
    //dtor
    _VectorsRam[IRQ_USBOTG] = usb_isr;
}




void USB_Host::usb_host_mode(void) {

    serial_print("host mode\n");

    // IN packages
    table[index(0, TX, EVEN)].desc = BDT_DESC(64, 0);
    table[index(0, TX, EVEN)].addr = ep0_tx0_buf;
    table[index(0, TX, ODD)].desc = BDT_DESC(64, 0);
    table[index(0, TX, ODD)].addr = ep0_tx1_buf;

    // clear interrupts
    USB0_ERRSTAT = 0xFF;
    USB0_ISTAT = 0xFF;
    USB0_OTGISTAT = 0xFF;

    //USB0_OTGCTL &= ~USB_OTGCTL_DPHIGH; // disable D+ pullup
    USB0_OTGCTL = USB_OTGCTL_DPLOW | USB_OTGCTL_DMLOW; // enable D+ and D- pulldowns, disable D+ pullup


    USB0_INTEN = USB_INTEN_ATTACHEN |
                USB_INTEN_TOKDNEEN |
                USB_INTEN_STALLEN |
                USB_INTEN_ERROREN |
                USB_INTEN_SLEEPEN; // enable attach interrupt and token done
    //USB0_OTGICR = USB_OTGICR_ONEMSECEN; // activate timer
    USB0_ERREN = 0xFF; // enable all error interrupts

    NVIC_DISABLE_IRQ(IRQ_USBOTG);
    _VectorsRam[IRQ_USBOTG+16] = usb_host_isr;
    NVIC_ENABLE_IRQ(IRQ_USBOTG);

    USB0_CTL |= USB_CTL_HOSTMODEEN; // host mode enable

    USB0_CTL &= ~USB_CTL_USBENSOFEN; // disable SOF generation to avoid noise until we detect attach



    serial_print("host mode done\n");

}



void USB_Host::endpoint0_transmit(const void *data, uint32_t len)
{
#if 1
    uint32_t i = 0;
    uint8_t *real_data = (uint8_t *)data; // recast to uint32_t
	serial_print("tx0: ");
	for(i=0; i<len; i++) {
        serial_phex( *(real_data + i) );
        serial_print(" ");
	}
	serial_print(",");
	serial_phex16(len);
	serial_print("\n");
	//serial_print(ep0_tx_bdt_bank ? ", odd" : ", even");
	//serial_print(ep0_tx_data_toggle ? ", d1\n" : ", d0\n");
#endif
	table[index(0, TX, ep0_tx_bdt_bank)].addr = (void *)data;
	table[index(0, TX, ep0_tx_bdt_bank)].desc = BDT_DESC(len, ep0_tx_data_toggle);
	ep0_tx_data_toggle ^= 1;
	ep0_tx_bdt_bank ^= 1;
}

void USB_Host::endpoint0_receive(const void *data, uint32_t len)
{
#if 1
	serial_print("rx: ");
	serial_phex16(len);
	serial_print("\n");
	//serial_print(ep0_rx_bdt_bank ? ", odd" : ", even");
	//serial_print(ep0_rx_data_toggle ? ", d1\n" : ", d0\n");
#endif
	table[index(0, RX, ep0_rx_bdt_bank)].addr = (void *)data;
	table[index(0, RX, ep0_rx_bdt_bank)].desc = BDT_DESC(len, ep0_rx_data_toggle);
	ep0_rx_data_toggle ^= 1;
	ep0_rx_bdt_bank ^= 1;
}



// HOST MODE ISR
void USB_Host::usb_host_isr(void)
{
	uint8_t status, otg_status, stat;

	//serial_print("host mode isr\n");
	//status = USB0_ISTAT;
	//serial_phex(status);
	//serial_print("\n");
	restart:
	status = USB0_ISTAT;
	//otg_status = USB0_OTGISTAT; // otg interrupts

	if ((status & USB_INTEN_SOFTOKEN /* 04 */ )) {
        //serial_print("sof_token \n");

        // in host mode manual says software should prepare for next SOF. what does it mean?
		if(USB0_CTL & USB_CTL_HOSTMODEEN) {
            //serial_print("sof token in host mode, do nothing. \n");
            USB0_ISTAT = USB_INTEN_SOFTOKEN;
            return;
        }

		USB0_ISTAT = USB_INTEN_SOFTOKEN;
	}

	if ((status & USB_ISTAT_TOKDNE /* 08 */ )) {


		// if in host mode, do nothing
		if(USB0_CTL & USB_CTL_HOSTMODEEN) {

            stat = USB0_STAT;

            #if 1
            bdt_t *b = stat2bufferdescriptor(stat);
            uint32_t pid = BDT_PID(b->desc);
            uint32_t count = b->desc >> 16;
            uint8_t *buf = (uint8_t *)b->addr;
            //uint8_t endpoint = stat >> 4;

            serial_print("TOKDNE. Pid: ");
            serial_phex(pid);
            serial_print(", count: ");
            serial_phex(count);
            serial_print(". Data: ");
            serial_phex32( *(uint32_t *)(buf) );
            serial_print(", ");
            serial_phex32( *(uint32_t *)(buf + 4) );
            serial_print("\n");
            #endif
            USB0_ISTAT = USB_ISTAT_TOKDNE;

            return;
        }

		USB0_ISTAT = USB_ISTAT_TOKDNE;
		goto restart;
	}



	if (status & USB_ISTAT_USBRST /* 01 */ ) {
		serial_print("reset\n");

		// reset is also called when in host mode for reasons I don't understand. Do nothing
		if(USB0_CTL & USB_CTL_HOSTMODEEN) {
            serial_print("reset in host mode, do nothing. \n");
            USB0_ISTAT = USB_ISTAT_USBRST;
            return;
        }

        USB0_ISTAT = USB_ISTAT_USBRST;
		return;
	}


	if ((status & USB_ISTAT_STALL /* 80 */ )) {
		serial_print("stall:\n");

		// in host mode stall means that the last ACK from device was stalled.
		if(USB0_CTL & USB_CTL_HOSTMODEEN) {
            serial_print("stall in host mode, do nothing. \n");
            USB0_ISTAT = USB_ISTAT_STALL;
            return;
        }
	}
	if ((status & USB_ISTAT_ERROR /* 02 */ )) {
		uint8_t err = USB0_ERRSTAT;
		USB0_ERRSTAT = err; // clear error
		serial_print("err:");
		serial_phex(err);
		serial_print("\n");
		USB0_ISTAT = USB_ISTAT_ERROR;
	}

	if ((status & USB_ISTAT_SLEEP /* 10 */ )) {
		serial_print("sleep\n");
		USB0_ISTAT = USB_ISTAT_SLEEP;
	}

	if ((status & USB_ISTAT_RESUME  )) {
		serial_print("resume\n");
		USB0_ISTAT = USB_ISTAT_RESUME;
	}


	// Host mode
	if ((status & USB_ISTAT_ATTACH  )) { // device attached to the usb
		serial_print("attach\n");

		uint8_t low_speed = 0;


		USB0_SOFTHLD = 0x4A; // set to 0x4A for 64 byte transfers, 0x12 for 8-byte, 0x1A=16-bytes

		// check whether the device wants low or full speed
		if( !(USB0_CTL & USB_CTL_JSTATE) ) { // low speed (what about SE0?)
		    #define USB_ADDR_LSEN  (0x80) // low speed enable bit
		    low_speed = 1;
            USB0_ADDR = USB_ADDR_LSEN; // low speed enable, address 0
            USB0_ENDPT0 |= USB_ENDPT_HOSTWOHUB; // no hub present, communicate directly with device
            USB0_SOFTHLD = 0x12; // low speed, 8 byte transfers

            maxPacketSize = 8;

            serial_print("low speed\n");

        }

        // send reset signal for 15 ms
        resetDevice(15);

        maxPacketSize = 64;

        USB0_CTL |= USB_CTL_USBENSOFEN; // start generating SOFs

        USB0_ADDR = 0x0; // address 0, low speed enable if detected

		// activate endpoint 0
		USB0_ENDPT0 = USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK; // bidirectional control transfers 0x4D // USB_ENDPT_RETRYDIS


        // start ennumeration

        // get the device descriptor
        // reset
        // set address to 1
        // get device descriptor
        // get configuration descriptor
        // get string descriptors

        // prototype of getDescriptor:
        // uint8_t getDescriptor(uint8_t descType, uint8_t descIndex, uint16_t langID, uint8_t length)

        uint8_t device_descriptor_length = getDescriptor(1, 0, 0, 0x40); // Get device descriptor of max length 64 bytes

        serial_print("maxPacketSize: ");
        serial_phex(maxPacketSize);
        serial_print("\n");

        serial_print("device_descriptor_length: ");
        serial_phex(device_descriptor_length);
        serial_print("\n");

        uint8_t device_descriptor[device_descriptor_length];
        memcpy(device_descriptor, data_in_buf, device_descriptor_length);
        int i=0;
        for(i=0; i<device_descriptor_length; i++) {
            serial_phex( device_descriptor[i] );
            serial_print(" ");
        }
        serial_print("\n");


        // reset again
        resetDevice(10);

        // Change the address
        setAddress(1);

        // Get device descriptor
		device_descriptor_length = getDescriptor(1, 0, 0, device_descriptor_length);
		memcpy(device_descriptor, data_in_buf, device_descriptor_length);

        // Get configuration descriptor
        uint8_t conf_descriptor_length = getDescriptor(2, 0, 0, 0xFF); // Get device descriptor of max length

		serial_print("conf_descriptor_length: ");
        serial_phex(conf_descriptor_length);
        serial_print("\n");

        uint8_t conf_descriptor[conf_descriptor_length];
        memcpy(conf_descriptor, data_in_buf, conf_descriptor_length);
        for(i=0; i<conf_descriptor_length; i++) {
            serial_phex( conf_descriptor[i] );
            serial_print(" ");
        }
        serial_print("\n");


        // Get string descriptor 3
        uint8_t str_descriptor_3_length = getDescriptor(3, 3, 0x0904, 0xFF); // Get device descriptor of max length

		serial_print("str_descriptor_3_length: ");
        serial_phex(str_descriptor_3_length);
        serial_print("\n");

        uint8_t str_descriptor_3[str_descriptor_3_length];
        memcpy(str_descriptor_3, data_in_buf, str_descriptor_3_length);
        for(i=0; i<str_descriptor_3_length; i++) {
            serial_phex( str_descriptor_3[i] );
            serial_print(" ");
        }
        serial_print("\n");


        // Get string descriptor 0
        uint8_t str_descriptor_0_length = getDescriptor(3, 0, 0, 0xFF); // Get device descriptor of max length

		serial_print("str_descriptor_0_length: ");
        serial_phex(str_descriptor_0_length);
        serial_print("\n");

        uint8_t str_descriptor_0[str_descriptor_0_length];
        memcpy(str_descriptor_0, data_in_buf, str_descriptor_0_length);

        uint32_t langID = str_descriptor_0[0] | (str_descriptor_0[1] << 0xFF);
        serial_print("langID: ");
        serial_phex(langID);
        serial_print("\n");

        // Get string descriptor 2
        uint8_t str_descriptor_2_length = getDescriptor(3, 2, 0x0904, 0xFF); // Get device descriptor of max length

		serial_print("str_descriptor_2_length: ");
        serial_phex(str_descriptor_2_length);
        serial_print("\n");

        uint8_t str_descriptor_2[str_descriptor_2_length];
        memcpy(str_descriptor_2, data_in_buf, str_descriptor_2_length);
        for(i=0; i<str_descriptor_2_length; i++) {
            serial_phex( str_descriptor_2[i] );
            serial_print(" ");
        }
        serial_print("\n");



        // Set configuration to 1
        setConfiguration(1);


		//USB0_CTL &= ~USB_CTL_USBENSOFEN; // disable SOF generation, send device to sleep

		USB0_INTEN &= ~USB_INTEN_ATTACHEN; // disable the attach interrupt os we don't go into a cycle of attach/reset/attach

		USB0_ISTAT = USB_ISTAT_ATTACH; // clear interrupt
		//goto restart;
		return;
	}

}



#define USB_TOKEN_SETUP (0xD0)
#define USB_TOKEN_DATA_IN (0x90)
#define USB_TOKEN_DATA_OUT (0x10)


void USB_Host::debug_tokdne(uint8_t stat) {
    #if 1
    bdt_t *b = stat2bufferdescriptor(stat);
    uint32_t pid = BDT_PID(b->desc);
    uint32_t count = b->desc >> 16;
    uint8_t *buf = (uint8_t *)b->addr;
    //uint8_t endpoint = stat >> 4;

    serial_print("TOKDNE. Pid: ");
    serial_phex(pid);
    serial_print(", count: ");
    serial_phex(count);
    if(count) {
        serial_print(". Data: ");
        serial_phex32( *(uint32_t *)(buf) );
        serial_print(", ");
        serial_phex32( *(uint32_t *)(buf + 4) );
    }
    serial_print("\n");
    #endif
}

void USB_Host::setupToken(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength) {



}

/* Get bytes length from the descritor number num from the device.
*  Maximum length is 255 bytes
* Descriptor stored in data_in_buf, the real number of bytes of data is returned
* If the descriptor is smaller than length only the descriptor is returned.

 num =
Device 1

Configuration 2
Request for this also returns OTG, interface and endpoint descriptors

String 3
Qualified by an index to specify which string is required

Interface 4
Not directly accessible

Endpoint 5
Not directly accessible

Device Qualifier 6
Only for high speed capable devices

Other Speed Configuration 7
Only for high speed capable devices

Interface Power 8
Obsolete

On-The-Go (OTG) 9
Not directly accessible

*/
uint8_t USB_Host::getDescriptor(uint8_t descType, uint8_t descIndex, uint32_t langID, uint8_t length) {

    if( length<8 )  {
        length = 8; // min size
    }

    const uint8_t *data = NULL;
    uint32_t datalen = 0;

    uint32_t totalBytes = 0;

    //uint8_t descriptor[length]; // reserve as much space as we may need

    // algorithm:
    // SETUP token to request length data from the descriptor number descType
    // IN data token:
    //      if length is less than the packet size we only need this token, go to status token
    //      if length is greater, we need more IN tokens
    // STATUS OUT token


    // SETUP phase
    serial_print("GET DESCRIPTOR\n");
    //serial_print("setup phase\n");

    // data is sent low byte first
    setup_command_buffer[0] = 0x80; // GET DESCRIPTOR = 0x80 06 00 xx 00 00 yy 00, yy bytes of desc #xx
    setup_command_buffer[1] = 0x06;
    setup_command_buffer[2] = descIndex & 0xFF; // usually 0
    setup_command_buffer[3] = descType & 0xFF; // only 1 byte for the number

    setup_command_buffer[4] = (uint8_t)(langID & 0x00FF); // only for string desc (descType=3)
    setup_command_buffer[5] = (uint8_t)(langID & 0xFF00);
    setup_command_buffer[6] = length & 0xFF;
    setup_command_buffer[7] = 0x00;

    datalen = 8;
    data = setup_command_buffer;
    ep0_tx_data_toggle = DATA0; // setup always uses DATA0
    endpoint0_transmit(data, datalen);

    USB0_TOKEN = USB_TOKEN_SETUP; //  SETUP token to endpoint 0.

    while(USB0_CTL & USB_CTL_TXSUSPENDTOKENBUSY) {} // wait in case there's another token being sent
    while(!(USB0_ISTAT & USB_ISTAT_TOKDNE)) {} // wait for last transfer to finish
    debug_tokdne(USB0_STAT);
    USB0_ISTAT = USB_ISTAT_TOKDNE;

    // DATA IN phase (we requested the descriptor)
    //serial_print("data phase \n");

    // get length or maxPacketSize bytes into data_in_buf, whichever is smaller
    if(length<maxPacketSize) {
        datalen = length;
    } else {
        datalen = maxPacketSize;
    }
    data = data_in_buf;
    ep0_rx_data_toggle = DATA1;
    endpoint0_receive(data, datalen); // the first data IN uses DATA1, then toggle.
    data += datalen; // advance the pointer



    USB0_TOKEN = USB_TOKEN_DATA_IN; //  DATA IN token to endpoint 0.

    while(USB0_CTL & USB_CTL_TXSUSPENDTOKENBUSY) {} // wait in case there's another token being sent
    while(!(USB0_ISTAT & USB_ISTAT_TOKDNE)) {} // wait for last transfer to finish
    debug_tokdne(USB0_STAT);
    USB0_ISTAT = USB_ISTAT_TOKDNE;

    // Now check how long is the data_in_buf
    uint8_t desc_length = data_in_buf[0];
    // If it's the configuration data_in_buf, the total length is in byte data_in_buf[2]
    if(descType == 2) {
        desc_length = data_in_buf[2];
    }

    // We need to get either length or desc_length, whichever is smaller
    totalBytes = length;
    if(length > desc_length) {
        length = desc_length;
        totalBytes = desc_length; // we don't care about the rest of the data
    }

    // Data we still need to get, if it's negative then we are done!
    int32_t data_left = length - maxPacketSize;

    // Set maxPacketSize to the 8th byte of the data_in_buf if this is a device descriptor
    if(descType == 1) {
        maxPacketSize = data_in_buf[7];
    }

    #if 0
    serial_print("length: ");
    serial_phex(length);
    serial_print(", desc_length: ");
    serial_phex(desc_length);
    serial_print(", maxPacketSize: ");
    serial_phex(maxPacketSize);
    serial_print("\n");
    #endif

    while(data_left>0) {

        //serial_print("data_left: ");
        //serial_phex(data_left);
        //serial_print("\n");

        // get data_left if it's less than a full packet
        if(data_left<maxPacketSize) {
            datalen = data_left;
        } else {
            datalen = maxPacketSize;
        }

        endpoint0_receive(data, datalen); // DATA0/1 toggle
        data += datalen; // advance the pointer

        USB0_TOKEN = USB_TOKEN_DATA_IN; //  DATA IN token to endpoint 0.

        while(USB0_CTL & USB_CTL_TXSUSPENDTOKENBUSY) {} // wait in case there's another token being sent
        while(!(USB0_ISTAT & USB_ISTAT_TOKDNE)) {} // wait for last transfer to finish
        debug_tokdne(USB0_STAT);
        USB0_ISTAT = USB_ISTAT_TOKDNE;

        data_left -= datalen;

    }

    // STATUS phase
    //serial_print("status phase \n");

    // status uses always DATA1, although teensy datasheet says is data0
    ep0_tx_data_toggle = DATA1;
    endpoint0_transmit(NULL, 0);

    USB0_TOKEN = USB_TOKEN_DATA_OUT; //  DATA OUT token to endpoint 0.

    while(USB0_CTL & USB_CTL_TXSUSPENDTOKENBUSY) {} // wait in case there's another token being sent
    while(!(USB0_ISTAT & USB_ISTAT_TOKDNE)) {} // wait for last transfer to finish

    debug_tokdne(USB0_STAT);

    USB0_ISTAT = USB_ISTAT_TOKDNE;

    // data_in_buf is a 255 bytes array to store incomming data.
    //memcpy(data_in_buf, descriptor, totalBytes);

    return totalBytes;

}

/* Set address of the device to num
    It updates the value of USB0_ADDR

*/
void USB_Host::setAddress(uint8_t num) {
    //
    // SET_ADDRESS
    //

    const uint8_t *data = NULL;
    uint32_t datalen = 0;

    // SETUP phase
    serial_print("SET_ADDRESS to 1\n");
    //serial_print("setup phase\n");

    // data is sent low byte first
    setup_command_buffer[0] = 0x00; // SET_ADDRESS = 0x00 05 01 00 00 00 00 00
    setup_command_buffer[1] = 0x05;
    setup_command_buffer[2] = num & 0xFF; // address 1
    setup_command_buffer[3] = 0x00;

    setup_command_buffer[4] = 0x00;
    setup_command_buffer[5] = 0x00;
    setup_command_buffer[6] = 0x00;
    setup_command_buffer[7] = 0x00;

    datalen = 8;
    data = setup_command_buffer;

    // setup always uses DATA0
    ep0_tx_data_toggle = DATA0;
    endpoint0_transmit(data, datalen);

    USB0_TOKEN = USB_TOKEN_SETUP; //  SETUP token to endpoint 0.

    while(USB0_CTL & USB_CTL_TXSUSPENDTOKENBUSY) {} // wait in case there's another token being sent
    while(!(USB0_ISTAT & USB_ISTAT_TOKDNE)) {} // wait for last transfer to finish

    debug_tokdne(USB0_STAT);

    USB0_ISTAT = USB_ISTAT_TOKDNE;


    // NO data phase

    // STATUS phase
    //serial_print("status phase \n");

    // status uses always DATA1, although teensy datasheet says is data0
    ep0_rx_data_toggle = DATA1;
    endpoint0_receive(NULL, 0);

    USB0_TOKEN = USB_TOKEN_DATA_IN; //  DATA IN token to endpoint 0.

    while(USB0_CTL & USB_CTL_TXSUSPENDTOKENBUSY) {} // wait in case there's another token being sent
    while(!(USB0_ISTAT & USB_ISTAT_TOKDNE)) {} // wait for last transfer to finish

    debug_tokdne(USB0_STAT);

    USB0_ISTAT = USB_ISTAT_TOKDNE;

    USB0_ADDR = num & 0xFF; // address 1, low speed enable if detected

}



/* Set the configuration to num

*/
void USB_Host::setConfiguration(uint8_t num) {
    //
    // SET_CONFIGURATION
    //

    const uint8_t *data = NULL;
    uint32_t datalen = 0;

    // SETUP phase
    serial_print("SET_CONFIGURATION to 1\n");
    //serial_print("setup phase\n");

    // data is sent low byte first
    setup_command_buffer[0] = 0x00; // SET_CONFIGURATION = 0x00 09 XX 00 00 00 00 00
    setup_command_buffer[1] = 0x09;
    setup_command_buffer[2] = num & 0xFF; // address 1
    setup_command_buffer[3] = 0x00;

    setup_command_buffer[4] = 0x00;
    setup_command_buffer[5] = 0x00;
    setup_command_buffer[6] = 0x00;
    setup_command_buffer[7] = 0x00;

    datalen = 8;
    data = setup_command_buffer;

    // setup always uses DATA0
    ep0_tx_data_toggle = DATA0;
    endpoint0_transmit(data, datalen);

    USB0_TOKEN = USB_TOKEN_SETUP; //  SETUP token to endpoint 0.

    while(USB0_CTL & USB_CTL_TXSUSPENDTOKENBUSY) {} // wait in case there's another token being sent
    while(!(USB0_ISTAT & USB_ISTAT_TOKDNE)) {} // wait for last transfer to finish

    debug_tokdne(USB0_STAT);

    USB0_ISTAT = USB_ISTAT_TOKDNE;


    // NO data phase

    // STATUS phase
    //serial_print("status phase \n");

    // status uses always DATA1, although teensy datasheet says is data0
    ep0_rx_data_toggle = DATA1;
    endpoint0_receive(NULL, 0);

    USB0_TOKEN = USB_TOKEN_DATA_IN; //  DATA IN token to endpoint 0.

    while(USB0_CTL & USB_CTL_TXSUSPENDTOKENBUSY) {} // wait in case there's another token being sent
    while(!(USB0_ISTAT & USB_ISTAT_TOKDNE)) {} // wait for last transfer to finish

    debug_tokdne(USB0_STAT);

    USB0_ISTAT = USB_ISTAT_TOKDNE;

}

/* Resets the attached USB device for time milliseconds (usually 10)

*/
void USB_Host::resetDevice(uint8_t time) {
    //
    // RESET AGAIN
    //

    // send reset signal for 10 ms
    USB0_CTL |= USB_CTL_RESET;
    serial_print("RESET\n");
    delay(time);
    USB0_CTL &= ~USB_CTL_RESET; // stop reset signals
    //serial_print("RESET DONE\n");

}
