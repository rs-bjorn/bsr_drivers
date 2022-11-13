# BSR Drivers Repository
This page will list and outline a number of drivers to be used with the buddy smart reel. 

## LoRa Driver (For RFM95)
The lora driver requires the "rslora.h" and "rslora.c" files to be located in the compiler directories. 
Once this has been done, set up all of the required pins in the "rslora.h" file.

initialised the module to start sending by using 
```c
/* USER CODE BEGIN 2 */
lora_initialise();
```

### Sending
the following example will demonstrate sending packets
```c
// create a character buffer
uint8_t tmp_buff[] = {'T','e','s','t','!'};

    // begin the lora packet and check return status is ok
    if (lora_begin_packet(0)){
      // write the data to the lora module
      lora_write(&tmp_buff, sizeof(tmp_buff));
      // end the packet and use true to enable GPIO interrupts 
      lora_end_packet(true);
    }
```

### Receiving
the following example will demonstrate receiving packets 
```c
// parse the current lora packet and assign its size to packet size
int packetSize = lora_parse_packet(0);

// check whether data is available and loop if true 
while(available()){
  // check whether the packet has a size greater than 0 
  if(packetSize){
     //print some packet associated data 
     printf("%d | %d | %c | rssi %d\r\n",lora_read_register(0x13),_packetIndex, lora_read(), lora_packet_rssi());
    }
}
```
