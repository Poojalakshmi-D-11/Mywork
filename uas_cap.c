/******************************************implemented on kernel version 3.13.0-170-generic ********/
#include<linux/kernel.h>
#include<linux/module.h>
#include<linux/usb.h>
#include <linux/slab.h>
#include<linux/types.h>




#define CARD_READER_VID 0xaaaa    //card reader MXT USB Device          
#define CARD_READER_PID  0x8816                

#define SONY_VID  0x054c
#define SONY_PID  0x05ba

#define SANDISK_VID 0x0781
#define SANDISK_PID 0x1303

#define GET_MAX_LUN        0xFE
#define REQ_TYPE		      0xA1

#define READ_CAPACITY_LENGTH          0x08
#define READ_INQ_LENGTH          0x24

#define CBW_LEN 31
#define CSW_LEN 13

u8 lun;   //max lun
u8 cdb[16]; //CBD
u32 exp_tag; //expected return tag
u8 endpoint_in_addr,endpoint_out_addr; //endpoint adress for bulk only devices

struct usb_device *udev;  //A pointer to the USB device that the descriptor should be retrieved from 

//to determine length of scsi command

static u8 cdb_length[256] = {
//	 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  0
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  1
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  2
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  3
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  4
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  5
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  6
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  7
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  8
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  9
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  A
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  B
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  C
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  D
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  E
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  F
};

//supported device table
static struct usb_device_id usbdev_table [] = {
	{USB_DEVICE(CARD_READER_VID, CARD_READER_PID)},
	{USB_DEVICE(SONY_VID, SONY_PID)},
	{USB_DEVICE(SANDISK_VID, SANDISK_PID)},
	{} /*terminating entry*/	
};


// Command Block Wrapper (CBW)
struct command_block_wrapper {
	u8 dCBWSignature[4];
	u32 dCBWTag;
	u32 dCBWDataTransferLength;
	u8 bmCBWFlags;
	u8 bCBWLUN;
	u8 bCBWCBLength;
	u8 CBWCB[16];
};


// Command Status Wrapper (CSW)
struct command_status_wrapper {
	u8 dCSWSignature[4];
	u32 dCSWTag;
	u32 dCSWDataResidue;
	u8 bCSWStatus;
};


//function prototype
int find_lun(void);
int fill_cbw(void);
int get_status(void);

//function defenisions


//------------get status of received data----------//
int get_status()
{	
	int size,k;
	struct command_status_wrapper csw;
	memset(&csw, 0, sizeof(csw));
	k=usb_bulk_msg(udev, usb_rcvbulkpipe (udev,endpoint_in_addr), (unsigned char*)&csw, CSW_LEN, &size, 3000);
	if(k!=0)
	{
		return k;
	}
	if(csw.bCSWStatus!=0)
	{
		printk(KERN_INFO "	status: failed\n");
		return -1;
	}
	if(csw.dCSWTag!=exp_tag) 
	{
		printk(KERN_INFO "tag not matched\n");
		return -1;
	}
	if(csw.dCSWSignature[0]!=85 || csw.dCSWSignature[1]!=83 || csw.dCSWSignature[2]!= 66 || csw.dCSWSignature[3]!=83)
	{
		printk(KERN_INFO "	signalture not matched\n");
		return -1;
	}
	
	return 0;

}




/*-----------finding maximum lun------------*/

int find_lun()
{
int r;   //number of bytes receivedd after control msg sent or error
void *lun1= kmalloc(1, GFP_KERNEL);


r=usb_control_msg(udev, usb_rcvctrlpipe (udev,0x00),GET_MAX_LUN,REQ_TYPE, 0, 0, lun1, 1, 10000);  //sending control signal
memcpy(&lun, lun1, 1);
kfree(lun1);


if(r==0)   //for stall
{
lun=0;
}

if(r<0)  //for error
{
return r;
}

  

printk("----------------------------\n");

return 0;
}




int fill_cbw()
{
	u8 buffer[64];  //buffer to store inquery data
	u8 buffer1[8];  //buffer to store capacity data
	u64 big,temp1,temp2,temp3,temp4,temp5,temp6,temp7,temp8,temp9;  //for capacity calculation
	unsigned char vid[9], pid[9], rev[5];            //to  veryfy PID and VID through SCSI command
	int i,r,k;			 //error codes for: bulk send , bulk recieve, stauts 
	int ei,ej;         //error codes for: inquiry send , inquiry recieve
	int size,size1;  //received sizes
	u8 cdb_len;          //CBWCB length
	u32  max_lba,block_size;  //maximum logical block adress and block size 
	static u32 tag = 1;
	int ct=0;   //incase of stall 

	struct command_block_wrapper cbw;


	

	
	memset(&cbw, 0, sizeof(cbw));
	memset(cdb, 0, sizeof(cdb));
	memset(buffer, 0, sizeof(buffer));
	
	cbw.dCBWSignature[0] = 'U';
	cbw.dCBWSignature[1] = 'S';
	cbw.dCBWSignature[2] = 'B';
	cbw.dCBWSignature[3] = 'C';	
	cbw.bCBWLUN = lun;
	cbw.bmCBWFlags = USB_DIR_IN; //since data transfer is in for both data Inquiry and capacity- 0x80

////-------------------------------------start of inquiry---------////
	
	cdb[0] = 0x12;	// Inquiry
	cdb[4] = 0x24;
	cdb_len = cdb_length[cdb[0]];
	cbw.dCBWTag = tag;
	cbw.dCBWDataTransferLength = READ_INQ_LENGTH; //
 	cbw.bCBWCBLength = cdb_len;
	memcpy(cbw.CBWCB, cdb, cdb_len);
	exp_tag=tag;
	

	//--------inqury send---------//
	
	ei=usb_bulk_msg(udev, usb_sndbulkpipe(udev,endpoint_out_addr),(unsigned char*)&cbw, 31, &size, 10000);
	if(ei!=0)
	{
	return ei;
	}
	
	//--------Inquiry receive----//
	ej=usb_bulk_msg(udev, usb_rcvbulkpipe(udev,endpoint_in_addr), (unsigned char*)&buffer, READ_INQ_LENGTH, &size, 1000);
	
	if(ej!=0)
	{
	return ej;
	}
	//---------get status----------//
	k=get_status();
	if(k==-1)
	{
		return -1;
	}

	for (i=0; i<8; i++) {
		vid[i] = buffer[8+i];
		pid[i] = buffer[16+i];
		rev[i/2] = buffer[32+i/2];	// instead of another loop
	}

	vid[8] = 0;
	pid[8] = 0;
	rev[4] = 0;
	

////-------------------------end of inquiry----------////

	

////---------------------start of capacity determination-------//
	
	memset(cdb, 0, sizeof(cdb));
	memset(buffer1, 0, sizeof(buffer1));
	cdb[0] = 0x25;
	cbw.dCBWTag = tag+1;
	cbw.dCBWDataTransferLength = READ_CAPACITY_LENGTH; //
	cdb_len = cdb_length[cdb[0]];	
	cbw.bCBWCBLength = cdb_len;
	memcpy(cbw.CBWCB, cdb, cdb_len);

	exp_tag = tag+1;

	// The transfer length must always be exactly 31 bytes.

	//--------capacity  send---------//
	
	r = usb_bulk_msg(udev, usb_sndbulkpipe(udev,endpoint_out_addr),(unsigned char*)&cbw, 31, &size, 10000);

   while(r!=0 && ct<5)  //if not sent first time
	{
		r=usb_clear_halt(udev,usb_sndbulkpipe(udev,endpoint_out_addr)) ;
		r = usb_bulk_msg(udev, usb_sndbulkpipe(udev,endpoint_out_addr),(unsigned char*)&cbw, 31, &size, 1000);
		ct++;

	}
	if(ct==5 && r!=0)
	{
		return r;
	}

	
	//--------capacity  recieve---------//
	i=usb_bulk_msg(udev, usb_rcvbulkpipe (udev,endpoint_in_addr), (unsigned char*)&buffer1,  READ_CAPACITY_LENGTH, &size1, 10000);
	
	if(i!=0)
	{
		return i;
	}
	k=get_status();  //check status
	if(k==-1)
	{
		return -1;
	}



	max_lba= (buffer1[0] << 24) | (buffer1[1] << 16) | ( buffer1[2] << 8 ) | (buffer1[3]);   //get max logical adress
	block_size=(buffer1[4] << 24) | (buffer1[5] << 16) | ( buffer1[6] << 8 ) | (buffer1[7]); //get block size
	big=(u64)(max_lba+1)*(u64)(block_size); //total memeory size in bytes
	temp1=big>>30;
	temp2=big%(1<<30);
	temp9=big%(1<<20);
	temp3=(temp2*10)>>30; //total memory size in GB
	temp8=big>>20;
	temp4=(temp9*10)>>20;//total memory size in MB
	temp5=big/(u64)(1000000000);
	temp6=big%(u64)(1000000000);
	temp7=(temp6*10)/(u64)(1000000000);

	printk(KERN_INFO "capacity of device is:\n");
	printk(KERN_INFO "	in hex:   Max LBA: %08X, Block Size: %08X \n	therefore total size: (%llx Bytes)\n", max_lba, block_size, big);
	printk(KERN_INFO "	in decimal:   Max LBA: %u, Block Size: %u Bytes\n	total size: (%lu Bytes) (%lu.%lu MB)(%lu.%lu GB/%lu.%lu GiB)\n", (unsigned int)max_lba,(unsigned int)block_size,(unsigned long) big,(unsigned int)temp8,(unsigned int)temp9,(unsigned long)temp5,(unsigned long)temp7,(unsigned long)temp1,(unsigned long)temp3);
	printk("----------------------------\n");
	return 0;
	///---------------------end of capacity determination-------//
	}



static void usbdev_disconnect(struct usb_interface *interface)
{
	printk(KERN_INFO "USBDEV Device Removed\n");
	return;
}


static int usbdev_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	int i;
	int err1;
	u8 epAddr, epAttr;  //for end point adress and end point attribute
	struct usb_endpoint_descriptor *ep_desc;
	udev = interface_to_usbdev(interface);
	
	if(id->idProduct == CARD_READER_PID)
	{
		printk(KERN_INFO "----------Known USB drivedetected----------\n");
		printk(KERN_INFO "card reader plugged in\n");
	}
	else if(id->idProduct == SONY_PID)
	{
		printk(KERN_INFO "-------------Known USB drivedetected---------\n");
		printk(KERN_INFO "sony plugged in\n");
	}
	else if(id->idVendor == SANDISK_PID)
	{
		printk(KERN_INFO "------Known USB drivedetected-------\n");
		printk(KERN_INFO "sandisk Plugged in\n");
	}

	printk(KERN_INFO "USB DEVICE VID : %x\n", id->idVendor);
	printk(KERN_INFO "USB DEVICE PID : %x\n", id->idProduct);
	printk(KERN_INFO "USB DEVICE CLASS : %x\n", interface->cur_altsetting->desc.bInterfaceClass);
	printk(KERN_INFO "USB DEVICE SUB CLASS : %x\n", interface->cur_altsetting->desc.bInterfaceSubClass);
	printk(KERN_INFO "USB DEVICE Protocol : %x\n", interface->cur_altsetting->desc.bInterfaceProtocol);
	printk(KERN_INFO "No. of Endpoints = %d\n", interface->cur_altsetting->desc.bNumEndpoints);
	printk(KERN_INFO "No. of Altsettings = %d\n",interface->num_altsetting);

	for(i=0;i<interface->cur_altsetting->desc.bNumEndpoints;i++)
	{
		ep_desc = &interface->cur_altsetting->endpoint[i].desc;
		epAddr = ep_desc->bEndpointAddress;
		epAttr = ep_desc->bmAttributes;
	
		if((epAttr & USB_ENDPOINT_XFERTYPE_MASK)==USB_ENDPOINT_XFER_BULK)
		{
			if(epAddr & USB_DIR_IN){
				endpoint_in_addr=epAddr;
				printk(KERN_INFO "EP %d is Bulk IN address=%x \n", i,endpoint_in_addr); 
				
		
				}
			else{
				endpoint_out_addr=epAddr;
				printk(KERN_INFO "EP %d is Bulk OUT address=%x\n", i, endpoint_out_addr);
				
				}
	
		}

	}

	

	printk(KERN_INFO "READING capacity:\n");
	err1=find_lun();
	if(err1!=0)
	{
		 return err1;
	}
	err1=fill_cbw();	
	if(err1!=0)
	{
		 return err1;
	}
	return 0;
}



/*Operations structure*/
static struct usb_driver usbdev_driver = {
	name: "usbdev",  //name of the device
	probe: usbdev_probe, // Whenever Device is plugged in
	disconnect: usbdev_disconnect, // When we remove a device
	id_table: usbdev_table, //  List of devices served by this driver
};


int device_init(void)
{	
	int res;
	res=usb_register(&usbdev_driver);
	if(res!=0)
	{
		printk("usb_register failed. Error number %d\n", res);
		return res;
	}
	printk(KERN_INFO "--------UAS READ Capacity Driver Inserted------\n");
	return 0;
}

void device_exit(void)
{
	usb_deregister(&usbdev_driver);
	printk(KERN_NOTICE "-----Leaving Kernel----\n");
	
	return 0;
}

module_init(device_init);
module_exit(device_exit);
MODULE_LICENSE("GPL");


