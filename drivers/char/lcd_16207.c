/*
 *  lcd16207.c - Create an input/output character device
 *  not all functions are working. But they could be used in 
 *  future. 
 */
//shell commands to compile the module in "drivers/char/"
//make ARCH=nios2 CROSS_COMPILE=nios2-linux-uclibc- -C ~/uClinux-dist/linux-2.6.x M=`pwd` modules 
//nios2-linux-uclibc-strip -R .comment -R .note -g --strip-unneeded hello.ko

#include <linux/kernel.h>	/* We're doing kernel work */
#include <linux/module.h>	/* Specifically, a module */
#include <linux/fs.h>
#include <asm/uaccess.h>	/* for get_user and put_user */
#include <linux/delay.h>

#include <linux/lcd_16207.h>

#define SUCCESS 0
//#define DEBUG 1

/* 
 * Is the device open right now? Used to prevent
 * concurent access into the same device 
 */
static int Device_Open = 0;

/* 
 * Waiting time for writing the next
 * characters of the buffer to the display 
 */
static unsigned long Display_Wait = 1500000;

/* 
 * The message the device will give when asked 
 */
static char Message[BUF_LCD_CHARS];

/* 
 * How far did the process reading the message get?
 * Useful if the message is larger than the size of the
 * buffer we get to fill in device_read. 
 */
static char *Message_Ptr;

/* 
 * Pointer to the Lines of the Display
 */
static char *Message_Ptr_Line1 = Message;
static char *Message_Ptr_Line2 = Message + BUF_LCD_LINE;
static char *Message_Ptr_Write;

/* 
 * This is called whenever a process attempts to open the device file 
 */
static int device_open(struct inode *inode, struct file *file)
{
#ifdef DEBUG
  printk(KERN_INFO "device_open(%p)\n", file);
#endif
  
  /* 
   * We don't want to talk to two processes at the same time 
   */
  if (Device_Open)
    return -EBUSY;
  
  Device_Open++;
  /*
   * Initialize the message 
   */
  memset(Message,32,BUF_LCD_CHARS);	
  
  try_module_get(THIS_MODULE);
  return SUCCESS;
}

static int device_release(struct inode *inode, struct file *file)
{
#ifdef DEBUG
  printk(KERN_INFO "device_release(%p,%p)\n", inode, file);
#endif
  
  /* 
   * We're now ready for our next caller 
   */
  Device_Open--;
  
  module_put(THIS_MODULE);
  return SUCCESS;
}

/* 
 * This function is called whenever a process which has already opened the
 * device file attempts to read from it.
 */
//not implemented yet
static ssize_t device_read(struct file *file,	/* see include/linux/fs.h   */
			   char __user * buffer,	/* buffer to be
							 * filled with data */
			   size_t length,	/* length of the buffer     */
			   loff_t * offset)
{
  /* 
   * Number of bytes actually written to the buffer 
   */
  int bytes_read = 0;
  Message_Ptr = Message;
  LcdReadLines();

#ifdef DEBUG
  printk(KERN_INFO "device_read(%p,%p,%d)\n", file, buffer, length);
  printk(KERN_INFO "Message:%s:End\n",Message);
#endif
  
  /* 
   * Actually put the data into the buffer 
   */
  while (length>0 && bytes_read<BUF_LCD_CHARS) {
    
    /* 
     * Because the buffer is in the user data segment,
     * not the kernel data segment, assignment wouldn't
     * work. Instead, we have to use put_user which
     * copies data from the kernel data segment to the
     * user data segment. 
     */
    put_user(*(Message_Ptr++), buffer++);
    length--;
    bytes_read++;
  }

#ifdef DEBUG
  printk(KERN_INFO "Read %d bytes, %d left\n", bytes_read, length);
#endif

  if (length>0) {
    /* 
     * Put a zero at the end of the buffer, so it will be 
     * properly terminated 
     */ 
    put_user('\0', buffer++);
    bytes_read++;
    length--;
  }

  /* 
   * Read functions are supposed to return the number
   * of bytes actually inserted into the buffer 
   */
 return bytes_read;
}

/* 
 * This function is called when somebody tries to
 * write into our device file. 
 */
static ssize_t device_write(struct file *file,
			    const char __user * buffer, size_t length, loff_t * offset)
{
  int ii;

#ifdef DEBUG
  printk(KERN_INFO "device_write(%p,%s,%d);\n", file, buffer, length);
#endif

  ii=0;
  while(ii<length)
    {
      strncpy(Message_Ptr_Line1, Message_Ptr_Line2, BUF_LCD_LINE);
      Message_Ptr=Message;

      if ( (length-ii) > BUF_LCD_LINE)
	{
	  copy_from_user(Message_Ptr_Line2, buffer+ii, BUF_LCD_LINE);
	  ii=ii+BUF_LCD_LINE;
	}
      else
	{
	  copy_from_user(Message_Ptr_Line2, buffer+ii, length-ii);
	  memset(Message_Ptr_Line2+(length-ii),32,BUF_LCD_LINE-(length-ii));  
	  ii=length;
	}

      WaitNios(Display_Wait);
      LcdWriteLines();
    }
#ifdef DEBUG
  printk(KERN_INFO "Message:%s:End\n",Message);
#endif
  /* 
   * Again, return the number of input characters used 
   */
  return length;
}

/* 
 * This function is called whenever a process tries to do an ioctl on our
 * device file. We get two extra parameters (additional to the inode and file
 * structures, which all device functions get): the number of the ioctl called
 * and the parameter given to the ioctl function.
 *
 * If the ioctl is write or read/write (meaning output is returned to the
 * calling process), the ioctl call returns the output of this function.
 *
 */
static int device_ioctl(struct inode *inode,	/* see include/linux/fs.h */
		 struct file *file,	/* ditto */
		 unsigned int ioctl_num,	/* number and param for ioctl */
		 unsigned long ioctl_param)
{
  int i;
  char *temp;
  char ch;

  /* 
   * Switch according to the ioctl called 
   */
  switch (ioctl_num) {
  case IOCTL_SET_MSG:
    /* 
     * Receive a pointer to a message (in user space) and set that
     * to be the device's message.  Get the parameter given to 
     * ioctl by the process. 
     */
    temp = (char *)ioctl_param;
    
    /* 
     * Find the length of the message 
     */
    get_user(ch, temp);
    if (ch=='\0')
      break;
    for (i = 0; ch!='\0' ; i++, temp++)
      get_user(ch, temp);
    
    device_write(file, (char *)ioctl_param, i-1, 0);

    break;

  case IOCTL_SET_DISP_WAIT:
    /* 
     * Set the time of the Display to wait before
     * printing the next 2x16 chars to the display.
     * 
     */
    Display_Wait=(unsigned long)ioctl_param;
#ifdef DEBUG
  printk(KERN_INFO "Display wait set to hex %08X\n", (unsigned int) Display_Wait);
#endif
    break;

  case IOCTL_GET_MSG:
    /* 
     * Give the current message to the calling process - 
     * the parameter we got is a pointer, fill it. 
     */
    i = device_read(file, (char *)ioctl_param, BUF_LCD_CHARS+1, 0);
    break;
    
    //not tested - still todo
  case IOCTL_GET_NTH_BYTE:
    /* 
     * This ioctl is both input (ioctl_param) and 
     * output (the return value of this function) 
     */
    return Message[ioctl_param];
    break;
  }
  
  return SUCCESS;
}

/* Module Declarations */

/* 
 * This structure will hold the functions to be called
 * when a process does something to the device we
 * created. Since a pointer to this structure is kept in
 * the devices table, it can't be local to
 * init_module. NULL is for unimplemented functions. 
 */
struct file_operations Fops = {
  .read = device_read,
  .write = device_write,
  .ioctl = device_ioctl,
  .open = device_open,
  .release = device_release,	/* a.k.a. close */
};

/* 
 * Initialize the module - Register the character device 
 */
static int __init lcd16207_module_init(void)
{
  int ret_val;
  /* 
   * Register the character device (atleast try) 
   */
  ret_val = register_chrdev(MAJOR_NUM, DEVICE_NAME, &Fops);
  
  /* 
   * Negative values signify an error 
   */
  if (ret_val < 0) {
    printk(KERN_ALERT "%s failed with %d\n",
	   "Sorry, registering the character device ", ret_val);
    return ret_val;
  }
  printk(KERN_INFO "Device %s registered\n", DEVICE_FILE_NAME);
/*  
  printk(KERN_INFO "%s The lcd16207 major device number is %d.\n",
	 "Registeration is a success", MAJOR_NUM);
  printk(KERN_INFO "If you want to talk to the device driver,\n");
  printk(KERN_INFO "you'll have to create a device file. \n");
  printk(KERN_INFO "We suggest you use:\n");
  printk(KERN_INFO "mknod %s c %d 0\n", DEVICE_FILE_NAME, MAJOR_NUM);
  printk(KERN_INFO "The device file name is important, because\n");
  printk(KERN_INFO "the ioctl program assumes that's the\n");
  printk(KERN_INFO "file you'll use.\n");
*/  
  mdelay(20);
  WriteNios(ADR_LCD_COMMAND,LCD_COM_RESET);
  mdelay(5);
  WriteNios(ADR_LCD_COMMAND,LCD_COM_RESET);
  udelay(200);
  WriteNios(ADR_LCD_COMMAND,LCD_COM_RESET);
  udelay(200);
  WriteNios(ADR_LCD_COMMAND,LCD_NUMROWS | LCD_INTERFACE | LCD_CMD_FUNC);
  udelay(50);
  WriteNios(ADR_LCD_COMMAND,LCD_CMD_ONOFF);
  udelay(50);
  WriteNios(ADR_LCD_COMMAND,LCD_CMD_CLEAR);
  mdelay(2);
  WriteNios(ADR_LCD_COMMAND,LCD_CMD_ONOFF | LCD_CMD_ENABLE_DISP | LCD_CMD_ENABLE_CURSOR); // | LCD_CMD_ENABLE_BLINK);
  udelay(50);
  //write " Hello uClinux! " to the display
  WriteNios(ADR_LCD_DATA,0x20);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x48);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x65);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x6C);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x6C);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x6F);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x20);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x75);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x43);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x6C);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x69);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x6E);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x75);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x78);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x21);
  udelay(50);
  WriteNios(ADR_LCD_DATA,0x20);
  udelay(50);

  return 0;
}

/* 
 * Cleanup - unregister the appropriate file from /proc 
 */
static void __exit lcd16207_module_exit(void)
{
  //int ret;
  
  /* 
   * Unregister the device 
   */
  unregister_chrdev(MAJOR_NUM, DEVICE_NAME);
  
  /* 
   * If there's an error, report it 
   */
  //if (ret < 0)
  //printk(KERN_ALERT "Error: unregister_chrdev: %d\n", ret);
}


//write all chars to the LCD
static void LcdWriteLines(void)
{
  int i;
  WriteNios(ADR_LCD_COMMAND,0x80);
  udelay(50);
  Message_Ptr_Write = Message_Ptr_Line1;
  for (i = 0; i < BUF_LCD_LINE; i++)
    {
      WriteNios(ADR_LCD_DATA, (unsigned long)*(Message_Ptr_Write+i) );
      udelay(50);
    }
  WriteNios(ADR_LCD_COMMAND,0x80 + 0x40);
  udelay(50);
  Message_Ptr_Write = Message_Ptr_Line2;
  for (i = 0; i < BUF_LCD_LINE; i++)
    {
      WriteNios(ADR_LCD_DATA, (unsigned long)*(Message_Ptr_Write+i) );
      udelay(50);
    }
}

//read all chars from the LCD
static void LcdReadLines(void)
{
  int i;
  WriteNios(ADR_LCD_COMMAND,0x80);
  udelay(50);
  Message_Ptr_Write = Message_Ptr_Line1;
  for (i = 0; i < BUF_LCD_LINE; i++)
    {
      *(Message_Ptr_Write+i)=ReadNios(ADR_LCD_READ);
      udelay(50);
    }
  WriteNios(ADR_LCD_COMMAND,0x80 + 0x40);
  udelay(50);
  Message_Ptr_Write = Message_Ptr_Line2;
  for (i = 0; i < BUF_LCD_LINE; i++)
    {
      *(Message_Ptr_Write+i)=ReadNios(ADR_LCD_READ);
      udelay(50);
    }
}

//write 32bit value to avalon bus address
static void WriteNios(unsigned long addr, unsigned long value)
{
  (* (volatile unsigned long *)(addr))=value;
}
  
//read 32bit value from avalon bus address
static unsigned long ReadNios(unsigned long addr)
{
  return (unsigned long)(* (volatile unsigned long *)(addr));
}

//wait for us
static void WaitNios(unsigned long us)
{
  unsigned long usLoop,i;
  if (us > 2000)
    {
      usLoop=us/2000;
      us = us % usLoop;
      for (i=0;i<usLoop;i++)
	{
	  udelay(2000);
	}
    }   
  udelay(us);
}

module_init(lcd16207_module_init);
module_exit(lcd16207_module_exit);


//#define author "The kernel programming guide, NIOS wiki users and mamba"
//#define description "A driver for lcd16207 device of the DE2 board and the lcd16207 core of altera."
//MODULE_AUTHOR(author);
//MODULE_DESCRIPTION(description);
MODULE_SUPPORTED_DEVICE(DEVICE_NAME);
MODULE_LICENSE("GPL");


