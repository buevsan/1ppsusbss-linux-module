#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,11,0)
  #include <linux/sched.h>
#else
  #include <uapi/linux/sched/types.h>
#endif

#define BULK_EP_OUT 0x01
#define BULK_EP_IN 0x81
#define MAX_PKT_SIZE 512
#define MIN(a,b) (((a) <= (b)) ? (a) : (b))

#define SIGNAL_PERIOD_NS (NSEC_PER_SEC)
#define SIGNAL_DUR_NS    (200000000)

#define STAT_PRINT_S     (2)
#define WAKEUP_LONG_US   (70)
#define WAKEUP_SHORT_US  (1)

static unsigned long signal_dur_ns = SIGNAL_DUR_NS;
MODULE_PARM_DESC(delay,
	"Delay between setting and dropping the signal (ns)");
module_param_named(delay, signal_dur_ns, ulong, 0);

static unsigned long signal_period_ns = SIGNAL_PERIOD_NS;
MODULE_PARM_DESC(period,
	"Signal period (ns)");
module_param_named(period, signal_period_ns, ulong, 0);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)
 #define ktime_t u64
#endif

#define PRIkt "llu"

struct time_stat_s {
  ktime_t min;
  ktime_t max;
  ktime_t avg;
};

struct m1pps_stat_s {
  struct time_stat_s wakeup_long;   /* duration beetween long wakeups */
  struct time_stat_s wakeup_short;  /* duration beetween short wakeups */
  struct time_stat_s signal_period; /* signal period */
  struct time_stat_s write;         /* USB write duration */
  struct time_stat_s signal_err;    /* current absolute signal error */
  u64 signal_cnt;                   /* count of generated signals */
  u64 short_cnt;                    /* count of short mode switching */
  ktime_t st;                       /* generation start time */
  ktime_t ct;                       /* current time */ 
};

static struct m1pps_stat_s stat;
static struct m1pps_stat_s proc_stat;
static char   m1pps_name[] = "1pps-tx";

/* internal per port structure */
struct pps_generator_cyfx3 {
	struct usb_device *usbd;	/* USB device */
	int attached;
  u8 signal_state;          /* current state of signal */
  struct task_struct *th;
};


void time_stat_init( struct time_stat_s *s)
{
  s->min = U64_MAX/2;
  s->max = 0;
  s->avg = 0;
}

void time_stat_calc( struct time_stat_s *s, ktime_t v)
{
  if ( v > s->max )
    s->max = v;
  if ( v < s->min )
    s->min = v;
  s->avg = (s->avg) ? (3 * s->avg + v) >> 2 : v;
}

void time_stat_print( const struct time_stat_s *s, const char *name)
{
  pr_info("%s: min/avg/max/err:  %"PRIkt"/%"PRIkt"/%"PRIkt"/%"PRIkt"\n", 
           name, s->min, s->avg, s->max, s->max - s->min );
}

int time_stat_sprint( char *str, const struct time_stat_s *s, const char *name)
{
  return sprintf(str, "%s: min/avg/max/err:  %"PRIkt"/%"PRIkt"/%"PRIkt"/%"PRIkt"\n", 
                 name, s->min, s->avg, s->max, s->max - s->min );
}

void time_stat_seq_print( struct seq_file *f, const struct time_stat_s *s, const char *name)
{
  seq_printf(f, "%-24s%-20"PRIkt"%-20"PRIkt"%-20"PRIkt"%-20"PRIkt"\n", 
                 name, s->min, s->avg, s->max, s->max - s->min );
}

static void stat_init( struct m1pps_stat_s *s) 
{
  time_stat_init( &s->wakeup_short );
  time_stat_init( &s->wakeup_long );
  time_stat_init( &s->signal_period );
  time_stat_init( &s->write );
  time_stat_init( &s->signal_err );
  s->short_cnt=0;
  s->signal_cnt=0;
  s->st = 0;
  s->ct = 0;
}

static int stat_proc_show(struct seq_file *f, void *v) {
  seq_printf(f, "%-24s%-20s%-20s%-20s%-20s\n", 
                 "", "min ", "avg ", "max ", "R " );
  time_stat_seq_print( f, &proc_stat.wakeup_short, "wakeup dur short (ns)"); 
  time_stat_seq_print( f, &proc_stat.wakeup_long, "wakeup dur long (ns)"); 
  time_stat_seq_print( f, &proc_stat.signal_period, "signal period (ns)");
  time_stat_seq_print( f, &proc_stat.signal_err, "signal error (ns)");
  time_stat_seq_print( f, &proc_stat.write, "USB write dur (ns)");
  seq_printf(f, "\n%-24s%-20i\n","long period (us)", WAKEUP_LONG_US);
  seq_printf(f, "%-24s%-20i\n","short period (us)", WAKEUP_SHORT_US);
  seq_printf(f, "%-24s%-20lu\n","signal period (ns)", signal_period_ns);
  seq_printf(f, "%-24s%-20lu\n","signal dur (ns)", signal_dur_ns);
  seq_printf(f, "%-24s%-20llu\n","total signals", proc_stat.signal_cnt);
  seq_printf(f, "%-24s%-20"PRIkt"\n","total duration (ns)", proc_stat.ct - proc_stat.st );
  seq_printf(f, "%-24s%-20llu\n","short count", proc_stat.short_cnt);
  return 0;
}

static int stat_proc_open(struct inode *inode, struct  file *file) {
  return single_open(file, stat_proc_show, NULL);
}

static const struct file_operations stat_proc_fops = {
  .owner = THIS_MODULE,
  .open = stat_proc_open,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

void gen_write_callback(struct urb *u)
{
	usb_free_urb(u);
  	return;
}

static int pps_write(struct usb_device *usbd, u8 value)
{
  	int ret = 0;
	unsigned char *buf;
	struct urb *urb;
	struct pps_generator_cyfx3 *gen = dev_get_drvdata(&usbd->dev);

	buf = kzalloc(sizeof(char) * MAX_PKT_SIZE, GFP_ATOMIC);
	buf[0] = value;
	
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		dev_err(&usbd->dev, "Failed to allocate a URB!\n");
		return -ENOMEM;
	}
	

	usb_fill_bulk_urb(urb, usbd,
	    			usb_sndbulkpipe(usbd, BULK_EP_OUT),
				buf,
				MAX_PKT_SIZE,
				gen_write_callback,
				gen);
	ret = usb_submit_urb(urb, GFP_ATOMIC);
	//while(gen->inprogress != 0);
/*	ret = usb_bulk_msg(usbd, usb_sndbulkpipe(usbd, BULK_EP_OUT),
			&value, sizeof(value), &wrote_cnt, 5000);
	dev_info(&usbd->dev, "wr %d, %d\n", wrote_cnt, ret);*/
	return ret;
}


int m1pps_thread(void *data)
{
  struct pps_generator_cyfx3 *gen = data;
	int ret = 0;
  ktime_t ct=0;   /* current time */
  ktime_t lut=0;  /* last statistics update time */
  ktime_t sst=0;  /* strong calculated time of last signal start */
  ktime_t rst=0;  /* real time of last signal start */
  ktime_t lwt=0;  /* last wakeup time */
  ktime_t t; 
  ktime_t period_ns = signal_period_ns;

  long sleep_us =  WAKEUP_LONG_US; 

  printk( KERN_INFO "%s started \n", m1pps_name);
  struct sched_param param = { .sched_priority = 1 };
	printk( KERN_INFO "%s set pid:%i priority:%i\n", 
          m1pps_name, current->pid, param.sched_priority);
  sched_setscheduler(current, SCHED_FIFO, &param);

  stat_init( &stat );

  while ( ! kthread_should_stop() ) {

    lwt = ct;

    usleep_range( sleep_us, sleep_us );

    ct = ktime_get_real_ns();

    stat.ct = ct;

    if ( ! lwt )  {
      stat.st = ct;
      continue;
    }  

    if ( sleep_us ==  WAKEUP_SHORT_US)
      time_stat_calc( &stat.wakeup_short, ct - lwt );
    else
      time_stat_calc( &stat.wakeup_long, ct - lwt );
   

    if ( gen->signal_state ) {
      
      if ( ( ct - rst ) >= signal_dur_ns ) {
        t = ktime_get_real_ns();
  	    pps_write(gen->usbd, 0);
        time_stat_calc( &stat.write, ktime_get_real_ns() - t);
        //pr_info("signal off\n"); 
  	    if (ret) 
  		    printk(KERN_CRIT "r %d\n", ret);
        gen->signal_state = 0;
      } 
  
    } else  {

      if ( ! rst )
        sst = ct/period_ns*period_ns;
      
      if ( ! ( (ct - sst)/period_ns  ) ) { 
        t = period_ns - ct % period_ns;
        if ( t > (period_ns>>9) ) {
          continue;
        } else {
    	    if ( t > 10000 ) {
            if ( sleep_us != WAKEUP_SHORT_US ) {  
              stat.short_cnt++;
              sleep_us = WAKEUP_SHORT_US;
            }
            continue;
          }  
        }
      }  
  
      sleep_us = WAKEUP_LONG_US;
      
      stat.signal_cnt++;
      t = ktime_get_real_ns();
  	  ret = pps_write(gen->usbd, 1);
  	  if (ret) 
  		  printk(KERN_CRIT "r %d\n", ret);
      time_stat_calc( &stat.write, ktime_get_real_ns() - t);
      
      if ( rst ) 
        time_stat_calc( &stat.signal_period, ct - rst ); 

      rst = ct;
      sst += period_ns;
      gen->signal_state = 1;
      time_stat_calc( &stat.signal_err, (ct > sst)?ct-sst:sst-ct);
      //pr_info("signal on ct:%"PRIkt" err:%"PRIkt"\n", ct, (ct > st)?ct-st:st-ct ); 

    }  

    if ( ((ct - lut)/(NSEC_PER_SEC*STAT_PRINT_S)) ) {
      proc_stat = stat;
      lut = ct;
    }

  }
  printk( KERN_INFO "%s finished \n", m1pps_name);
  return 0;
}


static int m1pps_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_device *usbd = NULL;
	struct pps_generator_cyfx3 *gen = NULL;

	usbd = interface_to_usbdev(interface);

	gen = kzalloc(sizeof(*gen), GFP_KERNEL);
	if (!gen)
		return -ENOMEM;
	dev_info(&usbd->dev, "allocate generator at 0x%p\n", gen);

	gen->usbd = usbd;
	dev_set_drvdata(&usbd->dev, gen);

	dev_info(&usbd->dev, "attached to usb device %d\n", usbd->devnum);
	gen->attached = 1;

  proc_create(m1pps_name, 0, NULL, &stat_proc_fops);

  gen->th = kthread_run(m1pps_thread, gen, "%s thread", m1pps_name);
	
	return 0;
}
 
static void m1pps_disconnect(struct usb_interface *interface)
{
 	struct usb_device *usbd = interface_to_usbdev(interface);
	struct pps_generator_cyfx3 *gen = dev_get_drvdata(&usbd->dev);

  kthread_stop( gen->th ) ;
  remove_proc_entry(m1pps_name, 0);
	dev_info(&usbd->dev, "free generator at 0x%p\n", gen);
	kfree(gen);
  	//usb_deregister_dev(interface, &class);
	printk(KERN_INFO "%s device removed\n", m1pps_name);
}
 
static struct usb_device_id m1pps_dev_table[] =
{
    { USB_DEVICE(0x04b4, 0x00fa) },
    {} /* Terminating entry */
};
MODULE_DEVICE_TABLE (usb, m1pps_dev_table);

static struct usb_driver m1pps_driver =
{
	.name = "1pps_tx",
	.id_table = m1pps_dev_table,
	.probe = m1pps_probe,
	.disconnect = m1pps_disconnect,
};

static int __init m1pps_init(void)
{
 	int ret = 0;
	ret = usb_register(&m1pps_driver);
	printk( KERN_INFO "usb_register: %d\n", ret);
	return ret;
}

static void __exit m1pps_exit(void)
{
   usb_deregister(&m1pps_driver);
}

module_init(m1pps_init);
module_exit(m1pps_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexander V. Buev <san@zzz.spb.ru>");
MODULE_AUTHOR("Vladimir Georgiev");
MODULE_AUTHOR("Alexander Gordeev <lasaine@lvk.cs.msu.su>");
MODULE_DESCRIPTION("Xypress FX3-based PPS board USB driver");
