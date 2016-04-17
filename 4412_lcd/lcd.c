#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <asm/io.h>
#include <asm/div64.h>
#include <asm/dma-mapping.h>

#define VSPW       9
#define VBPD       22
#define LINEVAL    599
#define VFPD       11 

#define HSPW       40
#define HBPD       159 
#define HOZVAL     1023
#define HFPD       159

MODULE_LICENSE("GPL");


unsigned int * gpf0con;
unsigned int *gpf0pendcon;
unsigned int * gpf1con;
unsigned int * gpf2con;
unsigned int * gpf3con;
unsigned  int  *gpf0pud;
unsigned  int  *gpf1pud;
unsigned  int  *gpf2pud;
unsigned  int  *gpf3pud;
unsigned  int  *gpf0drv;
unsigned  int  *gpf1drv;
unsigned  int  *gpf2drv;
unsigned  int  *gpf3drv;
unsigned  int  * gpd0con;
unsigned  int  * gpd0dat;
//CLK_SRC_LCD0
unsigned  int  * clk_src_lcd0;
//CLK_DIV_LCD
unsigned  int  *  clk_div_lcd;
//LCDBLK_CFG
unsigned  int  *lcdblk_cfg;
unsigned  int  *lcdblk_cfg2;

unsigned int * vidcon0;
unsigned int * vidcon1;
unsigned int * vidcon2;
unsigned int * vidtcon0;
unsigned int * vidtcon1;
unsigned int * vidtcon2;
unsigned int * wincon0;
unsigned int * win0map;
unsigned int * vidw00add0b0;
unsigned int * vidw00add1b0;
unsigned int * vidw00add2;
unsigned int *vidosd0a;
unsigned int *vidosd0b;
unsigned int *vidosd0c;
unsigned  int  * shadowcon;


struct fb_info  * fs_fb;
struct clk *bus_clk;
static unsigned int pseudo_palette[16];


 static inline unsigned int chan_to_field(unsigned int chan,struct fb_bitfield *bf)                                                                                                 
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
} 

static int fs4412_setcolreg(unsigned regno,
		                 unsigned red, unsigned green, unsigned blue,
		                         unsigned transp, struct fb_info *info)
{
	unsigned  int  val;
	if (regno < 16) 
	{
		unsigned  int  *pal = info->pseudo_palette;
		
		val  = chan_to_field(red,   &info->var.red);
		val |= chan_to_field(green, &info->var.green);
	    val |= chan_to_field(blue,  &info->var.blue);
        pal[regno] = val;
                                                                                                                                   
     }
     return  0;
}

static struct fb_ops   my_fops = {
	.owner= THIS_MODULE,
	.fb_setcolreg = fs4412_setcolreg,
	.fb_fillrect= cfb_fillrect,
	.fb_copyarea= cfb_copyarea,
	.fb_imageblit=cfb_imageblit,
};

int lcd_init(void)
{
	dma_addr_t map_dma;
	unsigned map_size = PAGE_ALIGN(1024*600*2);
	int ret;

	fs_fb = framebuffer_alloc(0,NULL);

	if(!fs_fb)
	{
		printk("failed to allocate framebuffer \n");
		return -ENOENT;
	}


	strcpy(fs_fb->fix.id  ,"lcd");
	fs_fb->fix.smem_len  =1024*600*16/8 ; 
	fs_fb->fix.type = FB_TYPE_PACKED_PIXELS;
	fs_fb->fix.visual = FB_VISUAL_TRUECOLOR;
	fs_fb->fix.line_length = 1024*16/8 ;
	
	
	
	fs_fb->var.xres  = 1024; 
	fs_fb->var.yres  = 600;
	fs_fb->var.xres_virtual = 1024; 
	fs_fb->var.yres_virtual =600;
	fs_fb->var.bits_per_pixel  = 16;
	                                
	fs_fb->var.red.offset  = 11;
	fs_fb->var.red.length  = 5;
	fs_fb->var.green.offset  = 5;
	fs_fb->var.green.length = 6;
	fs_fb->var.blue.offset  = 0;
	fs_fb->var.blue.length  = 5;
	
	fs_fb->var.activate = FB_ACTIVATE_NOW;
	
	fs_fb->fbops  = &my_fops;
	
	
	fs_fb->pseudo_palette=pseudo_palette;
	
	fs_fb->screen_size = 1024 * 600 * 16 / 8;

  

	printk("fs_fb screen_size   = %ld\n",fs_fb->screen_size);
	gpf0con = ioremap(0x11400180,8);
	gpf1con = ioremap(0x114001A0,4);
	gpf2con = ioremap(0x114001C0,4);
	gpf3con = ioremap(0x114001E0,4);
	gpd0con  = ioremap(0x114000A0,4);
	gpf0pud  = ioremap(0x11400188,4);
	gpf1pud  = ioremap(0x114001A8,4);
	gpf2pud  = ioremap(0x114001C8,4);
	gpf3pud  = ioremap(0x114001E8,4);
   gpf0drv  = ioremap(0x1140018c,4);
   gpf1drv  = ioremap(0x114001ac,4);
   gpf2drv  = ioremap(0x114001cc,4);
   gpf3drv  = ioremap(0x114001ec,4);
   shadowcon  = ioremap(0x11c00034,4);
    lcdblk_cfg  = ioremap(0x10010210,4);
    lcdblk_cfg2  = ioremap(0x10010214,4);
	*gpf0con = 0x22222222;
	*gpf1con = 0x22222222;
	*gpf2con = 0x22222222;
	*gpf3con = 0x00222222;
   *gpf0pud  =  0x0000ffff;
   *gpf1pud  =  0x0000ffff;
   *gpf2pud  =  0x0000ffff;
   *gpf3pud  =  0x0000ffff;

   *gpf0drv  =  0x0000ffff;
   *gpf1drv  =  0x0000ffff;
   *gpf2drv  =  0x0000ffff;
   *gpf3drv  =  0x0000ffff;
    clk_div_lcd  = ioremap(0x1003C534,4);
    clk_src_lcd0  =  ioremap(0x1003c234,4);
	*clk_div_lcd&= ~0xf;
       *clk_src_lcd0&= ~0xf;
       *clk_src_lcd0|= 6;
      *lcdblk_cfg|= 1 << 1;
     *lcdblk_cfg2|= 1;
	 printk("lcdblk_cfg2  \n");
	vidcon0 = ioremap(0x11C00000,4);
	vidcon1 = ioremap(0x11C00004,4);
	vidcon2 = ioremap(0x11C00008,4);
	
	vidtcon0 = ioremap(0x11C00010,4);
	vidtcon1 = ioremap(0x11C00014,4);
	vidtcon2 = ioremap(0x11C00018,4);
	printk("ioremap  vidosd0c  \n");
	wincon0 = ioremap(0x11C00020,4);
  // *gpd0dat  |=1;
	win0map = ioremap(0x11C00180,4);
	
	printk("ioremap  vidosd0c  \n");
	vidw00add0b0 = ioremap(0x11C000a0,4);
	vidw00add1b0 = ioremap(0x11C000d0,4);
	vidw00add2 = ioremap(0x11C00104,4);
	
	
	printk("ioremap  vidosd0c  \n");
	vidosd0a = ioremap(0x11C00040,4);
	vidosd0b = ioremap(0x11C00044,4);
	vidosd0c = ioremap(0x11C00048,4);
	
	
	//bus_clk = clk_get(NULL, "lcd");
	//clk_enable(bus_clk);	

	writel(0,wincon0);
	writel(0,vidosd0a);
	writel(0,vidosd0b);
	writel(0,vidosd0c);

   *gpd0con  &= ~(0xf <<4) ;
   *gpd0con  |=(0x1 <<4);
   gpd0dat  =  gpd0con  +1;
  // *gpd0dat  &=~1;
	//*vidcon0 &= ~((0b111 << 26) | (0b111 << 17) | (0xff << 6) | (1 << 2));
	*vidcon0 = (14 << 6) ; 
	*vidcon1 &= ~(1 << 7);
	*vidcon1 |= (1 << 6) | (1 << 5)|(1 <<9);
   
	*vidcon2 = (1 << 14);
	*vidtcon0 = (VBPD << 16) | (VFPD << 8) | (VSPW << 0);
	*vidtcon1 = (HBPD << 16) | (HFPD << 8) | (HSPW << 0);
	*vidtcon2 = (LINEVAL << 11) | (HOZVAL << 0);
	*wincon0 &= ~(0xf << 2);
	*wincon0 |= (5 << 2) | (1 << 16) | (0 << 9);
	*win0map = 0;

	

	fs_fb->screen_base = dma_alloc_writecombine(NULL, map_size,
		   &map_dma, GFP_KERNEL);	

	if(!fs_fb->screen_base)
	{
		printk("dma_alloc_writecombine fail \n");
		return -ENOMEM;
	}

	memset(fs_fb->screen_base, 0x0, map_size);
	fs_fb->fix.smem_start = map_dma;

	      

    *vidw00add0b0 = fs_fb->fix.smem_start;

	*vidw00add1b0 = fs_fb->fix.smem_start + fs_fb->fix.smem_len;
	
	*vidw00add2 = (0 << 13) | ((1024 * 16 / 8 )<< 0);
	
	
	
	*vidosd0a = (0 << 11) | (0 << 0);
	*vidosd0b = (1023 << 11) | (479 << 0);
	*vidosd0c = 1024 * 600;
   *shadowcon  |=1;


   *gpd0dat  |=(1<<1);
	*vidcon0 |= 3<<0;
	*wincon0 |= (1 << 0);


	ret = register_framebuffer(fs_fb);
	if(ret < 0)
	{
		printk("failed to register framebuffer \n");
		return ret;
	}
	printk("lcd_init  \n");
	return  0;
}
void  lcd_exit(void)
{

	unregister_framebuffer(fs_fb);
	
	dma_free_writecombine(NULL, fs_fb->fix.smem_len, fs_fb->screen_base,fs_fb->fix.smem_start);
	
	//iounmap
	iounmap(vidosd0c);
	iounmap(vidosd0b);
	iounmap(vidosd0a);
	iounmap(vidw00add2);
	iounmap(vidw00add1b0);
	iounmap(vidw00add0b0);
	iounmap(win0map);
	iounmap(wincon0);
	iounmap(vidtcon2);
	iounmap(vidtcon1);
	iounmap(vidtcon0);
	iounmap(vidcon2);
	iounmap(vidcon1);
	iounmap(vidcon0);
	iounmap(gpf3con);
	iounmap(gpf2con);
	iounmap(gpf1con);
	iounmap(gpf0con);

	clk_disable(bus_clk);
	
	framebuffer_release(fs_fb);
	printk("lcd_exit  \n");
}

module_init(lcd_init);
module_exit(lcd_exit);
