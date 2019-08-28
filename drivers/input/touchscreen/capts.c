/*
 * capts.c
 *
 *  Created on: Mar 14, 2017
 *      Author: turke
 */


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/input/capts.h>
#include <linux/string.h>
#include <linux/firmware.h>
#include <asm/uaccess.h>
#include <linux/of_gpio.h>







#define DRV_VERSION "1.8"
#define DRV_NAME	"Capastive TS driver"
#define FIRM_ID_1792_1024_1		0x01
#define FIRM_ID_1024_600_1		0x02
#define FIRM_ID_1024_600_2		0x03
#define FIRM_ID_800_480_1		0x0A
#define RES_800_480		1
#define RES_1792_1024 	2
#define RES_1024_600 	3
#define MULTITOUCH 1
#define I2C_RETRY_CNT 5 //Fixed value
#define PRESS_KEY 1 //Fixed value
#define RELEASE_KEY 0 //Fixed value
#define TS_READ_LEN_ADDR 0x0F //Fixed value
#define TS_READ_START_ADDR 0x10 //Fixed value
#define TS_READ_REGS_LEN 66 //Fixed value
#define TS_WRITE_REGS_LEN 16 //Fixed value
#ifdef MULTITOUCH
#define TS_MAX_TOUCH 5//Model Dependent
#else
#define TS_MAX_TOUCH 1//Model Dependent
#endif
#define TS_READ_HW_VER_ADDR 0xF1 //Model Dependent
#define TS_READ_SW_VER_ADDR 0xF5 //Model Dependent
#define MELFAS_HW_REVISON 0x01 //Model Dependent
#define MELFAS_FW_VERSION 0x02 //Model Dependent
#define FOCAL_FLAG_PUTDOWN	0
#define FOCAL_FLAG_PUTUP	1
#define FOCAL_FLAG_CONTACT	2
#ifdef MULTITOUCH
#define REPORT_MT(touch_number, x, y, width, strength) \
		do {     \
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, touch_number);\
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);             \
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);             \
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, strength);       \
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, width);			\
			input_report_key(ts->input_dev, BTN_TOUCH, strength ? 1 : 0);			\
			input_mt_sync(ts->input_dev); \
		} while (0)
#else
#define REPORT_MT(touch_number, x, y, width, strength) \
		do {     \
			input_event(ts->input_dev, EV_ABS, ABS_X, x);	\
			input_event(ts->input_dev, EV_ABS, ABS_Y, y);	\
			input_event(ts->input_dev, EV_KEY, BTN_TOUCH, strength ? 1 : 0);	\
			if(!strength)															\
			input_event(ts->input_dev, EV_KEY, BTN_TOUCH, strength ? 1 : 0);	\
			input_sync(ts->input_dev);	\
		} while (0)
#endif
static int focaltech_resolution;
static int reset_gpio;
#define LCD_DIFF_EDT 10
static int lcd_diff = 0;
struct capts_data
{
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	//struct capts_platform_data *pdata;
	struct work_struct work;
	uint32_t flags;
	int max_x;
	int max_y;
	int max_width;
	int max_weight;
};
typedef int (*ch_func)(struct i2c_client *);
typedef irqreturn_t (*irq_handler_fnc)(int, void *);
static struct muti_touch_info g_Mtouch_info[TS_MAX_TOUCH];
//static int tsp_keycodes[4] = {KEY_MENU, KEY_HOME, KEY_SEARCH, KEY_BACK};
static irq_handler_fnc select_touchscreen(struct capts_data *ts);
typedef struct
{
	int flag;
	int id;
	int posx;
	int posy;
	int weight;
	int width;
}Tposf;
typedef struct
{
	unsigned char gestid;
	unsigned char numOfTps;
	Tposf ptch[TS_MAX_TOUCH];
}focalT;

int ldb_display_is(int lcd);
int lcd_display_num=-1;
int lcd_mode=-1;

//returns RGB LCD LCD struct number from mxc_lcdif.c, like 0, 1, 2...
int lcd_display_is(void);

//returns mxcfb0 LCD section mode, 0:RGB LCD, 1:LVDS, 2:HDMI
int get_lcd_mode(void);

int focaltech_i2c_read(struct i2c_client *client, unsigned char adr, int len, unsigned char *p) {
	int ret;
	if (i2c_master_send(client, &adr, 0x01) != 0x01) {

		return -1;
	}
	ret = i2c_master_recv(client, p, len);
	return ret;
}
int focaltech_i2c_read2(struct i2c_client *client, unsigned char adr, int len, unsigned char *p) {
	int ret;

	ret = focaltech_i2c_read(client, adr, len, p);
	if (ret == len) {
		return 0; // i2c success
	}


	return -1;
}
void read_focaltech_pct_ts_data(struct capts_data *ts){

	int  i,ret,touchState=0;
	unsigned char buf[1024];
	focalT tch;
	int pointCount = TS_MAX_TOUCH;
	if (ts == NULL) {

		return ;
	}
	tch.gestid = 0;
	tch.numOfTps = 0;
	for(i=0; i<TS_MAX_TOUCH; i++)
	{
		tch.ptch[i].flag = -1;
		tch.ptch[i].id = 0;
		tch.ptch[i].posx = 0;
		tch.ptch[i].posy = 0;
		tch.ptch[i].weight = 0;
		tch.ptch[i].width = 0;
	}

	ret= focaltech_i2c_read2(ts->client, 0x01, 2, &buf[0]);

	if (ret != 0){

		//printk( "%s read GESTID and point number failed.\n", __func__);
		return;
	}

	tch.gestid = buf[0]& 0xFF;
	tch.numOfTps = buf[1] & 0x0F;


	//printk( "GESTID: %d, TOUCHPOINT :%d\n", tch.gestid,tch.numOfTps );

	if(pointCount > 0 ){
		ret = focaltech_i2c_read2(ts->client, 0x03, pointCount*6, &buf[0]);
		if (ret != 0){
			//printk( "%s read touchdata failed.\n", __func__);
			return;
		}
	}

	for(i = 0;i <pointCount ;i++)
	{
		tch.ptch[i].posx = (( buf[0 + (6 * i)]) & 0x0F) << 8 | (( buf[1 + (6 * i)])& 0xFF);
		tch.ptch[i].posy = (( buf[2 + (6 * i)]) & 0x0F) << 8 | (( buf[3 + (6 * i)])& 0xFF);
		tch.ptch[i].flag = (( buf[0 + (6 * i)] & (0x03 << 6)) >> 6);
		tch.ptch[i].id = 	(( buf[2 + (6 * i)] & (0xF0 )) >> 4);
		tch.ptch[i].weight = 	(( buf[4 + (6 * i)] & (0xFF )));
		tch.ptch[i].width = 	(( buf[5 + (6 * i)] & (0xF0 )) >> 4);

	}


	for (i = 0; i < pointCount ;i++)
	{
		touchState = -1;
		if(tch.ptch[i].flag  == FOCAL_FLAG_CONTACT || tch.ptch[i].flag  == FOCAL_FLAG_PUTDOWN)
			touchState = 0xFF;
		else if (tch.ptch[i].flag  == FOCAL_FLAG_PUTUP)
			touchState = 0x00;
		else
			touchState = -1;

		switch(focaltech_resolution)
		{
		case RES_800_480:
#if 0
			if(ts->pdata->max_x == 640){
				g_Mtouch_info[i].posX  =  abs(tch.ptch[i].posx - 800);
				//	g_Mtouch_info[i].posX  =  (g_Mtouch_info[i].posX-20)+(g_Mtouch_info[i].posX)*15/120;
#ifndef MULTITOUCH				/* IF NOT MULTUTOUCH */
				g_Mtouch_info[i].posX = (g_Mtouch_info[i].posX) > 620 ? 620  : g_Mtouch_info[i].posX;
				g_Mtouch_info[i].posX = (g_Mtouch_info[i].posX - 20)<0 ? 0 : (g_Mtouch_info[i].posX - 20);
				g_Mtouch_info[i].posX = (g_Mtouch_info[i].posX *640)/600;
#endif
				g_Mtouch_info[i].posX = (g_Mtouch_info[i].posX) > 640 ? 640  : g_Mtouch_info[i].posX;
			}else{
				g_Mtouch_info[i].posX  =  abs(tch.ptch[i].posx - 800);
			}
#else
			g_Mtouch_info[i].posX  =  abs(tch.ptch[i].posx - 800);
#endif
#ifdef MULTITOUCH
			g_Mtouch_info[i].posX += g_Mtouch_info[i].posX / 15;
#endif
			g_Mtouch_info[i].posY  =  abs(tch.ptch[i].posy - 480);
#ifdef MULTITOUCH
			g_Mtouch_info[i].posY += ( g_Mtouch_info[i].posY - 240 ) / 10;
#endif
			g_Mtouch_info[i].width =  (int)( touchState / 20 );
			break;
		case RES_1792_1024:
			g_Mtouch_info[i].posX  =  tch.ptch[i].posx  ;
			g_Mtouch_info[i].posY  =  tch.ptch[i].posy ;
			g_Mtouch_info[i].width =  tch.ptch[i].width;
			break;
		case RES_1024_600:
			if((lcd_diff != 0)&&(lcd_mode==1)){
				if(tch.ptch[i].posy >= lcd_diff && tch.ptch[i].posy <=  ts->max_y + lcd_diff){
					g_Mtouch_info[i].posY  =  tch.ptch[i].posy - lcd_diff;
					g_Mtouch_info[i].posX  =  tch.ptch[i].posx;
					g_Mtouch_info[i].width =  tch.ptch[i].width;
				}
				else {
					g_Mtouch_info[i].posY = tch.ptch[i].posy < lcd_diff ? 0 : tch.ptch[i].posy > ( ts->max_y + lcd_diff ) ? ( ts->max_y + lcd_diff ) : tch.ptch[i].posy;
					if(g_Mtouch_info[i].strength == -1 )
						touchState = -1;
					else
						touchState = 0;
					g_Mtouch_info[i].posX  =  tch.ptch[i].posx;
					g_Mtouch_info[i].width =  tch.ptch[i].width;
				}
			}

			if ((lcd_display_num==1)&&(lcd_mode==0)){
				g_Mtouch_info[i].posY  =  tch.ptch[i].posy;
				g_Mtouch_info[i].posX  =  tch.ptch[i].posx >= 810 ? 809 : tch.ptch[i].posx;
				g_Mtouch_info[i].posX  =  (g_Mtouch_info[i].posX * 810) / 640;
				g_Mtouch_info[i].width =  tch.ptch[i].width;
			}
			else {
				g_Mtouch_info[i].posY  =  tch.ptch[i].posy;
				g_Mtouch_info[i].posX  =  tch.ptch[i].posx;
				g_Mtouch_info[i].width =  tch.ptch[i].width;
			}
			break;
		default:
			break;
		}
		g_Mtouch_info[i].id  =  tch.ptch[i].id ;
		if(touchState == 0x00 || touchState == 0xFF){
			g_Mtouch_info[i].width    =  tch.ptch[i].width;
			g_Mtouch_info[i].strength =  touchState;
		}else{
			g_Mtouch_info[i].width =  0;
			if(g_Mtouch_info[i].strength != -1)
				g_Mtouch_info[i].strength = 0;
		}
	}
	for (i = 0; i < TS_MAX_TOUCH ; i++)
	{

		if (g_Mtouch_info[i].id >= TS_MAX_TOUCH || g_Mtouch_info[i].strength == -1)
			continue;
		if(g_Mtouch_info[i].posX <= ts->max_x  && g_Mtouch_info[i].posY <= ts->max_y-lcd_diff )
			REPORT_MT(g_Mtouch_info[i].id, g_Mtouch_info[i].posX,g_Mtouch_info[i].posY, g_Mtouch_info[i].width, g_Mtouch_info[i].strength);
		if (g_Mtouch_info[i].strength == 0){
			g_Mtouch_info[i].strength = -1;
#ifdef MULTITOUCH
			input_event(ts->input_dev, ABS_PRESSURE, BTN_TOUCH,  0 );
			input_event(ts->input_dev, EV_KEY, BTN_TOUCH,  0 );
			input_mt_sync(ts->input_dev);
			input_sync(ts->input_dev);
#else
			input_event(ts->input_dev, EV_ABS, ABS_X, 0);
			input_event(ts->input_dev, EV_ABS, ABS_Y, 0);
			input_event(ts->input_dev, EV_KEY, BTN_TOUCH,  1 );
			input_sync(ts->input_dev);
			input_event(ts->input_dev, EV_KEY, BTN_TOUCH,  0 );
			input_sync(ts->input_dev);
#endif
		}
	}
	input_sync(ts->input_dev);
	return ;
}
void read_pct_ts_data(struct capts_data *ts){
	int ret=-1, len=26,i,j,touchState = 0;
	char  bcc=0;
	unsigned char buf[128];
	char poll_request = 0xF9;
	if (ts == NULL) {

		return ;
	}
	for (i = 0; i < I2C_RETRY_CNT; i++){
		if((ret = i2c_master_send(ts->client, &poll_request, 1)) >= 0 ){

			bcc=0;
			ret = i2c_master_recv(ts->client, buf, len);
			if (ret < 0){

				continue;
			}
			for (j=0;j<len;j++)
				bcc ^= buf[j];
			if (bcc != 0){

				ret = -1;
				continue;
			}
			if(buf[0] != 0xAA || buf[1] != 0xAA  || (buf[2] & 0x3F )!= 26) {

				ret = -1;
				continue;
			}
			break; // i2c success
		}
	}
	if (ret < 0)
	{

		return;
	}
	for (i = 0; i < TS_MAX_TOUCH ;i++)
	{
#ifdef MULTITOUCH
		touchState = (( buf[3] == (0) ) ) ? 0 : 0xFF ;
#else
		touchState = (( buf[3] & (0x01 << i) ) ) ? 0xFF : 0 ;
#endif
		g_Mtouch_info[i].posX = (unsigned int)(buf[i * 4 + 5 ] & 0x0F) << 8 | buf[i * 4 + 6];
		g_Mtouch_info[i].posY = (unsigned int)(buf[i * 4 + 7 ] & 0x0F) << 8 | buf[i * 4 + 8];
		g_Mtouch_info[i].posX  =  ((int)((int)g_Mtouch_info[i].posX  * (int)800 / (int)(1792)));
		g_Mtouch_info[i].posY  =  ((int)((int)g_Mtouch_info[i].posY  * (int)480 / (int)(1024)));
		g_Mtouch_info[i].width =  (int)( touchState / 20 );

		if(g_Mtouch_info[i].posX < 800 && g_Mtouch_info[i].posY < 480 )
			g_Mtouch_info[i].strength = touchState;
		//else
		//g_Mtouch_info[i].strength = 0;
	}
	for (i = 0; i < TS_MAX_TOUCH ; i++)
	{

		if(g_Mtouch_info[i].posY <= 5 || g_Mtouch_info[i].posY >= 475 )
			continue;

		if(g_Mtouch_info[i].posY <= 15){
			g_Mtouch_info[i].posY = g_Mtouch_info[i].posY-5;
		} else if(g_Mtouch_info[i].posY >= 460){
			g_Mtouch_info[i].posY = g_Mtouch_info[i].posY+5;
		}

		if (g_Mtouch_info[i].strength == -1)
			continue;
		if(g_Mtouch_info[i].posX < 800 && g_Mtouch_info[i].posY < 480 )
			REPORT_MT(i, g_Mtouch_info[i].posX,g_Mtouch_info[i].posY, g_Mtouch_info[i].width, g_Mtouch_info[i].strength);
		if (g_Mtouch_info[i].strength == 0){
			g_Mtouch_info[i].strength = -1;
#ifdef MULTITOUCH
			input_event(ts->input_dev, ABS_PRESSURE, BTN_TOUCH,  0 );
			input_event(ts->input_dev, EV_KEY, BTN_TOUCH,  0 );
			input_mt_sync(ts->input_dev);
			input_sync(ts->input_dev);
#else
			input_event(ts->input_dev, EV_ABS, ABS_X, 0);
			input_event(ts->input_dev, EV_ABS, ABS_Y, 0);
			input_event(ts->input_dev, EV_KEY, BTN_TOUCH,  1 );
			input_sync(ts->input_dev);
			input_event(ts->input_dev, EV_KEY, BTN_TOUCH,  0 );
			input_sync(ts->input_dev);
#endif
		}
	}
	input_sync(ts->input_dev);
	return ;
}
static irqreturn_t quick_check_irq_handler(int irq, void *handle)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t capts_focaltech_irq_handler(int irq, void *handle)
{
	struct capts_data *ts = (struct capts_data *) handle;

	read_focaltech_pct_ts_data(ts);
	return IRQ_HANDLED;
}
static irqreturn_t capts_kingley_edt_irq_handler(int irq, void *handle)
{
	struct capts_data *ts = (struct capts_data *) handle;

	read_pct_ts_data(ts);
	return IRQ_HANDLED;
}

static int capts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct capts_data *ts;
	int ret = 0, i;
	irq_handler_fnc irq_handler;
	lcd_mode=get_lcd_mode();
	lcd_display_num=lcd_display_is();

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		pr_info("cap_ts_probe: need I2C_FUNC_I2C\n");
		return -ENODEV;

	}

	ts = kmalloc(sizeof(struct capts_data), GFP_KERNEL);
	if (ts == NULL)
	{
		return  -ENOMEM;

	}

	struct device_node *np = client->dev.of_node;
	//ts->client->irq= of_get_named_gpio(np, "gpio_ts_int", 0);
	/* setup reset gpio */
	reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio_is_valid(reset_gpio)) {
		ret = devm_gpio_request(&client->dev,reset_gpio, "pct-reset");
		if (ret) {
			dev_err(&client->dev,"failed to request gpio %d\n", reset_gpio);
			kfree(ts);
			return ret;
		}
		gpio_direction_output(reset_gpio, 1);
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);


	if (ts->client->irq)
	{
		ret = 1;
		irq_handler = select_touchscreen(ts);

		if(irq_handler == IRQ_NONE){
			kfree(ts);
			return  -ENXIO;
		}

		ret = request_threaded_irq(client->irq, quick_check_irq_handler,irq_handler,IRQF_TRIGGER_FALLING, ts->client->name, ts);
		if (ret > 0)
		{
			kfree(ts);
			return  -ENODEV;

		}
	}else{

		kfree(ts);
		return  -ENODEV;
	}
	for (i = 0; i < TS_MAX_TOUCH; i++) /* _SUPPORT_MULTITOUCH_ */
		g_Mtouch_info[i].strength = -1;


	ts->input_dev = input_allocate_device();
	if (!ts->input_dev)
	{
		pr_info("[TSP] %s: Not enough memory\n", __FUNCTION__);
		if(ts->client->irq)
			free_irq(client->irq, ts);
		kfree(ts);
		return  -ENOMEM;
	}
	ts->input_dev->name = ts->client->name;
	ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) ;
	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);
	__set_bit(ABS_X, ts->input_dev->absbit);
	__set_bit(ABS_Y, ts->input_dev->absbit);
	__set_bit(ABS_PRESSURE, ts->input_dev->absbit);
	__set_bit(BTN_TOUCH, ts->input_dev->keybit);
	ts->input_dev->dev.parent	= &client->dev;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor  = 0x0001;
	ts->input_dev->id.product = 0x0001;
	ts->input_dev->id.version = 0x0100;

	//input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 800, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);
#ifdef MULTITOUCH
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, TS_MAX_TOUCH - 1, 0, 0);
#endif
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, ts->max_width, 0, 0);
	pr_info("capts: %d,%d res registered \n",ts->max_x,ts->max_y);
	ret = input_register_device(ts->input_dev);
	if (ret)
	{
		pr_info("[TSP] %s: Failed to register device\n", __FUNCTION__);
		ret = -ENOMEM;
		input_free_device(ts->input_dev);
		if(ts->client->irq)
			free_irq(client->irq, ts);
		kfree(ts);
		return  -ENODEV;

	}


	return 0;

}
static int capts_remove(struct i2c_client *client)
{
	struct capts_data *ts = i2c_get_clientdata(client);
	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}
static int check_kingley_edt(struct i2c_client *client)
{
	int ret=-1, len=26,i;
	unsigned char buf[1000];
	char poll_request = 0xF9;
	if (client == NULL) {
		return -1;
	}
	for (i = 0; i < I2C_RETRY_CNT; i++){
		if((ret = i2c_master_send(client, &poll_request, 1)) >= 0 ){
			ret = i2c_master_recv(client, &buf[0], len);
			if (ret < 0){
				continue;
			}
			if(buf[0] != 0xAA || buf[1] != 0xAA ) {
				return -1;
			}
			else
				return 0;
		}
	}
	if (ret < 0)
	{
		return -1;
	}
	if(i < I2C_RETRY_CNT)
		return 0;
	return -1;
}
static int check_focal_tech_800_480(struct i2c_client *client)
{
	int ret=-1, i;
	unsigned char buf[12];
	if (client == NULL) {
		return -1;
	}
	for (i = 0; i < I2C_RETRY_CNT; i++){
		ret = focaltech_i2c_read2(client, 0xA6, 1, &buf[0]);
		if(ret)
			continue;
		if(buf[0] == FIRM_ID_800_480_1)
		{
			return 0;
		}
		else
		{
			return -1;
		}
	}
	if (ret < 0)
	{
		return -1;
	}
	return -1;
}
static int check_focal_tech_1792_1024(struct i2c_client *client)
{
	int ret=-1, i;
	unsigned char buf[12];
	if (client == NULL) {
		return -1;
	}
	for (i = 0; i < I2C_RETRY_CNT; i++){
		ret = focaltech_i2c_read2(client, 0xA6, 1, &buf[0]);
		if(ret)
			continue;
		if(buf[0] == FIRM_ID_1792_1024_1)
			return 0;
		else if(buf[0] == FIRM_ID_1024_600_1 || buf[0] == FIRM_ID_1024_600_2)
			return 1;
		else
			return -1;
	}
	if (ret < 0)
	{
		return -1;
	}
	return -1;
}

/*
 */
static irq_handler_fnc select_touchscreen(struct capts_data *ts )
{
	int ret = 1;
	struct i2c_client *client = ts->client;
	ts->max_weight = 255;
	ts->max_width = 16;
	ret = check_focal_tech_1792_1024(client);
	if(ret == 0)
	{
		focaltech_resolution = RES_1792_1024;
		ts->max_x = 1792;
		ts->max_y = 1024;
		return capts_focaltech_irq_handler;
	}
	if(ret == 1)
	{

		focaltech_resolution = RES_1024_600;
		ts->max_x = 1024;
		if(ldb_display_is(0)){
			lcd_diff = LCD_DIFF_EDT;

		}else
			lcd_diff = 0;

		ts->max_y = 600 -2 * lcd_diff;
		return capts_focaltech_irq_handler;
	}

	ret = check_focal_tech_800_480(client);
	if(!ret)
	{
		focaltech_resolution = RES_800_480;
		ts->max_x = 800;
		ts->max_y = 480;
		return capts_focaltech_irq_handler;
	}

/*	focaltech_resolution = RES_1024_600;
	ts->max_x = 1024;
	if(ldb_display_is(0))
		lcd_diff = LCD_DIFF_EDT;
	else
		lcd_diff = 0;

	ts->max_y = 600-2*lcd_diff;*/

	return IRQ_NONE;
}
static const struct i2c_device_id capts_id[] =
{
		//	{ "melfas", (int)check_melfas },
		{ "kingley-edt", 0 },
		//	{ "focaltech", (int)check_focaltech },
		{ }
};
MODULE_DEVICE_TABLE(i2c, capts_id);
static const struct of_device_id capts_of_match[] = {
		{ .compatible = "kentkart,edt", },
		{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, capts_of_match);
static struct i2c_driver capts_driver ={
		.driver =
		{
				.name = "capts",
				.of_match_table = of_match_ptr(capts_of_match),
		},
		.id_table = capts_id,
		.probe = capts_probe,
		.remove = capts_remove,
};
module_i2c_driver(capts_driver);
MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_NAME);
MODULE_LICENSE("GPL");
