#include "ican_curl_panel.h"
#include "wifi_debug_binding.h"
#include "rehab_wifi_panel.h"
#include "../../shared/ican_curl_ui.h"
#include <board.h>
#include <stdio.h>
#ifdef BSP_USING_LVGL
#include <lvgl.h>

LV_FONT_DECLARE(rehab_wifi_font);
LV_FONT_DECLARE(rehab_cjk_common_font);
#define UI ((volatile ican_curl_ui_t *)(uintptr_t)WIFI_DEBUG_CURL_UI)
#define BG 0x0B1220U
#define SURFACE 0x152137U
#define MUTED 0x99ABC3U
#define TEXT 0xEDF4FCU
#define ACCENT 0x38CDBBU

static const unsigned action_commands[]={ICAN_UI_START_CURL,ICAN_UI_START_LIFT,
    ICAN_UI_START_ROTATE,ICAN_UI_START_SMALL,ICAN_UI_START_MEDIUM,
    ICAN_UI_START_PAUSE,ICAN_UI_RECORD,ICAN_UI_PLAY,ICAN_UI_RESIST};
static const char *const action_names[]={"肘部屈伸","抬臂往返","前臂内旋",
    "小幅屈伸","中幅屈伸","屈肘停留","三轴示教录制","三轴轨迹回放","肘部阻力训练"};
static const char *const action_captions[]={"约90度 / 4、6号保持","约90度 / 5、6号保持",
    "约85度 / 4、5号保持","约30度 / 轻量往返","约60度 / 中幅往返",
    "约60度 / 停留2秒","手动引导 / 同步采集","相对轨迹 / 同步复现",
    "手动屈伸 / 4、6号保持 / 最大1A"};
static const char *const action_details[]={
    "仅5号往返，4、6号保持本轮起点",
    "仅4号往返，5、6号保持本轮起点",
    "仅6号往返，4、5号保持本轮起点",
    "屈肘约30度后返回，其他两轴保持",
    "屈肘约60度后返回，其他两轴保持",
    "屈肘约60度，停留2秒后返回",
    "停止保存轨迹，新录制会覆盖旧轨迹",
    "从当前位置复现相对位移，执行一次",
    "仅5号提供反向阻力，4、6号保持起点"};
#define ACTION_COUNT (sizeof(action_commands)/sizeof(action_commands[0]))

static lv_obj_t *pages[3],*tabs[3],*cards[ACTION_COUNT];
static lv_obj_t *status_label,*count_label,*start_button,*start_text,*stop_text;
static lv_obj_t *name_label,*detail_label,*connection_label,*page_label,*progress_bar,*guide_label;
static unsigned selected_action,viewed_page,page_selected[3];
static lv_timer_t *refresh_timer;
static uint32_t pending_seq,sent_at,notice_at;
static unsigned pending_command,notice_active;
static unsigned action_page(unsigned action) {return action<6U ? 0U:action<8U ? 1U:2U;}

static int compatible(void)
{
    const volatile uint32_t *d=(const volatile uint32_t *)(uintptr_t)WIFI_DEBUG_DESCRIPTOR;
    static const uint32_t expected[8]={0x57444231U,11U,WIFI_DEBUG_MAILBOX,64U,
        0xe7c19011U,0x326a9d41U,0x5f21ab88U,0x9d643207U};
    unsigned i;
    SCB_InvalidateDCache_by_Addr((void *)d,32);__DSB();
    for(i=0;i<8;i++) if(d[i]!=expected[i]) return 0;
    SCB_InvalidateDCache_by_Addr((void *)UI,64);__DSB();
    return UI->magic==ICAN_UI_MAGIC;
}
static void clean_request(void)
{
    SCB_CleanDCache_by_Addr((void *)UI,32);__DSB();
}
static void status(const char *text,unsigned color)
{
    lv_label_set_text(status_label,text);
    lv_obj_set_style_text_color(status_label,lv_color_hex(color),0);
}
static void disabled(lv_obj_t *obj,unsigned value)
{
    if(value) lv_obj_add_state(obj,LV_STATE_DISABLED);
    else lv_obj_remove_state(obj,LV_STATE_DISABLED);
}
static void selection_display(void)
{
    unsigned i;
    for(i=0;i<ACTION_COUNT;i++) {
        lv_obj_set_style_bg_color(cards[i],lv_color_hex(i==selected_action ? 0x183D45U:SURFACE),0);
        lv_obj_set_style_border_color(cards[i],lv_color_hex(i==selected_action ? ACCENT:0x27364FU),0);
        lv_obj_set_style_border_width(cards[i],i==selected_action ? 2:1,0);
    }
    lv_label_set_text(name_label,action_names[selected_action]);
    lv_label_set_text(detail_label,action_details[selected_action]);
    lv_label_set_text(guide_label,page_selected[1]==6U ?
        "1  打开电机动力，托稳机构\n2  开始录制，缓慢手动演示\n3  放回结束姿态，停止并保存":
        "1  将机构摆回接近示教起点\n2  点击开始回放，同步复现三轴\n3  完成后停止，也可随时点停止");
}
static void show_page(unsigned page)
{
    unsigned i;
    if(page>2U) return;
    viewed_page=page;
    for(i=0;i<3U;i++) {
        if(i==page) lv_obj_remove_flag(pages[i],LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(pages[i],LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_color(tabs[i],lv_color_hex(i==page ? ACCENT:SURFACE),0);
        lv_obj_set_style_text_color(tabs[i],lv_color_hex(i==page ? BG:MUTED),0);
    }
    lv_label_set_text(page_label,page==2U ? "3 / 3":page ? "2 / 3":"1 / 3");
    /* 翻页只切换视图，不发送任何控制命令，也不更改正在执行的动作。 */
    if(!pending_seq && !(compatible() && UI->active)) {
        selected_action=page_selected[page];notice_active=0;
    }
    selection_display();
}
static void send_command(unsigned command)
{
    /* UI只发布命令，不等待CAN；STOP始终可以覆盖待处理的启动。 */
    if(!compatible()) {status("控制未连接",0xF5B85AU);return;}
    if(command!=ICAN_UI_STOP && (UI->active || pending_seq || UI->fault)) return;
    if(UI->seq==UINT32_MAX) {status("请重启控制板",0xF5B85AU);return;}
    UI->command=command;UI->heartbeat++;__DMB();
    pending_seq=UI->seq+1U;UI->seq=pending_seq;
    pending_command=command;sent_at=rt_tick_get();notice_active=0;
    clean_request();disabled(start_button,1);
    status(command==ICAN_UI_STOP ? "正在停止":"正在准备",0xF5B85AU);
}
static void start_event(lv_event_t *e)
{
    (void)e;
    if(selected_action<ACTION_COUNT) send_command(action_commands[selected_action]);
}
static void stop_event(lv_event_t *e) {(void)e;send_command(ICAN_UI_STOP);}
static void select_event(lv_event_t *e)
{
    unsigned action=(unsigned)(uintptr_t)lv_event_get_user_data(e);
    if(action>=ACTION_COUNT || pending_seq || (compatible() && UI->active)) return;
    selected_action=action;page_selected[action_page(action)]=action;notice_active=0;
    selection_display();
}
static void tab_event(lv_event_t *e)
{
    show_page((unsigned)(uintptr_t)lv_event_get_user_data(e));
}
static void flip_event(lv_event_t *e)
{
    int step=(int)(intptr_t)lv_event_get_user_data(e);
    show_page((unsigned)((int)viewed_page+step+3)%3U);
}

static void refresh(lv_timer_t *timer)
{
    char count[80];
    unsigned active,fault,frames,i;
    (void)timer;
    if(!compatible()) {
        lv_label_set_text(connection_label,"控制离线");
        lv_obj_set_style_text_color(connection_label,lv_color_hex(0xF5B85AU),0);
        status("等待控制连接",0xF5B85AU);disabled(start_button,1);
        for(i=0;i<ACTION_COUNT;i++) disabled(cards[i],1);
        return;
    }
    /* 先invalidate再更新心跳，M33复位时不会回写旧启动请求。翻页不删除此定时器。 */
    UI->heartbeat++;clean_request();
    active=UI->active;fault=UI->fault;frames=UI->phase>>16;
    lv_label_set_text(connection_label,fault ? "控制故障":"控制在线");
    lv_obj_set_style_text_color(connection_label,lv_color_hex(fault ? 0xFF8989U:ACCENT),0);
    for(i=0;i<ACTION_COUNT;i++) disabled(cards[i],active || pending_seq || fault);
    if(!active && !pending_seq && action_page(selected_action)!=viewed_page) {
        selected_action=page_selected[viewed_page];selection_display();
    }
    lv_label_set_text(start_text,active ? "运行中":selected_action==6U ? "开始录制":
        selected_action==7U ? "开始回放":selected_action==8U ? "启动训练":"启动动作");
    lv_label_set_text(stop_text,selected_action==6U ? "停止并保存":"停止");
    if(selected_action==8U) {
        snprintf(count,sizeof(count),"训练  %lu / 120 秒  /  电流上限1A",(unsigned long)UI->cycles);
        lv_obj_remove_flag(progress_bar,LV_OBJ_FLAG_HIDDEN);
        lv_bar_set_value(progress_bar,UI->cycles>120U ? 512:(int)(UI->cycles*512U/120U),LV_ANIM_OFF);
    } else if(selected_action>=6U) {
        snprintf(count,sizeof(count),"轨迹  %lu / 512 帧%s",(unsigned long)frames,
            selected_action==6U ? "":"  /  回放一次");
        lv_obj_remove_flag(progress_bar,LV_OBJ_FLAG_HIDDEN);
        lv_bar_set_value(progress_bar,frames>512U ? 512:frames,LV_ANIM_OFF);
    } else {
        snprintf(count,sizeof(count),"已完成  %lu 次  /  启动后持续往返",(unsigned long)UI->cycles);
        lv_obj_add_flag(progress_bar,LV_OBJ_FLAG_HIDDEN);
    }
    lv_label_set_text(count_label,count);
    if(pending_seq && UI->done==pending_seq) {
        int result=(int32_t)UI->result;pending_seq=0;
        if(result) {
            status(result==ICAN_UI_NO_RECORD ? "暂无轨迹，请先录制":"启动失败，请检查控制",0xFF8989U);
            notice_at=rt_tick_get();notice_active=1;disabled(start_button,1);return;
        }
    }
    if(notice_active && (rt_tick_t)(rt_tick_get()-notice_at)<rt_tick_from_millisecond(2500)) return;
    notice_active=0;
    if(pending_seq) {
        if((rt_tick_t)(rt_tick_get()-sent_at)>rt_tick_from_millisecond(2000)) {
            if(pending_command!=ICAN_UI_STOP) send_command(ICAN_UI_STOP);
            status("控制超时，请检查",0xFF8989U);
        }
        return;
    }
    if(fault) {
        status("故障已停止，请检查后重启",0xFF8989U);disabled(start_button,1);
    } else if(active) {
        status(selected_action==6U ? "正在录制，缓慢移动":
            selected_action==7U ? "三轴同步回放中":selected_action==8U ?
            (UI->state==10U ? "手动屈伸，反向阻力训练":"正在准备，请保持静止"):"动作循环中",ACCENT);
        disabled(start_button,1);
    } else {
        status(selected_action==6U && frames>=2U ? "轨迹已保存":
            selected_action==7U && frames<2U ? "暂无轨迹，请先录制":"就绪，点击启动",TEXT);
        disabled(start_button,selected_action==7U && frames<2U);
    }
}
static lv_obj_t *panel(lv_obj_t *parent,int x,int y,int w,int h,unsigned color)
{
    lv_obj_t *obj=lv_obj_create(parent);
    lv_obj_set_pos(obj,x,y);lv_obj_set_size(obj,w,h);
    lv_obj_remove_flag(obj,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(obj,0,0);lv_obj_set_style_border_width(obj,0,0);
    lv_obj_set_style_radius(obj,16,0);lv_obj_set_style_bg_color(obj,lv_color_hex(color),0);
    return obj;
}
static lv_obj_t *label(lv_obj_t *parent,const char *text,int x,int y,int width,
                       unsigned color,const lv_font_t *font)
{
    lv_obj_t *obj=lv_label_create(parent);
    lv_obj_set_pos(obj,x,y);lv_obj_set_width(obj,width);
    lv_label_set_long_mode(obj,LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(obj,font,0);lv_obj_set_style_text_color(obj,lv_color_hex(color),0);
    lv_label_set_text(obj,text);return obj;
}
static lv_obj_t *button(lv_obj_t *parent,const char *text,int x,int y,int w,int h,
                        unsigned color,lv_event_cb_t cb,void *data,const lv_font_t *font)
{
    lv_obj_t *obj=lv_button_create(parent),*caption;
    lv_obj_set_pos(obj,x,y);lv_obj_set_size(obj,w,h);
    lv_obj_set_style_pad_all(obj,0,0);lv_obj_set_style_border_width(obj,0,0);
    lv_obj_set_style_shadow_width(obj,0,0);lv_obj_set_style_radius(obj,14,0);
    lv_obj_set_style_bg_color(obj,lv_color_hex(color),0);
    lv_obj_set_style_bg_color(obj,lv_color_hex(0x2F5264U),LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(obj,LV_OPA_40,LV_STATE_DISABLED);
    lv_obj_set_style_text_color(obj,lv_color_hex(TEXT),0);
    caption=lv_label_create(obj);lv_obj_set_style_text_font(caption,font,0);
    lv_label_set_text(caption,text);lv_obj_center(caption);
    lv_obj_add_event_cb(obj,cb,LV_EVENT_CLICKED,data);return obj;
}
static void settings_event(lv_event_t *e)
{
    (void)e;
    /* 只有离开整个运动界面才STOP并停止心跳，页内切换不会中断动作。 */
    send_command(ICAN_UI_STOP);
    if(refresh_timer) {lv_timer_delete(refresh_timer);refresh_timer=NULL;}
    rehab_wifi_panel_create();
}
static void deleted(lv_event_t *e)
{
    (void)e;
    if(refresh_timer) {lv_timer_delete(refresh_timer);refresh_timer=NULL;}
    status_label=NULL;start_button=NULL;
}
void ican_curl_panel_create(void)
{
    lv_obj_t *screen=lv_screen_active(),*root,*box,*obj;
    unsigned i;
    rehab_wifi_panel_suspend();
    if(refresh_timer) {lv_timer_delete(refresh_timer);refresh_timer=NULL;}
    lv_obj_clean(screen);
    root=panel(screen,0,0,480,800,BG);lv_obj_set_style_radius(root,0,0);
    lv_obj_add_event_cb(root,deleted,LV_EVENT_DELETE,NULL);
    label(root,"REHAB ARM  /  DEMO",24,18,290,ACCENT,&lv_font_montserrat_14);
    label(root,"康复机械臂",24,43,320,TEXT,&rehab_wifi_font);
    connection_label=label(root,"控制连接中",345,23,118,MUTED,&rehab_cjk_common_font);
    label(root,"三轴协同  /  动作、示教与阻力",24,91,420,MUTED,&rehab_cjk_common_font);
    tabs[0]=button(root,"固定动作",24,126,136,52,ACCENT,tab_event,(void *)0,&rehab_cjk_common_font);
    tabs[1]=button(root,"录制回放",172,126,136,52,SURFACE,tab_event,(void *)1,&rehab_cjk_common_font);
    tabs[2]=button(root,"阻力训练",320,126,136,52,SURFACE,tab_event,(void *)2,&rehab_cjk_common_font);
    pages[0]=panel(root,24,194,432,222,BG);
    pages[1]=panel(root,24,194,432,222,BG);
    pages[2]=panel(root,24,194,432,222,BG);
    for(i=0;i<ACTION_COUNT;i++) {
        unsigned page=action_page(i);
        int x=page==2U ? 0:page ? (int)(i-6U)*222:(int)(i%2U)*222;
        int y=page ? 0:(int)(i/2U)*78;
        int width=page==2U ? 432:210;
        obj=button(pages[page],"",x,y,width,page ? 80:66,SURFACE,
                   select_event,(void *)(uintptr_t)i,&rehab_cjk_common_font);
        cards[i]=obj;
        label(obj,action_names[i],14,page ? 15:9,width-26,TEXT,&rehab_cjk_common_font);
        label(obj,action_captions[i],14,page ? 45:37,width-26,MUTED,&rehab_cjk_common_font);
    }
    box=panel(pages[1],0,94,432,128,SURFACE);
    guide_label=label(box,"",16,12,402,MUTED,&rehab_cjk_common_font);
    lv_obj_set_style_text_line_space(guide_label,7,0);
    label(box,"仅存一条轨迹  /  重启后清空",16,104,402,0xF5B85AU,&rehab_cjk_common_font);
    box=panel(pages[2],0,94,432,128,SURFACE);
    obj=label(box,"1  托稳前臂，启动后先保持静止\n2  准备完成后，缓慢手动屈伸\n3  静止无主动电流，点停止结束",16,12,402,MUTED,&rehab_cjk_common_font);
    lv_obj_set_style_text_line_space(obj,7,0);
    label(box,"柔和阻力  /  单次最多120秒",16,104,402,0xF5B85AU,&rehab_cjk_common_font);
    box=panel(root,24,434,432,126,SURFACE);
    name_label=label(box,"",16,12,400,MUTED,&rehab_cjk_common_font);
    status_label=label(box,"控制连接中",16,39,400,TEXT,&rehab_wifi_font);
    detail_label=label(box,"",16,88,400,MUTED,&rehab_cjk_common_font);
    count_label=label(root,"",24,578,432,TEXT,&rehab_cjk_common_font);
    progress_bar=lv_bar_create(root);lv_obj_set_pos(progress_bar,24,608);lv_obj_set_size(progress_bar,432,5);
    lv_bar_set_range(progress_bar,0,512);lv_obj_set_style_bg_color(progress_bar,lv_color_hex(SURFACE),0);
    lv_obj_set_style_bg_color(progress_bar,lv_color_hex(ACCENT),LV_PART_INDICATOR);
    start_button=button(root,"启动动作",24,630,210,82,0x087F70U,start_event,NULL,&rehab_wifi_font);
    start_text=lv_obj_get_child(start_button,0);
    obj=button(root,"停止",246,630,210,82,0xAF3548U,stop_event,NULL,&rehab_wifi_font);
    stop_text=lv_obj_get_child(obj,0);
    label(root,"停止后释放保持，请保留机械支撑",24,718,432,MUTED,&rehab_cjk_common_font);
    button(root,"上一页",24,748,90,40,SURFACE,flip_event,(void *)(intptr_t)-1,&rehab_cjk_common_font);
    page_label=label(root,"1 / 3",126,758,60,MUTED,&lv_font_montserrat_14);
    button(root,"下一页",192,748,90,40,SURFACE,flip_event,(void *)(intptr_t)1,&rehab_cjk_common_font);
    button(root,"设置",344,748,112,40,SURFACE,settings_event,NULL,&rehab_cjk_common_font);
    selected_action=0;viewed_page=0;page_selected[0]=0;page_selected[1]=6;page_selected[2]=8;
    pending_seq=0;notice_active=0;
    show_page(0);
    refresh_timer=lv_timer_create(refresh,200,NULL);refresh(NULL);
}
#else
void ican_curl_panel_create(void) {}
#endif
