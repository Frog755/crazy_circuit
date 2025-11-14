#include "key.h"
#include "zf_device_ips200.h"
#include "zf_common_font.h"
#include "vofa_function.h"

// 菜单结构体定义
typedef struct {
    char* name;          // 菜单项名称
    float* value;        // 对应的参数值
    float step;          // 调节步长
    float min;           // 最小值
    float max;           // 最大值
} MenuItem;

// 菜单页面结构体
typedef struct {
    char* title;         // 页面标题
    MenuItem* items;     // 菜单项数组
    uint8 item_count;    // 菜单项数量
} MenuPage;

// 全局变量
uint8 current_page = 0;   // 当前页面
uint8 current_item = 0;   // 当前选中项
uint8 is_editing = 0;     // 是否在编辑模式
uint8 show_menu = 0;      // 是否显示菜单

// 外部变量声明
extern float speed_kp;
extern float speed_ki;
extern float speed_kd;
extern float turn_kp;
extern float turn_ki;
extern float turn_kd;
extern float base_speed;
extern float circle_speed;
extern float brake_speed;
extern float angle_kp;
extern float angle_ki;
extern float angle_kd;
extern float roll_angle_Kp;
extern float roll_angle_Ki;
extern float roll_angle_Kd;
extern float speed_Kp;
extern float speed_Ki;
extern float speed_Kd;

// 速度控制页面
MenuItem speed_items[] = {
    {"Base Speed", &base_speed, 0.5, 0, 100},
    {"Circle Speed", &circle_speed, 0.5, 0, 100},
    {"Brake Speed", &brake_speed, 0.5, 0, 100},
};

// 速度PID页面
MenuItem speed_pid_items[] = {
    {"Speed Kp", &speed_kp, 0.1, 0, 50},
    {"Speed Ki", &speed_ki, 0.01, 0, 10},
    {"Speed Kd", &speed_kd, 0.1, 0, 50},
};

// 转向PID页面
MenuItem turn_pid_items[] = {
    {"Turn Kp", &turn_kp, 0.1, 0, 50},
    {"Turn Ki", &turn_ki, 0.01, 0, 10},
    {"Turn Kd", &turn_kd, 0.1, 0, 50},
};

// 菜单页面定义
MenuPage menu_pages[] = {
    {"Speed Control", speed_items, sizeof(speed_items)/sizeof(MenuItem)},
    {"Speed PID", speed_pid_items, sizeof(speed_pid_items)/sizeof(MenuItem)},
    {"Turn PID", turn_pid_items, sizeof(turn_pid_items)/sizeof(MenuItem)},
};

#define TOTAL_PAGES (sizeof(menu_pages)/sizeof(MenuPage))

// 按键定义
#define KEY_UP      1
#define KEY_DOWN    2
#define KEY_LEFT    3
#define KEY_RIGHT   4
#define KEY_CENTER  5

// 显示菜单
void display_menu(void) {
    if (!show_menu) return;
    
    // 清屏
    ips200_clear(RGB565_BLACK);
    
    // 显示标题
    ips200_showstr(0, 0, menu_pages[current_page].title);
    ips200_showstr(180, 0, "Page:");
    ips200_showuint16(220, 0, current_page + 1);
    ips200_showstr(240, 0, "/");
    ips200_showuint16(250, 0, TOTAL_PAGES);
    
    // 显示菜单项
    for(uint8 i = 0; i < menu_pages[current_page].item_count; i++) {
        MenuItem* item = &menu_pages[current_page].items[i];
        
        // 显示选中标记
        if(i == current_item) {
            ips200_showstr(0, (i+1)*20, ">");
        }
        
        // 显示菜单项名称和值
        ips200_showstr(10, (i+1)*20, item->name);
        ips200_showfloat(150, (i+1)*20, *item->value, 2, 2);
        
        // 编辑模式显示
        if(is_editing && i == current_item) {
            ips200_showstr(200, (i+1)*20, "<EDIT>");
        }
    }
    
    // 显示操作提示
    ips200_showstr(0, 180, "UP/DOWN: Select");
    ips200_showstr(0, 200, "LEFT/RIGHT: Adjust");
    ips200_showstr(0, 220, "CENTER: Edit/Save");
}

// 处理按键
void handle_key(uint8 key) {
    if (!show_menu) {
        if (key == KEY_CENTER) {
            show_menu = 1;
            display_menu();
        }
        return;
    }
    
    switch(key) {
        case KEY_UP:
            if (!is_editing) {
                if (current_item > 0) current_item--;
                else current_item = menu_pages[current_page].item_count - 1;
            }
            break;
            
        case KEY_DOWN:
            if (!is_editing) {
                current_item++;
                if (current_item >= menu_pages[current_page].item_count)
                    current_item = 0;
            }
            break;
            
        case KEY_LEFT:
            if (is_editing) {
                MenuItem* item = &menu_pages[current_page].items[current_item];
                *item->value -= item->step;
                if (*item->value < item->min) *item->value = item->min;
            } else {
                if (current_page > 0) current_page--;
                else current_page = TOTAL_PAGES - 1;
                current_item = 0;
            }
            break;
            
        case KEY_RIGHT:
            if (is_editing) {
                MenuItem* item = &menu_pages[current_page].items[current_item];
                *item->value += item->step;
                if (*item->value > item->max) *item->value = item->max;
            } else {
                current_page++;
                if (current_page >= TOTAL_PAGES) current_page = 0;
                current_item = 0;
            }
            break;
            
        case KEY_CENTER:
            is_editing = !is_editing;
            if (!is_editing) {
                // 退出编辑模式时保存参数
                save_parameters();
            }
            break;
    }
    
    display_menu();
}

// 保存参数
void save_parameters(void) {
    // 这里添加参数保存到Flash的代码
    // 可以调用vofa发送参数到上位机
    vofa_send_parameter();
}

// 初始化菜单
void menu_init(void) {
    show_menu = 0;
    current_page = 0;
    current_item = 0;
    is_editing = 0;
}

// 按键中断处理函数
void key_handler(void) {
    // 根据实际硬件读取按键状态
    if(gpio_get(KEY_UP_PIN) == 0) handle_key(KEY_UP);
    if(gpio_get(KEY_DOWN_PIN) == 0) handle_key(KEY_DOWN);
    if(gpio_get(KEY_LEFT_PIN) == 0) handle_key(KEY_LEFT);
    if(gpio_get(KEY_RIGHT_PIN) == 0) handle_key(KEY_RIGHT);
    if(gpio_get(KEY_CENTER_PIN) == 0) handle_key(KEY_CENTER);
} 
 