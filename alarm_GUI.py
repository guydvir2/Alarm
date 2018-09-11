#from gpiozero import Button, OutputDevice
#from gpiozero.pins.pigpio import PiGPIOFactory

from sys import platform, path
import os
import datetime
import tkinter as tk
from tkinter import filedialog
from tkinter import ttk
#from time import sleep
#from signal import pause


class GPIOMonitor:
    def __init__(self, ip=None, alias='gpio_monitor',
                 listen_pins=[20, 21], trigger_pins=[16, 26], 
                 log_filename='/home/guy/Documents/AlarmMonitor.log'):
                # listen_pins = [sys.arm, alarm.on], trigger_pins=[full, home]
        self.last_state=[None for i in range(4)]
        self.cbit=cbit.CBit(1000)
        
        if ip is not None:
            self.factory = PiGPIOFactory(host=ip)
        else:
            self.factory=None
            self.ip_pi = getip.get_ip()[0]
            
        self.alias = alias
        self.fullarm_hw = OutputDevice(trigger_pins[0], pin_factory=self.factory)
        self.homearm_hw = OutputDevice(trigger_pins[1], pin_factory=self.factory)
        self.sysarm_hw = Button(listen_pins[0], pin_factory=self.factory)
        self.alarm_hw = Button(listen_pins[1], pin_factory=self.factory)
        
        self.logger = Log2File(log_filename, name_of_master=self.alias, 
            time_in_log=1, screen=1)
        
        # self.run_gmail_service()
        self.check_state_on_boot(trigger_pins, listen_pins)
        
        self.monitor_state()

    def monitor_state(self):
        def const_check_state():
            tmp_status=self.fullarm_hw.value, self.homearm_hw.value, self.sysarm_hw.value, self.alarm_hw.value 
            if tmp_status != self.last_state:
                for i, current_gpio in enumerate(tmp_status):
                    if self.last_state[i] != current_gpio:
                        self.last_state[i] = current_gpio
                        self.notify('Change State -%s :%s'%(msgs[i], current_gpio))
        
        msgs=['Full-mode Arm','Home-mode','System Arm state','Alarm state']
        
        self.cbit.append_process(const_check_state)
        self.cbit.init_thread()

    def check_state_on_boot(self, trigger_pins, listen_pins):
        # check triggers at boot
        self.notify("%s start" % self.alias)
        self.notify("IP [%s]" % self.ip_pi)
        self.notify("trigger IOs [%d, %d]" % (trigger_pins[0], trigger_pins[1]))
        self.notify("Indications IOs [%d, %d]" % (listen_pins[0], listen_pins[1]))

        if any([self.homearm_hw.value, self.fullarm_hw.value]):
            al_stat = '@BOOT- System Armed'
        else:
            al_stat = '@Boot -System Unarmed'

        self.notify(al_stat)

    def run_gmail_service(self):
        path = main_path + 'modules/'
        self.gmail = gmail_mod.GmailSender(sender_file=path + 'ufile.txt',
                                           password_file=path + 'pfile.txt')

    def email_notify(self, msg, sbj=None):
        self.gmail.compose_mail(recipients=['guydvir.tech@gmail.com'],
                                body=msg, subject=sbj)

    def notify(self, msg):
        self.logger.append_log(msg)
        try:
            self.write2log(msg)
        except AttributeError:
            pass
        # self.email_notify(msg=self.logger.msg, sbj='HomePi: %s' % (self.alias))


class AlarmControlGUI(ttk.Frame):
    def __init__(self, master, ip=None):
        self.arm_value = tk.StringVar()
        self.alarm_on_value = tk.StringVar()
        self.arm_home_value = tk.IntVar()
        self.arm_full_value = tk.IntVar()
        self.tx_value = tk.StringVar()

        self.common_bg = 'grey35'
        self.disarm_pwd = '1234'
        master.title('HomePi Alarm monitor')
        self.blink_status = None
        self.last_ind_state = ['','']
        self.last_ping_state = 0
        self.log_stack, self.ip_pi = [], ip

        ttk.Frame.__init__(self)
        #GPIOMonitor.__init__(self, ip=self.ip_pi, alias='Alarm Monitor')

        # Frames
        self.mainframe = tk.Frame(master, padx=5, pady=5, bg=self.common_bg)
        self.mainframe.grid()
        self.activity_log_frame = tk.LabelFrame(self.mainframe, text='History', bg=self.common_bg, fg='white', padx=3,
                                                pady=3)
        self.activity_log_frame.grid(row=1, column=0, sticky=tk.W + tk.E, pady=(0, 5), columnspan=3)
        self.activity_log_frame.grid_columnconfigure(0, weight=1)

        self.controls_frame = tk.Frame(self.mainframe, bg=self.common_bg)
        self.controls_frame.grid(row=0, column=0, columnspan=3, sticky=tk.E + tk.W)
        self.controls_frame.grid_columnconfigure(1, weight=2)

        self.status_frame = tk.Frame(self.mainframe, bg=self.common_bg)
        self.status_frame.grid(row=2, column=0, columnspan=3, sticky=tk.E + tk.W)
        self.status_frame.grid_columnconfigure(2, weight=1)
        ###################################################

        self.run_modules()
        
    def test1(self):
        print("HI")
        self.after(100, self.test1)

    def run_modules(self):
        self.log_window()
        self.arm_buttons()
        self.create_indicators()
        self.oper_buttons()
        self.chk_ind_state()
        self.status_bar()
        self.clock1()
        #self.set_arm_ind(0)
        #self.alarm_setoff_ind(0)
        self.blink_tx()
        #self.test1()
        #self.boot_notifications()
        #self.run_alltime()

    def boot_notifications(self):
        self.write2log("Boot GUI")

    def arm_buttons(self):
        pdx, pdy = 2, 2
        self.arm_buts_frame = tk.Frame(self.controls_frame, bg=self.common_bg, relief=tk.GROOVE, bd=2)
        self.arm_buts_frame.grid(row=0, column=0)
        self.full_arm_button = tk.Checkbutton(self.arm_buts_frame, text="Full", width=9, variable=self.arm_full_value,
                                              indicatoron=0, height=1, command=self.farm_cb)
        self.full_arm_button.grid(row=0, column=0, padx=pdx, pady=pdy)

        self.home_arm_button = tk.Checkbutton(self.arm_buts_frame, text="Home", width=9, variable=self.arm_home_value,
                                              indicatoron=0, height=1, command=self.harm_cb)
        self.home_arm_button.grid(row=0, column=1, padx=pdx, pady=pdy)

    def oper_buttons(self):
        pdx, pdy = 2, 2
        self.buts_frame = tk.Frame(self.controls_frame, bg=self.common_bg, relief=tk.GROOVE, bd=2)
        self.buts_frame.grid(row=0, column=4, sticky=tk.W)

        self.exit_button = tk.Button(self.buts_frame, text="Exit", command=self.exit_button_cb, width=8)
        self.exit_button.grid(row=0, column=1, padx=pdx, pady=pdy)
        self.save_button = tk.Button(self.buts_frame, text="Save", command=self.save_button_cb, width=8)
        self.save_button.grid(row=0, column=0, padx=pdx, pady=pdy)

    def create_indicators(self):
        pdx, pdy = 1, 1
        self.ind_frame = tk.Frame(self.controls_frame, bg=self.common_bg, relief=tk.GROOVE, bd=2)
        self.ind_frame.grid(row=0, column=1)

        self.system_armed_label = tk.Label(self.ind_frame, textvariable=self.arm_value, relief=tk.RIDGE, bd=2,
                                           width=13)
        self.system_armed_label.grid(row=0, column=0, padx=pdx, pady=pdy)
        self.setoff_alarm_label = tk.Label(self.ind_frame, textvariable=self.alarm_on_value, relief=tk.RIDGE, bd=2,
                                           width=13)
        self.setoff_alarm_label.grid(row=0, column=2, padx=pdx, pady=pdy)

    def chk_ind_state(self):
        #gpio_s = self.get_gpio_status()
        gpio_s = [[1, 1], [1, 1]]
        # arm ind
        if gpio_s[1][1] == True:
            self.set_arm_ind(1)
            if self.last_ind_state[0] != 'arm_on':
                self.last_ind_state[0] ='arm_on'
                self.write2log('System is Armed')
        else:
            self.set_arm_ind(0)
            if self.last_ind_state[0] != 'arm_off':
                self.last_ind_state[0] = 'arm_off'
                self.write2log('System Disarmed')
        # alert ind
        if gpio_s[1][0] == True:
            self.alarm_setoff_ind(1)
            if self.last_ind_state[1] != 'alarm_on':
                self.last_ind_state[1] = 'alarm_on'
                self.write2log('System is Alarming')
        else:
            self.alarm_setoff_ind(0)
            if self.last_ind_state[1] != 'alarm_off':
                self.last_ind_state[1] = 'alarm_off'
                self.write2log('System stopped alarming')

        self.after(1000, self.chk_ind_state)

    def set_arm_ind(self, value):
        if value == 1:
            self.arm_value.set('Sys.Armed')
            self.system_armed_label['bg'] = 'red'
        elif value == 0:
            self.arm_value.set('Sys.NotArmed')
            self.system_armed_label['bg'] = 'orange'

    def alarm_setoff_ind(self, value):
        if value == 1:
            self.alarm_on_value.set('Alarm.On')
            self.setoff_alarm_label['bg'] = 'red'
        elif value == 0:
            self.alarm_on_value.set('Alarm.Off')
            self.setoff_alarm_label['bg'] = 'orange'

    def farm_cb(self):
        # already armed - need to disable
        if self.fullarm_hw.value == 1:
            self.disable_with_pwd(self.fullarm_hw, self.arm_full_value)
            self.write2log("Full mode switched off")
        
        # set full_arm to ON
        elif self.fullarm_hw.value == 0 and self.homearm_hw.value == 0:
            self.fullarm_hw.on()
            self.write2log("Full mode switched on")

        # home mode is armed- need to disable home and then arm full
        elif self.fullarm_hw.value == 0 and self.homearm_hw.value == 1:
            # disable home arm
            if self.disable_with_pwd(self.homearm_hw, self.arm_home_value) == 1:
                self.write2log("Home mode switched off")
                self.fullarm_hw.on()
                self.write2log("Full mode switched on")
            else:
                self.arm_full_value.set(0)
                self.write2log("PWD err disarming Home mode")

    def harm_cb(self):
        # already armed - need to disable
        if self.homearm_hw.value == 1:
            self.disable_with_pwd(self.homearm_hw, self.arm_home_value)
            self.write2log("Home mode switched off")

        # set home_arm to ON
        elif self.homearm_hw.value == 0 and self.fullarm_hw.value == 0:
            self.homearm_hw.on()
            self.write2log("Home mode switched on")

        # full mode is armed- need to disable home and then arm home
        elif self.homearm_hw.value == 0 and self.fullarm_hw.value == 1:
            # disable full arm
            if self.disable_with_pwd(self.fullarm_hw, self.arm_full_value) == 1:
                self.write2log("Full mode switched off")
                self.homearm_hw.on()
                self.write2log("Home mode switched on")

            else:
                self.write2log("PWD err for disarming Full mode")
                self.arm_home_value.set(0)
        
    def disable_with_pwd(self, hw, ind_hw):
        if hw.value == 1:
            self.passwd_window()
            if self.pwd_ent_value.get() == self.disarm_pwd:
                hw.off()
                ind_hw.set(0)
                return 1
            else:
                print("wrong pwd")
                ind_hw.set(1)
                return 0

    def log_window(self):
        # Create log window

        self.log_text = tk.Text(self.activity_log_frame, height=15, wrap=tk.NONE, bd=1, relief=tk.RIDGE)
        self.log_text.grid(row=0, column=0, sticky=tk.E + tk.W)
        scrollbar_y = ttk.Scrollbar(self.activity_log_frame, orient=tk.VERTICAL)
        scrollbar_y.grid(row=0, column=1, sticky=tk.N + tk.S)
        scrollbar_y.config(command=self.log_text.yview)
        scrollbar_x = ttk.Scrollbar(self.activity_log_frame, orient=tk.HORIZONTAL)
        scrollbar_x.grid(row=1, column=0, sticky=tk.E + tk.W + tk.S, columnspan=2)
        scrollbar_x.config(command=self.log_text.xview)

        self.log_text.config(yscrollcommand=scrollbar_y.set)
        self.log_text.config(xscrollcommand=scrollbar_x.set)
        self.log_text.config(state=tk.DISABLED)

    def write2log(self, text_in):
        try:
            self.log_text.config(state=tk.NORMAL)
            time2log = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-5]
            start = tk.END
            if self.log_stack != []:
                for msg in self.log_stack:
                    self.log_text.insert(start, msg)
                    start = tk.END
                self.log_stack = []
            self.log_text.insert(start, "[" + str(time2log) + "] " + text_in + "\n")
            self.log_text.config(state=tk.DISABLED)
        except AttributeError:
            # Stack log prior to boot of log
            time2log = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            self.log_stack.append('[' + str(time2log) + '] ' + text_in + '\n')

    def passwd_window(self):

        bg = self.common_bg  # 'yellow2'#'RoyalBlue4'
        self.pwd_toplevel = tk.Toplevel(bg=bg, pady=5, padx=5)
        self.pwd_toplevel.grid()
        self.pwd_toplevel.title('Enter Disarm Code')
        self.pwd_button_value = tk.IntVar()

        self.pwd_window_frame = tk.Frame(self.pwd_toplevel, bg=bg)
        self.pwd_window_frame.grid()
        tk.Label(self.pwd_window_frame, text='Enter Password to disarm:', pady=5, bg=bg, fg='white').grid(row=0,
                                                                                                          column=0,
                                                                                                          columnspan=2)
        self.pwd_ent_value = tk.StringVar()
        self.pwd_entry = tk.Entry(self.pwd_window_frame, textvariable=self.pwd_ent_value, show="*")
        self.pwd_entry.grid(row=1, column=0)
        self.pwd_ok_button = tk.Button(self.pwd_window_frame, text='Send', width=5,
                                       command=lambda: self.pwd_button_value.set(1))
        self.pwd_ok_button.grid(row=1, column=1, padx=(10, 0))

        self.pwd_ok_button.wait_variable(self.pwd_button_value)
        self.pwd_toplevel.destroy()

    def status_bar(self):
        tk.Label(self.status_frame, text='Console IP:%s ,Remote IP:%s ,Start Time:%s' % (
            getip.get_ip()[0], self.ip_pi, str(datetime.datetime.now())[:-5]), bg=self.common_bg, fg='white').grid(
            row=0, column=0)
        self.tx_label = tk.Label(self.status_frame, textvariable=self.tx_value, relief=tk.GROOVE, width=2)
        self.tx_label.grid(row=0, column=4, sticky=tk.E)

        #self.clock()

    def clock1(self):
        def update_time():
            time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            clock_label.config(text=time)
            print("YU")
            
        def me2():
            print("DFGDFGDFG")


        clock_label = tk.Label(self.status_frame, bg=self.common_bg, fg='white', relief=tk.GROOVE, bd=2, padx=3)
        clock_label.grid(row=0, column=2, sticky=tk.E, padx=2)
        time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        clock_label.config(text=time)
        
        self.after(500, self.clock1)

    def exit_button_cb(self):
        quit()

    def save_button_cb(self):
        filetypes = [('Log file', '*.log'), ]
        filename = tk.filedialog.asksaveasfilename(defaultextension='.log', filetypes=filetypes)
        # copies data from log window
        text = self.log_text.get('1.0', tk.END)
        with open(filename, 'w')as file:
            file.write(text)
        # tk.messagebox.showinfo('FYI', 'File Saved')

    def blink_tx(self):
        def blink_1(color1, color2, txt):
            self.tx_label["bg"] = color1
            self.tx_value.set(txt)
            self.after(t_blink, blink_2, color2)

        def blink_2(color):
            self.tx_label["bg"] = color
            self.after(t_blink, self.blink_tx)

        t_blink = 1500

        ping_result = os.system('ping -c 1 %s > /dev/null 2>&1 ' % self.ip_pi)
        if ping_result == 0:
            self.blink_status = 1
        else:
            self.blink_status = 0

        if self.blink_status == 1:
            blink_1('green', 'orange', 'Tx')
        elif self.blink_status == 0:
            blink_1('red', 'yellow', '*')
        else:
            blink_1('red', 'red', 'X')
            
        if self.last_ping_state == ping_result:
            self.last_ping_state = [1,0][ping_result]
            m=["OK","Fail"][ping_result]
            self.write2log("Ping %s"%m)
    
    #def run_alltime(self):
        #self.clock()
        


os_type = platform
if os_type == 'darwin':
    main_path = '/Users/guy/Documents/github/Rpi/'
elif os_type == 'win32':
    main_path = 'd:/users/guydvir/Documents/git/Rpi/'
elif os_type == 'linux':
    main_path = '/home/guy/Documents/github/Rpi/'

path.append(main_path + 'GPIO_Projects/lcd')
path.append(main_path + 'SmartHome/LocalSwitch')
path.append(main_path + 'modules')

import gmail_mod
from localswitches import Log2File
import getip
#import cbit

root = tk.Tk()
AlarmControlGUI(root , ip='192.168.2.117')
root.mainloop()

