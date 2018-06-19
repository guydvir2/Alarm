# from gpiozero import Button
# from gpiozero.pins.pigpio import PiGPIOFactory


from sys import platform, path
import os
import datetime
import tkinter as tk
from tkinter import filedialog
from tkinter import ttk
from time import sleep
from signal import pause


class GPIOMonitor:
    def __init__(self, ip=None, alias='gpio_monitor',
                 trigger_pins=[21, 20], listen_pins=[16, 26]):

        if ip is None:
            ip = 'localhost'
        self.factory = PiGPIOFactory(host=ip)

        self.trigger_pins, self.listen_pins, self.ip_pi = trigger_pins, listen_pins, ip
        self.trigger_vector, self.listen_vector = [], []

        self.alias = alias
        self.create_gpios()
        self.change_state_notifications()

        self.logger = Log2File('/home/guy/Documents/AlarmMonitor.log',
                               name_of_master=self.alias, time_in_log=1, screen=0)
        self.run_gmail_service()
        self.check_state_on_boot(trigger_pins, listen_pins)

    # def run_status(self):
    # self.cbit = CBit(500)
    # self.cbit.append_process(self.run_status)
    # self.cbit.init_thread()
    # print(self.get_gpio_status())

    def create_gpios(self):
        for pin in self.trigger_pins:
            self.trigger_vector.append(Button(pin, pin_factory=self.factory))
        for pin1 in self.listen_pins:
            self.listen_vector.append(Button(pin1, pin_factory=self.factory))

    def get_gpio_status(self):
        trig_s, listen_s = [], []
        for pin in self.trigger_vector:
            trig_s.append(pin.is_pressed)
        for pin1 in self.listen_vector:
            listen_s.append(pin1.is_pressed)

        return trig_s, listen_s

    def change_state_notifications(self):
        # Triggers        
        msgs = [['Full-mode alarm switched OFF', 'Full-mode alarm switched ON'],
                ['Home-mode alarm switched OFF', 'Home-mode alarm switched ON']]
        for i, trig in enumerate(self.trigger_vector):
            trig.when_released = lambda arg=i: self.notify(msgs[arg][0])
            trig.when_pressed = lambda arg=i: self.notify(msgs[arg][1])

        # Indications 
        msgss = [['Alarm Stopped', 'Alarm is ON'],
                 ['System Armed', 'System Disarmed']]
        for n, listen in enumerate(self.listen_vector):
            listen.when_released = lambda arg=n: self.notify(msgss[arg][1])
            listen.when_pressed = lambda arg=n: self.notify(msgss[arg][0])

    def check_state_on_boot(self, trigger_pins, listen_pins):
        # check triggers at boot
        self.notify("%s start" % self.alias)
        self.notify("IP [%s]" % self.ip_pi)
        self.notify("trigger IOs [%d, %d]" % (trigger_pins[0], trigger_pins[1]))
        self.notify("Indications IOs [%d, %d]" % (listen_pins[0], listen_pins[1]))

        if any([trig.is_pressed for trig in (self.trigger_vector)]):
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


class AlarmControlGUI(ttk.Frame):  # , GPIOMonitor):
    def __init__(self, master):
        self.arm_value = tk.StringVar()
        self.alarm_on_value = tk.StringVar()
        self.arm_home_value = tk.IntVar()
        self.arm_full_value = tk.IntVar()
        self.tx_value = tk.StringVar()

        self.common_bg = 'grey35'
        self.disarm_pwd = '1234'
        master.title('HomePi Alarm monitor')
        self.blink_status = None
        self.log_stack, self.ip_pi = [], None

        ttk.Frame.__init__(self)
        # GPIOMonitor.__init__(self, ip=ip, alias='Alarm Monitor')

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

    def run_modules(self):
        self.log_window()
        self.arm_buttons()
        self.create_indicators()
        self.oper_buttons()
        # self.constant_chk_gpio()
        self.status_bar()
        self.set_arm_ind(0)
        self.alarm_setoff_ind(0)
        self.blink_tx()
        self.boot_notifications()

    def boot_notifications(self):
        self.write2log("Boot GUI")

    def arm_buttons(self):
        pdx, pdy = 2, 2
        self.arm_buts_frame = tk.Frame(self.controls_frame, bg=self.common_bg, relief=tk.GROOVE, bd=2)
        self.arm_buts_frame.grid(row=0, column=0)
        self.full_arm_button = tk.Checkbutton(self.arm_buts_frame, text="Full", width=9, variable=self.arm_full_value,
                                              indicatoron=0, height=1, command=lambda: self.arm_buttons_cb(0))
        self.full_arm_button.grid(row=0, column=0, padx=pdx, pady=pdy)

        self.home_arm_button = tk.Checkbutton(self.arm_buts_frame, text="Home", width=9, variable=self.arm_home_value,
                                              indicatoron=0, height=1, command=lambda: self.arm_buttons_cb(1))
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

    def constant_chk_gpio(self):
        gpio_s = self.get_gpio_status()

        # arm ind
        if gpio_s[1][1] == True:
            self.set_arm_ind(1)
        else:
            self.set_arm_ind(0)
        # alert ind
        if gpio_s[1][0] == True:
            self.alarm_setoff_ind(1)
        else:
            self.alarm_setoff_ind(0)

        root.after(1000, self.constant_chk_gpio)

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

    def arm_buttons_cb(self, but_num):
        gpio_s = self.get_gpio_status()
        # Full Arm was pressed
        if but_num == 0:
            # if other button is pressed on
            if self.arm_home_value.get() == 1:
                # ask for passwd
                self.passwd_window()
                # case of correct passwd
                if self.pwd_ent_value.get() == self.disarm_pwd:
                    sleep(0.5)
                    self.arm_home_value.set(0)
                    if self.gpio_s[0][1] == 0 and self.gpio_s[0][0] == 1:
                        self.write2log('System disarmed from Home Mode')
                        self.write2log('System armed to Full Mode')
                    else:
                        self.write2log('Fail to switch to Full Home mode')
                        self.arm_home_value.set(1)
                        self.arm_full_value.set(0)
                else:
                    self.write2log('Wrong Password')
                    self.arm_full_value.set(0)
            elif self.arm_full_value.get() == 1:
                # code to arm to Full mode
                sleep(0.5)
                if self.get_gpio_status[0][0] == 1:
                    self.write2log('System armed to Full Mode')
                else:
                    self.write2log('System fail to Full mode arm')
            elif self.arm_full_value.get() == 0:
                # code to arm to Full mode
                sleep(0.5)
                self.passwd_window()
                if self.pwd_ent_value.get() == self.disarm_pwd:
                    self.write2log('System disarmed from Full Mode')
                else:
                    self.write2log('Password fail to disarm Full Mode')
                    self.arm_full_value.set(1)
                    # code to disarm
        # Home Arm was pressed
        elif but_num == 1:
            # if other button is pressed on
            if self.arm_full_value.get() == 1:
                self.passwd_window()
                if self.pwd_ent_value.get() == self.disarm_pwd:
                    self.write2log('System disarmed from Full Mode')
                    self.arm_full_value.set(0)
                    sleep(0.5)
                    # Add deactivate code
                    self.write2log('System armed to Home Mode')
                else:
                    self.write2log('Password fail to disarm Home Mode')
                    self.arm_home_value.set(0)
            elif self.arm_full_value.get() == 1:
                # code to arm to Full mode
                sleep(0.5)
                self.write2log('System armed to Home Mode')
            elif self.arm_home_value.get() == 0:
                # code to arm to home mode
                sleep(0.5)
                self.passwd_window()
                if self.pwd_ent_value.get() == self.disarm_pwd:
                    self.write2log('System disarmed from Home Mode')
                else:
                    self.write2log('Password fail to disarm Home Mode')
                    self.arm_home_value.set(1)
                    # code to disarm

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

        self.clock()

    def clock(self):
        clock_label = tk.Label(self.status_frame, bg=self.common_bg, fg='white', relief=tk.GROOVE, bd=2, padx=3)
        clock_label.grid(row=0, column=2, sticky=tk.E, padx=2)
        time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        clock_label.config(text=time)
        root.after(500, self.clock)

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

        t_blink = 5500

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
import getip

# from localswitches import Log2File
# from cbit import CBit
getip.get_ip()

root = tk.Tk()
AlarmControlGUI(root)  # , ip='192.168.2.101')
root.mainloop()
# GPIOMonitor(ip='192.168.2.115')
# pause()
