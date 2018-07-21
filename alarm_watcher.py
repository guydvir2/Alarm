from gpiozero import Button, OutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
from sys import platform, path
import time


class GPIOMonitor:
    def __init__(self, ip=None, alias='HomePi-AlarmSys monitor',
                 listen_pins=[21, 20], trigger_pins=[16, 26],
                 log_filename='/home/guy/github/Alarm/AlarmMonitor.log'):
        # listen_pins = [sys.arm, alarm.on], trigger_pins=[full, home]
        self.last_state = [None for i in range(4)]
        self.cbit = cbit.CBit(1000)

        if ip is not None:
            self.factory = PiGPIOFactory(host=ip)
            self.ip_pi = ip

        else:
            self.factory = None
            self.ip_pi = getip.get_ip()[0]

        self.alias = alias
        self.fullarm_hw = OutputDevice(trigger_pins[0], pin_factory=self.factory, initial_value=None)
        self.homearm_hw = OutputDevice(trigger_pins[1], pin_factory=self.factory, initial_value=None)
        self.sysarm_hw = Button(listen_pins[0], pin_factory=self.factory)
        self.alarm_hw = Button(listen_pins[1], pin_factory=self.factory)

        self.logger = Log2File(log_filename, name_of_master=self.alias,
                               time_in_log=1, screen=1)

        self.check_state_on_boot(trigger_pins, listen_pins)

        self.monitor_state()

    def monitor_state(self):
        def const_check_state():
            tmp_status = self.fullarm_hw.value, self.homearm_hw.value, self.sysarm_hw.value, self.alarm_hw.value
            if tmp_status != self.last_state:
                for i, current_gpio in enumerate(tmp_status):
                    if self.last_state[i] != current_gpio:
                        self.last_state[i] = current_gpio
                        self.notify('[%s] :%s' % (msgs[i], current_gpio))

        msgs = ['Full-mode Arm', 'Home-mode', 'System Arm state', 'Alarm state']

        self.cbit.append_process(const_check_state)
        self.cbit.init_thread()
    
    def get_status(self):
        msg='Empty'
        #print(self.sysarm_hw.value)
        if self.sysarm_hw.value == True:
            if self.fullarm_hw.value == True:
                msg = 'System armed - Full Mode'
            elif self.homearm_hw.value == True:
                msg = 'System armed - Home Mode'
            else:
                msg = 'System armed - Manually'
        else:
            msg = 'System is unarmed'
            
        if self.alarm_hw.value == True:
            msg = 'System is in ALARM MODE'
        return msg
        tmp_status = [str(self.fullarm_hw.value), str(self.homearm_hw.value), str(self.sysarm_hw.value), str(self.alarm_hw.value)]
        print(tmp_status)
        return 1

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

    def notify(self, msg):
        self.logger.append_log(msg)

    def fullarm_cb(self, set_state):
        if set_state == 1:
            self.fullarm_hw.on()
        elif set_state == 0:
            # case of alarm operated via keypad
            #if self.sysarm_hw.value == True and self.fullarm_hw.value == False:
                #self.fullarm_hw.on()
                #time.sleep(0.5)
                #self.fullarm_hw.off()
            #else:
            self.fullarm_hw.off()
            
    def homearm_cb(self, set_state):
        if set_state == 1:
            self.homearm_hw.on()
        elif set_state == 0:
            # case of alarm operated via keypad
            #if self.sysarm_hw.value == True and self.homearm_hw.value == False:
                #self.homearm_hw.on()
                #time.sleep(0.5)
                #self.homearm_hw.off()
            #else:
            self.homearm_hw.off()

    def disarm (self):
        if self.sysarm_hw.value == True:
            if self.fullarm_hw.value == True:
                self.fullarm_cb(0)
            elif self.homearm_hw.value == True:
                self.homearm_cb(0)
            else:
                self.fullarm_cb(1)
                time.sleep(0.2)
                self.fullarm_cb(0)
                

class MQTTnotify:
    def __init__(self, mqtt_server='iot.eclipse.org'):
        # following lines as must in every class that ment to use MQTT_class
        self.mqtt = MQTTClient(topic='/HomePi/Dvir/AlarmSys', topic_qos=0, host='iot.eclipse.org')
        self.mqtt.call_externalf = lambda: self.commands(self.mqtt.arrived_msg)
        self.mqtt.start()
        time.sleep(1)
        self.mqtt.pub(topic='/HomePi/Dvir/Messages',payload='AlarmSys in ON and monitoring')

    def commands(self, mqtt_msg):
        if mqtt_msg.lower() == 'log':
            t_log = XTractLastLogEvent('/home/guy/github/Alarm/AlarmMonitor.log')
            self.mqtt.pub(topic='/HomePi/Dvir/Messages',payload=t_log.xport_chopped_log())#A.get_status())
        elif mqtt_msg.lower() == 'status':
            t_log = XTractLastLogEvent('/home/guy/github/Alarm/AlarmMonitor.log')
            self.mqtt.pub(topic='/HomePi/Dvir/Messages',payload=A.get_status())
        elif mqtt_msg.lower() == 'disarm':
            self.mqtt.pub(topic='/HomePi/Dvir/Messages',payload='Disarmed')
            A.disarm()
            time.sleep(0.5)
            if A.sysarm_hw.value == False:
                A.notify('System disarmed')
            else:
                A.notify('System failed to disarm')
        elif mqtt_msg.lower() == 'full_arm':
            self.mqtt.pub(topic='/HomePi/Dvir/Messages',payload='full mode armed')
            A.fullarm_cb(1)
            time.sleep(0.5)
            if A.sysarm_hw.value == True:
                A.notify('System Full armed')
            else:
                A.notify('System failed to arm')
        elif mqtt_msg.lower() == 'home_arm':
            self.mqtt.pub(topic='/HomePi/Dvir/Messages',payload='home mode armed')
            A.homearm_cb(1)
            time.sleep(0.5)
            if A.sysarm_hw.value == True:
                A.notify('System Home armed')
            else:
                A.notify('System failed to arm')


os_type = platform
if os_type == 'darwin':
    main_path = '/Users/guy/Documents/github/Rpi/'
elif os_type == 'win32':
    main_path = 'd:/users/guydvir/Documents/git/Rpi/'
elif os_type == 'linux':
    main_path = '/home/guy/Documents/github/Rpi/'
    main_path = '/home/guy/github/'

path.append(main_path + 'LocalSwitch')
path.append(main_path + 'RemoteSwitch')
path.append(main_path + 'modules')

from mqtt_switch import MQTTClient
from localswitches import Log2File, XTractLastLogEvent
import getip
import cbit

A=GPIOMonitor(ip='192.168.2.117')
B=MQTTnotify()
