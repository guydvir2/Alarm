from gpiozero import Button, OutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
from sys import platform, path
import time
import datetime


class GPIOMonitor:
    def __init__(self, ip=None, alias='HomePi-AlarmSys monitor',
                 listen_pins=[21, 20], trigger_pins=[16, 26],
                 log_filepath=''):
        # listen_pins = [sys.arm, alarm.on], trigger_pins=[full, home]
        self.factory = None
        self.log_filename = log_filepath + 'AlarmMonitor.log'
        self.last_state = [None for i in range(4)]
        self.cbit = cbit.CBit(1000)

        # operated from remote,but ip belongs to AlarmSys
        if ip is not None or ip != getip.get_ip()[0]:
            self.factory = PiGPIOFactory(host=ip)
            self.ip_pi = ip
        # case or run localy at AlarmSys
        else:
            self.ip_pi = getip.get_ip()[0]

        self.alias = alias
        self.fullarm_hw = OutputDevice(trigger_pins[0], pin_factory=self.factory, initial_value=False)
        self.homearm_hw = OutputDevice(trigger_pins[1], pin_factory=self.factory, initial_value=False)
        self.sysarm_hw = Button(listen_pins[0], pin_factory=self.factory)
        self.alarm_hw = Button(listen_pins[1], pin_factory=self.factory)

        self.logger = Log2File(self.log_filename, name_of_master=self.alias,
                               time_in_log=1, screen=1)

        self.check_state_on_boot(trigger_pins, listen_pins)
        self.notify('logfile: %s' % self.log_filename)

        self.monitor_state()

    def monitor_state(self):
        self.fullarm_hw.off()
        def const_check_state():
            tmp_status = self.fullarm_hw.value, self.homearm_hw.value, self.sysarm_hw.value, self.alarm_hw.value
            if tmp_status != self.last_state:
                for i, current_gpio in enumerate(tmp_status):
                    if self.last_state[i] != current_gpio:
                        self.last_state[i] = current_gpio
                        self.notify('[%s] :%s' % (msgs[i], current_gpio))
            print(tmp_status)

        msgs = ['Full-mode Arm', 'Home-mode', 'System Arm state', 'Alarm state']

        self.cbit.append_process(const_check_state)
        self.cbit.init_thread()

    def get_status(self):
        msg = 'Empty'
        # print(self.sysarm_hw.value)
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
        #tmp_status = [str(self.fullarm_hw.value), str(self.homearm_hw.value), str(self.sysarm_hw.value),
                      #str(self.alarm_hw.value)]
        #print(tmp_status)
        #return 1

    def check_state_on_boot(self, trigger_pins, listen_pins):
        # check triggers at boot
        self.notify("%s start" % self.alias)
        self.notify("IP [%s]" % self.ip_pi)
        self.notify("trigger IOs [%d, %d]" % (trigger_pins[0], trigger_pins[1]))
        self.notify("Indications IOs [%d, %d]" % (listen_pins[0], listen_pins[1]))

        if self.sysarm_hw.value is True:  #any([self.homearm_hw.value, self.fullarm_hw.value, ]):
            al_stat = '@BOOT- System Armed'
        else:
            al_stat = '@Boot -System Unarmed'

        self.notify(al_stat)

    def notify(self, msg):
        self.logger.append_log(msg)

    def fullarm_cb(self, set_state):
        if set_state == 1:
            self.fullarm_hw.on()
            print("FULLON")
        elif set_state == 0:
            self.fullarm_hw.off()

    def homearm_cb(self, set_state):
        if set_state == 1:
            self.homearm_hw.on()
            print("HOMEON")
        elif set_state == 0:
            hw.off()
            self.homearm_hw.off()

    def disarm(self):
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
    def __init__(self, sub_topic, msg_topic, mqtt_server='iot.eclipse.org'):
        self.msg_topic = msg_topic
        self.mqtt = MQTTClient(topic=sub_topic, topic_qos=0, host=mqtt_server)
        self.mqtt.call_externalf = lambda: self.commands(self.mqtt.arrived_msg)
        self.mqtt.start()
        time.sleep(1)

    def startup_notify(self):
        self.mqtt.pub(topic=self.msg_topic, payload='AlarmSys in ON and monitoring')

    def commands(self, mqtt_msg):
        if mqtt_msg.lower() == 'info' or mqtt_msg == '5':
            self.mqtt.pub(topic=self.msg_topic,
                          payload='Applicable commands:\n***********************\n0) disarm\n1) full_arm\n2) home_arm\n3) status\n4) log\n5) info')
        elif mqtt_msg.lower() == 'log' or mqtt_msg == '4':
            t_log = XTractLastLogEvent(A.log_filename)
            self.mqtt.pub(topic=self.msg_topic, payload=t_log.xport_chopped_log())
        elif mqtt_msg.lower() == 'status' or mqtt_msg == '3':
            self.mqtt.pub(topic=self.msg_topic, payload=A.get_status())
        elif mqtt_msg.lower() == 'disarm' or mqtt_msg == '0':
            self.mqtt.pub(topic=self.msg_topic, payload='Disarmed')
            A.disarm()
            time.sleep(0.5)
            if A.sysarm_hw.value is False:
                A.notify('System disarmed')
            else:
                A.notify('System failed to disarm')
        elif mqtt_msg.lower() == 'full_arm' or mqtt_msg.lower() == '1':
            self.mqtt.pub(topic=self.msg_topic, payload='full mode armed')
            A.fullarm_cb(1)
            time.sleep(0.5)
            if A.sysarm_hw.value is True:
                A.notify('System Full armed')
            else:
                A.notify('System failed to arm')
        elif mqtt_msg.lower() == 'home_arm' or mqtt_msg.lower() == '2':
            self.mqtt.pub(topic=self.msg_topic, payload='home mode armed')
            A.homearm_cb(1)
            time.sleep(0.5)
            if A.sysarm_hw.value is True:
                A.notify('System Home armed')
            else:
                A.notify('System failed to arm')


MAIN_PATH = '/home/guy/github/'
path.append(MAIN_PATH + 'LocalSwitch/main')
path.append(MAIN_PATH + 'RemoteSwitch')
path.append(MAIN_PATH + 'modules')

from mqtt_switch import MQTTClient
from localswitches import Log2File, XTractLastLogEvent
import getip
import cbit

def start_mqtt_service():
    global MQTT_Client
    MQTT_Client = MQTTClient(topics=['HomePi/Dvir/AlarmSystem'], topic_qos=0, host='192.168.2.113')
    MQTT_Client.call_externalf = lambda: mqtt_commands(MQTT_Client.arrived_msg)
    MQTT_Client.start()
    time.sleep(1)
    pub_msg(msg='System Boot')
    
def mqtt_commands(msg):
    global alarmsys_monitor
    if msg.upper() == 'HOME':
        alarmsys_monitor.homearm_cb(1)
        pub_msg('Home mode armed')
    elif msg.upper() == 'FULL':
        alarmsys_monitor.fullarm_cb(1)
        pub_msg('Full mode armed')
    elif msg.upper() == 'DOWN':
        lalarmsys_monitor.disarm()
        pub_msg('Disarmed')
    elif msg.upper() == 'STATUS':
        pub_msg(alarmsys_monitor.get_status())
    else:
        pass


def pub_msg(msg):
    global MQTT_Client
    msg_topic='HomePi/Dvir/Messages'
    device_name='AlarmSystem'
    time_stamp = '[' + str(datetime.datetime.now())[:-5] + ']'
    MQTT_Client.pub(payload='%s [%s] %s' % (time_stamp, device_name, msg), topic=msg_topic)
    


alarmsys_monitor = GPIOMonitor(ip='192.168.2.113', log_filepath=MAIN_PATH + 'Alarm/')
start_mqtt_service()
#B = MQTTnotify(sub_topic='HomePi/Dvir/AlarmSys', msg_topic='HomePi/Dvir/Messages', mqtt_server='iot.eclipse.org')
