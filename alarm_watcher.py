try:
    from gpiozero import Button, OutputDevice
except:
    pass
import time
from sys import path
import datetime
from threading import Thread

MAIN_PATH = '/home/guy/github/'
path.append(MAIN_PATH + 'LocalSwitch/main')
path.append(MAIN_PATH + 'modules')
path.append(MAIN_PATH + 'MQTTswitches')

from mqtt_switch import MQTTClient
from localswitches import Log2File, XTractLastLogEvent
import getip


class GPIOMonitor(Thread):
    def __init__(self, ip=None, alias='HomePi-AlarmSys monitor', listen_pins=[21, 20], trigger_pins=[16, 26],
                 log_filepath=''):
        Thread.__init__(self)
        self.mqtt_client = MQTTClient(sid='alarm_mqtt', topics=['HomePi/Dvir/AlarmSystem', 'HomePi/Dvir/All'],
                                      topic_qos=0, host='192.168.2.200', username="guy", password="kupelu9e")#,
                                      # alert_topic='HomePi/Dvir/Alerts')

        # listen_pins = [sys.arm, alarm.on], trigger_pins=[full, home]
        self.factory = None
        self.fullarm_hw = None
        self.homearm_hw = None
        self.sysarm_hw = None
        self.alarm_hw = None
        self.alias = alias
        self.log_filename = log_filepath + 'AlarmMonitor.log'
        self.last_state = [None for i in range(4)]

        # operated from remote,but ip belongs to AlarmSys
        if ip is not None and ip != getip.get_ip()[0]:
            self.factory = PiGPIOFactory(host=ip)
            self.ip_pi = ip
        # case or run localy at AlarmSys
        else:
            self.ip_pi = getip.get_ip()[0]

        self.logger = Log2File(self.log_filename, name_of_master=self.alias, time_in_log=1, screen=1)
        self.hardware_gpio(trigger_pins, listen_pins)
        self.notify('logfile: %s' % self.log_filename)

    def hardware_gpio(self, trigger_pins, listen_pins):
        self.fullarm_hw = OutputDevice(trigger_pins[0], pin_factory=self.factory, initial_value=False)
        self.homearm_hw = OutputDevice(trigger_pins[1], pin_factory=self.factory, initial_value=False)
        self.sysarm_hw = Button(listen_pins[0], pin_factory=self.factory)
        self.alarm_hw = Button(listen_pins[1], pin_factory=self.factory)

        self.check_state_on_boot(trigger_pins, listen_pins)

    def run(self):
        msgs = ['Full-mode Arm', 'Home-mode Arm', 'System Arm state', 'Alarm state']
        current_status = self.fullarm_hw.value, self.homearm_hw.value, self.sysarm_hw.value, self.alarm_hw.value
        if current_status != self.last_state:
            for i, current_gpio in enumerate(current_status):
                if self.last_state[i] != current_gpio:
                    self.last_state[i] = current_gpio
                    self.notify('[%s] :%s' % (msgs[i], current_gpio))

    def get_status(self):
        return ('Full-arm state:%s, Home-arm state:%s, System armed:%s,SystemAlarming:%s' % (
            self.fullarm_hw.value, self.homearm_hw.value, self.sysarm_hw.value, self.alarm_hw.value))

    def check_state_on_boot(self, trigger_pins, listen_pins):
        # check triggers at boot
        self.notify("%s start" % self.alias)
        self.notify("IP [%s]" % self.ip_pi)
        self.notify("trigger IOs [%d, %d]" % (trigger_pins[0], trigger_pins[1]))
        self.notify("Indications IOs [%d, %d]" % (listen_pins[0], listen_pins[1]))

        if self.sysarm_hw.value is True:  # any([self.homearm_hw.value, self.fullarm_hw.value, ]):
            al_stat = '@BOOT- System Armed'
        else:
            al_stat = '@Boot -System Unarmed'

        self.notify(al_stat)

    def notify(self, msg):
        self.logger.append_log(msg)

    def fullarm_cb(self, set_state=None):
        if set_state is None:
            return self.fullarm_hw.value
        if self.homearm_cb() == 1 and set_state == 1:
            self.homearm_cb(set_state=0)
            time.sleep(2)
        if set_state == 1:
            self.fullarm_hw.on()
        elif set_state == 0:
            self.fullarm_hw.off()

    def homearm_cb(self, set_state=None):
        if set_state is None:
            return self.homearm_hw.value
        if self.fullarm_cb() == 1 and set_state == 1:
            self.fullarm_cb(set_state == 0)
            time.sleep(2)
        if set_state == 1:
            self.homearm_hw.on()
        elif set_state == 0:
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
                self.homearm_cb(1)
                time.sleep(0.2)
                self.homearm_cb(0)

    def mqtt_service(self):
        self.mqtt_client.call_externalf = lambda: self.mqtt_commands(self.mqtt_client.arrived_msg)
        self.mqtt_client.start()
        time.sleep(1)
        self.pub_msg(msg='AlarmSystem Boot')

    def mqtt_commands(self, msg):
        if msg.upper() == 'HOME':
            self.homearm_cb(1)
            self.pub_msg('Home mode armed')
        elif msg.upper() == 'FULL':
            self.fullarm_cb(1)
            self.pub_msg('Full mode armed')
        elif msg.upper() == 'DISARM':
            self.disarm()
            self.pub_msg('Disarmed')
        elif msg.upper() == 'STATUS':
            self.pub_msg(alarmsys_monitor.get_status())
        else:
            pass

    def pub_msg(self, msg, topic=None):
        msg_topic = 'HomePi/Dvir/Messages'
        device_name = 'AlarmSystem'
        time_stamp = '[' + str(datetime.datetime.now())[:-4] + ']'
        self.mqtt_client.pub(payload='%s [%s] %s' % (time_stamp, device_name, msg), topic=msg_topic)


# def start_mqtt_service():
#     global MQTT_Client
#     MQTT_Client = MQTTClient(sid='alarm_mqtt', topics=['HomePi/Dvir/AlarmSystem', 'HomePi/Dvir/All'], topic_qos=0,
#                              host='192.168.2.200', username="guy", password="kupelu9e", alert_topic='HomePi/Dvir/Alerts')
#     MQTT_Client.call_externalf = lambda: mqtt_commands(MQTT_Client.arrived_msg)
#     MQTT_Client.start()
#     time.sleep(1)
#     pub_msg(msg='AlarmSystem Boot')
#


# def mqtt_commands(msg):
#     global alarmsys_monitor
#     if msg.upper() == 'HOME':
#         alarmsys_monitor.homearm_cb(1)
#         pub_msg('Home mode armed')
#     elif msg.upper() == 'FULL':
#         alarmsys_monitor.fullarm_cb(1)
#         pub_msg('Full mode armed')
#     elif msg.upper() == 'DISARM':
#         alarmsys_monitor.disarm()
#         pub_msg('Disarmed')
#     elif msg.upper() == 'STATUS':
#         pub_msg(alarmsys_monitor.get_status())
#     else:
#         pass


# def pub_msg(msg):
#     global MQTT_Client
#     msg_topic = 'HomePi/Dvir/Messages'
#     device_name = 'AlarmSystem'
#     time_stamp = '[' + str(datetime.datetime.now())[:-4] + ']'
#     MQTT_Client.pub(payload='%s [%s] %s' % (time_stamp, device_name, msg), topic=msg_topic)


# start_mqtt_service()
alarmsys_monitor = GPIOMonitor(ip=None, log_filepath=MAIN_PATH + 'Alarm/')
