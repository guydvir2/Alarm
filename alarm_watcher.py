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
path.append(MAIN_PATH + 'bots')

from mqtt_switch import MQTTClient
from localswitches import Log2File, XTractLastLogEvent
import getip
from tbot import TelegramBot


class GPIOMonitor(Thread):
    def __init__(self, ip=None, alias='HomePi-AlarmSys monitor', listen_pins=[21, 20], trigger_pins=[16, 26],
                 log_filepath='', device_topic=None, msg_topic=None, alert_topic=None, group_topics=None,
                 broker='192.168.2.120', qos=0,
                 username=None, password=None):
        # listen_pins = [sys.arm, alarm.on], trigger_pins=[full, home]

        # ## MQTT
        self.alert_topic = alert_topic
        self.msg_topic = msg_topic
        self.device_topic = device_topic
        # ##

        # ## GPIO
        self.factory = None
        self.fullarm_hw = None
        self.homearm_hw = None
        self.sysarm_hw = None
        self.alarm_hw = None
        self.alias = alias
        self.log_filename = log_filepath + 'AlarmMonitor.log'
        self.last_state = [None for i in range(4)]
        # ##

        # operated from remote,but ip belongs to AlarmSys
        if ip is not None and ip != getip.get_ip()[0]:
            self.factory = PiGPIOFactory(host=ip)
            self.ip_pi = ip
        # case or run localy at AlarmSys
        else:
            self.ip_pi = getip.get_ip()[0]

        # ## Start Services
        Thread.__init__(self)
        self.mqtt_client = MQTTClient(sid='alarm_mqtt', topics=[device_topic, group_topics], topic_qos=qos,
                                      host=broker,
                                      username=username, password=password)
        self.telegram_bot = TelegramBot()
        self.start_mqtt_service()
        self.start_telegram_service()
        self.logger = Log2File(self.log_filename, name_of_master=self.alias, time_in_log=1, screen=1)
        self.hardware_gpio(trigger_pins, listen_pins)
        self.notify('logfile: %s' % self.log_filename)
        # ##

    def hardware_gpio(self, trigger_pins, listen_pins):
        self.fullarm_hw = OutputDevice(trigger_pins[0], pin_factory=self.factory, initial_value=False)
        self.homearm_hw = OutputDevice(trigger_pins[1], pin_factory=self.factory, initial_value=False)
        self.sysarm_hw = Button(listen_pins[0], pin_factory=self.factory)
        self.alarm_hw = Button(listen_pins[1], pin_factory=self.factory)

        self.check_state_on_boot(trigger_pins, listen_pins)

    def run(self):
        msgs = ['Full-mode Arm', 'Home-mode Arm', 'System Arm state', 'Alarm state']
        while True:
            current_status = self.fullarm_hw.value, self.homearm_hw.value, self.sysarm_hw.value, self.alarm_hw.value
            if current_status != self.last_state:
                for i, current_gpio in enumerate(current_status):
                    if self.last_state[i] != current_gpio:
                        self.last_state[i] = current_gpio
                        msg1 = '[%s] :%s' % (msgs[i], current_gpio)
                        self.notify(msg1)
                        if i == 3:
                            self.alert(msg=msg1)

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

    def start_mqtt_service(self):
        self.mqtt_client.call_externalf = lambda: self.mqtt_commands(self.mqtt_client.arrived_msg)
        self.mqtt_client.start()
        time.sleep(1)
        self.pub_msg(msg='AlarmSystem Boot')

    def mqtt_commands(self, msg, origin=None):
        if msg.upper() == 'HOME':
            self.homearm_cb(1)
            msg1 = 'Home mode armed'
        elif msg.upper() == 'FULL':
            self.fullarm_cb(1)
            msg1 = 'Full mode armed'
        elif msg.upper() == 'DISARM':
            self.disarm()
            msg1 = 'Disarmed'
        elif msg.upper() == 'STATUS':
            msg1 = alarmsys_monitor.get_status()
        else:
            msg1 = "Nan"

        self.pub_msg(msg1)
        if origin == 't':
            self.telegram_bot.send_msg(msg1)

    def pub_msg(self, msg, topic=None):
        if topic is None:
            msg_topic = self.msg_topic
        else:
            msg_topic = topic

        device_name = 'AlarmSystem'
        time_stamp = '[' + str(datetime.datetime.now())[:-4] + ']'
        self.mqtt_client.pub(payload='%s [%s] %s' % (time_stamp, device_name, msg), topic=msg_topic)

    def start_telegram_service(self):
        self.telegram_bot.telbot_commands = lambda: self.mqtt_commands(self.telegram_bot.telbot_arrived_msg, origin='t')
        self.telegram_bot.start()
        time.sleep(1)
        self.pub_msg(msg='AlarmSystem Boot')

    def alert(self, msg):
        self.pub_msg(msg=msg, topic=self.alert_topic)
        self.telegram_bot.send_msg(msg)


# ############ Parameters ###############################
DEVICE_TOPIC = 'HomePi/Dvir/AlarmSystem'
MSG_TOPIC = 'HomePi/Dvir/Messages'
ALERT_TOPIC = 'HomePi/Dvir/Alerts'
GROUP_TOPICS = 'HomePi/Dvir/All'
BROKER = '192.168.2.200'
USER = "guy"
PASSWORD = "kupelu9e"
QOS = 0
ALIAS = 'HomePi AlarmSystem Communicator'
LISTEN_PINS = [21, 20]
TRIGGER_PINS = [16, 26]
LOG_FILE = '/home/guy/'
# #######################################################

alarmsys_monitor = GPIOMonitor(ip=None, listen_pins=LISTEN_PINS, trigger_pins=TRIGGER_PINS, alias=ALIAS,
                               log_filepath=LOG_FILE, device_topic=DEVICE_TOPIC, msg_topic=MSG_TOPIC,
                               group_topics=GROUP_TOPICS, alert_topic=ALERT_TOPIC, broker=BROKER, username=USER,
                               password=PASSWORD, qos=QOS)
alarmsys_monitor.start()
