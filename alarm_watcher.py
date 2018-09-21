try:
    from gpiozero import Button, OutputDevice
except:
    print("GPIOSHIT")

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
                 broker='192.168.2.120', qos=0, username=None, password=None):
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
        # ##

        self.alias = alias
        self.log_filename = log_filepath + 'AlarmMonitor.log'
        self.last_state = [None, None, None, None]
        self.alarm_on_flag = False
        self.alarm_start_time = None

        # ##

        # operated from remote,but ip belongs to AlarmSys
        if ip is not None and ip != getip.get_ip()[0]:
            self.factory = PiGPIOFactory(host=ip)
            self.ip_pi = ip

        # case or run locally at AlarmSys
        else:
            self.ip_pi = getip.get_ip()[0]

        # ## Start Services
        Thread.__init__(self)
        self.mqtt_client = MQTTClient(sid='alarm_mqtt', topics=[device_topic, group_topics], topic_qos=qos,
                                      host=broker, username=username, password=password)
        self.telegram_bot = TelegramBot()
        self.start_mqtt_service()
        self.start_telegram_service()
        self.logger = Log2File(self.log_filename, name_of_master=self.alias, time_in_log=1, screen=1)
        self.last_log_record = None  # define below
        self.hardware_gpio(trigger_pins, listen_pins)
        self.notify('logfile: [%s]' % self.log_filename)
        # ##

    def hardware_gpio(self, trigger_pins, listen_pins):
        self.fullarm_hw = OutputDevice(trigger_pins[0], pin_factory=self.factory, initial_value=None)
        self.homearm_hw = OutputDevice(trigger_pins[1], pin_factory=self.factory, initial_value=None)
        self.sysarm_hw = Button(listen_pins[0], pin_factory=self.factory)
        self.alarm_hw = Button(listen_pins[1], pin_factory=self.factory)

        self.check_state_on_boot(trigger_pins, listen_pins)

    def run(self):
        self.alert(msg="AlarmSystem started")
        msgs = ['Full-mode Arm', 'Home-mode Arm', 'System Arm state', 'Alarm state']

        while True:
            current_status = [self.fullarm_hw.value, self.homearm_hw.value, self.sysarm_hw.value, self.alarm_hw.value]
            if current_status != self.last_state:
                for i, current_gpio in enumerate(current_status):
                    if self.last_state[i] != current_gpio:
                        msg1 = '[watchdog] [%s] :%s' % (msgs[i], current_gpio)
                        self.notify(msg1)
                    self.last_state[i] = current_gpio

                if current_status[3] is True:
                    if self.alarm_start_time is None:
                        self.alarm_start_time = time.time()
                        self.notify(msg="System is ALARMING!", platform='mt')

                elif current_status[3] is False and self.alarm_start_time is not None:
                    self.alarm_start_time = None
                    self.notify(msg="System stopped Alarming", platform='mt')

            time.sleep(1)

    def get_status(self):
        msg = 'Empty status result'
        if self.sysarm_hw.value is True:
            msg = 'System armed: '
            if self.homearm_hw.value is True:
                msg = msg + 'Home mode'
            elif self.fullarm_hw.value is True:
                msg = msg + 'Full mode'
            else:
                msg = msg + 'Error'
        elif self.sysarm_hw.value is not True:
            msg = 'System is not Armed'

        if self.alarm_hw.value is True:
            msg = 'System is ALARMING!'

        return 'Status CMD: ' + msg

    def check_state_on_boot(self, trigger_pins, listen_pins):
        # check triggers at boot
        self.notify(msg="%s start" % self.alias)
        self.notify(msg="IP [%s]" % self.ip_pi)
        self.notify(msg="trigger IOs [%d, %d]" % (trigger_pins[0], trigger_pins[1]))
        self.notify(msg="Indications IOs [%d, %d]" % (listen_pins[0], listen_pins[1]))
        self.notify(msg="MQTT topics: [%s], [%s], [%s]" % (self.device_topic, self.msg_topic, self.alert_topic))
        if self.sysarm_hw.value is True:
            al_stat = ' System @boot :[Armed]'
        else:
            al_stat = 'System @boot :[Unarmed]'

        self.notify(msg=al_stat)

    def notify(self, msg, platform=None):
        self.logger.append_log(msg)

        # choose notification platform:
        if platform == 'm':
            self.pub_msg(msg=msg)
        if platform == 't':
            self.telegram_bot.send_msg(msg)
        if platform == 'mt' or platform == 'tm':
            self.alert(msg)

    def fullarm_cb(self, set_state=None):
        if set_state is None:
            return self.fullarm_hw.value

        if set_state == 1:
            if self.homearm_cb() == 0:
                self.fullarm_hw.on()
                self.notify(msg="[Hardware CMD]: Full-arm [ON]")  # , platform='mt')
                return 1
            elif self.homearm_cb() == 1:
                self.homearm_cb(set_state=0)
                time.sleep(2)
                self.fullarm_hw.on()
                self.notify(msg="[Hardware CMD]: Full-arm [ON]")  # , platform='mt')
                return 1

        elif set_state == 0:
            self.fullarm_hw.off()
            self.notify(msg="[Hardware CMD]: Full-arm [OFF]")  # , platform='mt')
            return 0

    def homearm_cb(self, set_state=None):
        if set_state is None:
            return self.homearm_hw.value

        if set_state == 1:
            if self.fullarm_cb() == 1:
                self.fullarm_cb(set_state=0)
                time.sleep(2)
                self.homearm_hw.on()
                self.notify(msg="[Hardware CMD]: Home-arm [ON]")  # , platform='mt')

            elif self.fullarm_cb() == 0:
                self.homearm_hw.on()
                self.notify(msg="[Hardware CMD]: Home-arm [ON]")  # , platform='mt')

        elif set_state == 0:
            self.homearm_hw.off()
            self.notify(msg="[Hardware CMD]: Home-arm [OFF]")  # , platform='mt')

    def disarm(self):
        if self.sysarm_hw.value is True:
            if self.fullarm_hw.value is True:
                self.fullarm_cb(set_state=0)
            if self.homearm_hw.value is True:
                self.homearm_cb(set_state=0)

            if self.homearm_cb() == 0 and self.fullarm_cb() == 0:
                return 1

        # verify in case system stucked in armed state
        time.sleep(1)
        if self.sysarm_hw.value is True:
            self.fullarm_cb(1)
            time.sleep(0.2)
            self.fullarm_cb(0)
            self.homearm_cb(1)
            time.sleep(0.2)
            self.homearm_cb(0)

        if self.sysarm_hw.value is True: # treated as an alert situation
            self.notify(msg="[Hardware CMD]: Disarm, fail", platform='mt')
            return 0
        else:
            self.notify(msg="[Hardware CMD]: Disarm, ok")
            return 1

    def xport_last_log(self):
        self.last_log_record = XTractLastLogEvent(self.log_filename)
        return self.last_log_record.xport_chopped_log()

    def start_mqtt_service(self):
        self.mqtt_client.call_externalf = lambda: self.mqtt_commands(self.mqtt_client.arrived_msg)
        self.mqtt_client.start()
        time.sleep(1)

    def mqtt_commands(self, msg, origin=None):
        if msg.upper() == 'HOME':
            self.homearm_cb(1)
            if self.homearm_cb() == 1:
                msg1 = '[Remote CMD] System status: Home mode armed'
            else:
                msg1 = "[Remote CMD] failed arming Home mode "

        elif msg.upper() == 'FULL':
            self.fullarm_cb(1)
            if self.fullarm_cb() == 1:
                msg1 = '[Remote CMD] System status: Full mode armed'
            else:
                msg1 = "[Remote CMD] failed arming to Full mode"

        elif msg.upper() == 'DISARM':
            if self.disarm() == 1:
                msg1 = '[Remote CMD] System status: Disarmed'
            else:
                msg1 = "[Remote CMD] System status: failed to disarm"

        elif msg.upper() == 'STATUS':
            msg1 = self.get_status()

        elif msg.upper() == 'LOG':
            msg1 = self.xport_last_log()
        else:
            msg1 = "Nan"

        if origin == 't':
            self.notify(msg=msg1, platform='t')
        else:
            self.notify(msg=msg1, platform='m')

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

    def alert(self, msg):
        self.pub_msg(msg=msg)
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
