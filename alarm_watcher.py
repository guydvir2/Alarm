try:
    from gpiozero import Button, OutputDevice
except:
    print("can't find gpios ")

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
                 broker='192.168.2.200', qos=0, username=None, password=None, state_topic=None,
                 MAN_state_topic=None):

        # EXPLAIN: listen_pins = [sys.arm, alarm.on], trigger_pins=[full, home]
        # ## MQTT
        self.alert_topic = alert_topic
        self.msg_topic = msg_topic
        self.device_topic = device_topic
        self.state_topic = state_topic
        self.man_state_topic = MAN_state_topic
        # ##

        # ## GPIO
        self.factory = None
        self.armed_away_hw = None
        self.armed_home_hw = None
        self.armed_indication = None
        self.triggered_indication = None
        # ##

        self.alias = alias
        self.log_filename = log_filepath + 'AlarmMonitor.log'
        self.last_state = [None, None, None, None]
        self.current_state = [None, None, None, None]
        # Do not change values below !
        self.system_states = ['armed_away', 'armed_home', 'disarmed', 'triggered', 'pending']
        self.alarm_pwd = "5161"
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
        self.start_mqtt_service()

        try:
            self.telegram_bot = TelegramBot()
            self.start_telegram_service()
        except:
            self.notify("Telegram service failed")

        self.logger = Log2File(self.log_filename, name_of_master=self.alias, time_in_log=1, screen=1)
        self.last_log_record = None  # define below
        self.hardware_gpio(trigger_pins, listen_pins)
        self.notify('logfile: [%s]' % self.log_filename)
        # ##

    def hardware_gpio(self, trigger_pins, listen_pins):
        self.armed_away_hw = OutputDevice(trigger_pins[0], pin_factory=self.factory, initial_value=None)
        self.armed_home_hw = OutputDevice(trigger_pins[1], pin_factory=self.factory, initial_value=None)
        self.armed_indication = Button(listen_pins[0], pin_factory=self.factory)
        self.triggered_indication = Button(listen_pins[1], pin_factory=self.factory)

        self.check_state_on_boot(trigger_pins, listen_pins)

    def run(self):
        self.alert(msg="AlarmSystem started")

        # first run
        self.current_state = [self.armed_away_hw.value, self.armed_home_hw.value, self.armed_indication.value,
                              self.triggered_indication.value]
        self.last_state = self.current_state.copy()

        while True:
            self.current_state = [self.armed_away_hw.value, self.armed_home_hw.value, self.armed_indication.value,
                                  self.triggered_indication.value]

            if self.current_state != self.last_state:
                self.detect_hardware_state()

            time.sleep(1)

    def detect_hardware_state(self):
        for i, current_gpio in enumerate(self.current_state):
            if self.last_state[i] != current_gpio:
                if i == 2:
                    msg1 = '[watchdog] [%s] :%s' % (self.system_states[i], "False")
                    # return feedback - armed
                    if current_gpio is True:
                        self.mqtt_client.pub(payload="armed", topic=self.man_state_topic, retain=True)
                        # armed using keypad
                        if self.current_state[0] is False and self.current_state[1] is False:
                            self.mqtt_client.pub(payload=self.system_states[4], topic=self.state_topic, retain=True)
                    # return feedback - disarmed
                    else:
                        self.mqtt_client.pub(payload=self.system_states[i], topic=self.state_topic, retain=True)
                        self.mqtt_client.pub(payload="disarmed", topic=self.man_state_topic, retain=True)
                else:
                    msg1 = '[watchdog] [%s] :%s' % (self.system_states[i], current_gpio)
                    if current_gpio is True:
                        self.mqtt_client.pub(payload=self.system_states[i], topic=self.state_topic, retain=True)
                        if i == 3:
                            self.notify(msg="ALARM!", platform='mt')

                self.notify(msg1)

        self.last_state = self.current_state

    def get_status(self):
        msg = 'Empty status result'
        if self.armed_indication.value is True:
            msg = 'System armed: '
            if self.armed_home_hw.value is True:
                msg = msg + 'Home mode'
            elif self.armed_away_hw.value is True:
                msg = msg + 'Full mode'
            else:
                msg = msg + "Manually ( can't tell what state is it )"
        elif self.armed_indication.value is not True:
            msg = 'System is not Armed'

        if self.triggered_indication.value is True:
            msg = 'System is ALARMING!'

        return 'Status CMD: ' + msg

    def check_state_on_boot(self, trigger_pins, listen_pins):
        # check triggers at boot
        self.notify(msg="%s start" % self.alias)
        self.notify(msg="IP [%s]" % self.ip_pi)
        self.notify(msg="trigger IOs [%d, %d]" % (trigger_pins[0], trigger_pins[1]))
        self.notify(msg="Indications IOs [%d, %d]" % (listen_pins[0], listen_pins[1]))
        self.notify(msg="MQTT topics: [%s], [%s], [%s]" % (self.device_topic, self.msg_topic, self.alert_topic))
        if self.armed_indication.value is True:
            al_stat = ' System @boot :[Armed]'
        else:
            al_stat = 'System @boot :[Unarmed]'

        self.notify(msg=al_stat)

    def notify(self, msg, platform=None):
        self.logger.append_log(msg)

        # choose notification platform:
        if platform == 'm' or platform is None:
            self.pub_msg(msg=msg)
        if platform == 't':
            self.telegram_bot.send_msg(msg)
        if platform == 'mt' or platform == 'tm':
            self.alert(msg)

    def fullarm_cb(self, set_state=None):
        if set_state is None:
            return self.armed_away_hw.value

        #  arm full
        if set_state == 1:
            # case it wa not home arm before
            if self.homearm_cb() == 0:
                self.armed_away_hw.on()
                if self.armed_away_hw.value == 1:
                    self.notify(msg="[Hardware]: Full-arm [ON]")
                    return 1
                else:
                    self.notify(msg="[Hardware]: Full-arm [ON], failed")
                    return 0

            # case it was home armed before
            elif self.homearm_cb() == 1:
                self.homearm_cb(set_state=0)
                time.sleep(2)
                if self.homearm_cb() == 0:
                    self.armed_away_hw.on()
                    if self.armed_away_hw.value == 1:
                        self.notify(msg="[Hardware]: Full-arm [ON]")
                        return 1
                    else:
                        self.notify(msg="[Hardware]: Full-arm [ON], failed")
                        return 0
                else:
                    self.notify(msg="[Hardware]: Full-arm [ON], failed")
                    return 0

        elif set_state == 0:
            self.armed_away_hw.off()
            if self.armed_away_hw.value == 0:
                self.notify(msg="[Hardware]: Full-arm [OFF]")
                return 0
            else:
                self.notify(msg="[Hardware]: Full-arm [OFF], failed")
                return 1

    def homearm_cb(self, set_state=None):
        if set_state is None:
            return self.armed_home_hw.value

        # arm home
        if set_state == 1:
            # switch from full state
            if self.fullarm_cb() == 1:
                self.fullarm_cb(set_state=0)
                time.sleep(2)
                if self.armed_away_hw.value == 0:
                    self.armed_home_hw.on()
                    if self.armed_home_hw.value == 1:
                        self.notify(msg="[Hardware]: Home-arm [ON]")
                        return 1
                    else:
                        self.notify(msg="[Hardware]: Home-arm [ON], failed")
                        return 0
                else:
                    self.notify(msg="[Hardware]: Home-arm [ON], failed")
                    return 0
            # case it was not full armed
            elif self.fullarm_cb() == 0:
                self.armed_home_hw.on()
                if self.armed_home_hw.value == 1:
                    self.notify(msg="[Hardware]: Home-arm [ON]")
                    return 1
                else:
                    self.notify(msg="[Hardware]: Home-arm [ON], failed")
                    return 0

        elif set_state == 0:
            self.armed_home_hw.off()
            if self.armed_home_hw.value == 0:
                self.notify(msg="[Hardware]: Home-arm [OFF]")
                return 0
            else:
                self.notify(msg="[Hardware]: Home-arm [OFF], failed")
                return 1

    def disarm(self):
        # case 1 : armed by software ( has a relay indication )
        if self.armed_indication.value is True and any([self.armed_away_hw.value, self.armed_home_hw.value]):
            if self.armed_away_hw.value is True:
                self.fullarm_cb(set_state=0)
            if self.armed_home_hw.value is True:
                self.homearm_cb(set_state=0)

            # verify all is off
            if all([self.armed_home_hw.value, self.armed_away_hw.value, self.armed_indication.value]) is False:
                self.notify(msg="[Hardware]: Disarm, ok")
                return 1

        # case 2: armed manually by user ( no indication by relay )
        if self.armed_indication.value is True and self.armed_away_hw.value == 0 and self.armed_home_hw.value == 0:
            # arm ( not knowing what state it realy is )
            self.fullarm_cb(1)
            time.sleep(1)
            # and now disarm
            self.fullarm_cb(0)

            # self.homearm_cb(0)

            # verify
            if all([self.armed_home_hw.value, self.armed_away_hw.value, self.armed_indication.value]) is False:
                self.notify(msg="[Hardware]: Disarm, ok")
                return 1

        # case 3: some error
        if any([self.armed_home_hw.value, self.armed_away_hw.value, self.armed_indication.value]):
            self.fullarm_cb(1)
            time.sleep(0.2)
            self.fullarm_cb(0)
            self.homearm_cb(1)
            time.sleep(0.2)
            self.homearm_cb(0)

            # verify
            if all([self.armed_home_hw.value, self.armed_away_hw.value, self.armed_indication.value]) is False:
                self.notify(msg="[Hardware]: Disarm, ok")
                return 1

        if any([self.armed_home_hw.value, self.armed_away_hw.value, self.armed_indication.value]) is True:
            self.notify(msg="[Hardware]: Disarm, fail", platform='mt')
            return 0

    def xport_last_log(self):
        self.last_log_record = XTractLastLogEvent(self.log_filename)
        return self.last_log_record.xport_chopped_log()

    def start_mqtt_service(self):
        self.mqtt_client.call_externalf = lambda: self.mqtt_commands(self.mqtt_client.arrived_msg)
        self.mqtt_client.start()
        time.sleep(1)

    def mqtt_commands(self, msg, origin=None):
        # armed_home
        if msg.lower() == self.system_states[1]:
            self.homearm_cb(1)
            if self.homearm_cb() == 1:
                msg1 = '[Remote]: Home mode arm'
            else:
                msg1 = "[Remote] failed arming Home mode "

        # armed_away
        elif msg.lower() == self.system_states[0]:
            self.fullarm_cb(1)
            if self.fullarm_cb() == 1:
                msg1 = '[Remote]: Full mode arm'
            else:
                msg1 = "[Remote] failed arming to Full mode"

        # disarmed
        elif msg.lower() == self.system_states[2]:
            if self.disarm() == 1:
                msg1 = '[Remote]: Disarm'
            else:
                msg1 = "[Remote]: failed to disarm"

        elif msg.upper() == 'STATUS':
            msg1 = self.get_status()

        elif msg.upper() == 'LOG':
            msg1 = self.xport_last_log()
        else:
            msg1 = "Nan"

        if origin == 't':
            self.notify(msg=msg1, platform='t')
        # else:
        #     self.notify(msg=msg1, platform='m')

    def pub_msg(self, msg, topic=None):
        if topic is None:
            msg_topic = self.msg_topic
        else:
            msg_topic = topic

        device_name = 'AlarmSystem'
        time_stamp = '[' + str(datetime.datetime.now())[:-4] + ']'
        self.mqtt_client.pub(payload='%s [%s] %s' % (time_stamp, device_name, msg), topic=msg_topic)

    def start_telegram_service(self):
        # this way I use MQTT command to be same when reaching from Telegram
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
STATE_TOPIC = 'HomePi/Dvir/AlarmSystem/State'
MAN_STATE_TOPIC = 'HomePi/Dvir/AlarmSystem/MAN_State'
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
                               group_topics=GROUP_TOPICS, alert_topic=ALERT_TOPIC, state_topic=STATE_TOPIC,
                               broker=BROKER, username=USER, MAN_state_topic=MAN_STATE_TOPIC,
                               password=PASSWORD, qos=QOS)
alarmsys_monitor.start()
