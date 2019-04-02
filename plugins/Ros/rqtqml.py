import sys

try:
    sys.argv
except:
    setattr(sys, 'argv', [])
    setattr(sys, 'argc', 0)

from python_qt_binding.QtCore import qDebug
from python_qt_binding.QtCore import pyqtProperty, pyqtSlot, pyqtSignal, QVariant
from python_qt_binding.QtCore import Q_CLASSINFO
from python_qt_binding.QtQml import QQmlListProperty, QQmlComponent, QQmlParserStatus
from python_qt_binding.QtQuick import QQuickItem

import rospy
import rospkg
import roslib
from rospy_message_converter import message_converter, json_message_converter


class Topic(QQuickItem, QQmlParserStatus):
    def __init__(self, parent = None):
        super(Topic, self).__init__(parent)
        self._name = None
        self._type = None

    @pyqtProperty(str)
    def name(self):
        return self._name

    @name.setter
    def name(self, name):
        self._name = str(name)

    @pyqtProperty(str)
    def type(self):
        return self._type

    @type.setter
    def type(self, type):
        self._type = str(type)

    def componentComplete(self):
        if not self.name:
            rospy.logfatal('QML property Topic.name is not set')
            sys.exit(1)

        if not self.type:
            rospy.logfatal('QML property Topic.type is not set')
            sys.exit(1)


class Subscriber(QQuickItem, QQmlParserStatus):
    on_message = pyqtSignal('QVariantMap', name = 'messageChanged', arguments = ['msg'])

    def __init__(self, parent = None):
        super(Subscriber, self).__init__(parent)
        self._topic = None
        self._queue_size = None
        self._sub = None
        self._msg_dict = dict()

    @pyqtProperty(Topic)
    def topic(self):
        return self._topic

    @topic.setter
    def topic(self, topic):
        self._topic = topic

    @pyqtProperty(int)
    def queueSize(self):
        return self._queue_size

    @queueSize.setter
    def queueSize(self, queue_size):
        self._queue_size = queue_size

    @pyqtProperty('QVariantMap', notify=on_message)
    def message(self):
        return self._msg_dict

    @message.setter
    def message(self, msg):
        self._msg_dict = msg
        self.on_message.emit(self._msg_dict)

    def componentComplete(self):
        if not self.topic:
            rospy.logfatal('QML property Subscriber.topic is not set')
            sys.exit(1)

    def message_callback(self, data):
        msg_dict = message_converter.convert_ros_message_to_dictionary(data)
        self.message = msg_dict

    def subscribe(self):
        message_class = roslib.message.get_message_class(self.topic.type)
        self._sub = rospy.Subscriber(
            self._topic.name, message_class, self.message_callback,
            queue_size = self._queue_size)


class Publisher(QQuickItem, QQmlParserStatus):
    def __init__(self, parent = None):
        super(Publisher, self).__init__(parent)
        self._topic = None
        self._queue_size = None
        self._pub = None

    @pyqtProperty(Topic)
    def topic(self):
        return self._topic

    @topic.setter
    def topic(self, topic):
        self._topic = topic

    @pyqtProperty(int)
    def queueSize(self):
        return self._queue_size

    @queueSize.setter
    def queueSize(self, queue_size):
        self._queue_size = queue_size

    def componentComplete(self):
        if self._topic is None:
            rospy.logfatal('QML property Publisher.topic is not set')
            sys.exit(1)

        message_class = roslib.message.get_message_class(self._topic.type)
        self._pub = rospy.Publisher(self._topic.name, message_class, queue_size = self._queue_size)

    @pyqtSlot(str, name = 'publish')
    @pyqtSlot('QVariantMap', name = 'publish')
    def publish(self, msg):
        if isinstance(msg, str) or isinstance(msg, unicode):
            # Message is a JSON string
            self._pub.publish(json_message_converter.convert_json_to_ros_message(self.topic.type, msg))
        elif isinstance(msg, dict):
            # Message is Python dictionary
            self._pub.publish(message_converter.convert_dictionary_to_ros_message(self.topic.type, msg))


class Ros(QQuickItem, QQmlParserStatus):
    Q_CLASSINFO('DefaultProperty', 'objects')

    def __init__(self, parent = None):
        super(Ros, self).__init__(parent)

        self._publishers = []
        self._subscribers = []

    @pyqtProperty(str)
    def name(self):
        return rospy.get_name()

    @pyqtProperty(QQmlListProperty)
    def objects(self):
        return QQmlListProperty(QQuickItem, self, append = self.append_object)

    @pyqtProperty(bool)
    def isInitialized(self):
        return rospy.core.is_initialized()

    def append_object(self, node, object):
        if isinstance(object, Subscriber):
            self._subscribers.append(object)
        elif isinstance(object, Publisher):
            self._publishers.append(object)

    def componentComplete(self):
        for s in self._subscribers:
            s.subscribe()
            rospy.logdebug('Subscribed to: ' + s.topic.name)

        for p in self._publishers:
            rospy.logdebug('Will publish to: ' + p.topic.name)

    @pyqtSlot(name = 'now', result = 'QVariantMap')
    def now(self):
        now = rospy.Time.now()
        return {'secs': now.secs, 'nsecs': now.nsecs}

    @pyqtSlot(str, name = 'logdebug')
    def logdebug(self, s):
        rospy.logdebug(s)

    @pyqtSlot(str, name = 'loginfo')
    def loginfo(self, s):
        rospy.loginfo(s)

    @pyqtSlot(str, name = 'logwarn')
    def logwarn(self, s):
        rospy.logwarn(s)

    @pyqtSlot(str, name = 'logerr')
    def logerr(self, s):
        rospy.logerr(s)

    @pyqtSlot(str, name = 'logfatal')
    def logfatal(self, s):
        rospy.logfatal(s)
