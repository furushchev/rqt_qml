from python_qt_binding.QtCore import qDebug
from PyQt5.QtQml import qmlRegisterType, QQmlExtensionPlugin
from rqtqml import Ros, Topic, Subscriber, Publisher

class RosPlugin(QQmlExtensionPlugin):
    def registerTypes(self, uri):
        qmlRegisterType(Ros, "Ros", 1, 0, "Ros")
        qmlRegisterType(Topic, "Ros", 1, 0, "Topic")
        qmlRegisterType(Subscriber, "Ros", 1, 0, "Subscriber")
        qmlRegisterType(Publisher, "Ros", 1, 0, "Publisher")
