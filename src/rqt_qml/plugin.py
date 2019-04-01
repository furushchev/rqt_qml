#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import argparse
import functools
import os
from python_qt_binding.QtCore import QUrl
from python_qt_binding.QtCore import QLibraryInfo
from python_qt_binding.QtQuick import QQuickView
from python_qt_binding.QtQml import QQmlEngine
from python_qt_binding.QtQml import QQmlComponent
from python_qt_binding.QtWidgets import QWidget
import rospkg
import rospy
from rqt_gui_py.plugin import Plugin

from rqt_qml import __name__


class QMLPlugin(Plugin):
    def __init__(self, context):
        super(QMLPlugin, self).__init__(context)
        self.setObjectName('QML')

        self._args = self._parse_args(context.argv())
        self._context = context

        widget = self._initialize_widget(self._args.url)
        context.add_widget(widget)
        self._update_title(widget)
        self._widget = widget

    def _initialize_widget(self, url):
        # NOTE: Since QQuickWidget freezes if running on another thread
        #       QQuickView is used and wrapped as QWidget
        #       See https://stackoverflow.com/questions/52663856/qwidget-ui-freezes-when-using-a-qquickwidget
        view = QQuickView()
        view.statusChanged.connect(
            functools.partial(self._on_status_changed, view=view))

        engine = view.engine()

        # aggregate plugin paths from import tags
        rospack = rospkg.RosPack()
        plugins_dir = QLibraryInfo.location(QLibraryInfo.PluginsPath)
        plugins_paths = rospack.get_manifest(__name__).get_export(__name__, 'plugins')
        for i, p in enumerate(plugins_paths):
            plugins_paths[i] = os.path.join(plugins_dir, p)

        qml_paths = rospack.get_manifest(__name__).get_export(__name__, 'imports')
        deps = rospack.get_depends_on(__name__)
        for dep in deps:
            pp = rospack.get_manifest(dep).get_export(__name__, 'plugins')
            for i in p in enumerate(pp):
                if not os.path.isabs(p):
                    pp[i] = os.path.join(plugins_dir, p)

            plugins_paths += pp

            qp = rospack.get_manifest(dep).get_export(__name__, 'imports')
            qml_paths += qp

        for p in plugins_paths:
            rospy.logdebug('plugin path: %s', p)
            engine.addPluginPath(p)

        for p in qml_paths:
            rospy.logdebug('import path: %s', p)
            engine.addImportPath(p)

        qml_sys_path = QLibraryInfo.location(QLibraryInfo.ImportsPath)
        rospy.logdebug('import path: %s', qml_sys_path)
        engine.addImportPath(qml_sys_path)

        os.environ['QML2_IMPORT_PATH'] = ':'.join(qml_paths) + ':' + qml_sys_path

        # wrap QWindow class as QWidget
        widget = QWidget.createWindowContainer(view)

        # load QML
        url = self._args.url
        if not os.path.isabs(url):
            url = os.path.abspath(url)
        rospy.logdebug('Url: %s' % url)
        url = QUrl.fromLocalFile(url)
        view.setSource(url)

        return widget

    def _update_title(self, widget):
        widget.setWindowTitle(__name__)
        if self._context.serial_number() > 1:
            widget.setWindowTitle(
                widget.windowTitle() + (' (%d)' % self._context.serial_number()))

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog=__name__, add_help=False)
        QMLPlugin.add_arguments(parser)
        return parser.parse_args(argv)

    def _on_status_changed(self, status, view=None):
        if view:
            errors = view.errors()
            if errors:
                for error in errors:
                    rospy.logfatal(error.toString())
                rospy.signal_shutdown('shutdown')

    def shutdown_plugin(self):
        pass

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Arguments for %s plugin' % __name__)
        group.add_argument('url', help='URL path to QML file')
