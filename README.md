rqt_qml
=======

Qt QML Integrated GUI for Robots

## Installation

This package is built on the top of Qt5 and its python binding called PyQt5.


``` bash
sudo apt install \
    python-pyqt5.qtquick \
    qtdeclarative5-controls-plugin \
    qml-module-qtquick-dialogs
```

## Usage

1. Create GUI using QML. (See example in `qml` directory)
1. Load QML in RQT Plugin

    ```bash
    rqt_qml $(rospack find rqt_qml)/qml/simple.qml
    ```

The launch file additionally defines `LIBOVERLAY_SCROLLBAR` environment variable and set it to 0 to avoid QT GTK+ bug on Ubuntu. If you do not have any scrollbars in your QML graphical interface then the variable can be omitted.

## For Developers

### Create a new plugin

To add your own QML plugins (either written in Python or C++) use `<rqt_qml>` tag in export section of a `package.xml` in your package, for instance:

```xml
  <export>
    <rqt_qml plugins="${prefix}/my_qml_plugins_dir"/>
  </export>
```

If a relative path supplied to `plugins` attribute, it will be treated as relative to Qt plugins directory, for example:

```xml
  <export>
    <rqt_qml plugins="PyQt5" /> <!-- This may resolve to /usr/lib/x86_64-linux-gnu/qt5/plugins/PyQt5 --/>
  </export>
```

### Import your own QML

To add your own extension modules (*.qml files) use `imports` attribute.

```xml
  <export>
    <rqt_qml imports="${prefix}/my_qml_extensions_dir"/>
  </export>
```

Note, however, that a path provided with `imports` attribute is always treated as absolute.

## QML Plugin API

There are four components provided by the plugin:

 * `Ros`: top-level container for other components. It holds the reference to the actual ROS node and provides logging and time facilities.
 * `Subscriber`: subscribes to ROS topic and handles incomming messages. A topic to be used is defined with `Topic` component (see below).
 * `Publisher`: provides function to publish to ROS topic. Similarly to `Subscriber` uses `Topic` component.
 * `Topic`: just a container for topic specific properties, i.e. `name` and `type`.

See [qml/Chat.qml](qml/Chat.qml).

Documentation is [here](https://doc.qt.io/qt-5.6/qmlapplications.html).

### Ros

```qml
Ros {
  /* Allows access to logging and time facilities */
  id: my_ros
  
  /* Array of publishers and subscribers.
   * That is a default container, so the name can be omitted (see below)
   */
  objects: [
    Subscriber {
      ...
    }
    Publisher {
      ...
    }
  ]
  
  /* The publishers and subscribers defined like this
   * will be automatically added to the 'objects' container
   */
  Publisher {
    ...
  }
}

Button {
  text: "Log message to ROS"
  onClicked: {
    var now = my_ros.now() // remember time stamp
    
    /* Logging functions correspond to those defined in rospy */
    my_ros.logdebug('Sample debug message')
    my_ros.loginfo('Sample info message with user data: ' + JSON.stringify(now))
    my_ros.logwarn('Sample warn message')
    my_ros.logerr('Sample error message')
    my_ros.logfatal('Sample fatal message')
  }
}
```

### Subscriber

```qml
Subscriber {
  /* Not used at the moment */
  id: sub1
  
  /* Corresponds to ROS subscriber queue_size parameter */
  queueSize: 10
  
  /* Holds topic name and type (see below) */
  topic: Topic {
    ...
  }
  
  /* Handles 'message' signal emitted by the plugin backend. 
   * Passes 'msg' as sole parameter, which is a JavaScript object
   */
  onMessage: {
    // QML JavaScript code goes here
  }
}
```

### Publisher

```qml
Publisher {
  /* Allows access to 'publish()' function */
  id: pub1
  
  /* Corresponds to ROS publisher queue_size parameter */
  queueSize: 10
  
  /* Holds topic name and type (see below) */
  topic: Topic {
    ...
  }
}
...
Button {
  text: "Publish"
  onClicked: {
    var my_msg = {
      field1: 'value1',
      field2: 'value2',
      ...
    }
    ...
    // call publish() method of Publisher component
    pub1.publish(my_msg)
  }
}
```

### Topic

```qml
Topic {
  name: "/my_qml_topic"
  type: "std_msgs/Header"
}
```

## Author

Yuki Furuta <<me@furushchev.ru>>

Inspired from Boris Gromov <<boris.a.gromov@gmail.com>>
