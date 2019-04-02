import QtQuick 2.0
import QtQuick.Controls 1.0
import Ros 1.0

Rectangle {
  id: chat
  width: 320; height: 320
  color: "lightgray"

  Ros {
    id: ros

    Publisher {
      id: pub
      queueSize: 10
      topic: Topic {
        name: 'chatter'
        type: 'std_msgs/String'
      }
    }

    Subscriber {
      id: sub
      queueSize: 1
      topic: Topic {
        name: 'chatter'
        type: 'std_msgs/String'
      }
      onMessageChanged: {
        display.text += msg.data + '\n'
      }
    }
  }

  Item {
    id: input
    anchors.horizontalCenter: parent.horizontalCenter
    anchors.top: parent.top
    anchors.topMargin: 20

    TextField {
      id: textField
      anchors.right: send.left
      anchors.top: input.top
      placeholderText: 'Enter'
    }

    Button {
      id: send
      anchors.top: input.top
      text: 'Send'
      y: 30
      onClicked: pub.publish({'data': textField.text})
    }
  }

  Text {
    id: display
    anchors.top: input.bottom
    anchors.topMargin: 50
    anchors.leftMargin: 10
  }
}
