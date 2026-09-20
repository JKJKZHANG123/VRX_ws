import QtQuick 2.0
import QtQuick.Controls 1.4

// Waypoint control panel for the gz_click_to_goal plugin.
// The C++ plugin is exposed as context property "plugin".
//  - "Mark Point" toggle: when ON, clicking the scene places a persistent
//    marker (red sphere) and records a waypoint.
//  - Single marker = single nav goal; multiple markers = multi-point nav.
Rectangle {
  id: panel
  width: 220
  height: 140
  color: "#282C34"
  opacity: 0.9
  radius: 6

  Column {
    anchors.fill: parent
    anchors.margins: 8
    spacing: 6

    Text {
      text: "Waypoint Control"
      color: "white"
      font.pixelSize: 13
      font.bold: true
    }

    // Mark-point toggle (first line, per user request)
    Button {
      id: markBtn
      width: parent.width
      text: plugin.markMode ? "Mark Point: ON" : "Mark Point: OFF"
      onClicked: plugin.markMode = !plugin.markMode
    }

    // Clear markers + list
    Button {
      width: parent.width
      text: "Clear Markers"
      onClicked: plugin.sendCommand("clear")
    }

    // Navigate accumulated waypoints
    Button {
      width: parent.width
      text: "Navigate"
      onClicked: plugin.sendCommand("navigate")
    }
  }
}
