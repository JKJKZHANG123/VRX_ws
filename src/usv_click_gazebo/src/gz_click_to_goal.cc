// Gazebo GUI plugin: mark points + waypoint navigation.
//
// Redesigned per user request:
//   - Remove the OFF/direct-click mode entirely.
//   - First line toggle: "Mark Point: ON/OFF". When ON, clicking the scene
//     places a PERSISTENT visible marker (red sphere via gz MarkerManager on
//     /marker) AND records the point. Markers stay in the scene so the user
//     can see exactly where they clicked.
//   - Single marker = single navigation goal; multiple markers = multi-point
//     navigation (NavigateThroughPoses).
//   - Buttons: [Mark Point toggle] [Clear Markers] [Navigate]
//
// Click coordinates come from gz::gui::events::LeftClickToScene (needs
// ray-hittable geometry; the invisible pick_plane in the world file makes
// open water clickable).

#include <string>

#include <gz/common/Console.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Plugin.hh>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <QApplication>
#include <QEvent>
#include <QQmlContext>

namespace usv_click_gazebo
{

class GzClickToGoal : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(bool markMode READ markMode WRITE setMarkMode NOTIFY markModeChanged)

public:
  GzClickToGoal() = default;

  bool markMode() const { return this->markMode_; }
  void setMarkMode(bool _on)
  {
    if (this->markMode_ == _on)
      return;
    this->markMode_ = _on;
    emit markModeChanged();
    gzmsg << "mark mode: " << (_on ? "ON (click to place markers)"
                                   : "OFF (clicks do nothing)") << std::endl;
  }

signals:
  void markModeChanged();

public:
  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override
  {
    if (qApp)
      qApp->installEventFilter(this);

    // Expose this plugin object to the QML buttons as context property "plugin".
    if (auto ctx = this->Context())
    {
      ctx->setContextProperty("plugin", this);
    }

    // Markers are shown by gz-sim GUI's MarkerManager, which subscribes to /marker.
    this->pubMarker = this->node.Advertise<gz::msgs::Marker>("/marker");
    this->pubPoint = this->node.Advertise<gz::msgs::Vector3d>(
      "/gazebo/click/point");
    this->pubBoat = this->node.Advertise<gz::msgs::Vector3d>(
      "/gazebo/boat_pose");
    this->pubCmd = this->node.Advertise<gz::msgs::StringMsg>(
      "/gazebo/waypoint_cmd");
    if (!this->pubMarker || !this->pubPoint || !this->pubBoat || !this->pubCmd)
    {
      gzerr << "Failed to advertise gz topics" << std::endl;
      return;
    }
    gzmsg << "gz_click_to_goal: marker->/marker, point->/gazebo/click/point" << std::endl;

    if (!this->node.Subscribe(
        "/world/sydney_regatta/pose/info",
        &GzClickToGoal::OnPoseInfo, this))
    {
      gzerr << "Failed to subscribe /world/sydney_regatta/pose/info" << std::endl;
    }
  }

  void OnPoseInfo(const gz::msgs::Pose_V &_msg)
  {
    for (const auto &p : _msg.pose())
    {
      if (p.name() == "wamv")
      {
        gz::msgs::Vector3d b;
        b.set_x(p.position().x());
        b.set_y(p.position().y());
        b.set_z(p.position().z());
        this->pubBoat.Publish(b);
        return;
      }
    }
  }

  // QML buttons: send "clear" / "navigate" commands.
  Q_INVOKABLE void sendCommand(const QString &_cmd)
  {
    gz::msgs::StringMsg m;
    m.set_data(_cmd.toStdString());
    if (this->pubCmd)
      this->pubCmd.Publish(m);

    if (_cmd == "clear")
    {
      // Delete all markers from the scene (namespace "wamv_waypoints").
      this->DeleteAllMarkers();
      gzmsg << "waypoint cmd: clear" << std::endl;
    }
    else if (_cmd == "navigate")
    {
      gzmsg << "waypoint cmd: navigate" << std::endl;
    }
  }

  bool eventFilter(QObject *_obj, QEvent *_event) override
  {
    if (_event &&
        _event->type() == gz::gui::events::LeftClickToScene::kType)
    {
      auto ev = static_cast<gz::gui::events::LeftClickToScene *>(_event);
      auto pt = ev->Point();  // world-frame 3D point

      if (!this->markMode_)
      {
        gzmsg << "mark mode OFF - click ignored (turn on Mark Point first)"
              << std::endl;
        return gz::gui::Plugin::eventFilter(_obj, _event);
      }

      // 1) Show a persistent marker at the click (red sphere).
      this->PublishMarker(pt.X(), pt.Y(), pt.Z(), this->markerSeq_);
      this->markerSeq_++;

      // 2) Record the point for the ROS node (world->odom conversion there).
      gz::msgs::Vector3d msg;
      msg.set_x(pt.X());
      msg.set_y(pt.Y());
      msg.set_z(pt.Z());
      if (this->pubPoint)
        this->pubPoint.Publish(msg);

      gzmsg << "mark point #" << this->markerSeq_
            << " -> world (" << pt.X() << ", " << pt.Y() << ")" << std::endl;
    }
    return gz::gui::Plugin::eventFilter(_obj, _event);
  }

  // Show a red sphere marker at (x,y,z) via gz MarkerManager (/marker).
  void PublishMarker(double _x, double _y, double _z, uint64_t _id)
  {
    gz::msgs::Marker m;
    m.set_ns("wamv_waypoints");
    m.set_id(_id);
    m.set_action(gz::msgs::Marker::ADD_MODIFY);
    m.set_type(gz::msgs::Marker::SPHERE);
    m.set_visibility(gz::msgs::Marker::GUI);

    auto *pose = m.mutable_pose();
    pose->mutable_position()->set_x(_x);
    pose->mutable_position()->set_y(_y);
    pose->mutable_position()->set_z(_z + 0.5);  // hover slightly above water
    pose->mutable_orientation()->set_w(1.0);

    auto *scale = m.mutable_scale();
    scale->set_x(1.0);
    scale->set_y(1.0);
    scale->set_z(1.0);

    // Red color
    auto *mat = m.mutable_material();
    mat->mutable_ambient()->set_r(1.0); mat->mutable_ambient()->set_g(0.2f);
    mat->mutable_ambient()->set_b(0.2f); mat->mutable_ambient()->set_a(1.0);
    mat->mutable_diffuse()->set_r(1.0); mat->mutable_diffuse()->set_g(0.2f);
    mat->mutable_diffuse()->set_b(0.2f); mat->mutable_diffuse()->set_a(1.0);

    // gz-sim Harmonic's GUI Marker Manager serves the marker as a SERVICE
    // (/marker), not a topic subscription -- the /marker topic has zero
    // subscribers, so a topic publish never displays. Request() is exactly how
    // VRX's WaypointMarkers draws its waypoint markers. Keep the topic publish
    // as a no-op fallback for other gz versions.
    this->pubMarker.Publish(m);
    this->node.Request("/marker", m);
  }

  void DeleteAllMarkers()
  {
    gz::msgs::Marker m;
    m.set_ns("wamv_waypoints");
    m.set_action(gz::msgs::Marker::DELETE_ALL);
    this->pubMarker.Publish(m);
    this->node.Request("/marker", m);
    this->markerSeq_ = 0;
  }

private:
  gz::transport::Node node;
  gz::transport::Node::Publisher pubMarker;
  gz::transport::Node::Publisher pubPoint;
  gz::transport::Node::Publisher pubBoat;
  gz::transport::Node::Publisher pubCmd;
  bool markMode_ = false;
  uint64_t markerSeq_ = 0;
};

}  // namespace usv_click_gazebo

GZ_ADD_PLUGIN(
  usv_click_gazebo::GzClickToGoal,
  gz::gui::Plugin)

// AUTOMOC requirement: Q_OBJECT in a .cc file -> include the generated moc.
#include "gz_click_to_goal.moc"
