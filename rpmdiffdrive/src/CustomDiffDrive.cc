/*
 * CustomDiffDrive.cc
 *
 * A simple custom differential drive plugin that subscribes to left and
 * right RPM topics, converts RPM to rad/s, and commands the corresponding
 * joints.
 *
 * SDF Parameters:
 *   - <left_wheel_joint>: Name of the left wheel joint.
 *   - <right_wheel_joint>: Name of the right wheel joint.
 *   - <wheel_base>: Distance between the wheels.
 *   - <wheel_radius>: Radius of the wheels.
 *   - <left_rpm_topic>: Topic for left RPM (default: "/left_rpm").
 *   - <right_rpm_topic>: Topic for right RPM (default: "/right_rpm").
 */

 #include "CustomDiffDrive.hh"

 #include <gz/msgs/float.pb.h>
 #include <gz/plugin/Register.hh>
 #include <gz/transport/Node.hh>
 #include <gz/sim/Model.hh>
 #include <gz/sim/components/JointVelocityCmd.hh>
 
 #include <cmath>
 #include <mutex>
 #include <string>
 
 using namespace gz;
 using namespace sim;
 using namespace systems;
 
 //------------------------------------------------------------------------------
 // Private implementation of CustomDiffDrive
 //------------------------------------------------------------------------------
 class gz::sim::systems::CustomDiffDrivePrivate
 {
   public:
     /// \brief Callback for left RPM subscription.
     void OnLeftRPM(const gz::msgs::Float &_msg)
     {
       std::lock_guard<std::mutex> lock(mutex);
       leftRPM = _msg.data();
     }
 
     /// \brief Callback for right RPM subscription.
     void OnRightRPM(const gz::msgs::Float &_msg)
     {
       std::lock_guard<std::mutex> lock(mutex);
       rightRPM = _msg.data();
     }
 
     // Communication node.
     transport::Node node;
 
     // SDF parameters.
     std::string leftWheelJoint;
     std::string rightWheelJoint;
     double wheelBase{1.0};
     double wheelRadius{0.2};
 
     // Topic names.
     std::string leftRpmTopic{"/left_rpm"};
     std::string rightRpmTopic{"/right_rpm"};
 
     // Latest RPM values.
     double leftRPM{0.0};
     double rightRPM{0.0};
 
     // Entities for joints.
     Entity leftJointEntity{kNullEntity};
     Entity rightJointEntity{kNullEntity};
 
     // Model interface.
     Model model{kNullEntity};
 
     // Mutex for protecting RPM values.
     std::mutex mutex;
 };
 
 //------------------------------------------------------------------------------
 // CustomDiffDrive class implementation
 //------------------------------------------------------------------------------
 CustomDiffDrive::CustomDiffDrive()
   : dataPtr(std::make_unique<CustomDiffDrivePrivate>())
 {
 }
 
 void CustomDiffDrive::Configure(const Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 EntityComponentManager &_ecm,
                                 EventManager &_eventMgr)
 {
   // Store the model.
   dataPtr->model = Model(_entity);
 
   // Retrieve SDF parameters.
   if (_sdf->HasElement("left_wheel_joint"))
     dataPtr->leftWheelJoint = _sdf->Get<std::string>("left_wheel_joint");
 
   if (_sdf->HasElement("right_wheel_joint"))
     dataPtr->rightWheelJoint = _sdf->Get<std::string>("right_wheel_joint");
 
   dataPtr->wheelBase = _sdf->Get<double>("wheel_base", dataPtr->wheelBase).first;
   dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius", dataPtr->wheelRadius).first;
 
   if (_sdf->HasElement("left_rpm_topic"))
     dataPtr->leftRpmTopic = _sdf->Get<std::string>("left_rpm_topic");
 
   if (_sdf->HasElement("right_rpm_topic"))
     dataPtr->rightRpmTopic = _sdf->Get<std::string>("right_rpm_topic");
 
   // Subscribe to RPM topics.
   dataPtr->node.Subscribe(dataPtr->leftRpmTopic,
                           &CustomDiffDrivePrivate::OnLeftRPM,
                           dataPtr.get());
   dataPtr->node.Subscribe(dataPtr->rightRpmTopic,
                           &CustomDiffDrivePrivate::OnRightRPM,
                           dataPtr.get());
 
   gzmsg << "CustomDiffDrive subscribed to RPM topics: ["
         << dataPtr->leftRpmTopic << "] and ["
         << dataPtr->rightRpmTopic << "]" << std::endl;
 
   // Lookup joint entities using the model interface.
   dataPtr->leftJointEntity = dataPtr->model.JointByName(_ecm, dataPtr->leftWheelJoint);
   dataPtr->rightJointEntity = dataPtr->model.JointByName(_ecm, dataPtr->rightWheelJoint);
 
   if (dataPtr->leftJointEntity == kNullEntity)
     gzerr << "Left wheel joint [" << dataPtr->leftWheelJoint << "] not found." << std::endl;
   if (dataPtr->rightJointEntity == kNullEntity)
     gzerr << "Right wheel joint [" << dataPtr->rightWheelJoint << "] not found." << std::endl;
 }
 
 void CustomDiffDrive::PreUpdate(const UpdateInfo &_info,
                                 EntityComponentManager &_ecm)
 {
   // If either joint is invalid, do nothing.
   if (dataPtr->leftJointEntity == kNullEntity || dataPtr->rightJointEntity == kNullEntity)
     return;
 
   double left_rpm, right_rpm;
   {
     std::lock_guard<std::mutex> lock(dataPtr->mutex);
     left_rpm = dataPtr->leftRPM;
     right_rpm = dataPtr->rightRPM;
   }
 
   // Convert RPM to radians per second.
   double leftVel = left_rpm * 2.0 * M_PI / 60.0;
   double rightVel = right_rpm * 2.0 * M_PI / 60.0;
 
   // Update the left joint velocity.
   if (_ecm.HasEntity(dataPtr->leftJointEntity))
   {
     auto jointVelComp = _ecm.Component<components::JointVelocityCmd>(dataPtr->leftJointEntity);
     if (!jointVelComp)
     {
       _ecm.CreateComponent(dataPtr->leftJointEntity,
                            components::JointVelocityCmd({leftVel}));
     }
     else
     {
       *jointVelComp = components::JointVelocityCmd({leftVel});
     }
   }
 
   // Update the right joint velocity.
   if (_ecm.HasEntity(dataPtr->rightJointEntity))
   {
     auto jointVelComp = _ecm.Component<components::JointVelocityCmd>(dataPtr->rightJointEntity);
     if (!jointVelComp)
     {
       _ecm.CreateComponent(dataPtr->rightJointEntity,
                            components::JointVelocityCmd({rightVel}));
     }
     else
     {
       *jointVelComp = components::JointVelocityCmd({rightVel});
     }
   }
 }
 
 //------------------------------------------------------------------------------
 // Plugin registration
 //------------------------------------------------------------------------------
 GZ_ADD_PLUGIN(CustomDiffDrive,
               System,
               CustomDiffDrive::ISystemConfigure,
               CustomDiffDrive::ISystemPreUpdate)
 
 GZ_ADD_PLUGIN_ALIAS(CustomDiffDrive, "custom::CustomDiffDrive")
 