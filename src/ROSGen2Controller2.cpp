/**
   ROS Gen2 Controller 2
   @author Kenta Suzuki
*/

#include <cnoid/EigenUtil>
#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include <mutex>
#include <kinova_msgs/JointVelocity.h>

using namespace std;
using namespace cnoid;

class ROSGen2Controller2 : public SimpleController
{
    ros::NodeHandle node;
    ros::Subscriber gen2Subscriber;
    kinova_msgs::JointVelocity joint_velocity_msg;
    std::mutex gen2Mutex;
    
    Body* ioBody;

public:

    virtual bool configure(SimpleControllerConfig* config) override
    {
        //config->sigChanged().connect();
        return true;
    }
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        ostream& os = io->os();
        ioBody = io->body();

        for(int i = 0; i < ioBody->numJoints(); ++i) {
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JointVelocity);
            io->enableIO(joint);
        }

        gen2Subscriber = node.subscribe("/j2s7s300_driver/in/joint_velocity", 2, &ROSGen2Controller2::gen2Callback, this);

        return true;
    }

    void gen2Callback(const kinova_msgs::JointVelocity& msg)
    {
        std::lock_guard<std::mutex> lock(gen2Mutex);
        joint_velocity_msg = msg;
    }

    virtual bool control() override
    {
        kinova_msgs::JointVelocity joint_velocity;
        {
            std::lock_guard<std::mutex> lock(mutex);
            joint_velocity = joint_velocity_msg;
        }

        double dq[7] = { 0.0 };
        dq[0] = joint_velocity.joint1;
        dq[1] = joint_velocity.joint2;
        dq[2] = joint_velocity.joint3;
        dq[3] = joint_velocity.joint4;
        dq[4] = joint_velocity.joint5;
        dq[5] = joint_velocity.joint6;
        dq[6] = joint_velocity.joint7;

        for(int i = 0; i < ioBody->numJoints(); ++i) {
            Link* joint = ioBody->joint(i);
            joint->dq_target() = radian(dq[i]);
        }

        return true;
    }

    virtual void stop() override
    {
        gen2Subscriber.shutdown();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ROSGen2Controller2)
