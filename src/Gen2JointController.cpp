/**
   Gen2 Joint Controller
   @author Kenta Suzuki
*/

#include <cnoid/EigenUtil>
#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include <mutex>

#include <kinova_msgs/JointVelocity.h>

using namespace cnoid;

class Gen2JointController : public SimpleController
{
    std::unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    kinova_msgs::JointVelocity joint_velocity_msg;
    std::mutex velocityMutex;
    
    Body* ioBody;

public:
    virtual bool configure(SimpleControllerConfig* config) override
    {
        node.reset(new ros::NodeHandle);
        return true;
    }
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        std::ostream& os = io->os();
        ioBody = io->body();

        for(int i = 0; i < ioBody->numJoints(); ++i) {
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JointVelocity);
            io->enableIO(joint);
        }

        subscriber = node->subscribe("/j2s7s300_driver/in/joint_velocity", 2, &Gen2JointController::velocityCallback, this);

        return true;
    }

    void velocityCallback(const kinova_msgs::JointVelocity& msg)
    {
        std::lock_guard<std::mutex> lock(velocityMutex);
        joint_velocity_msg = msg;
    }

    virtual bool control() override
    {
        kinova_msgs::JointVelocity joint_velocity;
        {
            std::lock_guard<std::mutex> lock(velocityMutex);
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
        subscriber.shutdown();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Gen2JointController)
