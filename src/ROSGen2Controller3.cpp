/**
   ROS Gen2 Controller 3
   @author Kenta Suzuki
*/

#include <cnoid/EigenUtil>
#include <cnoid/JointPath>
#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include <mutex>
#include <kinova_msgs/PoseVelocity.h>

using namespace std;
using namespace cnoid;

class ROSGen2Controller3 : public SimpleController
{
    ros::NodeHandle node;
    ros::Subscriber gen2Subscriber;
    kinova_msgs::PoseVelocity pose_velocity_msg;
    std::mutex gen2Mutex;
    
    Body* ioBody;
    BodyPtr ikBody;
    Link* ikWrist;
    shared_ptr<JointPath> baseToWrist;
    VectorXd qref, qref_old, qold;
    double dt;

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

        ikBody = ioBody->clone();
        ikWrist = ikBody->link("WRIST_ORIGIN");
        Link* base = ikBody->rootLink();
        baseToWrist = JointPath::getCustomPath(base, ikWrist);
        base->p().setZero();
        base->R().setIdentity();

        const int nj = ioBody->numJoints();
        qold.resize(nj);
        for(int i = 0; i < nj; ++i) {
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JointVelocity);
            io->enableIO(joint);
            double q = joint->q();
            ikBody->joint(i)->q() = q;
            qold[i] = q;
        }

        baseToWrist->calcForwardKinematics();
        qref = qold;
        qref_old = qold;

        dt = io->timeStep();

        gen2Subscriber = node.subscribe("/j2s7s300_driver/in/cartesian_velocity", 2, &ROSGen2Controller3::gen2Callback, this);

        return true;
    }

    void gen2Callback(const kinova_msgs::PoseVelocity& msg)
    {
        std::lock_guard<std::mutex> lock(gen2Mutex);
        pose_velocity_msg = msg;
    }

    virtual bool control() override
    {
        kinova_msgs::PoseVelocity pose_velocity;
        {
            std::lock_guard<std::mutex> lock(mutex);
            pose_velocity = pose_velocity_msg;
        }

        double linear_x = pose_velocity_msg.twist_linear_x;
        double linear_y = pose_velocity_msg.twist_linear_y;
        double linear_z = pose_velocity_msg.twist_linear_z;
        double angular_x = pose_velocity_msg.twist_angular_x;
        double angular_y = pose_velocity_msg.twist_angular_y;
        double angular_z = pose_velocity_msg.twist_angular_z;

        Vector3 linear(-linear_x, linear_y, linear_z);
        Vector3 angular(angular_x, -angular_y, angular_z);
        Matrix3 rot = rotFromRpy(angular * dt);

        VectorXd p(6);
        p.head<3>() = ikWrist->p() + ikBody->rootLink()->R() * (linear * dt);
        p.tail<3>() = degree(rpyFromRot(ikWrist->R() * rot));

        Isometry3 T;
        T.linear() = rotFromRpy(radian(p.tail<3>()));
        T.translation() = p.head<3>();
        if(baseToWrist->calcInverseKinematics(T)) {
            for(int i = 0; i < baseToWrist->numJoints(); ++i) {
                Link* joint = baseToWrist->joint(i);
                qref[joint->jointId()] = joint->q();
            }
        }

        for(int i = 0; i < ioBody->numJoints(); ++i) {
            Link* joint = ioBody->joint(i);
            double q = joint->q();
            double dq = (q - qold[i]) / dt;
            double dq_ref = (qref[i] - qref_old[i]) / dt;

            joint->dq_target() = dq_ref;
            qold[i] = q;
        }
        qref_old = qref;

        return true;
    }

    virtual void stop() override
    {
        gen2Subscriber.shutdown();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ROSGen2Controller3)
