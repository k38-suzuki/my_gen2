/**
   Gen2 Cartesian Controller
   @author Kenta Suzuki
*/

#include <cnoid/EigenUtil>
#include <cnoid/JointPath>
#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include <mutex>

#include <kinova_msgs/PoseVelocity.h>

using namespace cnoid;

class Gen2CartesianController : public SimpleController
{
    std::unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    kinova_msgs::PoseVelocity pose_velocity_msg;
    std::mutex velocityMutex;
    
    Body* ioBody;
    BodyPtr ikBody;
    Link* ikWrist;
    std::shared_ptr<JointPath> baseToWrist;
    VectorXd qref, qold, qref_old;
    double timeStep;

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

        timeStep = io->timeStep();

        subscriber = node->subscribe("/j2s7s300_driver/in/cartesian_velocity", 2, &Gen2CartesianController::velocityCallback, this);

        return true;
    }

    void velocityCallback(const kinova_msgs::PoseVelocity& msg)
    {
        std::lock_guard<std::mutex> lock(velocityMutex);
        pose_velocity_msg = msg;
    }

    virtual bool control() override
    {
        kinova_msgs::PoseVelocity pose_velocity;
        {
            std::lock_guard<std::mutex> lock(velocityMutex);
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
        Matrix3 rot = rotFromRpy(angular * timeStep);

        VectorXd p(6);
        p.head<3>() = ikWrist->p() + ikBody->rootLink()->R() * (linear * timeStep);
        p.tail<3>() = rpyFromRot(ikWrist->R() * rot);

        Isometry3 T;
        T.linear() = rotFromRpy(Vector3(p.tail<3>()));
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
            double dq = (q - qold[i]) / timeStep;
            double dq_ref = (qref[i] - qref_old[i]) / timeStep;

            joint->dq_target() = dq_ref;
            qold[i] = q;
        }
        qref_old = qref;

        return true;
    }

    virtual void stop() override
    {
        subscriber.shutdown();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Gen2CartesianController)
