#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <thread>

using namespace XBot::Cartesian;

int main(int argc, char **argv)
{
    // an option structure which is needed to make a model
    XBot::ConfigOptions xbot_cfg;

    // set the urdf and srdf path..
    xbot_cfg.set_urdf_path("/opt/xbot/share/cartesian_interface/examples_reemc/urdf/reemc.urdf");
    xbot_cfg.set_srdf_path("/opt/xbot/share/cartesian_interface/examples_reemc/srdf/reemc.srdf");

    // the following call is needed to generate some default joint IDs
    xbot_cfg.generate_jidmap();

    // some additional parameters..
    xbot_cfg.set_parameter("is_model_floating_base", true);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");

    // and we can make the model class
    auto model = XBot::ModelInterface::getModel(xbot_cfg);

    // initialize to a homing configuration
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();

    // before constructing the problem description, let us build a
    // context object which stores some information, such as
    // the control period
    const double dt = 0.01;
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(dt),
                model
            );

    // load the ik problem given a yaml file
    auto ik_pb_yaml = YAML::LoadFile("/opt/xbot/share/cartesian_interface/examples_reemc/configs/reemc_simple_stack.yaml");
    ProblemDescription ik_pb(ik_pb_yaml, ctx);

    // we are finally ready to make the CartesIO solver "OpenSot"
    auto solver = CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                       ik_pb, ctx
                                                       );

    auto larm_task=solver->getTask("left_hand");
    auto larm_cartesian = std::dynamic_pointer_cast<CartesianTask>(larm_task);                             
    int current_state = 0; // hand-crafted finite state machine!
    double time = 0;
    Eigen::VectorXd q, qdot, qddot;
    Eigen::Affine3d Tref;
    while(true)
    {
        if(current_state == 0) // here we command a reaching motion
        {
            std::cout << "Commanding left hand forward 0.3m in 3.0 secs" << std::endl;

            larm_cartesian->getPoseReference(Tref);
            Tref.translation()[0] += 0.3;
            double target_time = 3.0;
            larm_cartesian->setPoseTarget(Tref, target_time);

            current_state++;
        }

        if(current_state == 1) // here we check that the reaching started
        {
            if(larm_cartesian->getTaskState() == State::Reaching)
            {
                std::cout << "Motion started!" << std::endl;
                current_state++;
            }
        }

        if(current_state == 2) // here we wait for it to be completed
        {
            if(larm_cartesian->getTaskState() == State::Online)
            {
                Eigen::Affine3d T;
                larm_cartesian->getCurrentPose(T);

                std::cout << "Motion completed, final error is " <<
                            (T.inverse()*Tref).translation().norm() << std::endl;

                current_state++;
            }
        }

        if(current_state == 3) // here we wait the robot to come to a stop
        {
            std::cout << "qdot norm is " << qdot.norm() << std::endl;
            if(qdot.norm() < 1e-3)
            {
                std::cout << "Robot came to a stop, press ENTER to exit.. \n";
                std::cin.ignore();
                current_state++;
            }

        }

        if(current_state == 4) break;

        // update and integrate model state
        solver->update(time, dt);

        model->getJointPosition(q);
        model->getJointVelocity(qdot);
        model->getJointAcceleration(qddot);

        q += dt * qdot + 0.5 * std::pow(dt, 2) * qddot;
        qdot += dt * qddot;

        model->setJointPosition(q);
        model->setJointVelocity(qdot);
        model->update();


        // roughly loop at 100 Hz
        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        time += dt;
    }
}