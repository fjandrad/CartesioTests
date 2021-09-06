#include <cartesian_interface/ros/RosClient.h>

using namespace XBot::Cartesian;

int main(int argc, char ** argv)
{
    RosClient cli;
    auto taskList = cli.getTaskList();
    std::cout << "task list # " << taskList.size() << std::endl;
    for (uint i=0 ; i < taskList.size() ; i++){
        std::cout << "task " << i<< " "  << taskList[i] << std::endl;
    }
    
    auto task = cli.getTask(taskList[2]);

    // ..use task
    task->setLambda(0.5);

    // ..convert to Cartesian task
    auto task_cartesian = std::dynamic_pointer_cast<CartesianTask>(task);

    // if conversion was successful...
    if(task_cartesian)
    {
        Eigen::Affine3d Ttgt;
        double time = 5.0;

        // fill Ttgt...
        task_cartesian->getPoseReference(Ttgt);
        Ttgt.translation().x() += 0.2;

        // command reaching motion
        task_cartesian->setPoseTarget(Ttgt, time);

        // sleep time
        const double sleep_dt = 0.1;

        // wait until motion started
        while(task_cartesian->getTaskState() == State::Online)
        {
            cli.update(0, 0);
            ros::Duration(sleep_dt).sleep();
        }

        std::cout << "Motion started" << std::endl;

        // wait until motion completed
        while(task_cartesian->getTaskState() == State::Reaching)
        {
            cli.update(0, 0);
            ros::Duration(sleep_dt).sleep();
        }

        std::cout << "Motion completed" << std::endl;
    }
}