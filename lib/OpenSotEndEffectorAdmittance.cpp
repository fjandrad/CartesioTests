#include "OpenSotEndEffectorAdmittance.h"

#include <boost/make_shared.hpp>

using namespace XBot::Cartesian;

OpenSotEndEffectorAdmittance::OpenSotEndEffectorAdmittance(TaskDescription::Ptr task,
                                               Context::ConstPtr context):
    OpenSotTaskAdapter(task, context)
{
    _ci_angmom = std::dynamic_pointer_cast<EndEffectorAdmittance>(task);

    if(!_ci_angmom) throw std::runtime_error("Provided task description "
                                             "does not have expected type 'EndEffectorAdmittance'");

}

TaskPtr OpenSotEndEffectorAdmittance::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _sot_angmom = boost::make_shared<EndEffectorAdmittanceSoT>(q,
                                                         const_cast<ModelInterface&>(*_model));

    return _sot_angmom;
}

bool OpenSotEndEffectorAdmittance::initialize(const OpenSoT::OptvarHelper& vars)
{
    if(vars.getAllVariables().size() > 0)
    {
        throw BadVariables("[OpenSotEndEffectorAdmittance] requires default variables definition");
    }

    return OpenSotTaskAdapter::initialize();
}

void OpenSotEndEffectorAdmittance::update(double time, double period)
{
    OpenSotTaskAdapter::update(time, period);

    _sot_angmom->setReference(_ci_angmom->getReference() * period);
}

CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotEndEffectorAdmittance, EndEffectorAdmittance)