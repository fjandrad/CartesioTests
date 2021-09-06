#ifndef OPENSOTENDEFFECTORADMITTANCE_H
#define OPENSOTENDEFFECTORADMITTANCE_H

#include "EndEffectorAdmittance.h"
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <OpenSoT/tasks/velocity/EndEffectorAdmittance.h>

using EndEffectorAdmittanceSoT = OpenSoT::tasks::velocity::EndEffectorAdmittance;

namespace XBot { namespace Cartesian {

class OpenSotEndEffectorAdmittance : public OpenSotTaskAdapter
{

public:

    OpenSotEndEffectorAdmittance(TaskDescription::Ptr task,
                           Context::ConstPtr context);

    virtual TaskPtr constructTask() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

private:

    EndEffectorAdmittanceSoT::Ptr _sot_angmom;
    EndEffectorAdmittance::Ptr _ci_angmom;


};

} }

#endif // OPENSOTENDEFFECTORADMITTANCE_H