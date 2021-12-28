#include "control/hierarchical_qp.hpp"

namespace control
{
	HierarchicalQP::HierarchicalQP()
	{
		prog_.NewContinuousVariables(1,1,"c");
	}
}
