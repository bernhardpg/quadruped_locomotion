#pragma once

#include <drake/solvers/mathematical_program.h>
#include <Eigen/Core>

namespace control
{
	class HierarchicalQP
	{
		public:
			HierarchicalQP();
			~HierarchicalQP(){};

		private:
			drake::solvers::MathematicalProgram prog_;

	};
}
