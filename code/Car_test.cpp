#define _USE_MATH_DEFINES
#include <levelset/levelset.hpp>
#include <helperOC/helperOC.hpp>
#include <helperOC/DynSys/DynSys/DynSysSchemeData.hpp>
#include <helperOC/DynSys/Plane/Plane.hpp>
#include <cmath>
#include <numeric>
#include <functional>
#include <cfloat>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cstring>
#include "until.cpp"

/**
	@brief Tests the Plane class by computing a reachable set and then computing 
	the optimal trajectory from the reachable set.
	*/
int main(int argc, char *argv[])
{
	bool dump_file = false;
	if (argc >= 2) {
		dump_file = (atoi(argv[1]) == 0) ? false : true;
	}
	bool useTempFile = false;
	if (argc >= 3) {
		useTempFile = (atoi(argv[2]) == 0) ? false : true;
	}
	const bool keepLast = false;
	const bool calculateTTRduringSolving = false;
	levelset::DelayedDerivMinMax_Type delayedDerivMinMax = 
		levelset::DelayedDerivMinMax_Disable;
	if (argc >= 4) {
		switch (atoi(argv[3])) {
		default:
		case 0:
			delayedDerivMinMax = levelset::DelayedDerivMinMax_Disable;
			break;
		case 1:
			delayedDerivMinMax = levelset::DelayedDerivMinMax_Always;
			break;
		case 2:
			delayedDerivMinMax = levelset::DelayedDerivMinMax_Adaptive;
			break;
		}
	}

	bool useCuda = false;
	if (argc >= 5) {
		useCuda = (atoi(argv[4]) == 0) ? false : true;
	}
	int num_of_threads = 0;
	if (argc >= 6) {
		num_of_threads = atoi(argv[5]);
	}
	int num_of_gpus = 0;
	if (argc >= 7) {
		num_of_gpus = atoi(argv[6]);
	}
	size_t line_length_of_chunk = 1;
	if (argc >= 8) {
		line_length_of_chunk = atoi(argv[7]);
	}

	bool enable_user_defined_dynamics_on_gpu = true;
	if (argc >= 9) {
		enable_user_defined_dynamics_on_gpu = (atoi(argv[8]) == 0) ? false : true;
	}
	//!< Plane parameters
	const beacls::FloatVec initState{ (FLOAT_TYPE)100, (FLOAT_TYPE)75, 
		(FLOAT_TYPE)(220 * M_PI / 180) };
	const FLOAT_TYPE wMax = (FLOAT_TYPE)1.2;
	const beacls::FloatVec vrange{ (FLOAT_TYPE)1.1, (FLOAT_TYPE)1.3 };
	const beacls::FloatVec dMax{ (FLOAT_TYPE)0, (FLOAT_TYPE)0 };
	helperOC::Plane* pl = new helperOC::Plane(initState, wMax, vrange, dMax);

	const FLOAT_TYPE inf = std::numeric_limits<FLOAT_TYPE>::infinity();
	//!< Target and obstacle
	levelset::HJI_Grid* g = helperOC::createGrid(
		beacls::FloatVec{(FLOAT_TYPE)0, (FLOAT_TYPE)0, (FLOAT_TYPE)0}, 
		beacls::FloatVec{(FLOAT_TYPE)150, (FLOAT_TYPE)150, (FLOAT_TYPE)(2*M_PI)}, 
		beacls::IntegerVec{41,41,11});

	//!< Compute reachable set
	const FLOAT_TYPE tMax = 5;
	const FLOAT_TYPE dt = 0.1;
	beacls::FloatVec tau = generateArithmeticSequence<FLOAT_TYPE>(0., dt, tMax);

	// Dynamical system parameters
	helperOC::DynSysSchemeData* schemeData = new helperOC::DynSysSchemeData;
	schemeData->set_grid(g);
	schemeData->dynSys = pl;
	schemeData->uMode = helperOC::DynSys_UMode_Min;
	schemeData->dMode = helperOC::DynSys_DMode_Max;

  beacls::FloatVec alpha, beta;

  const size_t numel = g->get_numel();
  const size_t num_dim = g->get_num_of_dimensions();
  
  alpha.assign(numel, 0);
  beta.assign(numel, 0);

  for (size_t dim = 0; dim < num_dim; ++dim) {
  	const beacls::FloatVec &xs = g->get_xs(dim);
  	std::transform(xs.cbegin(), xs.cend(), alpha.begin(), alpha.begin(), 
  	               [](const auto &xs_i, const auto &alpha_i) {
                     return alpha_i + std::pow(xs_i, 2); });

  	std::transform(xs.cbegin(), xs.cend(), beta.begin(), beta.begin(), 
                   [](const auto &xs_i, const auto &beta_i) {
  	                 return beta_i + std::pow(xs_i - 2., 2); });
  }
 
	// Target set and visualization
	helperOC::HJIPDE_extraArgs extraArgs;

	extraArgs.visualize = true;
	extraArgs.plotData.plotDims = beacls::IntegerVec{ 1, 1, 0 };
	extraArgs.plotData.projpt = beacls::FloatVec{ pl->get_x()[2] };
	extraArgs.deleteLastPlot = true;
	extraArgs.fig_filename = "Plane_test_BRS";

	extraArgs.execParameters.line_length_of_chunk = line_length_of_chunk;
	extraArgs.execParameters.calcTTR = calculateTTRduringSolving;
	extraArgs.keepLast = keepLast;
	extraArgs.execParameters.useCuda = useCuda;
	extraArgs.execParameters.num_of_gpus = num_of_gpus;
	extraArgs.execParameters.num_of_threads = num_of_threads;
	extraArgs.execParameters.delayedDerivMinMax = delayedDerivMinMax;
	extraArgs.execParameters.enable_user_defined_dynamics_on_gpu = 
	  enable_user_defined_dynamics_on_gpu;

  FLOAT_TYPE tau1 = 2.;
  FLOAT_TYPE tau2 = 4.;

  std::vector<beacls::FloatVec> aub;
  until(aub, alpha, beta, tau1, tau2, schemeData, tau, extraArgs);
  
	printf("Done\n");

	if (schemeData) delete schemeData;
	if (pl) delete pl;
	if (g) delete g;
	return 0;
}

