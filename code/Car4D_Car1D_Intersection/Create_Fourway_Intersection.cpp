#define _USE_MATH_DEFINES
#include <levelset/levelset.hpp>
#include <helperOC/helperOC.hpp>
#include <helperOC/DynSys/DynSys/DynSysSchemeData.hpp>
#include "Car4D_Car1D.cpp"

#include <cmath>
#include <numeric>
#include <functional>
#include <cfloat>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cstring>
#include "fourway_intersection.cpp"
#include "until.cpp"
#include "def_extraArgs.cpp"

int main(int argc, char *argv[])
{
	bool dump_file = true;
	if (argc >= 2) {
		dump_file = (atoi(argv[1]) == 0) ? false : true;
	}

//!< Compute reachable set
	const FLOAT_TYPE tMax = 3;
	const FLOAT_TYPE dt = tMax/12.;
	beacls::FloatVec tau = generateArithmeticSequence<FLOAT_TYPE>(0., dt, tMax);

//!< Plane parameters
	const FLOAT_TYPE wMax = (FLOAT_TYPE)1;
	const beacls::FloatVec vrange{ (FLOAT_TYPE)0, (FLOAT_TYPE)5 };
	const beacls::FloatVec arange{ (FLOAT_TYPE)0, (FLOAT_TYPE)3 }; //may need to be changed if considering acceleration
	const beacls::FloatVec dMax{ (FLOAT_TYPE)0, (FLOAT_TYPE)0, (FLOAT_TYPE)5 };

	const FLOAT_TYPE inf = std::numeric_limits<FLOAT_TYPE>::infinity();
  const beacls::IntegerVec pdDim{2};

// Grid Target and obstacle
	bool accel = true;
  const beacls::FloatVec initState{(FLOAT_TYPE)0, (FLOAT_TYPE)25,
			(FLOAT_TYPE)(270 * M_PI / 180), (FLOAT_TYPE)15, (FLOAT_TYPE)0};

  const beacls::FloatVec
		gmin{(FLOAT_TYPE)(-20), (FLOAT_TYPE)(-20), (FLOAT_TYPE)0, (FLOAT_TYPE)0, (FLOAT_TYPE)-12};

  const beacls::FloatVec
    gmax{(FLOAT_TYPE)20, (FLOAT_TYPE)20, (FLOAT_TYPE)(2*M_PI),(FLOAT_TYPE)5,(FLOAT_TYPE)12};

  levelset::HJI_Grid* g;
  helperOC::Car4D_Car1D* p4D1D = new helperOC::Car4D_Car1D(initState, wMax, arange, dMax);
	if (accel) {
  	g = helperOC::createGrid(gmin, gmax,
				beacls::IntegerVec{20,20,20,20,20}, pdDim);
	}
	else {
		g = helperOC::createGrid(
			beacls::FloatVec{gmin[0], gmin[1], gmin[2]},
			beacls::FloatVec{gmax[0], gmax[1], gmax[2]}, beacls::IntegerVec{120,120,120},
			pdDim);
		}

		const size_t numel = g->get_numel();
		const size_t num_dim = g->get_num_of_dimensions();

/* Define parameters for the until operator
  // satisfy beta if tau1 < tau < tau2 (reach beta)
  // satisfy alpha until beta is satisfied
*/
// Define tau1 and tau2
		FLOAT_TYPE tau1 = 0.;
		FLOAT_TYPE tau2 = 3.;

// Define alpha and beta
		int Command;

		beacls::FloatVec alpha;
		//alpha.assign(numel, -12.);
		Command = 0; // 0-Go straight; 1-Turn left
  	fourway_intersection(alpha,g,gmin,gmax,Command);

		beacls::FloatVec beta;
		//beta.assign(numel, -12.);
		Command = 1; // 0-Go straight; 1-Turn left
		fourway_intersection(beta,g,gmin,gmax,Command);

    // Dynamical system parameters
		helperOC::DynSysSchemeData* schemeData = new helperOC::DynSysSchemeData;
		// helperOC::HJIPDE_extraArgs extraArgs;

    // Target set and visualization
		//extraArgs.visualize = true;

		schemeData->set_grid(g);
		schemeData->dynSys = p4D1D;

		helperOC::HJIPDE_extraArgs extraArgs =
			def_extraArgs(accel, schemeData->dynSys);

		// std::vector<beacls::FloatVec> alpha_U_beta;
		// int resultU = until(alpha_U_beta, alpha, beta, tau1, tau2, schemeData, tau,
		// extraArgs);

    // std::vector<beacls::FloatVec> event_beta;
		// int resultF = eventually(event_beta, beta, tau1, tau2, schemeData, tau,
		// 	extraArgs);
		//
    // std::vector<beacls::FloatVec> always_alpha;
		// int resultG = always(always_alpha, alpha, tau1, tau2, schemeData, tau,
		// 	extraArgs);

	  // save mat file
		 std::string Car_test_filename("alpha.mat");
		 beacls::MatFStream* fs = beacls::openMatFStream(Car_test_filename,
		 beacls::MatOpenMode_Write);
		 if (dump_file) {
		 	beacls::IntegerVec Ns = g->get_Ns();
		 	g->save_grid(std::string("g"), fs);
			// if (!alpha_U_beta.empty()) {
			// 	save_vector_of_vectors(alpha_U_beta, std::string("alpha_U_beta"), Ns,
			// 		false, fs);
			// 	}

			if (!alpha.empty()) {
 			  save_vector(alpha, std::string("data"), Ns, false, fs);
 			  }

			// if (!beta.empty()) {
			// 	save_vector(beta, std::string("data"), Ns, false, fs);
			// 	}
		}
	  	beacls::closeMatFStream(fs);

		if (schemeData) delete schemeData;
		if (p4D1D) delete p4D1D;
		if (g) delete g;
		return 0;
	}
