#define _USE_MATH_DEFINES
#include <levelset/levelset.hpp>
#include <helperOC/helperOC.hpp>
#include <helperOC/DynSys/DynSys/DynSysSchemeData.hpp>
#include <helperOC/DynSys/Plane/Plane.hpp>
#include <helperOC/DynSys/Plane4D/Plane4D.hpp>
#include <cmath>
#include <numeric>
#include <functional>
#include <cfloat>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cstring>
#include "until.cpp"
#include "eventually.cpp"
#include "always.cpp"
#include "def_extraArgs.cpp"

/**
@brief Tests the Plane class by computing a reachable set and then computing the optimal trajectory from the reachable set.
*/
int main(int argc, char *argv[])
{
	bool dump_file = true;
	if (argc >= 2) {
		dump_file = (atoi(argv[1]) == 0) ? false : true;
	}

//!< Compute reachable set
	const FLOAT_TYPE tMax = 5;
	const FLOAT_TYPE dt = 0.25;
	beacls::FloatVec tau = generateArithmeticSequence<FLOAT_TYPE>(0., dt, tMax);

//!< Plane parameters
	const FLOAT_TYPE wMax = (FLOAT_TYPE)1;
	const beacls::FloatVec vrange{ (FLOAT_TYPE)10, (FLOAT_TYPE)15 };
	const beacls::FloatVec arange{ (FLOAT_TYPE)0, (FLOAT_TYPE)5 };
	const beacls::FloatVec dMax{ (FLOAT_TYPE)0, (FLOAT_TYPE)0 };

	const FLOAT_TYPE inf = std::numeric_limits<FLOAT_TYPE>::infinity();
  const beacls::IntegerVec pdDim{2};

// Grid Target and obstacle
	bool accel = false;
  const beacls::FloatVec initState{(FLOAT_TYPE)0, (FLOAT_TYPE)25,
			(FLOAT_TYPE)(270 * M_PI / 180), (FLOAT_TYPE)15};

  const beacls::FloatVec
    gmin{(FLOAT_TYPE)(-75), (FLOAT_TYPE)(-75), (FLOAT_TYPE)0, (FLOAT_TYPE)5};

  const beacls::FloatVec
    gmax{(FLOAT_TYPE)75, (FLOAT_TYPE)75, (FLOAT_TYPE)(2*M_PI),(FLOAT_TYPE)25};

  levelset::HJI_Grid* g;
  helperOC::Plane* p3D = new helperOC::Plane(
  	beacls::FloatVec{initState[0], initState[1], initState[2]},
  	wMax, vrange, dMax);
  helperOC::Plane4D* p4D = new helperOC::Plane4D(initState, wMax, arange, dMax);

	if (accel) {
  	g = helperOC::createGrid(gmin, gmax,
				beacls::IntegerVec{15,15,15,15}, pdDim);
	}
	else {
		g = helperOC::createGrid(
			beacls::FloatVec{gmin[0], gmin[1], gmin[2]},
			beacls::FloatVec{gmax[0], gmax[1], gmax[2]}, beacls::IntegerVec{35,35,25},
			pdDim);
		}

		beacls::FloatVec alpha, beta;

		const size_t numel = g->get_numel();
		const size_t num_dim = g->get_num_of_dimensions();

/* Define parameters for the until operator
  // satisfy beta if tau1 < tau < tau2 (reach beta)
  // satisfy alpha until beta is satisfied
*/
// Define tau1 and tau2
		FLOAT_TYPE tau1 = 0.;
		FLOAT_TYPE tau2 = 2.;

// Define alpha and beta
		FLOAT_TYPE alpha_offset = -30.;
		FLOAT_TYPE beta_radius = 20.;
		FLOAT_TYPE beta_offset = 0.;

		alpha.assign(numel, 0.);
		beta.assign(numel, 0.);

		for (size_t dim = 0; dim < num_dim; ++dim) {
			const beacls::FloatVec &xs = g->get_xs(dim);

			if (dim == 1) { // alpha = x1 - alpha offset
				std::transform(xs.cbegin(), xs.cend(), alpha.begin(),
					  [alpha_offset](const auto &xs_i) {
						return xs_i - alpha_offset; });
			}

			if (dim == 0 || dim == 1) {
			  // beta = (x0-beta_offset)^2 + (x1-beta_offset)^2
				std::transform(xs.cbegin(), xs.cend(), beta.begin(), beta.begin(),
					  [beta_offset](const auto &xs_i, const auto &beta_i) {
						return beta_i - std::pow((xs_i - beta_offset), 2); });
			}
		}

    // beta = beta - beta_radius^2
		std::transform(beta.cbegin(), beta.cend(), beta.begin(),
			  [beta_radius](const auto &beta_i) {
				return beta_i + std::pow(beta_radius, 2); });

    // Dynamical system parameters
		helperOC::DynSysSchemeData* schemeData = new helperOC::DynSysSchemeData;
    schemeData->set_grid(g);

		if (accel) {
			schemeData->dynSys = p4D;
		}
		else {
			schemeData->dynSys = p3D;
		}

    helperOC::HJIPDE_extraArgs extraArgs =
      def_extraArgs(accel, schemeData->dynSys);

		std::vector<beacls::FloatVec> alpha_U_beta;
		int resultU = until(alpha_U_beta, alpha, beta, tau1, tau2, schemeData, tau,
			extraArgs);

    std::vector<beacls::FloatVec> event_beta;
		int resultF = eventually(event_beta, beta, tau1, tau2, schemeData, tau,
			extraArgs);

    std::vector<beacls::FloatVec> always_alpha;
		int resultG = always(always_alpha, alpha, tau1, tau2, schemeData, tau,
			extraArgs);

	  // save mat file


		if (dump_file) {
			std::string Car_test_filename("Car_test.mat");
			beacls::MatFStream* fs = beacls::openMatFStream(Car_test_filename,
				beacls::MatOpenMode_Write);
			beacls::IntegerVec Ns = g->get_Ns();

			g->save_grid(std::string("g"), fs);
			if (!alpha_U_beta.empty()) {
				save_vector_of_vectors(alpha_U_beta, std::string("alpha_U_beta"), Ns,
					false, fs);
			}

			if (!event_beta.empty()) {
				save_vector_of_vectors(event_beta, std::string("event_beta"), Ns,
					false, fs);
			}

			if (!always_alpha.empty()) {
				save_vector_of_vectors(always_alpha, std::string("always_alpha"), Ns,
					false, fs);
			}

			beacls::closeMatFStream(fs);
		}



		if (schemeData) delete schemeData;
		if (p3D) delete p3D;
		if (p4D) delete p4D;
		if (g) delete g;
		return 0;
	}
