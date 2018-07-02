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
#include "eventually.cpp"
#include "def_extraArgs.cpp"
#include "always.cpp"

int main(int argc, char *argv[])
{
	bool dump_file = true;
	if (argc >= 2) {
		dump_file = (atoi(argv[1]) == 0) ? false : true;
	}



	//!< Plane parameters
	const FLOAT_TYPE wMax = (FLOAT_TYPE)1.;
	const beacls::FloatVec thetarange{ (FLOAT_TYPE)M_PI/8., (FLOAT_TYPE)7.*M_PI/8.};
	const beacls::FloatVec vrange{ (FLOAT_TYPE)0., (FLOAT_TYPE)0.09 };
	const beacls::FloatVec v2range{ (FLOAT_TYPE)0.15, (FLOAT_TYPE)0.2 };
	const beacls::FloatVec y2range{ (FLOAT_TYPE)-.6, (FLOAT_TYPE).8};
	const beacls::FloatVec arange{ (FLOAT_TYPE)-0.2, (FLOAT_TYPE)0.2 }; //may need to be changed if considering acceleration
	const beacls::FloatVec dMax{ (FLOAT_TYPE)0, (FLOAT_TYPE)0 };

	const FLOAT_TYPE inf = std::numeric_limits<FLOAT_TYPE>::infinity();
	const beacls::IntegerVec pdDim{};

	// Grid Target and obstacle
	bool accel = true;
	const beacls::FloatVec initState{(FLOAT_TYPE)0, (FLOAT_TYPE)0,
		(FLOAT_TYPE)(270 * M_PI / 180), (FLOAT_TYPE)0, (FLOAT_TYPE)0};

		const beacls::FloatVec
		gmin{(FLOAT_TYPE)(-0.4), (FLOAT_TYPE)(-0.8), (FLOAT_TYPE)0., (FLOAT_TYPE)-0.02, (FLOAT_TYPE)-0.8};

		const beacls::FloatVec
		gmax{(FLOAT_TYPE)0.4, (FLOAT_TYPE)1., (FLOAT_TYPE)(M_PI),(FLOAT_TYPE)0.11,(FLOAT_TYPE)1.};

		levelset::HJI_Grid* g;

		if (accel) {
			g = helperOC::createGrid(gmin, gmax,
				//beacls::IntegerVec{21,21,21,21,21}, pdDim);
				beacls::IntegerVec{17,19,9,14,19}, pdDim);
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


				// Define alpha and beta
				beacls::IntegerVec Command_set {0,1,2,3}; //0-alpha1(Eventually); 1-alpha2; 2-beta1; 3-beta2
				beacls::FloatVec alpha_1, beta_1, alpha_2, beta_2;
				int Command;

				Command = Command_set[0]; //0-alpha1(Eventually); 1-alpha2; 2-beta1; 3-beta2
				fourway_intersection(alpha_1,g,gmin,gmax,thetarange,vrange,y2range,Command);
				Command = Command_set[1]; //0-alpha1(Eventually); 1-alpha2; 2-beta1; 3-beta2
				fourway_intersection(alpha_2,g,gmin,gmax,thetarange,vrange,y2range,Command);
				Command = Command_set[2]; //0-alpha1(Eventually); 1-alpha2; 2-beta1; 3-beta2
				fourway_intersection(beta_1,g,gmin,gmax,thetarange,vrange,y2range,Command);
				Command = Command_set[3]; //0-alpha1(Eventually); 1-alpha2; 2-beta1; 3-beta2
				fourway_intersection(beta_2,g,gmin,gmax,thetarange,vrange,y2range,Command);

				FLOAT_TYPE tau1, tau2, tMax, dt;
				std::vector<beacls::FloatVec> event_beta, always_alpha, alpha_U_beta;
				// // Define tau for Eventually

					tau1 = 0.;
					tau2 = 15.;
					tMax = 15.;
					dt = 0.25;
					beacls::FloatVec tau = generateArithmeticSequence<FLOAT_TYPE>(0., dt, tMax);
					helperOC::Car4D_Car1D* p4D1D = new helperOC::Car4D_Car1D(initState, wMax, arange, dMax, v2range, y2range, dt);
					helperOC::DynSysSchemeData* schemeData = new helperOC::DynSysSchemeData;
					schemeData->set_grid(g);
					schemeData->dynSys = p4D1D;
					helperOC::HJIPDE_extraArgs extraArgs = def_extraArgs(accel, schemeData->dynSys);
					int resultF = until(event_beta, alpha_1, beta_1, tau1, tau2, schemeData, tau, extraArgs);


					tau1 = 0.;
					tau2 = 20.;
					tMax = 20.;
					beacls::FloatVec tau_2 = generateArithmeticSequence<FLOAT_TYPE>(0., dt, tMax);
					helperOC::Car4D_Car1D* p4D1D_2 = new helperOC::Car4D_Car1D(initState, wMax, arange, dMax, v2range, y2range, dt);
					helperOC::DynSysSchemeData* schemeData_2 = new helperOC::DynSysSchemeData;
					schemeData_2->set_grid(g);
					schemeData_2->dynSys = p4D1D_2;
					helperOC::HJIPDE_extraArgs extraArgs_2 = def_extraArgs(accel, schemeData_2->dynSys);
					int resultG = always(always_alpha, event_beta[event_beta.size()-1], tau1, tau2, schemeData_2, tau_2, extraArgs_2);


					tau1 = 0.;
					tau2 = 20.;
					tMax = 20.;
					dt = 0.25;
					beacls::FloatVec tau_3 = generateArithmeticSequence<FLOAT_TYPE>(0., dt, tMax);
					helperOC::Car4D_Car1D* p4D1D_3 = new helperOC::Car4D_Car1D(initState, wMax, arange, dMax, v2range, y2range, dt);
					helperOC::DynSysSchemeData* schemeData_3 = new helperOC::DynSysSchemeData;
					schemeData_3->set_grid(g);
					schemeData_3->dynSys = p4D1D_3;
					helperOC::HJIPDE_extraArgs extraArgs_3 = def_extraArgs(accel, schemeData->dynSys);
					int resultU = until(alpha_U_beta, alpha_2, beta_2, tau1, tau2, schemeData_3, tau_3, extraArgs_3);




				// save mat file
				std::string Car_test_filename("overtake_output.mat");
				beacls::MatFStream* fs = beacls::openMatFStream(Car_test_filename, beacls::MatOpenMode_Write);
				printf("always_alpha.size():%lu\n",always_alpha.size());
				if (dump_file) {
					beacls::IntegerVec Ns = g->get_Ns();
					g->save_grid(std::string("g"), fs);

					if (!event_beta.empty()) {
						save_vector_of_vectors(event_beta, std::string("event_beta"), Ns,
							false, fs);
						}

					if (!alpha_U_beta.empty()) {
						save_vector_of_vectors(alpha_U_beta, std::string("alpha_U_beta"), Ns,
							false, fs);
						}


					if (!always_alpha.empty()) {
						save_vector_of_vectors(always_alpha, std::string("always_alpha"), Ns,
							false, fs);
						}

					if (!alpha_1.empty()) {
						save_vector(alpha_1, std::string("alpha_1"), Ns, false, fs);
						}

					if (!beta_1.empty()) {
					  save_vector(beta_1, std::string("beta_1"), Ns, false, fs);
						}

					if (!alpha_2.empty()) {
						save_vector(alpha_2, std::string("alpha_2"), Ns, false, fs);
						}

					if (!beta_2.empty()) {
					  save_vector(beta_2, std::string("beta_2"), Ns, false, fs);
						}
				}
				beacls::closeMatFStream(fs);

				if (schemeData) delete schemeData;
				if (schemeData_2) delete schemeData_2;
				if (schemeData_3) delete schemeData_3;
				if (p4D1D) delete p4D1D;
				if (p4D1D_2) delete p4D1D_2;
				if (p4D1D_3) delete p4D1D_3;
				if (g) delete g;
				return 0;
			}
