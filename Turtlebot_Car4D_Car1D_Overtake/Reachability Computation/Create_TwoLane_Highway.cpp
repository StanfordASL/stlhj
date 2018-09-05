#define _USE_MATH_DEFINES
#include <levelset/levelset.hpp>
#include <helperOC/helperOC.hpp>
#include <helperOC/DynSys/DynSys/DynSysSchemeData.hpp>
#include "Car4D_Car1D.hpp"
#include "Car4D_Car1D_mod.hpp"

#include <cmath>
#include <numeric>
#include <functional>
#include <cfloat>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cstring>
#include "twolane_highway.cpp"
#include "until.cpp"
#include "eventually.cpp"
#include "def_extraArgs.cpp"
#include "always.cpp"
#include "ComputeHyperplaneCoefficients.cpp"

int main(int argc, char *argv[])
{
	bool dump_file = true;
	if (argc >= 2) {
		dump_file = (atoi(argv[1]) == 0) ? false : true;
	}


	//!< Plane parameters
	const FLOAT_TYPE wMax = (FLOAT_TYPE)1.;
	const beacls::FloatVec thetarange{ (FLOAT_TYPE)40/180.*M_PI, (FLOAT_TYPE)140/180.*M_PI};
	const beacls::FloatVec vrange{ (FLOAT_TYPE)0., (FLOAT_TYPE)0.11 };
	const beacls::FloatVec v2range{ (FLOAT_TYPE)0.04, (FLOAT_TYPE)1. };
	//const beacls::FloatVec y2range{ (FLOAT_TYPE)-.8, (FLOAT_TYPE).9};
	const beacls::FloatVec y2range{ (FLOAT_TYPE)-.6, (FLOAT_TYPE).8};
	const beacls::FloatVec arange{ (FLOAT_TYPE)-0.2, (FLOAT_TYPE)0.2 }; //may need to be changed if considering acceleration
	const beacls::FloatVec dMax{ (FLOAT_TYPE)0, (FLOAT_TYPE)0 };

	const FLOAT_TYPE inf = std::numeric_limits<FLOAT_TYPE>::infinity();
	const beacls::IntegerVec pdDim{};

	// Grid Target and obstacle
	bool accel = true;
	const beacls::FloatVec initState{(FLOAT_TYPE)0, (FLOAT_TYPE)0,
		(FLOAT_TYPE)(90 * M_PI / 180), (FLOAT_TYPE)0, (FLOAT_TYPE)0};

		const beacls::FloatVec
		//gmin{(FLOAT_TYPE)(-0.4), (FLOAT_TYPE)(-1.), (FLOAT_TYPE)(22.5/180.*M_PI), (FLOAT_TYPE)-0.02, (FLOAT_TYPE)-1.};
		gmin{(FLOAT_TYPE)(-.8), (FLOAT_TYPE)(-1.), (FLOAT_TYPE)(15/180.*M_PI), (FLOAT_TYPE)-0.02, (FLOAT_TYPE)-1.};

		const beacls::FloatVec
		gmax{(FLOAT_TYPE).8, (FLOAT_TYPE)1., (FLOAT_TYPE)(165/180.*M_PI),(FLOAT_TYPE)0.12,(FLOAT_TYPE)1.};

		levelset::HJI_Grid* g;

		if (accel) {
			g = helperOC::createGrid(gmin, gmax,
				beacls::IntegerVec{17,21,13,15,21}, pdDim);
				//beacls::IntegerVec{5,5,5,5,5}, pdDim);
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
				beacls::FloatVec alpha_1, beta_1, alpha_2, beta_2, beta_2_top, beta_2_bot;
				int Command;

				Command = Command_set[0]; //0-alpha1(Eventually); 1-alpha2; 2-beta1; 3-beta2
				twolane_highway(alpha_1,g,gmin,gmax,thetarange,vrange,y2range,Command);
				Command = Command_set[1]; //0-alpha1(Eventually); 1-alpha2; 2-beta1; 3-beta2
				twolane_highway(alpha_2,g,gmin,gmax,thetarange,vrange,y2range,Command);
				Command = Command_set[2]; //0-alpha1(Eventually); 1-alpha2; 2-beta1; 3-beta2
				twolane_highway(beta_1,g,gmin,gmax,thetarange,vrange,y2range,Command);
				Command = Command_set[3]; //0-alpha1(Eventually); 1-alpha2; 2-beta1; 3-beta2
				twolane_highway(beta_2,g,gmin,gmax,thetarange,vrange,y2range,Command);

				Command = 4; //top of beta2
				twolane_highway(beta_2_top,g,gmin,gmax,thetarange,vrange,y2range,Command);
				Command = 5; //bottom of beta2
				twolane_highway(beta_2_bot,g,gmin,gmax,thetarange,vrange,y2range,Command);

				FLOAT_TYPE tau1, tau2, tMax, dt;
				std::vector<beacls::FloatVec> event_beta, always_alpha, alpha_U_beta, alpha_U_beta2;
				// // Define tau for Eventually

					tau1 = 0.;
					tau2 = 5.;
					tMax = 5.;
					dt = 0.25;
					beacls::FloatVec tau = generateArithmeticSequence<FLOAT_TYPE>(0., dt, tMax);
					helperOC::Car4D_Car1D* p4D1D = new helperOC::Car4D_Car1D(initState, wMax, arange, dMax, v2range, y2range, dt);
					helperOC::DynSysSchemeData* schemeData = new helperOC::DynSysSchemeData;
					schemeData->set_grid(g);
					schemeData->dynSys = p4D1D;
					helperOC::HJIPDE_extraArgs extraArgs = def_extraArgs(accel, schemeData->dynSys);
					int resultF = until(event_beta, alpha_1, beta_1, tau1, tau2, schemeData, tau, extraArgs);


					tau1 = 0.;
					tau2 = 25.;
					tMax = 25.;
					beacls::FloatVec tau_2 = generateArithmeticSequence<FLOAT_TYPE>(0., dt, tMax);
					helperOC::Car4D_Car1D* p4D1D_2 = new helperOC::Car4D_Car1D(initState, wMax, arange, dMax, v2range, y2range, dt);
					helperOC::DynSysSchemeData* schemeData_2 = new helperOC::DynSysSchemeData;
					schemeData_2->set_grid(g);
					schemeData_2->dynSys = p4D1D_2;
					helperOC::HJIPDE_extraArgs extraArgs_2 = def_extraArgs(accel, schemeData_2->dynSys);
					int resultG = always(always_alpha, event_beta[event_beta.size()-1], tau1, tau2, schemeData_2, tau_2, extraArgs_2);

					std::vector<beacls::FloatVec> slope_Coefficient(tau_2.size());
					std::vector<beacls::FloatVec> intercept_Coefficient(tau_2.size());
					const beacls::FloatVec &xs_vel = g->get_xs(3);
					const beacls::FloatVec &xs_th = g->get_xs(2);
					ComputeHyperplaneCoefficients(slope_Coefficient,intercept_Coefficient,g,always_alpha,xs_vel,xs_th,v2range,dt); //u3 = -P4/P3*u4 - P5/P3*y2
					printf("slope_Coefficient.size()=%lu\n",slope_Coefficient.size());
					printf("intercept_Coefficient.size()=%lu\n",intercept_Coefficient.size());
					// std::vector<const beacls::FloatVec*> Xq(11); //
					//
					// for (size_t dim = 0; dim < 5; ++dim) {
					// 		xs[dim] = &g->get_xs(dim);
					// 	}


					tau1 = 0.;
					tau2 = 25;
					tMax = 25.;
					dt = 0.25;
					beacls::FloatVec tau_3 = generateArithmeticSequence<FLOAT_TYPE>(0., dt, tMax);
					//helperOC::Car4D_Car1D_mod* p4D1D_3 = new helperOC::Car4D_Car1D_mod(initState,always_alpha,slope_Coefficient,intercept_Coefficient, wMax, arange, dMax, v2range, y2range, dt);
					helperOC::Car4D_Car1D* p4D1D_3 = new helperOC::Car4D_Car1D(initState, wMax, arange, dMax, v2range, y2range, dt);;
					helperOC::DynSysSchemeData* schemeData_3 = new helperOC::DynSysSchemeData;
					schemeData_3->set_grid(g);
					schemeData_3->dynSys = p4D1D_3;
					helperOC::HJIPDE_extraArgs extraArgs_3 = def_extraArgs(accel, schemeData->dynSys);
					int resultU = until(alpha_U_beta, alpha_2, beta_2, tau1, tau2, schemeData_3, tau_3, extraArgs_3);
					printf("made it out alive!");

					beacls::FloatVec tau_4 = generateArithmeticSequence<FLOAT_TYPE>(0., dt, tMax);
					//helperOC::Car4D_Car1D_mod* p4D1D_3 = new helperOC::Car4D_Car1D_mod(initState,always_alpha,slope_Coefficient,intercept_Coefficient, wMax, arange, dMax, v2range, y2range, dt);
					helperOC::Car4D_Car1D* p4D1D_4 = new helperOC::Car4D_Car1D(initState, wMax, arange, dMax, v2range, y2range, dt);;
					helperOC::DynSysSchemeData* schemeData_4 = new helperOC::DynSysSchemeData;
					schemeData_4->set_grid(g);
					schemeData_4->dynSys = p4D1D_4;
					helperOC::HJIPDE_extraArgs extraArgs_4 = def_extraArgs(accel, schemeData->dynSys);
					//int resultU2 = until(alpha_U_beta, alpha_2, beta_2, tau1, tau2, schemeData_4, tau_4, extraArgs_4);



				// save mat file
				std::string Car_test_filename("test_output.mat");
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

						if (!alpha_U_beta2.empty()) {
							save_vector_of_vectors(alpha_U_beta2, std::string("alpha_U_beta2"), Ns,
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

					if (!beta_2_top.empty()) {
						save_vector(beta_2_top, std::string("beta_2_top"), Ns, false, fs);
						}

					if (!beta_2_bot.empty()) {
						save_vector(beta_2_bot, std::string("beta_2_bot"), Ns, false, fs);
						}

					if (!slope_Coefficient.empty()) {
						save_vector_of_vectors(slope_Coefficient, std::string("slope_Coefficient"), Ns, false, fs);
					}

					if (!intercept_Coefficient.empty()) {
						save_vector_of_vectors(intercept_Coefficient, std::string("intercept_Coefficient"), Ns, false, fs);
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
