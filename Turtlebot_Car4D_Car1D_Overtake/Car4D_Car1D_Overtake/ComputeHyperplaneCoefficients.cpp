#include <helperOC/ComputeGradients.hpp>
void ComputeHyperplaneCoefficients(
  std::vector<beacls::FloatVec>& a,
  std::vector<beacls::FloatVec>& b,
  const levelset::HJI_Grid* g,
  const std::vector<beacls::FloatVec>& data,
  const beacls::FloatVec& xs_vel,
  const beacls::FloatVec& xs_th,
  const beacls::FloatVec& v2Range,
  const FLOAT_TYPE dt)
  {

    const bool calculateTTRduringSolving = false;
    bool useCuda = false;
    int num_of_threads = 0;
    int num_of_gpus = 0;
    size_t line_length_of_chunk = 1;
    bool enable_user_defined_dynamics_on_gpu = true;

    helperOC::HJIPDE_extraArgs extraArgs;
      levelset::DelayedDerivMinMax_Type delayedDerivMinMax = levelset::DelayedDerivMinMax_Disable;
    extraArgs.execParameters.line_length_of_chunk = line_length_of_chunk;
    extraArgs.execParameters.calcTTR = calculateTTRduringSolving;
    extraArgs.execParameters.useCuda = useCuda;
    extraArgs.execParameters.num_of_gpus = num_of_gpus;
    extraArgs.execParameters.num_of_threads = num_of_threads;
    extraArgs.execParameters.delayedDerivMinMax = delayedDerivMinMax;
    extraArgs.execParameters.enable_user_defined_dynamics_on_gpu =
    enable_user_defined_dynamics_on_gpu;
    const helperOC::ExecParameters execParameters = extraArgs.execParameters;
    const beacls::UVecType execType = (execParameters.useCuda) ? beacls::UVecType_Cuda : beacls::UVecType_Vector;
    //bool computeGradients = new ComputeGradients(g, helperOC::ApproximationAccuracy_veryHigh, execType);

    helperOC::ComputeGradients* computeGradients;
    computeGradients = new helperOC::ComputeGradients(g, helperOC::ApproximationAccuracy_veryHigh, execType);
    beacls::FloatVec BRS_at_t;

    std::vector<beacls::FloatVec> derivC;
    std::vector<beacls::FloatVec> derivL;
    std::vector<beacls::FloatVec> derivR;
    std::vector<beacls::FloatVec> derivC_x;
    std::vector<beacls::FloatVec> derivC_y;
    std::vector<beacls::FloatVec> derivC_th;
    std::vector<beacls::FloatVec> derivC_vel;
    std::vector<beacls::FloatVec> derivC_y2;
    const size_t tau_length = data.size();

    //compute derivatives
    for(size_t i = 0; i < tau_length; ++i){
      BRS_at_t = data[i];
      computeGradients->operator()(derivC, derivL, derivR, g, data[i], data[i].size(), false, execParameters);
      derivC_x.push_back(derivC[0]);
      derivC_y.push_back(derivC[1]);
      derivC_th.push_back(derivC[2]);
      derivC_vel.push_back(derivC[3]);
      derivC_y2.push_back(derivC[4]);
    }

    //find coefficients
    FLOAT_TYPE dOpt, d_x, d_y, d_vel, d_th, d_y2, d_t, dJ, vel, th, x_dot, y_dot;
    for(size_t tau = 0; tau < tau_length; ++tau){
      for(size_t state = 0; state < derivC_th[tau].size(); ++state){
        //printf("tau=%lu\n",tau);
        //u3 = -P4/P3*u4 - P5/P3*y2
        if (tau == tau_length-1){
          d_t = 0;
        } else {
        d_t = (data[tau][state]-data[tau+1][state])/dt;
      }
        d_x = derivC_x[tau][state];
        d_y = derivC_y[tau][state];
        d_vel = derivC_vel[tau][state];
        d_th = derivC_th[tau][state];
        d_y2 = derivC_y2[tau][state];
        vel = xs_vel[state];
        th = xs_th[state];

        a[tau].push_back(d_vel/d_th);

        x_dot = vel*std::cos(th);
        y_dot = vel*std::sin(th);
        dOpt = (d_y2 >= 0) ? v2Range[1] : v2Range[0];
        dJ = d_t + d_x*x_dot + d_y*y_dot + d_y2*dOpt;

        b[tau].push_back(dJ/d_th);
      }
    }


    derivC_th.clear();
    derivC_vel.clear();
    derivC_y2.clear();

  }
