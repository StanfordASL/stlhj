
#include <helperOC/ComputeGradients.hpp>

void ComputeGradient_C(
  std::vector<beacls::FloatVec>& derivC_th_mat,
  std::vector<beacls::FloatVec>& derivC_vel_mat,
  std::vector<beacls::FloatVec>& derivC_y2_mat,
  const levelset::HJI_Grid* g,
  const std::vector<beacls::FloatVec>& data,
  const size_t tau_length)
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

    ComputeGradients* computeGradients;
    computeGradients = new ComputeGradients(g, helperOC::ApproximationAccuracy_veryHigh, execType);
    beacls::FloatVec BRS_at_t;

    std::vector<beacls::FloatVec> derivC;
    std::vector<beacls::FloatVec> derivL;
    std::vector<beacls::FloatVec> derivR;

    for(size_t i = 0; i < tau_length; ++i){
      BRS_at_t = data[i];
      computeGradients->operator()(derivC, derivL, derivR, g, data[i], data[i].size(), false, execParameters);
      derivC_th_mat.push_back(derivC[2]);
      derivC_vel_mat.push_back(derivC[3]);
      derivC_y2_mat.push_back(derivC[4]);
    }
  }
