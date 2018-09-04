
#include <helperOC/ComputeGradients.hpp>

void ComputeGradient_C(
  std::vector<beacls::FloatVec>& derivC,
  std::vector<beacls::FloatVec>& derivL,
  std::vector<beacls::FloatVec>& derivR,
  levelset::HJI_Grid* g,
  std::vector<beacls::FloatVec>& data)
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
    computeGradients->operator()(derivC, derivL, derivR, g, data, data.size(), false, execParameters);
  }
