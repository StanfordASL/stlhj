void ComputeHyperplaneCoefficients(
  std::vector<beacls::FloatVec>& a,
  std::vector<beacls::FloatVec>& b,
  const levelset::HJI_Grid* g,
  const std::vector<beacls::FloatVec>& data,
  const beacls::FloatVec& v2Range)
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
    std::vector<beacls::FloatVec> derivC_th;
    std::vector<beacls::FloatVec> derivC_vel;
    std::vector<beacls::FloatVec> derivC_y2;
    const size_t tau_length = data.size();

    //compute derivatives
    for(size_t i = 0; i < tau_length; ++i){
      BRS_at_t = data[i];
      computeGradients->operator()(derivC, derivL, derivR, g, data[i], data[i].size(), false, execParameters);
      derivC_th.push_back(derivC[2]);
      derivC_vel.push_back(derivC[3]);
      derivC_y2.push_back(derivC[4]);
    }

    //find coefficients
    FLOAT_TYPE dOpt, d_vel, d_th, d_y2;
    for(size_t tau = 0; tau < tau_length; ++tau){
      for(size_t state = 0; state < derivC_th[tau].size(); ++state){
        //u3 = -P4/P3*u4 - P5/P3*y2
        d_vel = derivC_vel[tau][state];
        d_th = derivC_th[tau][state];
        d_y2 = derivC_y2[tau][state];

        a[tau].push_back(d_vel/d_th);

        dOpt = (d_y2 >= 0) ? v2Range[1] : v2Range[0];
        b[tau].push_back(dOpt*d_y2/d_th);
      }
    }

    derivC_th.clear();
    derivC_vel.clear();
    derivC_y2.clear();

  }
