int always(
    std::vector<beacls::FloatVec> datas,
    beacls::FloatVec alpha,
    FLOAT_TYPE tau1,
    FLOAT_TYPE tau2,
    helperOC::DynSysSchemeData* schemeData, 
    beacls::FloatVec tau,
    helperOC::HJIPDE_extraArgs extraArgs){

const FLOAT_TYPE small = 1e-3;
const size_t numel = schemeData->get_grid()->get_numel();

  std::vector<beacls::FloatVec> targets(tau.size());
  for (size_t i = 0; i < tau.size(); ++i) {
    targets[i].assign(numel, 100.);
    if (tau[i] > tau1 - small && tau[i] < tau2 + small) { 
      // avoid alpha if tau1 < tau < tau2
     std::copy(alpha.begin(), alpha.end(), targets[i].begin());
    }
  }
  
  schemeData->uMode = helperOC::DynSys_UMode_Max;
  schemeData->dMode = helperOC::DynSys_DMode_Min;

  helperOC::HJIPDE_extraOuts extraOuts;

  extraArgs.targets = targets;

  helperOC::HJIPDE* hjipde;
  hjipde = new helperOC::HJIPDE();

  beacls::FloatVec tau_out;

  hjipde->solve(datas, tau_out, extraOuts, targets, tau, schemeData, 
      helperOC::HJIPDE::MinWithType_None, extraArgs);

  if (hjipde) delete hjipde;
  return 0;
}