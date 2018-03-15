std::vector<beacls::FloatVec> until(beacls::FloatVec alpha, 
  beacls::FloatVec beta, FLOAT_TYPE tau1, FLOAT_TYPE tau2, 
  helperOC::DynSysSchemeData* schemeData,   helperOC::HJIPDE_extraArgs extraArgs, 
  beacls::FloatVec tau){

const FLOAT_TYPE small = 1e-3;
beacls::FloatVec targets;

levelset::ShapeCylinder(beacls::IntegerVec{2}, beacls::FloatVec{75., 50., 0.}, 
  (FLOAT_TYPE)10).execute(g, targets[0]);

std::vector<beacls::FloatVec > obstacles(1);
obstacles[0].resize(obs1.size());
std::transform(obs1.cbegin(), obs1.cend(), obs2.cbegin(), obstacles[0].begin(), 
  std::ptr_fun<const FLOAT_TYPE&, const FLOAT_TYPE&>(std::min<FLOAT_TYPE>));

extraArgs.targets = beta;
extraArgs.obstacles = obstacles;


// solve HJIPDE
helperOC::HJIPDE* hjipde;
beacls::FloatVec tau_out;

std::vector<beacls::FloatVec> datas;
hjipde->solve(datas, tau_out, extraOuts, targets, tau, schemeData, 
  helperOC::HJIPDE::MinWithType_None, extraArgs);

}