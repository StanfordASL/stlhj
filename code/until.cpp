int until(std::vector<beacls::FloatVec> datas, beacls::FloatVec alpha, 
  beacls::FloatVec beta, FLOAT_TYPE tau1, FLOAT_TYPE tau2, 
  helperOC::DynSysSchemeData* schemeData, beacls::FloatVec tau,
  helperOC::HJIPDE_extraArgs extraArgs){

const FLOAT_TYPE small = 1e-3;
const size_t numel = schemeData->get_grid()->get_numel();

// Initialize obstacle function from alpha (memory inefficient)

std::vector<beacls::FloatVec> obstacles(tau.size());
for (size_t i = 0; i < tau.size(); ++i) {
  obstacles[i].assign(numel, 0);
  if (tau[i] < tau1) { // satisfy alpha if tau < tau1
    std::copy(alpha.cbegin(), alpha.cend(), obstacles[i].begin());
  } 
  else {
    obstacles[i].assign(numel, -1.);
  }
}

// Initialize target function from beta (memory inefficient) 
std::vector<beacls::FloatVec> targets(tau.size());

for (size_t i = 0; i < tau.size(); ++i) {
  targets[i].assign(numel, 0);
  if (tau[i] > tau1 && tau[i] < tau2) { // satisfy beta if tau1 < tau < tau2
    std::copy(beta.begin(), beta.end(), targets[i].begin());
  } 
  else {
    targets[i].assign(numel, -1.);
  }
}

// Test targets and obstacles
std::vector<beacls::FloatVec > targets(1);
  levelset::ShapeCylinder(beacls::IntegerVec{ 2 }, 
    beacls::FloatVec{ 75., 50., 0. }, (FLOAT_TYPE)10).execute(g, targets[0]);

  beacls::FloatVec obs1, obs2;
  levelset::ShapeRectangleByCorner(beacls::FloatVec{ 300, 250, -inf }, 
    beacls::FloatVec{ 350, 300, inf }).execute(g, obs1);
  levelset::ShapeRectangleByCorner(beacls::FloatVec{ 5, 5, -inf }, 
    beacls::FloatVec{ 145, 145, inf }).execute(g, obs2);
  std::transform(obs2.cbegin(), obs2.cend(), obs2.begin(), 
    std::negate<FLOAT_TYPE>());

  std::vector<beacls::FloatVec > obstacles(1);
  obstacles[0].resize(obs1.size());
  std::transform(obs1.cbegin(), obs1.cend(), obs2.cbegin(), 
    obstacles[0].begin(), std::ptr_fun<const FLOAT_TYPE&, 
    const FLOAT_TYPE&>(std::min<FLOAT_TYPE>));

extraArgs.targets = targets;
extraArgs.obstacles = obstacles;

// solve HJIPDE
beacls::FloatVec tau_out;
helperOC::HJIPDE_extraOuts extraOuts;

helperOC::HJIPDE* hjipde;
hjipde->solve(datas, tau_out, extraOuts, targets, tau, schemeData, 
  helperOC::HJIPDE::MinWithType_None, extraArgs);

if (hjipde) delete hjipde;

return 1;
}