int eventually(
    std::vector<beacls::FloatVec>& datas,
    beacls::FloatVec beta,
    FLOAT_TYPE tau1,
    FLOAT_TYPE tau2,
    helperOC::DynSysSchemeData* schemeData,
    beacls::FloatVec tau,
    helperOC::HJIPDE_extraArgs extraArgs){

  const size_t numel = schemeData->get_grid()->get_numel();

  beacls::FloatVec alpha;
  alpha.assign(numel, 100.);

  int result = until(datas, alpha, beta, tau1, tau2, schemeData, tau,
      extraArgs);
}
