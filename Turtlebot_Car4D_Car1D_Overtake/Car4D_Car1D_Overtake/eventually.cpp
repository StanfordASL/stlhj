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

  // const size_t num_dim = schemeData->get_grid()->get_num_of_dimensions();
  // FLOAT_TYPE alpha_offset = 20.;
  // for (size_t dim = 0; dim < num_dim; ++dim) {
  //   const beacls::FloatVec &xs = schemeData->get_grid()->get_xs(dim);

  //   if (dim == 0) { // alpha = x0 - alpha offset
  //     std::transform(xs.cbegin(), xs.cend(), alpha.begin(),
  //         [alpha_offset](const auto &xs_i) {
  //         return xs_i - alpha_offset; });
  //   }
  // }

  int result = until(datas, alpha, beta, tau1, tau2, schemeData, tau,
      extraArgs);
}
