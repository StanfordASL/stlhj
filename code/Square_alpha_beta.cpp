void Square_alpha_beta(
  beacls::FloatVec& alpha,
  beacls::FloatVec& beta,
  levelset::HJI_Grid* g){

    FLOAT_TYPE alpha_offset = 0.;
    FLOAT_TYPE beta_offset = 0.;
    FLOAT_TYPE length = 4.;

    const size_t numel = g->get_numel();
    const size_t num_dim = g->get_num_of_dimensions();

    alpha.assign(numel, 0.);
    beta.assign(numel, 0.);

    for (size_t dim = 0; dim < num_dim; ++dim) {
      const beacls::FloatVec &xs = g->get_xs(dim);


      if (dim == 0){
        std::transform(xs.cbegin(), xs.cend(), alpha.begin(),
        [alpha_offset, length](const auto &xs_i) {
          return 1- std::pow(((xs_i - alpha_offset)/length),2); });
        }
        else if (dim == 1) {
          beacls::FloatVec alpha_temp;
          alpha_temp.assign(numel, 0.);
          std::transform(xs.cbegin(), xs.cend(), alpha_temp.begin(),
          [alpha_offset, length](const auto &xs_i) {
            return 1- std::pow(((xs_i - alpha_offset)/length),2); });

            std::transform(alpha_temp.cbegin(), alpha_temp.cend(), alpha.begin(), alpha.begin(),
            [](const auto &alpha_temp_i, const auto &alpha_i) {
              return std::min(alpha_temp_i,alpha_i); });
            }


          }
        }
