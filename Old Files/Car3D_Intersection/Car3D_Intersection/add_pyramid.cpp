void add_pyramid(
  beacls::FloatVec& alpha,
  beacls::FloatVec xs,
  FLOAT_TYPE dim,
  const size_t numel,
  FLOAT_TYPE alpha_offset,
  FLOAT_TYPE length){

    if (dim == 0){
      //reset the default values
      for (int i = 0; i<alpha.size(); ++i){
        //if (alpha[i] <= -10.){
          alpha[i]=0.;
        //}
      }
      std::transform(xs.cbegin(), xs.cend(), alpha.begin(),
      [alpha_offset, length](const auto &xs_i) {
        return 1-std::pow(((xs_i - alpha_offset)/(length/2.)),2); });
      }
      else if (dim == 1) {

        beacls::FloatVec alpha_temp;
        alpha_temp.assign(alpha.size(), 0.);
        std::transform(xs.cbegin(), xs.cend(), alpha_temp.begin(),
        [alpha_offset, length](const auto &xs_i) {
          return 1-std::pow(((xs_i - alpha_offset)/(length/2.)),2); });

          std::transform(alpha_temp.cbegin(), alpha_temp.cend(), alpha.begin(), alpha.begin(),
          [](const auto &alpha_temp_i, const auto &alpha_i) {
            return 5*(-std::min(std::abs(alpha_temp_i),std::abs(alpha_i))); });
          }
        }
