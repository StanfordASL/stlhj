void add_lane_plain(
  beacls::FloatVec& lane,
  FLOAT_TYPE lane_width,
  beacls::FloatVec xs,
  beacls::FloatVec xs0,
  beacls::FloatVec xs1,
  beacls::FloatVec xs2,
  size_t dim,
  beacls::FloatVec vrange,
  beacls::FloatVec y2range,
  beacls::FloatVec theta_range,
  beacls::FloatVec range){

    FLOAT_TYPE r = (range[1]-range[0])/2.;
    FLOAT_TYPE theta_x, theta_y;


    if (dim == 0){
      // lane.assign(xs.size(),2.5);
      beacls::FloatVec lane_temp0, lane_temp1, theta_atan2;
      lane_temp0.assign(xs.size(),-5.);
      lane_temp1.assign(xs.size(),-5.);
      theta_atan2.assign(xs.size(),-5.);

      std::transform(xs0.cbegin(), xs0.cend(), lane_temp0.begin(),
      [lane_width,range](const auto &xs_i) {
        return 2.5*(1. - std::pow(((xs_i - (range[1]+range[0])/2.)/(lane_width/2.)),2)); });

        std::transform(xs1.cbegin(), xs1.cend(), lane_temp1.begin(),
        [lane_width,range](const auto &xs_i) {
          return 2.5*(1. - std::pow(((xs_i - (range[2]+range[3])/2.)/(lane_width/2.)),2)); });

          std::transform(xs0.cbegin(), xs0.cend(), xs1.cbegin(), lane.begin(),
          [lane_width,range,r](const auto &xs0_i,const auto &xs1_i) {
            return 2.5*(1.-std::pow(std::abs(std::sqrt(std::pow(xs0_i,2)+std::pow(xs1_i,2))-r)/(lane_width/2.),2)); });

            if (theta_range[0]<M_PI/4. || (theta_range[0]>M_PI*3./4. && theta_range[0]<5.*M_PI/4.)) {
              theta_x = theta_range[0];
              theta_y = theta_range[1];
            } else {
              theta_x = theta_range[1];
              theta_y = theta_range[0];
            }


           //theta penalty for x-path (dim=0)
            std::transform(xs2.cbegin(), xs2.cend(), lane_temp0.begin(), lane_temp0.begin(),
            [theta_y](const auto &xs_i, const auto &lane_i) {
              return (lane_i - std::pow(theta_diff(xs_i,theta_y),2)); });

              //theta penalty for y-path (dim=1)
              std::transform(xs2.cbegin(), xs2.cend(), lane_temp1.begin(), lane_temp1.begin(),
              [theta_x](const auto &xs_i, const auto &lane_i) {
                return (lane_i - std::pow(theta_diff(xs_i,theta_x),2)); });

                //theta penalty for turn (dim=0 & dim=1)
                std::transform(xs0.cbegin(), xs0.cend(), xs1.cbegin(), theta_atan2.begin(),
                [](const auto &xs0_i, const auto &xs1_i) {
                  return std::fmod((std::atan2(xs1_i,xs0_i)+2.*M_PI),2.*M_PI)+M_PI/2.  ; });


                  std::transform(xs2.cbegin(), xs2.cend(), theta_atan2.begin(), theta_atan2.begin(),
                  [](const auto &xs_i, const auto &theta_atan2_i) {
                    return std::pow(theta_diff(theta_atan2_i,xs_i),2)  ; });

                std::transform(theta_atan2.cbegin(), theta_atan2.cend(), lane.begin(), lane.begin(),
                [r](const auto &theta_atan2_i, const auto &lane_i) {
                  return (lane_i - theta_atan2_i); });

              //compare and keep max values
              std::transform(lane_temp0.cbegin(), lane_temp0.cend(), lane_temp1.begin(), lane_temp1.begin(),
              [](const auto &lane_temp_i, const auto &lane_i) {
                return std::max(lane_temp_i,lane_i); });

                std::transform(lane_temp1.cbegin(), lane_temp1.cend(), lane.begin(), lane.begin(),
                [](const auto &lane_temp_i, const auto &lane_i) {
                  return std::max(lane_temp_i,lane_i); });

                }

                else if (dim == 2){

                  // std::transform(xs.cbegin(), xs.cend(), lane.begin(),
                  // [theta_range](const auto &xs_i) {
                  //   return 2.*(1. - std::pow(((xs_i - (range[3]+range[2])/2.)/(lane_width/2.)),2)); });


                }
                else if (dim == 3) {
                  for (int i = 0; i <= lane.size()-1; ++i) {
                    if (xs[i]<vrange[0] || xs[i]>vrange[1]){
                      lane[i] = -10.;
                    }
                  }
                }
                else if (dim == 4) {
                  for (int i = 0; i <= lane.size()-1; ++i) {
                    if (xs[i]<y2range[0] || xs[i]>y2range[1]){
                      lane[i] = -10.;
                    }
                  }
                }
              }
