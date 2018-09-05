void add_lane_beta_2top(
  beacls::FloatVec& lane,
  beacls::FloatVec xs,
  FLOAT_TYPE lane_offset,
  FLOAT_TYPE lane_width,
  FLOAT_TYPE theta_offset,
  FLOAT_TYPE vehicle_width,
  size_t dim,
  beacls::FloatVec thetarange,
  beacls::FloatVec vrange,
  beacls::FloatVec y2range){

    FLOAT_TYPE enhance;
    enhance = 3.;

    if (dim == 0){
      //reset the default values
      std::fill(lane.begin(),lane.end(),0.);
      std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
      [lane_offset, lane_width, vehicle_width, enhance](const auto &xs_i, const auto &lane_i) {
        return enhance*(1-std::pow(((xs_i - lane_offset)/((lane_width-vehicle_width)/2.)),2)); });

      } else if (dim == 1) {
        beacls::FloatVec lane_temp = lane;

        std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
        [lane_width, vehicle_width, enhance](const auto &xs_i, const auto &lane_i) {
          return lane_i*(1-std::pow((xs_i - .6)/1.2,2)); });

        // std::transform(lane_temp.cbegin(), lane_temp.cend(), lane.begin(), lane.begin(),
        // [lane_width, vehicle_width, enhance](const auto &lane_temp_i, const auto &lane_i) {
        //   return std::max(lane_temp_i,lane_i); });

        } else if (dim == 2) {
          std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
          [theta_offset,enhance](const auto &xs_i, const auto &lane_i) {
            return lane_i*(1-std::pow((theta_diff(xs_i,theta_offset)/(5./18.*M_PI)),2)); });

            for (int i = 0; i <= lane.size()-1; ++i) {
              if (xs[i]<thetarange[0]-0.0001 || xs[i]>thetarange[1]+0.0001){
                lane[i] = -3.;
              }
            }
          } else if (dim == 3) {
            for (int i = 0; i <= lane.size()-1; ++i) {
              if (xs[i]<vrange[0]-0.0001 || xs[i]>vrange[1]+0.0001){
                lane[i] = -3.;
              }
            }
          } else if (dim == 4) {
            for (int i = 0; i <= lane.size()-1; ++i) {
              if (xs[i]<y2range[0]-0.0001 || xs[i]>y2range[1]+0.0001){
                lane[i] = -3.;}

              }
            }
          }
