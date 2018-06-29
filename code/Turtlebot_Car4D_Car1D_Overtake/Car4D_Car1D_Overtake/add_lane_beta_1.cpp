void add_lane_beta_1(
  beacls::FloatVec& lane,
  beacls::FloatVec xs,
  FLOAT_TYPE lane_offset,
  FLOAT_TYPE lane_width,
  FLOAT_TYPE theta_offset,
  FLOAT_TYPE vehicle_width,
  int dim_long,
  FLOAT_TYPE dim_long_offset,
  size_t dim,
  beacls::FloatVec vrange,
  beacls::FloatVec y2range,
  int position){

    FLOAT_TYPE enhance;
    enhance = 2.;

    if (dim == 0){
      //reset the default values
      std::fill(lane.begin(),lane.end(),0.);
      std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
      [lane_offset, lane_width, vehicle_width, enhance](const auto &xs_i, const auto &lane_i) {
        return lane_i + enhance*(1-std::pow(((xs_i - lane_offset)/((lane_width-vehicle_width)/2.)),2)); });

      } else if (dim == 1) {
        std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
        [dim_long_offset, lane_width, vehicle_width, enhance](const auto &xs_i, const auto &lane_i) {
          return lane_i - std::pow((xs_i - 2.)/2.,2); });

        } else if (dim == 2) {
          std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
          [theta_offset,enhance](const auto &xs_i, const auto &lane_i) {
            return (lane_i - std::pow((theta_diff(xs_i,theta_offset)/(M_PI)),2)); });

          } else if (dim == 3) {
            for (int i = 0; i <= lane.size()-1; ++i) {
              if (xs[i]<vrange[0] || xs[i]>vrange[1]){
                lane[i] = -10.;
              }
            }
          } else if (dim == 4) {
            for (int i = 0; i <= lane.size()-1; ++i) {
              if (xs[i]<y2range[0] || xs[i]>y2range[1]){
                lane[i] = -10.;}

              }
            }
          }
