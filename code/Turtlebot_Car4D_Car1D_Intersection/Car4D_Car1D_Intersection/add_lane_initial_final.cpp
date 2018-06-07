void add_lane_initial_final(
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
  int position){
    FLOAT_TYPE enhance;
  //printf("%i",position);
    if (position == 0) { //0-initial position, 1-final position
      enhance = 1.;
    } else if (position == 1){
      enhance = 4.;
    }


    if (dim == 0){
      //reset the default values
      std::fill(lane.begin(),lane.end(),0.);
      // for (int i = 0; i<lane.size(); ++i){
      //   if (lane[i] <= -10.){
      //     lane[i]=0.;
      //   }

      if (dim_long == 0) {
        std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
        [dim_long_offset, lane_width, vehicle_width](const auto &xs_i, const auto &lane_i) {
          return lane_i - std::pow((xs_i - dim_long_offset),2); });

        } else if (dim_long == 1) {
          std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
          [lane_offset, lane_width, vehicle_width, enhance](const auto &xs_i, const auto &lane_i) {
            return lane_i + 1.*enhance - std::pow(((xs_i - lane_offset)/((lane_width-vehicle_width)/2.)),2); });
          }

        } else if (dim == 1){

          if (dim_long == 0) {
            std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
            [lane_offset, lane_width, vehicle_width, enhance](const auto &xs_i, const auto &lane_i) {
              return lane_i + 1.*enhance - std::pow(((xs_i - lane_offset)/((lane_width-vehicle_width)/2.)),2); });

            } else if (dim_long == 1) {
              std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
              [dim_long_offset, lane_width, vehicle_width](const auto &xs_i, const auto &lane_i) {
                return lane_i - std::pow((xs_i - dim_long_offset),2); });
              }


        } else if (dim == 2) {

              std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
              [theta_offset](const auto &xs_i, const auto &lane_i) {
                return lane_i - 5.*std::pow((theta_diff(xs_i,theta_offset)/(M_PI)),2); });

                if (position == 1) {

                  std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
                  [theta_offset](const auto &xs_i, const auto &lane_i) {
                    return lane_i - 10.*std::pow((theta_diff(xs_i,theta_offset)/(M_PI)),2); });
                }

        } else if (dim == 3) {
                for (int i = 0; i <= lane.size()-1; ++i) {
                  if (xs[i]>vrange[1]){
                    lane[i] = -1.;
                    //lane[i] = lane[i]-std::pow((xs[i]-vrange[1])/vrange[1],2);
                  }
                }
              }
            }
