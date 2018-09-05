void add_lane_frame(
  beacls::FloatVec& lane,
  beacls::FloatVec xs,
  FLOAT_TYPE lane_offset,
  FLOAT_TYPE lane_width,
  FLOAT_TYPE theta_offset,
  FLOAT_TYPE vehicle_width,
  size_t dim,
  beacls::FloatVec vrange,
  beacls::FloatVec y2range){


    if (dim == 0 || dim == 1){
      FLOAT_TYPE x_unit = (static_cast<float>(shape[0])-1)/(gmax[0]-gmin[0]);
      FLOAT_TYPE y_unit = (static_cast<float>(shape[1])-1)/(gmax[1]-gmin[1]);

      long unsigned int x_size, y_size, z_size, a_size, b_size, y, z, a, b;

      FLOAT_TYPE y_add = (y_unit*vehicle_width);
      FLOAT_TYPE x_add = (x_unit*vehicle_width*0.78);

      x_size = x2-x1+1;
      y_size = y2-y1+1;
      z_size = shape[2];
      a_size = shape[3];
      b_size = shape[4];

      beacls::IntegerVec y_addvec{(long unsigned int)((static_cast<float>(y_size)-1.)/2.-y_add/2.+0.5),(long unsigned int)((static_cast<float>(x_size)-1.)/2.+y_add/2.+0.5)};

      std::transform(xs.cbegin(), xs.cend(), lane.begin(),
      [lane_offset, lane_width, vehicle_width](const auto &xs_i) {
        return 2.5*(1. - std::pow(((xs_i - lane_offset)/((lane_width-vehicle_width)/2.)),2)); });

      }
    }
