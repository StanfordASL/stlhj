void add_lane_disturbance(
  beacls::FloatVec& lane,
  const std::vector<size_t> shape,
  beacls::FloatVec range,
  beacls::FloatVec gmin,
  beacls::FloatVec gmax,
  size_t dim){

    FLOAT_TYPE x_unit = (shape[0]-1)/(gmax[0]-gmin[0]);
    FLOAT_TYPE y_unit = (shape[1]-1)/(gmax[1]-gmin[1]);
    long unsigned int x1 = (long unsigned int)(x_unit*(range[0]-gmin[0])+0.5);
    long unsigned int x2 = (long unsigned int)(x_unit*(range[1]-gmin[0])+0.5);
    beacls::IntegerVec x_index{x1,x2};
    long unsigned int y1 = (long unsigned int)(y_unit*(range[2]-gmin[1])+0.5);
    long unsigned int y2 = (long unsigned int)(y_unit*(range[3]-gmin[1])+0.5);
    beacls::IntegerVec y_index{y1,y2};

    long unsigned int x_size, y_size, z_size, a_size, b_size, y, z, a, b, b2y, yz_unit;

    beacls::IntegerVec n;
    x_size = x2-x1+1;
    y_size = y2-y1+1;
    z_size = shape[2];
    a_size = shape[3];
    b_size = shape[4];
    FLOAT_TYPE by_unit = b_size/y_size;
    beacls::IntegerVec b2y_index;

    for (b = 0; b <= b_size-1; ++b) {
      b2y = (long unsigned int)(b/by_unit+0.5);
      b2y_index.push_back(b2y);
    }

    for (b = 0; b <= b_size-1; ++b) {
      for (a = 0; a <= a_size-1; ++a) {
        for(z = 0; z <= z_size-1; ++z) {
          y = b2y_index[b];
          n = {b*a_size*z_size*y_size*x_size + a*z_size*y_size*x_size + z*y_size*x_size + y*x_size + 0, b*a_size*z_size*y_size*x_size + a*z_size*y_size*x_size + z*y_size*x_size + y*x_size + (x_size-1)};
          std::fill(lane.begin()+n[0],lane.begin()+n[1],-5.);
        }
      }
    }
  }
