void add_lane_disturbance_v2(
  beacls::FloatVec& lane,
  const std::vector<size_t> shape,
  beacls::FloatVec range,
  beacls::FloatVec gmin,
  beacls::FloatVec gmax,
  FLOAT_TYPE vehicle_width,
  size_t dim){

    FLOAT_TYPE x_unit = (static_cast<float>(shape[0])-1)/(gmax[0]-gmin[0]);
    FLOAT_TYPE y_unit = (static_cast<float>(shape[1])-1)/(gmax[1]-gmin[1]);
    FLOAT_TYPE b_unit = (static_cast<float>(shape[4])-1)/(gmax[4]-gmin[4]);
    long unsigned int x1 = (long unsigned int)(x_unit*(range[0]-gmin[0])+0.5);
    long unsigned int x2 = (long unsigned int)(x_unit*(range[1]-gmin[0])+0.5);
    beacls::IntegerVec x_index{x1,x2};
    long unsigned int y1 = (long unsigned int)(y_unit*(range[2]-gmin[1])+0.5);
    long unsigned int y2 = (long unsigned int)(y_unit*(range[3]-gmin[1])+0.5);
    beacls::IntegerVec y_index{y1,y2};

    long unsigned int x_size, y_size, z_size, a_size, b_size, y, z, a, b, b2y, yz_unit;

    FLOAT_TYPE y_add = (y_unit*vehicle_width*0.78);
    // printf("y_unit=%f\n",y_unit);
    // printf("y_add=%f\n",y_add);
    FLOAT_TYPE x_add = (x_unit*vehicle_width);
    // printf("x_unit=%f\n",x_unit);
    // printf("x_add=%f\n",x_add);

    beacls::IntegerVec n;
    x_size = x2-x1+1;
    y_size = y2-y1+1;
    z_size = shape[2];
    a_size = shape[3];
    b_size = shape[4];

    beacls::IntegerVec b2y_index;
    beacls::IntegerVec y_addvec{0,0};
    beacls::IntegerVec x_addvec{(long unsigned int)((static_cast<float>(x_size)-1.)/2.-x_add/2.+0.5),(long unsigned int)((static_cast<float>(x_size)-1.)/2.+x_add/2.+0.5)};
    beacls::IntegerVec b_addvec{(long unsigned int)((range[2]-gmin[4])*b_unit+0.5),(long unsigned int)((range[3]-gmin[4])*b_unit+0.5)};

    FLOAT_TYPE by_unit = static_cast<float>(b_addvec[1]-b_addvec[0]+1)/y_size;
    printf("b_addvec_1:%lu\n",b_addvec[0]);
    printf("b_addvec_1:%f\n",(range[2]-gmin[4])*b_unit);
    printf("range_2:%f\n",range[2]);
    printf("gmin_4:%f\n",gmin[4]);
    printf("b_unit:%f\n",b_unit);

    printf("b_addvec_2:%lu\n",b_addvec[1]);
    // printf("b2y:%f\n",b_unit);
    // for (b = b_addvec[0]; b < b_addvec[1]+1; ++b) {
    //   b2y = (long unsigned int)(b/by_unit+0.5);
    //   printf("b2y:%lu,%lu\n",b,b2y);
    //   b2y_index.push_back(b2y);
    // }

    for (b = b_addvec[0]; b < b_addvec[1]; ++b) {
      for (a = 0; a < a_size; ++a) {
        for(z = 0; z < z_size; ++z) {
          y_addvec[0] = (long unsigned int)((b-b_addvec[0])/by_unit-y_add/2.+0.5);
          y_addvec[1] = (long unsigned int)((b-b_addvec[0])/by_unit+y_add/2.+0.5);

          if (y_addvec[0]<0){
            y_addvec[0] == 0;
          }
          if (y_addvec[1]>b_size-1){
            y_addvec[1] == b_size-1;
          }

          for (y = y_addvec[0]; y < y_addvec[1]+1; ++y){
            n = {b*a_size*z_size*y_size*x_size + a*z_size*y_size*x_size + z*y_size*x_size + y*x_size + x_addvec[0], b*a_size*z_size*y_size*x_size + a*z_size*y_size*x_size + z*y_size*x_size + y*x_size + x_addvec[1]};
            std::fill(lane.begin()+n[0],lane.begin()+n[1]+1,-10.);
}
        }
      }
    }
  }
