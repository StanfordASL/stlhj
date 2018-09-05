#include "add_lane.cpp"
#include "add_sq.cpp"
#include "get_subvector.cpp"
#include "update_lane.cpp"
void fourway_intersection(
  beacls::FloatVec& lane,
  levelset::HJI_Grid* g,
  beacls::FloatVec gmin,
  beacls::FloatVec gmax,
  int Command)
  {


    FLOAT_TYPE vehicle_width = 1.;
    FLOAT_TYPE lane_width = 4.;

    const size_t numel = g->get_numel();
    const size_t num_dim = g->get_num_of_dimensions();
    const std::vector<size_t> shape = g->get_shape();

    beacls::FloatVec lane_temp;
    beacls::FloatVec lane_temp_calc;

    lane.assign(numel, -12.);
    //lane_temp.assign(numel, -20.);
    //lane_temp_calc.assign(numel,0.);
    beacls::FloatVec xs_temp;
    beacls::FloatVec range;
    FLOAT_TYPE theta_offset;
    FLOAT_TYPE lane_offset;
    int enhance;

    enhance = 0; //1-enhance value function

    for (size_t dim = 0; dim < num_dim; ++dim) {

      if (dim == 0 || dim == 2){
        const beacls::FloatVec &xs = g->get_xs(dim);

        if (Command == 0){ //0-Go straight; 1-Turn left
          theta_offset = M_PI/2;
          range = {0.,4.,-12.,12.}; //{xmin,xmax,ymin,ymax}
          lane_offset = (range[1]+range[0])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,enhance);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

          theta_offset = 3.*M_PI/2.;
          range = {-4.,0.,-12.,12.}; //{xmin,xmax,ymin,ymax}
          lane_offset = (range[1]+range[0])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,enhance);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);
        }

        if (Command == 1){ //0-Go straight; 1-Turn left
          theta_offset = M_PI/2;
          range = {0.,4.,-12.,-4.}; //{xmin,xmax,ymin,ymax}
          lane_offset = (range[1]+range[0])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,enhance);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

          range = {0.,4.,4.,12.}; //{xmin,xmax,ymin,ymax}
          lane_offset = (range[1]+range[0])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,enhance);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

          theta_offset = 3.*M_PI/2.;
          range = {-4.,0.,-12.,-4.}; //{xmin,xmax,ymin,ymax}
          lane_offset = (range[1]+range[0])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,enhance);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

          range = {-4.,0.,4.,12.}; //{xmin,xmax,ymin,ymax}
          lane_offset = (range[1]+range[0])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,enhance);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);
        }

      }

      if (dim == 1 || dim == 2){
        const beacls::FloatVec &xs = g->get_xs(dim);

        if (Command == 0){
          theta_offset = M_PI;
          range = {-12.,-4.,0.,4.}; //{xmin,xmax,ymin,ymax}
          lane_offset = (range[3]+range[2])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,enhance);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

          range = {4.,12.,0.,4.}; //{xmin,xmax,ymin,ymax}
          lane_offset = (range[3]+range[2])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,enhance);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

          theta_offset = 0.;
          range = {-12.,-4.,-4.,0.}; //{xmin,xmax,ymin,ymax}
          lane_offset = (range[3]+range[2])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,enhance);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

          range = {4.,12.,-4.,0.}; //{xmin,xmax,ymin,ymax}
          lane_offset = (range[3]+range[2])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,enhance);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);
        }

        if (Command == 1){
          enhance = 1;
          theta_offset = M_PI;
          range = {-12.,-4.,0.,4.}; //{xmin,xmax,ymin,ymax}
          lane_offset = (range[3]+range[2])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,enhance);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);
          enhance = 0;

          range = {-4.,12.,0.,4.}; //{xmin,xmax,ymin,ymax}
          lane_offset = (range[3]+range[2])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,enhance);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

          theta_offset = 0.;
          range = {-12.,12.,-4.,0.}; //{xmin,xmax,ymin,ymax}
          lane_offset = (range[3]+range[2])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,enhance);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

        }
      }
    }
  }
