#include "theta_diff.cpp"
#include "add_lane.cpp"
#include "get_subvector.cpp"
#include "update_lane.cpp"
#include "add_lane_disturbance.cpp"
#include "add_lane_initial_final.cpp"
#include "add_sq.cpp"

void fourway_intersection(
  beacls::FloatVec& lane,
  levelset::HJI_Grid* g,
  beacls::FloatVec gmin,
  beacls::FloatVec gmax,
  beacls::FloatVec vrange,
  int Command)
  {

    FLOAT_TYPE square_width;
    FLOAT_TYPE vehicle_width = 0.05; //for car 1
    FLOAT_TYPE vehicle_width2 = 0.138; //for car 2
    FLOAT_TYPE lane_width = 0.25;

    const size_t numel = g->get_numel();
    const size_t num_dim = g->get_num_of_dimensions();
    const std::vector<size_t> shape = g->get_shape();

    beacls::FloatVec lane_temp;
    beacls::FloatVec lane_temp_calc;

    lane.assign(numel, -10.);
    //lane_temp.assign(numel, -20.);
    //lane_temp_calc.assign(numel,0.);
    beacls::FloatVec xs_temp;
    beacls::FloatVec range;
    FLOAT_TYPE theta_offset;
    FLOAT_TYPE lane_offset;
    FLOAT_TYPE sq_offset;
    FLOAT_TYPE theta_min;
    FLOAT_TYPE theta_max;
    FLOAT_TYPE dim_long_offset;
    FLOAT_TYPE xg, yg, x0, y0;
    int position;
    int dim_long; //longitudinal dimension of lane

    xg = -0.25;
    yg = 0.125;
    x0 = 0.125;
    y0 = -0.25;
    position = 0; //1-enhance value function

    for (size_t dim = 0; dim < 4; ++dim) {
      // Assign values for the vertical lanes.
      if (dim == 0 || dim == 1 || dim == 2 || dim == 3){
        const beacls::FloatVec &xs = g->get_xs(dim);

        if (Command == 0){ //0-Green; 1-Red

          theta_offset = M_PI/2;

          dim_long = 1;
          range = {0.,0.25,-0.5,-0.25}; //{xmin,xmax,ymin,ymax}
          dim_long_offset = range[3];
          lane_offset = (range[1]+range[0])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          //lane_temp.assign(lane_temp.size(), 10.);
          add_lane_initial_final(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim_long,dim_long_offset,dim,vrange,position);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

          if (dim != 1 && dim != 4){
            range = {0.,0.25,0.25,0.5}; //{xmin,xmax,ymin,ymax}
            lane_offset = (range[1]+range[0])/2.;
            get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
            get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
            //lane_temp.assign(lane_temp.size(), 10.);
            add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,vrange);
            update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);


            theta_offset = 3.*M_PI/2.;

            range = {-0.25,0.,-0.5,-0.25}; //{xmin,xmax,ymin,ymax}
            lane_offset = (range[1]+range[0])/2.;
            get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
            get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
            //lane_temp.assign(lane_temp.size(), 10.);
            add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,vrange);
            update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

            range = {-0.25,0.,0.25,0.5}; //{xmin,xmax,ymin,ymax}
            lane_offset = (range[1]+range[0])/2.;
            get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
            get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
            //lane_temp.assign(lane_temp.size(), 10.);
            add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,vrange);
            update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);
          }
        }

        if (Command == 1){ //0-Green; 1-Red

          theta_offset = M_PI/2;

          dim_long = 1;
          range = {0.,0.25,-0.5,-0.25}; //{xmin,xmax,ymin,ymax}
          dim_long_offset = range[3];
          lane_offset = (range[1]+range[0])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          //lane_temp.assign(lane_temp.size(), 10.);
          add_lane_initial_final(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim_long,dim_long_offset,dim,vrange,position);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

        }
      }

      // Assign values for the horizontal lanes.
      if (dim == 0 || dim == 1 || dim == 2 || dim == 3){
        const beacls::FloatVec &xs = g->get_xs(dim);

        if (Command == 0){ //0-Green; 1-Red
          position = 1;
          dim_long = 0;
          theta_offset = M_PI;
          range = {-0.5,-0.25,0.,0.25}; //{xmin,xmax,ymin,ymax}
          dim_long_offset = range[1];
          lane_offset = (range[3]+range[2])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          //lane_temp.assign(lane_temp.size(), 10.);
          add_lane_initial_final(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim_long,dim_long_offset,dim,vrange,position);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);
          position = 0;

          if (dim != 0 && dim != 4){
            range = {0.25,0.5,0.,0.25}; //{xmin,xmax,ymin,ymax}
            lane_offset = (range[3]+range[2])/2.;
            get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
            get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
            //lane_temp.assign(lane_temp.size(), 10.);
            add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,vrange);
            update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

            theta_offset = 0.;
            range = {-0.5,-0.25,-0.25,0.}; //{xmin,xmax,ymin,ymax}
            lane_offset = (range[3]+range[2])/2.;
            get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
            get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
            //lane_temp.assign(lane_temp.size(), 10.);
            add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,vrange);
            update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

            range = {0.25,0.5,-0.25,0.}; //{xmin,xmax,ymin,ymax}
            lane_offset = (range[3]+range[2])/2.;
            get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
            get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
            //lane_temp.assign(lane_temp.size(), 10.);
            add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,vrange);
            update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);
          }
        }

        if (Command == 1){ //0-Green; 1-Red
          position = 1;
          dim_long = 0;
          theta_offset = M_PI;
          range = {-0.5,-0.25,0.,0.25}; //{xmin,xmax,ymin,ymax}
          dim_long_offset = range[1];
          lane_offset = (range[3]+range[2])/2.;
          get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
          get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
          //lane_temp.assign(lane_temp.size(), 10.);
          add_lane_initial_final(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim_long,dim_long_offset,dim,vrange,position);
          update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);
          position = 0;

        }
      }
    }
    for (size_t dim = 0; dim < 4; ++dim) {
      //Assign values for the Intersection Square
      if (Command==0 && (dim == 0 || dim == 1 || dim == 3)){ //0-Green; 1-Red
        const beacls::FloatVec &xs = g->get_xs(dim);
        theta_min = -M_PI/6.; //right turn
        theta_max = M_PI*(1/2+1/6); //forwards
        range = {0.,0.25,-0.25,0}; //{xmin,xmax,ymin,ymax}
        if (dim==0){lane_offset = (range[1]+range[0])/2.;}
        else if (dim==1){lane_offset = (range[3]+range[2])/2.;}
        get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
        get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
        add_sq(lane_temp,xs_temp,xg,yg,x0,y0,dim);
        //lane_temp.assign(lane_temp.size(), 2.);
        if (dim==3){
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,vrange);
        }
        update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

        theta_min = M_PI*(1/2-1/6); //forwards
        theta_max = M_PI*(1+1/6); //left turn
        range = {0.,0.25,0.,0.25}; //{xmin,xmax,ymin,ymax}
        if (dim==0){lane_offset = (range[1]+range[0])/2.;}
        else if (dim==1){lane_offset = (range[3]+range[2])/2.;}
        get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
        get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
        add_sq(lane_temp,xs_temp,xg,yg,x0,y0,dim);
        //lane_temp.assign(lane_temp.size(), 2.);
        if (dim==3){
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,vrange);
        }
        update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

        theta_min = M_PI*(1-1/6); //left turn
        theta_max = M_PI*(3/2+1/6); //backwards
        range = {-0.25,0.,0.,0.25}; //{xmin,xmax,ymin,ymax}
        if (dim==0){lane_offset = (range[1]+range[0])/2.;}
        else if (dim==1){lane_offset = (range[3]+range[2])/2.;}
        get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
        get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
        add_sq(lane_temp,xs_temp,xg,yg,x0,y0,dim);
        //lane_temp.assign(lane_temp.size(), 2.);
        if (dim==3){
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,vrange);
        }
        update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

        theta_min = M_PI*(3/2-1/6); //backwards
        theta_max = 2.*M_PI; //right turn
        range = {-0.25,0.,-0.25,0.}; //{xmin,xmax,ymin,ymax}
        if (dim==0){lane_offset = (range[1]+range[0])/2.;}
        else if (dim==1){lane_offset = (range[3]+range[2])/2.;}
        get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
        get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
        //lane_temp.assign(lane_temp.size(), 2.);
        add_sq(lane_temp,xs_temp,xg,yg,x0,y0,dim);
        if (dim==3){
          add_lane(lane_temp,xs_temp,lane_offset,lane_width,theta_offset,vehicle_width,dim,vrange);
        }
        update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

        //Create barriers at the relevant sections of the intersection.
        if(dim==3){
          if (Command==0){
            range = {0.25,0.3,0.,0.25};
            get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
            lane_temp.assign(lane_temp.size(), -5.);
            update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

            range = {-0.3,-0.25,-0.25,0.};
            get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
            lane_temp.assign(lane_temp.size(), -5.);
            update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);

          }
        }
      }
    }


    //Account for 2nd Car's position
    if (Command == 0){
      size_t dim = 4;
      range = {-0.275,0.025,-0.5,0.5};
      get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
      add_lane_disturbance(lane_temp,shape,range,gmin,gmax,vehicle_width2,dim);
      update_lane(lane_temp,lane,shape,range,gmin,gmax,dim);
    }
  }
