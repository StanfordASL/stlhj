#include "Car4D_Car1D_mod.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <typeinfo>
#include <levelset/Grids/HJI_Grid.hpp>
using namespace helperOC;

/*
Need v2Range: cannot use dMax since other vehicle probably has asymmetric speed range
  - go through file to add it in all appropriate places
  - look at any place where the class properties are being used
dMax should just be 2D for now (for disturbance acting on xdot and ydot)
  - note that number of disturbances is still 3
  - just need to compute last component of disturbance according to v2Range
 */

Car4D_Car1D_mod::Car4D_Car1D_mod(
    const beacls::FloatVec& x,
    const std::vector<beacls::FloatVec>& data,
    const std::vector<beacls::FloatVec>& slope_Coefficient,
    const std::vector<beacls::FloatVec>& intercept_Coefficient,
    const FLOAT_TYPE wMax,
    const beacls::FloatVec& aRange,
    const beacls::FloatVec& dMax,
    const beacls::FloatVec& v2Range,
    const beacls::FloatVec& y2Range,
    const FLOAT_TYPE dt,
    const beacls::IntegerVec& dims):
    DynSys(5, // number of states
	       2, // number of controls
		   3 // number of disturbances
    ),
    wMax(wMax), aRange(aRange), dMax(dMax), v2Range(v2Range), y2Range(y2Range), dt(dt),
    data(data), slope_Coefficient(slope_Coefficient),
    intercept_Coefficient(intercept_Coefficient), dims(dims) {

  if (x.size() != DynSys::get_nx()) {
    std::cerr << "Error: " << __func__ <<
      " : Initial state does not have right dimension!" << std::endl;
  }

  DynSys::set_x(x);
  DynSys::push_back_xhist(x);
}

Car4D_Car1D_mod::Car4D_Car1D_mod(
    beacls::MatFStream* fs,
    beacls::MatVariable* variable_ptr):
    DynSys(fs, variable_ptr),
    data(std::vector<beacls::FloatVec>()),
    slope_Coefficient(std::vector<beacls::FloatVec>()),
    intercept_Coefficient(std::vector<beacls::FloatVec>()),
    wMax(0),
    aRange(beacls::FloatVec()),
    dMax(beacls::FloatVec()),
    v2Range(beacls::FloatVec()),
    y2Range(beacls::FloatVec()),
    dt(0),
    dims(beacls::IntegerVec()) {

  beacls::IntegerVec dummy;
  std::vector<beacls::FloatVec> dummy2;
  load_value(wMax, std::string("wMax"), true, fs, variable_ptr);
  load_vector(aRange, std::string("aRange"), dummy, true, fs, variable_ptr);
  load_vector(dMax, std::string("dMax"), dummy, true, fs, variable_ptr);
  load_vector(v2Range, std::string("v2Range"), dummy, true, fs, variable_ptr);
  load_vector(y2Range, std::string("y2Range"), dummy, true, fs, variable_ptr);
  load_value(dt, std::string("dt"), true, fs, variable_ptr);
  //load_vector(data, std::string("data"), dummy2, true, fs, variable_ptr);
  //load_vector(slope_Coefficient, std::string("slope_Coefficient"), dummy2, true, fs, variable_ptr);
  //load_vector(intercept_Coefficient, std::string("intercept_Coefficient"), dummy2, true, fs, variable_ptr);
  load_vector(dims, std::string("dims"), dummy, true, fs, variable_ptr);
}

Car4D_Car1D_mod::~Car4D_Car1D_mod() {
}

bool Car4D_Car1D_mod::operator==(const Car4D_Car1D_mod& rhs) const {
  if (this == &rhs) return true;
  else if (!DynSys::operator==(rhs)) return false;
  else if (wMax != rhs.wMax) return false;  //!< Angular control bounds
  else if ((aRange.size() != rhs.aRange.size()) ||
      !std::equal(aRange.cbegin(), aRange.cend(), rhs.aRange.cbegin())) {
    return false; //!< Acceleration control bounds
  }

  else if ((dMax.size() != rhs.dMax.size()) ||
      !std::equal(dMax.cbegin(), dMax.cend(), rhs.dMax.cbegin())) {
    return false; //!< Disturbance
  }

  else if ((v2Range.size() != rhs.v2Range.size()) ||
      !std::equal(v2Range.cbegin(), v2Range.cend(), rhs.v2Range.cbegin())) {
    return false; //!< Disturbance
  }

  else if ((y2Range.size() != rhs.y2Range.size()) ||
      !std::equal(y2Range.cbegin(), y2Range.cend(), rhs.y2Range.cbegin())) {
    return false; //!< Disturbance
  }

  else if (dt != rhs.dt) return false;

  else if ((dims.size() != rhs.dims.size()) ||
      !std::equal(dims.cbegin(), dims.cend(), rhs.dims.cbegin())) {
    return false; //!< Dimensions that are active
  }

  return true;
}

bool Car4D_Car1D_mod::operator==(const DynSys& rhs) const {
  if (this == &rhs) return true;
  else if (typeid(*this) != typeid(rhs)) return false;
  else return operator==(dynamic_cast<const Car4D_Car1D_mod&>(rhs));
}

bool Car4D_Car1D_mod::save(
    beacls::MatFStream* fs,
    beacls::MatVariable* variable_ptr) {
  bool result = DynSys::save( fs, variable_ptr);
  result &= save_value(wMax, std::string("wMax"), true, fs, variable_ptr);
  if (!aRange.empty()) {
    result &= save_vector(aRange, std::string("aRange"), beacls::IntegerVec(),
      true, fs, variable_ptr);
  }

  if (!dMax.empty()) {
    result &= save_vector(dMax, std::string("dMax"), beacls::IntegerVec(),
      true, fs, variable_ptr);
  }

  if (!v2Range.empty()) {
    result &= save_vector(v2Range, std::string("v2Range"), beacls::IntegerVec(),
      true, fs, variable_ptr);
  }

  if (!y2Range.empty()) {
    result &= save_vector(y2Range, std::string("y2Range"), beacls::IntegerVec(),
      true, fs, variable_ptr);
  }

  result &= save_value(dt, std::string("dt"), true, fs, variable_ptr);

  if (!dims.empty()) {
    result &= save_vector(dims, std::string("dims"), beacls::IntegerVec(),
      true, fs, variable_ptr);
  }
  return result;
}

bool Car4D_Car1D_mod::optCtrl_cell_helper(
    beacls::FloatVec& uOpt0,
    beacls::FloatVec& uOpt1,
    const std::vector<const FLOAT_TYPE* >& derivs,
    const beacls::IntegerVec& deriv_sizes,
    const helperOC::DynSys_UMode_Type uMode,
    const size_t src_target_dim2_index,
    const size_t src_target_dim3_index,
    const FLOAT_TYPE t) const {

    const size_t tau = size_t (t/0.25);

  if (src_target_dim2_index < dims.size()) {
    const FLOAT_TYPE* deriv2 = derivs[src_target_dim2_index];
    const FLOAT_TYPE* deriv3 = derivs[src_target_dim3_index];
    const size_t length = deriv_sizes[src_target_dim2_index];

    if (length == 0 || deriv2 == NULL || deriv3 == NULL) return false;
    uOpt0.resize(length);
    uOpt1.resize(length);
    switch (uMode) {
      case helperOC::DynSys_UMode_Max:
        for (size_t i = 0; i < length; ++i) {
          uOpt0[i] = (deriv2[i] >= 0) ? wMax : -wMax;
          uOpt1[i] = (deriv3[i] >= 0) ? aRange[1] : aRange[0];
        }
        break;

      case helperOC::DynSys_UMode_Min:
        for (size_t i = 0; i < length; ++i) {
          if (data[tau][i] > 0.) {
            uOpt0[i] = (deriv2[i] >= 0) ? -wMax : wMax;
            uOpt1[i] = (deriv3[i] >= 0) ? aRange[0] : aRange[1];
          } else {
            ComputeCompatibleController(uOpt0[i],uOpt1[i],deriv2[i],deriv3[i],tau,i);
          }
        }
        break;

      case helperOC::DynSys_UMode_Invalid:

      default:
        std::cerr << "Unknown uMode!: " << uMode << std::endl;
        return false;
    }
  }
  return true;
}

void Car4D_Car1D_mod::ComputeCompatibleController(
    FLOAT_TYPE& uOpt0_i,
    FLOAT_TYPE& uOpt1_i,
    const FLOAT_TYPE& deriv2_i,
    const FLOAT_TYPE& deriv3_i,
    const size_t tau,
    const size_t i) const {

    const beacls::FloatVec wrange{-wMax,wMax};
    const FLOAT_TYPE a = intercept_Coefficient[tau][i];
    const FLOAT_TYPE b = slope_Coefficient[tau][i];
    FLOAT_TYPE Value;
    beacls::IntegerVec intersection_side;
    beacls::FloatVec intersection_point_a, intersection_point_w,
                      set1_w, set2_w, set1_a, set2_a,
                      compatible_set_a, compatible_set_w;

    for (size_t w_iter = 0; w_iter < 2; ++w_iter) {
      Value = (-b-wrange[w_iter])/a;
      if (Value>aRange[0] && Value<aRange[1]){
        intersection_side.push_back(w_iter);
        intersection_point_w.push_back(wrange[w_iter]);
        intersection_point_a.push_back(Value);
      }
    }

    for (size_t a_iter = 0; a_iter < 2; ++a_iter) {
      Value = -a*aRange[a_iter]-b;
      if (Value>wrange[0] && Value<wrange[1]){
        intersection_side.push_back(a_iter+2);
        intersection_point_w.push_back(Value);
        intersection_point_a.push_back(aRange[a_iter]);
      }
    }

    if (intersection_side.size()==2){
      if (intersection_side[0]==0){
        set1_w.push_back(wrange[0]); set1_a.push_back(aRange[0]);
        set2_w.push_back(wrange[0]); set2_a.push_back(aRange[1]);
        if (intersection_side[1]==1){
          set1_w.push_back(wrange[1]); set1_a.push_back(aRange[0]);
          set2_w.push_back(wrange[1]); set2_a.push_back(aRange[1]);
        } else if (intersection_side[1]==2){
          set2_w.push_back(wrange[1]); set2_a.push_back(aRange[0]);
          set2_w.push_back(wrange[1]); set2_a.push_back(aRange[1]);
        } else if (intersection_side[1]==3){
          set1_w.push_back(wrange[1]); set1_a.push_back(aRange[0]);
          set1_w.push_back(wrange[1]); set1_a.push_back(aRange[1]);
        }
      } else if (intersection_side[0]==1){
        set1_w.push_back(wrange[1]); set1_a.push_back(aRange[0]);
        set2_w.push_back(wrange[1]); set2_a.push_back(aRange[1]);
        if (intersection_side[1]==2){
          set2_w.push_back(wrange[0]); set2_a.push_back(aRange[0]);
          set2_w.push_back(wrange[0]); set2_a.push_back(aRange[1]);
        } else if (intersection_side[1]==3){
          set1_w.push_back(wrange[0]); set1_a.push_back(aRange[0]);
          set1_w.push_back(wrange[0]); set1_a.push_back(aRange[1]);
        }
      } else {
        set1_w.push_back(wrange[0]); set1_a.push_back(aRange[0]);
        set1_w.push_back(wrange[0]); set1_a.push_back(aRange[1]);
        set2_w.push_back(wrange[1]); set2_a.push_back(aRange[0]);
        set2_w.push_back(wrange[1]); set2_a.push_back(aRange[1]);
      }

      Value = set1_w[0]+a*set1_a[0]+b;
      if (Value<=0){
        beacls::FloatVec compatible_set_w = set1_w;
        beacls::FloatVec compatible_set_a = set1_a;
      } else {
        beacls::FloatVec compatible_set_w = set2_w;
        beacls::FloatVec compatible_set_a = set2_a;
      }

      for (size_t i = 0; i<2; ++i){
        compatible_set_w.push_back(intersection_point_w[i]);
        compatible_set_a.push_back(intersection_point_a[i]);
      }
      FLOAT_TYPE Value_min = 1000.;
      for (size_t j = 0; j<compatible_set_w.size(); ++j){
        Value = deriv2_i*compatible_set_w[j] + deriv3_i*compatible_set_a[j];
        if (Value<Value_min){
          Value_min = Value;
          uOpt0_i = compatible_set_w[j];
          uOpt1_i = compatible_set_a[j];
        }
      }

    } else {
      uOpt0_i = (deriv2_i >= 0) ? -wMax : wMax;
      uOpt1_i = (deriv3_i >= 0) ? aRange[0] : aRange[1];
    }

}

bool Car4D_Car1D_mod::optCtrl1_cell_helper(
    beacls::FloatVec& uOpt1,
    const std::vector<const FLOAT_TYPE* >& derivs,
    const beacls::IntegerVec& deriv_sizes,
    const helperOC::DynSys_UMode_Type uMode,
    const size_t src_target_dim_index) const {

  if (src_target_dim_index < dims.size()) {
    const FLOAT_TYPE* deriv = derivs[src_target_dim_index];
    const size_t length = deriv_sizes[src_target_dim_index];

    if (length == 0 || deriv == NULL) return false;

    uOpt1.resize(length);
    switch (uMode) {
      case helperOC::DynSys_UMode_Max:
        for (size_t i = 0; i < length; ++i) {
          uOpt1[i] = (deriv[i] >= 0) ? aRange[1] : aRange[0];
        }
        break;

      case helperOC::DynSys_UMode_Min:
        for (size_t i = 0; i < length; ++i) {
          uOpt1[i] = (deriv[i] >= 0) ? aRange[0] : aRange[1];
        }
        break;

      case helperOC::DynSys_UMode_Invalid:

      default:
        std::cerr << "Unknown uMode!: " << uMode << std::endl;
        return false;
    }
  }
  return true;
}

bool Car4D_Car1D_mod::optDstb0_cell_helper(
    beacls::FloatVec& dOpt0,
    const std::vector<const FLOAT_TYPE* >& derivs,
    const beacls::IntegerVec& deriv_sizes,
    const helperOC::DynSys_DMode_Type dMode,
    const size_t src_target_dim_index) const {

  if (src_target_dim_index != dims.size()) {
    const FLOAT_TYPE* deriv = derivs[src_target_dim_index];
    const size_t length = deriv_sizes[src_target_dim_index];

    if (length == 0 || deriv == NULL) return false;

    dOpt0.resize(length);
    const FLOAT_TYPE dMax0 = dMax[0];
    switch (dMode) {
      case helperOC::DynSys_DMode_Max:
        for (size_t i = 0; i < length; ++i) {
          dOpt0[i] = (deriv[i] >= 0) ? dMax0 : -dMax0;
        }
        break;

      case helperOC::DynSys_DMode_Min:
        for (size_t i = 0; i < length; ++i) {
          dOpt0[i] = (deriv[i] >= 0) ? -dMax0 : dMax0;
        }
        break;

      case helperOC::DynSys_UMode_Invalid:

      default:
        std::cerr << "Unknown uMode!: " << dMode << std::endl;
        return false;
    }
  }
  return true;

}
bool Car4D_Car1D_mod::optDstb1_cell_helper(
    beacls::FloatVec& dOpt1,
    const std::vector<const FLOAT_TYPE* >& derivs,
    const beacls::IntegerVec& deriv_sizes,
    const helperOC::DynSys_DMode_Type dMode,
    const size_t src_target_dim_index) const {

  if (src_target_dim_index != dims.size()) {
    const FLOAT_TYPE* deriv = derivs[src_target_dim_index];
    const size_t length = deriv_sizes[src_target_dim_index];

    if (length == 0 || deriv == NULL) return false;

    dOpt1.resize(length);
    const FLOAT_TYPE dMax1 = dMax[1];
    switch (dMode) {
      case helperOC::DynSys_DMode_Max:
        for (size_t i = 0; i < length; ++i) {
          dOpt1[i] = (deriv[i] >= 0) ? dMax1 : -dMax1;
        }
        break;

      case helperOC::DynSys_DMode_Min:
        for (size_t i = 0; i < length; ++i) {
          dOpt1[i] = (deriv[i] >= 0) ? -dMax1 : dMax1;
        }
        break;

      case helperOC::DynSys_UMode_Invalid:

      default:
        std::cerr << "Unknown uMode!: " << dMode << std::endl;
        return false;


    }
  }
  return true;
}


bool Car4D_Car1D_mod::optDstb2_cell_helper(
    beacls::FloatVec& dOpt2,
    const std::vector<const FLOAT_TYPE* >& derivs,
    const beacls::IntegerVec& deriv_sizes,
    const helperOC::DynSys_DMode_Type dMode,
    const size_t src_target_dim_index) const {

  if (src_target_dim_index != dims.size()) {
    const FLOAT_TYPE* deriv = derivs[src_target_dim_index];
    const size_t length = deriv_sizes[src_target_dim_index];

    if (length == 0 || deriv == NULL) return false;

    dOpt2.resize(length);
    switch (dMode) {
      case helperOC::DynSys_DMode_Max:
        for (size_t i = 0; i < length; ++i) {
          dOpt2[i] = (deriv[i] >= 0) ? v2Range[1] : v2Range[0];
        }
        break;

      case helperOC::DynSys_DMode_Min:
        for (size_t i = 0; i < length; ++i) {
          dOpt2[i] = (deriv[i] >= 0) ? v2Range[0] : v2Range[1];
        }
        break;

      case helperOC::DynSys_UMode_Invalid:

      default:
        std::cerr << "Unknown uMode!: " << dMode << std::endl;
        return false;
    }
  }
  return true;
}

bool Car4D_Car1D_mod::optCtrl(
    std::vector<beacls::FloatVec>& uOpts,
    const FLOAT_TYPE t,
    const std::vector<beacls::FloatVec::const_iterator>&,
    const std::vector<const FLOAT_TYPE*>& deriv_ptrs,
    const beacls::IntegerVec&,
    const beacls::IntegerVec& deriv_sizes,
    const helperOC::DynSys_UMode_Type uMode) const {

  const helperOC::DynSys_UMode_Type modified_uMode =
    (uMode == helperOC::DynSys_UMode_Default) ?
    helperOC::DynSys_UMode_Max : uMode;
  const size_t src_target_dim2_index = find_val(dims, 2);
  const size_t src_target_dim3_index = find_val(dims, 3);

  uOpts.resize(get_nu());

  bool result = true;
  result &= optCtrl_cell_helper(uOpts[0], uOpts[1], deriv_ptrs, deriv_sizes,
    modified_uMode, src_target_dim2_index, src_target_dim3_index, t);
  //result &= optCtrl0_cell_helper(uOpts[0], deriv_ptrs, deriv_sizes,
    //modified_uMode, src_target_dim2_index);
  //result &= optCtrl1_cell_helper(uOpts[1], deriv_ptrs, deriv_sizes,
    //modified_uMode, src_target_dim3_index);

  return result;
}


bool Car4D_Car1D_mod::optDstb(
    std::vector<beacls::FloatVec>& dOpts,
    const FLOAT_TYPE,
    const std::vector<beacls::FloatVec::const_iterator >&,
    const std::vector<const FLOAT_TYPE*>& deriv_ptrs,
    const beacls::IntegerVec&,
    const beacls::IntegerVec& deriv_sizes,
    const helperOC::DynSys_DMode_Type dMode) const {

  const helperOC::DynSys_DMode_Type modified_dMode =
    (dMode == helperOC::DynSys_DMode_Default) ?
    helperOC::DynSys_DMode_Min : dMode;
  const size_t src_target_dim0_index = find_val(dims, 0);
  const size_t src_target_dim1_index = find_val(dims, 1);
  const size_t src_target_dim2_index = find_val(dims, 4);

  dOpts.resize(get_nd());
  bool result = true;
  result &= optDstb0_cell_helper(dOpts[0], deriv_ptrs, deriv_sizes,
    modified_dMode, src_target_dim0_index);
  result &= optDstb1_cell_helper(dOpts[1], deriv_ptrs, deriv_sizes,
    modified_dMode, src_target_dim1_index);
  result &= optDstb2_cell_helper(dOpts[2], deriv_ptrs, deriv_sizes,
    modified_dMode, src_target_dim2_index);

  return result;
}
bool Car4D_Car1D_mod::dynamics_cell_helper(
    std::vector<beacls::FloatVec>& dxs,
    const beacls::FloatVec::const_iterator& x_ites2,
    const beacls::FloatVec::const_iterator& x_ites3,
    const beacls::FloatVec::const_iterator& x_ites4,
    const std::vector<beacls::FloatVec>& us,
    const std::vector<beacls::FloatVec>& ds,
    const size_t x2_size,
    const size_t,
    const size_t dim) const {

  beacls::FloatVec& dx_i = dxs[dim];
  bool result = true;

  switch (dims[dim]) {
    case 0: {
        //dx_i.assign(x2_size, 10.);
        dx_i.resize(x2_size);

        const beacls::FloatVec& ds_0s = ds[0];
        FLOAT_TYPE ds_0;

        for (size_t index = 0; index < x2_size; ++index) {
          dx_i[index] = (x_ites3[index])*std::cos(x_ites2[index]);
        }
      }
      break;

      case 1: {
        //  dx_i.assign(x2_size, 10.);
        dx_i.resize(x2_size);
        const beacls::FloatVec& ds_1s = ds[1];
        FLOAT_TYPE ds_1;

        for (size_t index = 0; index < x2_size; ++index) {
          dx_i[index] = x_ites3[index]*std::sin(x_ites2[index])-0.01;
        }
      }
      break;

    case 2:{
//      dx_i.assign(x2_size, 10.);
      dx_i.resize(us[0].size());
      std::copy(us[0].cbegin(), us[0].cend(), dx_i.begin());
    }
      break;

    case 3:{
//      dx_i.assign(x2_size, 10.);
      dx_i.resize(us[1].size());
      std::copy(us[1].cbegin(), us[1].cend(), dx_i.begin());
    }
      break;

      case 4:{
        //dx_i.assign(x2_size, 10.);
        dx_i.resize(x2_size);
        const beacls::FloatVec& ds_2s = ds[2];
        FLOAT_TYPE ds_2;

        for (size_t index = 0; index < x2_size; ++index) {
          if (ds[2].size() == x2_size) {
            ds_2 = ds_2s[index];
          }
          else {
            ds_2 = ds_2s[0];

          }
          if (x_ites4[index] > 0.7) {//y2Range[1]) {
            dx_i[index] = 0.;
          } else {
            dx_i[index] = ds_2;
          }
        }
      }
      break;
    default: {
      std::cerr << "Only dimension 1-5 are defined for dynamics of Car4D_Car1D_mod!"
      << std::endl;
      result = false;
    }

      break;
  }


  return result;
}
bool Car4D_Car1D_mod::dynamics(
    std::vector<beacls::FloatVec>& dx,
    const FLOAT_TYPE,
    const std::vector<beacls::FloatVec::const_iterator >& x_ites,
    const std::vector<beacls::FloatVec >& us,
    const std::vector<beacls::FloatVec >& ds,
    const beacls::IntegerVec& x_sizes,
    const size_t dst_target_dim) const {

  const size_t src_target_dim2_index = find_val(dims, 2);
  const size_t src_target_dim3_index = find_val(dims, 3);
  const size_t src_target_dim4_index = find_val(dims, 4);

  if ((src_target_dim2_index == dims.size())
    || (src_target_dim3_index == dims.size())) {
    return false;
  }

  beacls::FloatVec::const_iterator x_ites2 = x_ites[src_target_dim2_index];
  beacls::FloatVec::const_iterator x_ites3 = x_ites[src_target_dim3_index];
  beacls::FloatVec::const_iterator x_ites4 = x_ites[src_target_dim4_index];
  bool result = true;
  if (dst_target_dim == std::numeric_limits<size_t>::max()) {
    for (size_t dim = 0; dim < dims.size(); ++dim) {
      result &= dynamics_cell_helper(dx, x_ites2, x_ites3, x_ites4, us, ds,
        x_sizes[src_target_dim2_index], x_sizes[src_target_dim3_index], dim);
    }
  }
  else {
    if (dst_target_dim < dims.size())
      result &= dynamics_cell_helper(dx, x_ites2, x_ites3, x_ites4, us, ds,
        x_sizes[src_target_dim2_index], x_sizes[src_target_dim3_index],
        dst_target_dim);
    else {
      std::cerr << "Invalid target dimension for dynamics: " << dst_target_dim
        << std::endl;
      result = false;
    }
  }
  return result;
}
