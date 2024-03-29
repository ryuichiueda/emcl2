//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/OdomModel.h"
#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <cmath>

namespace emcl2 {

OdomModel::OdomModel(double ff, double fr, double rf, double rr)
	: std_norm_dist_(0.0, 1.0), fw_dev_(0.0), rot_dev_(0.0), engine_(seed_gen_())
{
	fw_var_per_fw_ = ff*ff;
	fw_var_per_rot_ = fr*fr;
	rot_var_per_fw_ = rf*rf;
	rot_var_per_rot_ = rr*rr;
}

void OdomModel::setDev(double length, double angle)
{
	fw_dev_ = sqrt( fabs(length)*fw_var_per_fw_ + fabs(angle)*fw_var_per_rot_ );
	rot_dev_ = sqrt( fabs(length)*rot_var_per_fw_ + fabs(angle)*rot_var_per_rot_ );
}

double OdomModel::drawFwNoise(void)
{
	return std_norm_dist_(engine_) * fw_dev_;
}

double OdomModel::drawRotNoise(void)
{
	return std_norm_dist_(engine_) * rot_dev_;
}

}
