#pragma once

#include "opencv2/core/mat.hpp"

namespace pisar::mcp
{

void thinningZhangSuen(cv::InputArray input, cv::OutputArray output);
void thinningGuoHall(cv::InputArray input, cv::OutputArray output);

} // namespace pisar::mcp
