#pragma once

#include "xosm/model.hpp"
#include "xosm/options.hpp"

namespace xosm::xodr {

void write_file(const model::MapModel& model, const Options& options);

} // namespace xosm::xodr
