#include "gui_internal.h"

namespace tinglecam_gui {

void release_render_resources() { heightmap_view.reset(); setup_view.reset(); surface_depth_view.reset(); }

}  // namespace tinglecam_gui
