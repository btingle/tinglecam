#include "gui_internal.h"

namespace tinglecam_gui {

#include "prism_view.h"

void release_prism_view() {
  heightmap_view.reset();
  prism_view.release();
}

}  // namespace tinglecam_gui
