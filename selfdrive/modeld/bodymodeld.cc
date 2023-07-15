#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <cmath>

#include <eigen3/Eigen/Dense>

#include "cereal/messaging/messaging.h"
#include "common/transformations/orientation.hpp"

#include "cereal/visionipc/visionipc_client.h"
#include "common/clutil.h"
#include "common/params.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "system/hardware/hw.h"
#include "selfdrive/modeld/models/body.h"


ExitHandler do_exit;


void run_model(BodyModelState *model, VisionIpcClient &vipc_client) {
  PubMaster pm({"logMessage"});

  double last_ts = 0;
  uint32_t last_frame_id = 0;
  VisionIpcBufExtra extra = {};

  while (!do_exit) {
    VisionBuf *buf = vipc_client.recv(&extra);
    if (buf == nullptr) continue;

    double t1 = millis_since_boot();
    BodyModelResult *model_res = bodymodel_eval_frame(model, buf);
    double t2 = millis_since_boot();

    // send body packet
    bodymodel_publish(pm, extra.frame_id, *model_res, (t2 - t1) / 1000.0);

    //printf("dmonitoring process: %.2fms, from last %.2fms\n", t2 - t1, t1 - last);
    last_ts = t1;
    last_frame_id = extra.frame_id;
  }
}

int main(int argc, char **argv) {
  if (!Hardware::PC()) {
    int ret;
    ret = util::set_realtime_priority(54);
    assert(ret == 0);
    util::set_core_affinity({7});
    assert(ret == 0);
  }

  // cl init
  cl_device_id device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  cl_context context = CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id, NULL, NULL, &err));

  // init the models
  // have to malloc since the output array is too big for the stack!
  BodyModelState *model = (BodyModelState*)malloc(sizeof(BodyModelState));
  bodymodel_init(model, device_id, context);
  LOGW("models loaded, modeld starting");

  VisionIpcClient vipc_client = VisionIpcClient("camerad", VISION_STREAM_DRIVER, true);
  while (!do_exit && !vipc_client.connect(false)) {
    util::sleep_for(100);
  }

  // run the models
  if (vipc_client.connected) {
    LOGW("connected with buffer size: %d", vipc_client.buffers[0].len);
    run_model(model, vipc_client);
  }

  bodymodel_free(model);
  free(model);
  CL_CHECK(clReleaseContext(context));
  return 0;
}
