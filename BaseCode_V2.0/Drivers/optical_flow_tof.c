#include "optical_flow_tof.h"
#include "tasks/task_manager.h"

#if (ENABLE_OPTICAL_FLOW_TOF == 1)

static optical_flow_tof_data_t s_flow_tof_state;

void optical_flow_tof_init(void)
{
    s_flow_tof_state.flow_x       = 0.0f;
    s_flow_tof_state.flow_y       = 0.0f;
    s_flow_tof_state.tof_distance = 0.0f;
    s_flow_tof_state.flow_quality = 255;
    s_flow_tof_state.valid        = true;
}

void optical_flow_tof_read(optical_flow_tof_data_t *out_data)
{
    if (!out_data) return;
    *out_data = s_flow_tof_state;
}

#else

/* When disabled, provide no-op stubs if ever referenced */
void optical_flow_tof_init(void) {}
void optical_flow_tof_read(optical_flow_tof_data_t *out_data) { (void)out_data; }

#endif /* ENABLE_OPTICAL_FLOW_TOF */
