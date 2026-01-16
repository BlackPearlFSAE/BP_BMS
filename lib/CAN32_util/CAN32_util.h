#include <cstdint>

void CAN32_initCANBus(int can_tx,int can_rx, bool &readyflag,twai_timing_config_t t_config);
void CAN32_initCANBus(int can_tx,int can_rx, bool &readyflag,
                twai_timing_config_t t_config, twai_filter_config_t f_config);
int CAN32_sendCAN(twai_message_t* tx_msg);
int CAN32_receiveCAN(twai_message_t* rx_msg);
void CAN32_twai_debug(uint32_t alerts_trigger);

void CAN32_debugFrame(twai_message_t* rx_msg);

// Example to write for filter -> Make it an output of twai_filter_config_t type , to get the data strcuture need