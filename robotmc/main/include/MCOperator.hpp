#pragma once

#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"

class MCOperator
{
private:
    mcpwm_oper_handle_t oper_handle;
    int group_id;

public:
    MCOperator(int group_id, mcpwm_timer_handle_t timer_handle)
        : oper_handle(nullptr), group_id(group_id)
    {
        mcpwm_operator_config_t operator_config = {};
        operator_config.group_id = group_id;
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper_handle));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_handle, timer_handle));
    }

    operator mcpwm_oper_handle_t() const
    {
        return oper_handle;
    }

    ~MCOperator()
    {
        if (oper_handle)
        {
            ESP_ERROR_CHECK(mcpwm_del_operator(oper_handle));
            oper_handle = nullptr;
        }
    }
};