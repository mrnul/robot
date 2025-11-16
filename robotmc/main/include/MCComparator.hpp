#pragma once

#include "driver/mcpwm_cmpr.h"

class MCComparator
{
private:
    mcpwm_cmpr_handle_t cmpr_handle;

public:
    MCComparator(mcpwm_oper_handle_t oper_handle) : cmpr_handle(nullptr)
    {
        mcpwm_comparator_config_t comparator_config = {};
        comparator_config.flags.update_cmp_on_tez = true;
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper_handle, &comparator_config, &cmpr_handle));
        setValue(0);
    }

    void setValue(uint32_t value) const
    {
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_handle, value));
    }

    operator mcpwm_cmpr_handle_t() const
    {
        return cmpr_handle;
    }

    ~MCComparator()
    {
        if (cmpr_handle)
        {
            ESP_ERROR_CHECK(mcpwm_del_comparator(cmpr_handle));
            cmpr_handle = nullptr;
        }
    }
};