/*
 * SPDX-FileCopyrightText: 2026 Pedro Minatel
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ddp.h"
#include "unity.h"
#include "unity_test_runner.h"

TEST_CASE("default server configuration is bounded", "[ddp]")
{
    const ddp_server_config_t config = DDP_SERVER_CONFIG_DEFAULT();
    TEST_ASSERT_EQUAL_UINT16(DDP_DEFAULT_PORT, config.port);
    TEST_ASSERT_EQUAL_UINT8(DDP_TRANSPORT_UDP, config.transports);
    TEST_ASSERT_GREATER_THAN_UINT16(0, config.max_payload_length);
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(DDP_MAX_PAYLOAD_LENGTH, config.max_payload_length);
}

TEST_CASE("server rejects invalid configuration", "[ddp]")
{
    ddp_server_handle_t server = NULL;
    const ddp_callbacks_t callbacks = {0};
    ddp_server_config_t config = DDP_SERVER_CONFIG_DEFAULT();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ddp_server_start(NULL, &callbacks, NULL, &server));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ddp_server_start(&config, NULL, NULL, &server));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ddp_server_start(&config, &callbacks, NULL, NULL));

    config.transports = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ddp_server_start(&config, &callbacks, NULL, &server));
    config = (ddp_server_config_t)DDP_SERVER_CONFIG_DEFAULT();
    config.max_payload_length = DDP_MAX_PAYLOAD_LENGTH + 1U;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ddp_server_start(&config, &callbacks, NULL, &server));
}

TEST_CASE("server accessors reject null handles", "[ddp]")
{
    ddp_server_stats_t stats = {0};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ddp_server_stop(NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ddp_server_get_stats(NULL, &stats));
}

void app_main(void)
{
    unity_run_all_tests();
}
