/*
 * SPDX-FileCopyrightText: 2026 Pedro Minatel
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ddp.h"

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#define DDP_BASE_HEADER_LENGTH 10U
#define DDP_TIMECODE_LENGTH     4U
#define DDP_MAX_HEADER_LENGTH  14U
#define DDP_MAX_TCP_CLIENTS     4U
#define DDP_DUPLICATE_SOURCES   4U

typedef struct {
    int fd;
    uint32_t source_ipv4;
    uint8_t header[DDP_MAX_HEADER_LENGTH];
    size_t header_used;
    size_t header_length;
    uint8_t payload[DDP_MAX_PAYLOAD_LENGTH];
    size_t payload_used;
    size_t payload_length;
    bool query;
} ddp_tcp_client_t;

typedef struct {
    bool used;
    uint32_t source_ipv4;
    uint8_t sequence;
    uint8_t destination_id;
    uint32_t offset;
    uint16_t length;
    uint32_t hash;
    uint32_t age;
} ddp_duplicate_source_t;

struct ddp_server {
    ddp_server_config_t config;
    ddp_callbacks_t callbacks;
    void *callback_context;
    int udp_fd;
    int tcp_fd;
    volatile bool running;
    TaskHandle_t task;
    SemaphoreHandle_t stopped;
    SemaphoreHandle_t stats_lock;
    ddp_server_stats_t stats;
    ddp_tcp_client_t clients[DDP_MAX_TCP_CLIENTS];
    ddp_duplicate_source_t sources[DDP_DUPLICATE_SOURCES];
    uint32_t source_age;
    uint8_t udp_buffer[DDP_MAX_HEADER_LENGTH + DDP_MAX_PAYLOAD_LENGTH + 1U];
    uint8_t query_response[DDP_MAX_PAYLOAD_LENGTH];
    uint8_t reply_packet[DDP_BASE_HEADER_LENGTH + DDP_MAX_PAYLOAD_LENGTH];
};

static const char *TAG = "ddp";

static uint16_t read_be16(const uint8_t *bytes)
{
    return (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

static uint32_t read_be32(const uint8_t *bytes)
{
    return ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8) | bytes[3];
}

static void write_be16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)(value >> 8);
    bytes[1] = (uint8_t)value;
}

static void write_be32(uint8_t *bytes, uint32_t value)
{
    bytes[0] = (uint8_t)(value >> 24);
    bytes[1] = (uint8_t)(value >> 16);
    bytes[2] = (uint8_t)(value >> 8);
    bytes[3] = (uint8_t)value;
}

static uint64_t stats_increment(ddp_server_handle_t server, uint64_t *counter)
{
    xSemaphoreTake(server->stats_lock, portMAX_DELAY);
    uint64_t value = ++(*counter);
    xSemaphoreGive(server->stats_lock);
    return value;
}

static uint32_t packet_hash(const ddp_packet_t *packet)
{
    uint32_t hash = 2166136261U;
    const uint8_t metadata[] = {
        packet->flags, packet->sequence, packet->data_type, packet->destination_id,
        (uint8_t)(packet->offset >> 24), (uint8_t)(packet->offset >> 16),
        (uint8_t)(packet->offset >> 8), (uint8_t)packet->offset,
        (uint8_t)(packet->data_length >> 8), (uint8_t)packet->data_length,
    };
    for (size_t index = 0; index < sizeof(metadata); ++index) {
        hash = (hash ^ metadata[index]) * 16777619U;
    }
    for (size_t index = 0; index < packet->data_length; ++index) {
        hash = (hash ^ packet->data[index]) * 16777619U;
    }
    return hash;
}

static bool is_duplicate_and_track(ddp_server_handle_t server, const ddp_packet_t *packet)
{
    if (packet->sequence == 0 || (packet->flags & (DDP_FLAG_QUERY | DDP_FLAG_REPLY)) != 0) {
        return false;
    }

    uint32_t hash = packet_hash(packet);
    ddp_duplicate_source_t *entry = NULL;
    ddp_duplicate_source_t *oldest = &server->sources[0];
    for (size_t index = 0; index < DDP_DUPLICATE_SOURCES; ++index) {
        ddp_duplicate_source_t *candidate = &server->sources[index];
        if (candidate->used && candidate->source_ipv4 == packet->source_ipv4) {
            entry = candidate;
            break;
        }
        if (!candidate->used) {
            oldest = candidate;
        } else if (oldest->used && candidate->age < oldest->age) {
            oldest = candidate;
        }
    }
    if (entry == NULL) {
        entry = oldest;
        memset(entry, 0, sizeof(*entry));
        entry->used = true;
        entry->source_ipv4 = packet->source_ipv4;
    }

    bool duplicate = entry->sequence == packet->sequence &&
                     entry->destination_id == packet->destination_id &&
                     entry->offset == packet->offset && entry->length == packet->data_length &&
                     entry->hash == hash;
    if (!duplicate && entry->sequence != 0) {
        uint8_t expected = entry->sequence == 15 ? 1 : (uint8_t)(entry->sequence + 1);
        if (packet->sequence != expected) {
            uint64_t gaps = stats_increment(server, &server->stats.sequence_gaps);
            if (gaps == 1 || gaps % 100 == 0) {
                ESP_LOGW(TAG, "Sequence gap from " IPSTR ": expected=%u received=%u "
                              "(%" PRIu64 " gaps)",
                         IP2STR((esp_ip4_addr_t *)&packet->source_ipv4), expected,
                         packet->sequence, gaps);
            }
        }
    }

    entry->sequence = packet->sequence;
    entry->destination_id = packet->destination_id;
    entry->offset = packet->offset;
    entry->length = packet->data_length;
    entry->hash = hash;
    entry->age = ++server->source_age;
    if (duplicate) {
        uint64_t duplicates = stats_increment(server, &server->stats.duplicate_packets);
        if (duplicates == 1 || duplicates % 100 == 0) {
            ESP_LOGI(TAG, "Ignored duplicate DDP packet from " IPSTR " (%" PRIu64 " duplicates)",
                     IP2STR((esp_ip4_addr_t *)&packet->source_ipv4), duplicates);
        }
    }
    return duplicate;
}

static esp_err_t send_all(int fd, const uint8_t *data, size_t length)
{
    size_t sent = 0;
    while (sent < length) {
        int result = send(fd, data + sent, length - sent, 0);
        if (result < 0 && errno == EINTR) {
            continue;
        }
        if (result <= 0) {
            return ESP_FAIL;
        }
        sent += (size_t)result;
    }
    return ESP_OK;
}

static esp_err_t send_query_reply(ddp_server_handle_t server, const ddp_packet_t *query,
                                  int socket_fd, const struct sockaddr_in *udp_address)
{
    uint8_t *response = server->query_response;
    size_t response_length = 0;
    esp_err_t err = ESP_ERR_NOT_SUPPORTED;
    if (server->callbacks.on_query != NULL) {
        err = server->callbacks.on_query(server->callback_context, query, response,
                                         server->config.max_payload_length, &response_length);
    }
    if (err != ESP_OK) {
        response_length = 0;
    }
    if (response_length > server->config.max_payload_length || response_length > UINT16_MAX) {
        stats_increment(server, &server->stats.callback_errors);
        return ESP_ERR_INVALID_SIZE;
    }

    uint32_t response_offset = 0;
    if (err == ESP_OK) {
        response_offset = query->offset;
        if (response_offset >= response_length) {
            response_length = 0;
        } else {
            response_length -= response_offset;
            if (query->data_length != 0 && response_length > query->data_length) {
                response_length = query->data_length;
            }
            memmove(response, response + response_offset, response_length);
        }
    }

    uint8_t *packet = server->reply_packet;
    packet[0] = DDP_FLAG_VERSION_1 | DDP_FLAG_REPLY | DDP_FLAG_PUSH;
    packet[1] = query->sequence & 0x0fU;
    packet[2] = query->data_type;
    packet[3] = query->destination_id;
    write_be32(&packet[4], response_offset);
    write_be16(&packet[8], (uint16_t)response_length);
    if (response_length > 0) {
        memcpy(&packet[DDP_BASE_HEADER_LENGTH], response, response_length);
    }

    size_t packet_length = DDP_BASE_HEADER_LENGTH + response_length;
    if (query->transport == DDP_TRANSPORT_UDP) {
        int sent = sendto(socket_fd, packet, packet_length, 0,
                          (const struct sockaddr *)udp_address, sizeof(*udp_address));
        err = sent == (int)packet_length ? ESP_OK : ESP_FAIL;
    } else {
        err = send_all(socket_fd, packet, packet_length);
    }
    if (err == ESP_OK) {
        stats_increment(server, &server->stats.reply_packets);
    }
    return err;
}

static esp_err_t dispatch_packet(ddp_server_handle_t server, const uint8_t *header,
                                 const uint8_t *payload, uint32_t source_ipv4,
                                 ddp_transport_t transport, int socket_fd,
                                 const struct sockaddr_in *udp_address)
{
    ddp_packet_t packet = {
        .flags = header[0],
        .sequence = header[1] & 0x0fU,
        .data_type = header[2],
        .destination_id = header[3],
        .offset = read_be32(&header[4]),
        .data = payload,
        .data_length = read_be16(&header[8]),
        .source_ipv4 = source_ipv4,
        .transport = transport,
    };

    uint64_t received = stats_increment(server, &server->stats.received_packets);
    if (received == 1 || received % 1000 == 0) {
        ESP_LOGI(TAG, "Received DDP packet %" PRIu64 " from " IPSTR
                      ": id=%u offset=%" PRIu32 " len=%u via %s",
                 received, IP2STR((esp_ip4_addr_t *)&packet.source_ipv4),
                 packet.destination_id, packet.offset, packet.data_length,
                 transport == DDP_TRANSPORT_UDP ? "UDP" : "TCP");
    }
    if ((packet.flags & 0xc0U) != DDP_FLAG_VERSION_1 ||
        (packet.flags & DDP_FLAG_REPLY) != 0) {
        uint64_t malformed = stats_increment(server, &server->stats.malformed_packets);
        if (malformed == 1 || malformed % 100 == 0) {
            ESP_LOGW(TAG, "Rejected packet from " IPSTR ": flags=0x%02x "
                          "(%" PRIu64 " malformed)",
                     IP2STR((esp_ip4_addr_t *)&packet.source_ipv4), packet.flags, malformed);
        }
        return ESP_ERR_INVALID_ARG;
    }
    if (is_duplicate_and_track(server, &packet)) {
        ESP_LOGV(TAG, "Ignored duplicate from " IPSTR ": seq=%u offset=%" PRIu32 " len=%u",
                 IP2STR((esp_ip4_addr_t *)&packet.source_ipv4), packet.sequence,
                 packet.offset, packet.data_length);
        return ESP_OK;
    }

    if ((packet.flags & DDP_FLAG_QUERY) != 0) {
        stats_increment(server, &server->stats.query_packets);
        ESP_LOGI(TAG, "Query from " IPSTR ": id=%u offset=%" PRIu32 " requested=%u via %s",
                 IP2STR((esp_ip4_addr_t *)&packet.source_ipv4), packet.destination_id,
                 packet.offset, packet.data_length,
                 transport == DDP_TRANSPORT_UDP ? "UDP" : "TCP");
        return send_query_reply(server, &packet, socket_fd, udp_address);
    }

    ESP_LOGV(TAG, "Data from " IPSTR ": seq=%u id=%u type=0x%02x offset=%" PRIu32
                  " len=%u push=%u via %s",
             IP2STR((esp_ip4_addr_t *)&packet.source_ipv4), packet.sequence,
             packet.destination_id, packet.data_type, packet.offset, packet.data_length,
             (packet.flags & DDP_FLAG_PUSH) != 0,
             transport == DDP_TRANSPORT_UDP ? "UDP" : "TCP");
    esp_err_t err = ESP_OK;
    if (packet.data_length > 0) {
        stats_increment(server, &server->stats.data_packets);
        if (server->callbacks.on_data != NULL) {
            err = server->callbacks.on_data(server->callback_context, &packet);
        }
    }
    if (err == ESP_OK && (packet.flags & DDP_FLAG_PUSH) != 0) {
        stats_increment(server, &server->stats.push_packets);
        if (server->callbacks.on_push != NULL) {
            err = server->callbacks.on_push(server->callback_context, &packet);
        }
    }
    if (err != ESP_OK) {
        uint64_t errors = stats_increment(server, &server->stats.callback_errors);
        if (errors == 1 || errors % 100 == 0) {
            ESP_LOGW(TAG, "Application rejected packet: %s (%" PRIu64 " callback errors)",
                     esp_err_to_name(err), errors);
        }
    }
    return err;
}

static void process_udp(ddp_server_handle_t server)
{
    uint8_t *datagram = server->udp_buffer;
    struct sockaddr_in source = {0};
    socklen_t source_length = sizeof(source);
    int received = recvfrom(server->udp_fd, datagram, sizeof(server->udp_buffer), 0,
                            (struct sockaddr *)&source, &source_length);
    if (received < 0) {
        if (errno != EINTR && errno != EAGAIN) {
            ESP_LOGW(TAG, "UDP receive failed: errno %d", errno);
        }
        return;
    }
    if ((size_t)received > DDP_MAX_HEADER_LENGTH + server->config.max_payload_length) {
        uint64_t oversized = stats_increment(server, &server->stats.oversized_packets);
        if (oversized == 1 || oversized % 100 == 0) {
            ESP_LOGW(TAG, "Rejected oversized UDP datagram from " IPSTR
                          ": %d bytes (%" PRIu64 " oversized)",
                     IP2STR((esp_ip4_addr_t *)&source.sin_addr.s_addr), received, oversized);
        }
        return;
    }
    if (received < DDP_BASE_HEADER_LENGTH) {
        uint64_t malformed = stats_increment(server, &server->stats.malformed_packets);
        if (malformed == 1 || malformed % 100 == 0) {
            ESP_LOGW(TAG, "Rejected short UDP datagram from " IPSTR
                          ": %d bytes (%" PRIu64 " malformed)",
                     IP2STR((esp_ip4_addr_t *)&source.sin_addr.s_addr), received, malformed);
        }
        return;
    }

    size_t header_length = (datagram[0] & DDP_FLAG_TIMECODE) != 0
                               ? DDP_MAX_HEADER_LENGTH : DDP_BASE_HEADER_LENGTH;
    bool query = (datagram[0] & DDP_FLAG_QUERY) != 0;
    uint16_t payload_length = read_be16(&datagram[8]);
    size_t expected = header_length + (query ? 0U : payload_length);
    if ((size_t)received != expected || (!query && payload_length > server->config.max_payload_length)) {
        bool oversized_packet = !query && payload_length > server->config.max_payload_length;
        uint64_t rejected = stats_increment(server, oversized_packet
                                                        ? &server->stats.oversized_packets
                                                        : &server->stats.malformed_packets);
        if (rejected == 1 || rejected % 100 == 0) {
            ESP_LOGW(TAG, "Rejected UDP datagram from " IPSTR
                          ": received=%d header=%u declared=%u query=%u (%" PRIu64 " %s)",
                     IP2STR((esp_ip4_addr_t *)&source.sin_addr.s_addr), received,
                     (unsigned)header_length, payload_length, query, rejected,
                     oversized_packet ? "oversized" : "malformed");
        }
        return;
    }

    dispatch_packet(server, datagram, &datagram[header_length], source.sin_addr.s_addr,
                    DDP_TRANSPORT_UDP, server->udp_fd, &source);
}

static void reset_client_packet(ddp_tcp_client_t *client)
{
    client->header_used = 0;
    client->header_length = DDP_BASE_HEADER_LENGTH;
    client->payload_used = 0;
    client->payload_length = 0;
    client->query = false;
}

static void disconnect_client(ddp_server_handle_t server, ddp_tcp_client_t *client)
{
    if (client->fd >= 0) {
        ESP_LOGI(TAG, "TCP client " IPSTR " disconnected",
                 IP2STR((esp_ip4_addr_t *)&client->source_ipv4));
        shutdown(client->fd, SHUT_RDWR);
        close(client->fd);
        client->fd = -1;
        stats_increment(server, &server->stats.tcp_disconnects);
    }
    reset_client_packet(client);
}

static void process_tcp_client(ddp_server_handle_t server, ddp_tcp_client_t *client)
{
    uint8_t *destination;
    size_t remaining;
    if (client->header_used < client->header_length) {
        destination = client->header + client->header_used;
        remaining = client->header_length - client->header_used;
    } else {
        destination = client->payload + client->payload_used;
        remaining = client->payload_length - client->payload_used;
    }

    int received = recv(client->fd, destination, remaining, 0);
    if (received <= 0) {
        if (received == 0 || (errno != EINTR && errno != EAGAIN)) {
            disconnect_client(server, client);
        }
        return;
    }

    if (client->header_used < client->header_length) {
        client->header_used += (size_t)received;
        if (client->header_used == DDP_BASE_HEADER_LENGTH &&
            client->header_length == DDP_BASE_HEADER_LENGTH) {
            if ((client->header[0] & 0xc0U) != DDP_FLAG_VERSION_1) {
                uint64_t malformed = stats_increment(server, &server->stats.malformed_packets);
                ESP_LOGW(TAG, "Closing TCP client " IPSTR
                              " after invalid version flags 0x%02x (%" PRIu64 " malformed)",
                         IP2STR((esp_ip4_addr_t *)&client->source_ipv4), client->header[0],
                         malformed);
                disconnect_client(server, client);
                return;
            }
            client->query = (client->header[0] & DDP_FLAG_QUERY) != 0;
            client->payload_length = client->query ? 0U : read_be16(&client->header[8]);
            if (client->payload_length > server->config.max_payload_length) {
                uint64_t oversized = stats_increment(server, &server->stats.oversized_packets);
                ESP_LOGW(TAG, "Closing TCP client " IPSTR
                              " after oversized payload: %u (%" PRIu64 " oversized)",
                         IP2STR((esp_ip4_addr_t *)&client->source_ipv4),
                         (unsigned)client->payload_length, oversized);
                disconnect_client(server, client);
                return;
            }
            if ((client->header[0] & DDP_FLAG_TIMECODE) != 0) {
                client->header_length = DDP_MAX_HEADER_LENGTH;
                return;
            }
        }
        if (client->header_used < client->header_length) {
            return;
        }
    } else {
        client->payload_used += (size_t)received;
    }

    if (client->header_used == client->header_length &&
        client->payload_used == client->payload_length) {
        dispatch_packet(server, client->header, client->payload, client->source_ipv4,
                        DDP_TRANSPORT_TCP, client->fd, NULL);
        reset_client_packet(client);
    }
}

static void accept_tcp_client(ddp_server_handle_t server)
{
    struct sockaddr_in source = {0};
    socklen_t source_length = sizeof(source);
    int fd = accept(server->tcp_fd, (struct sockaddr *)&source, &source_length);
    if (fd < 0) {
        return;
    }

    ddp_tcp_client_t *slot = NULL;
    for (size_t index = 0; index < server->config.max_tcp_clients; ++index) {
        if (server->clients[index].fd < 0) {
            slot = &server->clients[index];
            break;
        }
    }
    if (slot == NULL) {
        ESP_LOGW(TAG, "Rejected TCP client " IPSTR ": client limit reached",
                 IP2STR((esp_ip4_addr_t *)&source.sin_addr.s_addr));
        close(fd);
        return;
    }

    struct timeval timeout = { .tv_sec = 1 };
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    int keepalive = 1;
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
    slot->fd = fd;
    slot->source_ipv4 = source.sin_addr.s_addr;
    reset_client_packet(slot);
    stats_increment(server, &server->stats.tcp_connections);
    ESP_LOGI(TAG, "TCP client " IPSTR " connected",
             IP2STR((esp_ip4_addr_t *)&slot->source_ipv4));
}

static void server_task(void *argument)
{
    ddp_server_handle_t server = argument;

    while (server->running) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        int max_fd = -1;
        if (server->udp_fd >= 0) {
            FD_SET(server->udp_fd, &read_fds);
            max_fd = server->udp_fd;
        }
        if (server->tcp_fd >= 0) {
            FD_SET(server->tcp_fd, &read_fds);
            if (server->tcp_fd > max_fd) {
                max_fd = server->tcp_fd;
            }
        }
        for (size_t index = 0; index < server->config.max_tcp_clients; ++index) {
            int fd = server->clients[index].fd;
            if (fd >= 0) {
                FD_SET(fd, &read_fds);
                if (fd > max_fd) {
                    max_fd = fd;
                }
            }
        }

        struct timeval timeout = { .tv_sec = 0, .tv_usec = 250000 };
        int ready = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
        if (ready < 0) {
            if (errno != EINTR && server->running) {
                ESP_LOGW(TAG, "select failed: errno %d", errno);
            }
            continue;
        }
        if (ready == 0) {
            continue;
        }
        if (server->udp_fd >= 0 && FD_ISSET(server->udp_fd, &read_fds)) {
            process_udp(server);
        }
        if (server->tcp_fd >= 0 && FD_ISSET(server->tcp_fd, &read_fds)) {
            accept_tcp_client(server);
        }
        for (size_t index = 0; index < server->config.max_tcp_clients; ++index) {
            ddp_tcp_client_t *client = &server->clients[index];
            if (client->fd >= 0 && FD_ISSET(client->fd, &read_fds)) {
                process_tcp_client(server, client);
            }
        }
    }

    for (size_t index = 0; index < server->config.max_tcp_clients; ++index) {
        disconnect_client(server, &server->clients[index]);
    }
    xSemaphoreGive(server->stopped);
    vTaskDelete(NULL);
}

static esp_err_t create_bound_socket(int type, uint16_t port, bool listen_socket, int *out_fd)
{
    int fd = socket(AF_INET, type, IPPROTO_IP);
    if (fd < 0) {
        return ESP_FAIL;
    }
    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(fd, (struct sockaddr *)&address, sizeof(address)) != 0 ||
        (listen_socket && listen(fd, DDP_MAX_TCP_CLIENTS) != 0)) {
        ESP_LOGE(TAG, "Could not bind %s socket on port %u: errno %d",
                 type == SOCK_STREAM ? "TCP" : "UDP", port, errno);
        close(fd);
        return ESP_FAIL;
    }
    *out_fd = fd;
    return ESP_OK;
}

esp_err_t ddp_server_start(const ddp_server_config_t *config,
                           const ddp_callbacks_t *callbacks,
                           void *context,
                           ddp_server_handle_t *out_handle)
{
    if (config == NULL || callbacks == NULL || out_handle == NULL || config->port == 0 ||
        (config->transports & ~(DDP_TRANSPORT_UDP | DDP_TRANSPORT_TCP)) != 0 ||
        config->transports == 0 || config->max_tcp_clients == 0 ||
        config->max_tcp_clients > DDP_MAX_TCP_CLIENTS || config->max_payload_length == 0 ||
        config->max_payload_length > DDP_MAX_PAYLOAD_LENGTH || config->task_stack_size < 4096) {
        return ESP_ERR_INVALID_ARG;
    }

    ddp_server_handle_t server = calloc(1, sizeof(*server));
    if (server == NULL) {
        return ESP_ERR_NO_MEM;
    }
    server->config = *config;
    server->callbacks = *callbacks;
    server->callback_context = context;
    server->udp_fd = -1;
    server->tcp_fd = -1;
    for (size_t index = 0; index < DDP_MAX_TCP_CLIENTS; ++index) {
        server->clients[index].fd = -1;
        reset_client_packet(&server->clients[index]);
    }
    server->stopped = xSemaphoreCreateBinary();
    server->stats_lock = xSemaphoreCreateMutex();
    if (server->stopped == NULL || server->stats_lock == NULL) {
        ddp_server_stop(server);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = ESP_OK;
    if ((config->transports & DDP_TRANSPORT_UDP) != 0) {
        err = create_bound_socket(SOCK_DGRAM, config->port, false, &server->udp_fd);
    }
    if (err == ESP_OK && (config->transports & DDP_TRANSPORT_TCP) != 0) {
        err = create_bound_socket(SOCK_STREAM, config->port, true, &server->tcp_fd);
    }
    if (err != ESP_OK) {
        ddp_server_stop(server);
        return err;
    }

    server->running = true;
    if (xTaskCreate(server_task, "ddp_server", config->task_stack_size, server,
                    config->task_priority, &server->task) != pdPASS) {
        server->running = false;
        ddp_server_stop(server);
        return ESP_ERR_NO_MEM;
    }
    *out_handle = server;
    ESP_LOGI(TAG, "Listening on port %u (%s%s), max payload %u, max TCP clients %u",
             config->port,
             (config->transports & DDP_TRANSPORT_UDP) != 0 ? "UDP" : "",
             config->transports == (DDP_TRANSPORT_UDP | DDP_TRANSPORT_TCP) ? "+TCP" :
             ((config->transports & DDP_TRANSPORT_TCP) != 0 ? "TCP" : ""),
             config->max_payload_length, config->max_tcp_clients);
    return ESP_OK;
}

esp_err_t ddp_server_stop(ddp_server_handle_t server)
{
    if (server == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    bool had_task = server->task != NULL;
    server->running = false;
    if (had_task && xSemaphoreTake(server->stopped, pdMS_TO_TICKS(1500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (had_task) {
        ESP_LOGI(TAG, "Stopped: received=%" PRIu64 " data=%" PRIu64 " push=%" PRIu64
                      " query=%" PRIu64 " replies=%" PRIu64 " malformed=%" PRIu64
                      " oversized=%" PRIu64 " duplicates=%" PRIu64 " gaps=%" PRIu64
                      " callback_errors=%" PRIu64,
                 server->stats.received_packets, server->stats.data_packets,
                 server->stats.push_packets, server->stats.query_packets,
                 server->stats.reply_packets, server->stats.malformed_packets,
                 server->stats.oversized_packets, server->stats.duplicate_packets,
                 server->stats.sequence_gaps, server->stats.callback_errors);
    }
    if (server->udp_fd >= 0) {
        close(server->udp_fd);
    }
    if (server->tcp_fd >= 0) {
        close(server->tcp_fd);
    }
    if (server->stopped != NULL) {
        vSemaphoreDelete(server->stopped);
    }
    if (server->stats_lock != NULL) {
        vSemaphoreDelete(server->stats_lock);
    }
    free(server);
    return ESP_OK;
}

esp_err_t ddp_server_get_stats(ddp_server_handle_t server, ddp_server_stats_t *stats)
{
    if (server == NULL || stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(server->stats_lock, portMAX_DELAY);
    *stats = server->stats;
    xSemaphoreGive(server->stats_lock);
    return ESP_OK;
}
