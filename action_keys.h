#ifndef ACTION_KEYS_H
#define ACTION_KEYS_H

#include <stddef.h>
#include <sys/socket.h>

#define ACTION_MAX                     32
#define ACTION_MAX_BODY_LEN            512
#define ACTION_MAX_HEADER_LEN          128
#define ACTION_MAX_HEADERS             8
#define ACTION_MAX_DEST_LEN            256
#define ACTION_HTTP_TIMEOUT_MS_DEFAULT 1500
#define ACTION_KEYS_DEFAULT_CONF       "/etc/joystick2crsf.conf"

typedef enum {
    ACTION_TRANSPORT_UDP = 0,
    ACTION_TRANSPORT_HTTP
} action_transport_t;

typedef enum {
    ACTION_HTTP_GET = 0,
    ACTION_HTTP_POST
} action_http_method_t;

typedef enum {
    ACTION_EDGE_HIGH = 0,
    ACTION_EDGE_LOW
} action_edge_t;

typedef enum {
    ACTION_PRESS_ANY = 0,
    ACTION_PRESS_SHORT,
    ACTION_PRESS_LONG
} action_press_t;

typedef struct {
    int channel; /* zero-based channel index */
    action_edge_t edge;
    action_press_t press;
    action_transport_t transport;
    action_http_method_t method;
    char destination[ACTION_MAX_DEST_LEN];
    char body[ACTION_MAX_BODY_LEN];
    size_t body_len;
    char headers[ACTION_MAX_HEADERS][ACTION_MAX_HEADER_LEN];
    size_t header_count;
    int timeout_ms;
} action_spec_t;

typedef struct {
    action_spec_t spec;
    int udp_fd;
    struct sockaddr_storage udp_addr;
    socklen_t udp_addrlen;
    int prepared;
    int http_ai_family;
    int http_ai_socktype;
    int http_ai_protocol;
    struct sockaddr_storage http_addr;
    socklen_t http_addrlen;
    char http_host[ACTION_MAX_DEST_LEN];
    char http_port[16];
    char http_path[ACTION_MAX_DEST_LEN];
} action_binding_t;

typedef struct {
    int http_timeout_ms;
    int verbose;
    size_t action_count;
    action_spec_t actions[ACTION_MAX];
} action_keys_config_t;

void action_keys_config_defaults(action_keys_config_t *cfg);
int action_keys_config_load(action_keys_config_t *cfg, const char *path);

void action_keys_bindings_init(const action_keys_config_t *cfg,
                               action_binding_t bindings[ACTION_MAX]);
void action_keys_free_bindings(action_binding_t bindings[ACTION_MAX]);

void action_keys_handle_press(const action_keys_config_t *cfg,
                              action_binding_t bindings[ACTION_MAX],
                              int channel_index,
                              action_edge_t edge,
                              action_press_t press);

void action_keys_build_watchlist(const action_keys_config_t *cfg,
                                 int watch_high[16],
                                 int watch_low[16]);

#endif
