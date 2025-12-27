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
#define ACTION_KEYS_DEFAULT_CONF       "/etc/action_keys.conf"

typedef enum {
    ACTION_TRANSPORT_UDP = 0,
    ACTION_TRANSPORT_HTTP
} action_transport_t;

typedef enum {
    ACTION_HTTP_GET = 0,
    ACTION_HTTP_POST
} action_http_method_t;

typedef enum {
    ACTION_KEY_NONE = 0,
    ACTION_KEY_CHAR,
    ACTION_KEY_UP,
    ACTION_KEY_DOWN,
    ACTION_KEY_LEFT,
    ACTION_KEY_RIGHT,
    ACTION_KEY_ENTER,
    ACTION_KEY_SPACE
} action_keycode_t;

typedef struct {
    action_keycode_t key_code;
    char key_char;
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
} action_binding_t;

typedef struct {
    int http_timeout_ms;
    int verbose;
    char input_device[256];
    size_t action_count;
    action_spec_t actions[ACTION_MAX];
} action_keys_config_t;

void action_keys_config_defaults(action_keys_config_t *cfg);
int action_keys_config_load(action_keys_config_t *cfg, const char *path);

int action_keys_map_evdev_key(int code, action_keycode_t *out_code, char *out_char);

void action_keys_bindings_init(const action_keys_config_t *cfg,
                               action_binding_t bindings[ACTION_MAX]);
void action_keys_free_bindings(action_binding_t bindings[ACTION_MAX]);

void action_keys_handle_keycode(action_keycode_t code, char ch,
                                const action_keys_config_t *cfg,
                                action_binding_t bindings[ACTION_MAX]);

#endif
