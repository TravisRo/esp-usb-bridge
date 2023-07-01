#include "ubp_config.h"

#ifndef _ESP_LOG_LEVEL
#define _ESP_LOG_LEVEL

#define LOG_NL "\r\n"

#if (LOGGING_ENABLED())
#define ESP_LOG_LEVEL_LOCAL(level, tag, format, ...) do {                                               \
		if (level <= LOG_LEVEL) printf("[%s] " format LOG_NL, tag, ## __VA_ARGS__); \
}while(0)
#else
#define ESP_LOG_LEVEL_LOCAL(level, tag, format, ...)
#endif

#define ESP_LOGE( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   tag, format, ## __VA_ARGS__)
#define ESP_LOGW( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    tag, format, ## __VA_ARGS__)
#define ESP_LOGI( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    tag, format, ## __VA_ARGS__)
#define ESP_LOGD( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,   tag, format, ## __VA_ARGS__)
#define ESP_LOGV( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, tag, format, ## __VA_ARGS__)
#define ESP_LOG_BUFFER_HEXDUMP(tag,buf,size,level)
#endif
