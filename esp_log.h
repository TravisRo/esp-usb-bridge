#pragma once

#define ESP_LOG_ERROR 0
#define ESP_LOG_WARN 1
#define ESP_LOG_INFO 2
#define ESP_LOG_DEBUG 3
#define ESP_LOG_VERBOSE 4

#ifdef DISABLED
#define ESP_LOG_LEVEL_LOCAL(level, tag, format, ...) do {                                               \
		if (level <= ESP_LOG_INFO) printf("[%s] " format "\r\n", tag, ## __VA_ARGS__); \
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
