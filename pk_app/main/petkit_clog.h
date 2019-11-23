/*
 * petkit_comm_log.h
 *
 *  Created on: 2015-9-23
 *      Author: match0131
 */

#ifndef INCLUDE_PETKIT_CLOG_H_
#define INCLUDE_PETKIT_CLOG_H_

/******************************************************************************************/
/**
 * log level,0~7,large number,large messages
 *      0 - pkLOG_EMERG(fmt, args...)
 *      1 - pkLOG_CRIT(fmt, args...)
 *      2 - pkLOG_ALERT(fmt, args...)
 *      3 - pkLOG_ERR(fmt, args...)
 *      4 - pkLOG_WARNING(fmt, args...)
 *      5 - pkLOG_NOTICE(fmt, args...)
 *      6 - pkLOG_INFO(fmt, args...)
 *      7 - pkLOG_DEBUG(fmt, args...)
 **/
#define LOG_EMERG   1   /* system is unusable */
#define LOG_ERR     2   /* error conditions */
#define LOG_WARNING 3   /* warning conditions */
#define LOG_NOTCIE	4   /* warning conditions */
#define LOG_INFO    5   /* informational */
#define LOG_DEBUG   6   /* debug-level messages */
#define LOG_ALL		7   /* debug-level messages */

extern int petkit_log_level;

#define pkLOG_EMERG(fmt, args...) \
    if(petkit_log_level>=(LOG_EMERG)) { \
        printf(fmt, ##args); \
    }

#define pkLOG_ERR(fmt, args...) \
    if(petkit_log_level>=(LOG_ERR)) { \
        printf(fmt, ##args); \
    }

#define pkLOG_WARNING(fmt, args...) \
    if(petkit_log_level>=(LOG_WARNING)) { \
        printf(fmt, ##args); \
    }

#define pkLOG_NOTICE(fmt, args...) \
    if(petkit_log_level>=(LOG_NOTCIE)) { \
        printf(fmt, ##args); \
    }

#define pkLOG_INFO(fmt, args...) \
    if(petkit_log_level>=(LOG_INFO)) { \
        printf(fmt, ##args); \
    }

#define pkLOG_DEBUG(fmt, args...) \
    if(petkit_log_level>=(LOG_DEBUG)) { \
        printf(fmt, ##args); \
    }

#define pkLOG_ALL(fmt, args...) \
    if(petkit_log_level>=(LOG_ALL)) { \
        printf(fmt, ##args); \
    }

#define DBG(fmt, args...) \
    if(petkit_log_level>=(LOG_DEBUG)) { \
        printf(fmt, ##args); \
    }

#define petkit_set_log_lvl(lvl) do {\
    if ((LOG_ALL >= lvl) && (LOG_EMERG <= lvl)) { \
        petkit_log_level = lvl; \
    } } while(0)

#define petkit_get_log_lvl() do { \
        return petkit_log_level; \
    } while(0)
/******************************************************************************************/

#endif /* INCLUDE_PETKIT_CLOG_H_ */
