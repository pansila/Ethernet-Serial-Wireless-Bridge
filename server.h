#ifndef __SERVER_H_
#define __SERVER_H_

#define AT                  "AT"
#define AT_RST              "AT+RST"
#define AT_CWMODE_CUR       "AT+CWMODE_CUR"
    #define WIFI_MODE_STATION    1
    #define WIFI_MODE_SOFTAAP    2
    #define WIFI_MODE_CONCURR    3
/* AT+CWJAP_CUR="abc","0123456789" */
#define AT_CWJAP_CUR       "AT+CWJAP_CUR"
/* AT+CWDHCP_CUR=<mode>,<en> */
    #define DHCP_MODE_SOFTAAP    0
    #define DHCP_MODE_STATION    1
    #define DHCP_MODE_CONCURR    2
#define AT_CWDHCP_CUR       "AT+CWDHCP_CUR"

#endif /* __SERVER_H_ */
