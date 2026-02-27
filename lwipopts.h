#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

/* ----------------- Básico / Memória ----------------- */
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    4000
#define PBUF_POOL_SIZE              24
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_TCP_PCB            10
#define MEMP_NUM_UDP_PCB            6
#define MEMP_NUM_ARP_QUEUE          10

/* ----------------- Protocolos ----------------- */
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    1
#define LWIP_UDP                    1
#define LWIP_TCP                    1
#define LWIP_DHCP                   1
#define LWIP_DNS                    1
#define LWIP_IGMP                   1

/* ----------------- TCP Tunings ----------------- */
#define TCP_MSS                     1460
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_SND_BUF                 (8 * TCP_MSS)
#define TCP_SND_QUEUELEN            16

/* ----------------- Netif callbacks ----------------- */
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1

/* ----------------- Sockets/Netconn (desligados) ----------------- */
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0
#define MEMP_NUM_NETCONN            0
#define LWIP_COMPAT_SOCKETS         0

/* ----------------- MQTT ----------------- */
#define LWIP_ALTCP                  1
#define LWIP_ALTCP_TLS              0
#define MQTT_OUTPUT_RINGBUF_SIZE    512
#define MQTT_VAR_HEADER_BUFFER_LEN  128

/* ----------------- FreeRTOS / sys_arch -----------------
   Como estamos usando pico_cyw43_arch_lwip_sys_freertos, precisamos:
*/
#define NO_SYS                      0
#define SYS_LIGHTWEIGHT_PROT        1
#define LWIP_FREERTOS_CHECK_CORE_LOCKING 1

/* ----------------- Checksums / Stats ----------------- */
#define LWIP_CHKSUM_ALGORITHM       3
#define LWIP_STATS                  0

#endif /* _LWIPOPTS_H */