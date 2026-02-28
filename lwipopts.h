#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// =============================================================================
// lwipopts.h — Configuração lwIP para Pico W + FreeRTOS + MQTT
// =============================================================================

#define NO_SYS                          0
#define LWIP_SOCKET                     0
#define LWIP_NETCONN                    0
#define LWIP_DHCP                       1

// Memória
#define MEM_LIBC_MALLOC                 0
#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        8000    // aumentado (era 4000)

// CORREÇÃO: MEMP_SYS_TIMEOUT is empty
// Cada serviço consome slots: DHCP(2) + DNS(1) + MQTT keep-alive(1)
// + TCP retransmit(2) + ARP(1) + margem = mínimo 16
#define MEMP_NUM_SYS_TIMEOUT            16      // era ausente (default=3, muito pequeno)

#define MEMP_NUM_TCP_SEG                32
#define MEMP_NUM_ARP_QUEUE              10
#define PBUF_POOL_SIZE                  24

// Protocolos
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_ICMP                       1
#define LWIP_RAW                        1
#define LWIP_TCP                        1
#define LWIP_TCP_KEEPALIVE              1
#define LWIP_UDP                        1
#define LWIP_DNS                        1

// TCP
#define TCP_MSS                         1460
#define TCP_WND                         (8 * TCP_MSS)
#define TCP_SND_BUF                     (8 * TCP_MSS)
#define TCP_SND_QUEUELEN                ((4 * TCP_SND_BUF) / TCP_MSS)

// MQTT
#define LWIP_ALTCP                      1
#define LWIP_ALTCP_TLS                  0

// Integração FreeRTOS
#define LWIP_TCPIP_CORE_LOCKING         1
#define LWIP_TCPIP_CORE_LOCKING_INPUT   0

// Threads
#define TCPIP_THREAD_STACKSIZE          1024
#define TCPIP_MBOX_SIZE                 16
#define DEFAULT_UDP_RECVMBOX_SIZE       16
#define DEFAULT_TCP_RECVMBOX_SIZE       16
#define DEFAULT_ACCEPTMBOX_SIZE         16
#define DEFAULT_THREAD_STACKSIZE        1024
#define TCPIP_THREAD_PRIO               (configMAX_PRIORITIES - 2)
#define DEFAULT_THREAD_PRIO             1

#define LWIP_STATS                      0
#define LWIP_STATS_DISPLAY              0
#define LWIP_DEBUG                      0

#endif /* _LWIPOPTS_H */