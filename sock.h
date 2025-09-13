#ifndef _SOCK_H
#define _SOCK_H

/* Sn_SR values */
/**
 * @brief Closed
 * @details This indicates that Socket n is released.\n
 * When DICON, CLOSE command is ordered, or when a timeout occurs, it is changed to @ref SOCK_CLOSED regardless of previous status.
 */
#define SOCK_CLOSED                  0x00

/**
 * @brief Initiate state
 * @details This indicates Socket n is opened with TCP mode.\n
 * It is changed to @ref SOCK_INIT when @ref Sn_MR(P[3:0]) = 001 and OPEN command is ordered.\n
 * After @ref SOCK_INIT, user can use LISTEN /CONNECT command.
 */
#define SOCK_INIT                    0x13

/**
 * @brief Listen state
 * @details This indicates Socket n is operating as <b>TCP server</b>mode and waiting for connection-request (SYN packet) from a peer <b>TCP client</b>.\n
 * It will change to @ref SOCK_ESTALBLISHED when the connection-request is successfully accepted.\n
 * Otherwise it will change to @ref SOCK_CLOSED after TCPTO @ref Sn_IR(TIMEOUT) = '1') is occurred.
 */
#define SOCK_LISTEN                  0x14

/**
 * @brief Connection state
 * @details This indicates Socket n sent the connect-request packet (SYN packet) to a peer.\n
 * It is temporarily shown when @ref Sn_SR is changed from @ref SOCK_INIT to @ref SOCK_ESTABLISHED by CONNECT command.\n
 * If connect-accept(SYN/ACK packet) is received from the peer at SOCK_SYNSENT, it changes to @ref SOCK_ESTABLISHED.\n
 * Otherwise, it changes to @ref SOCK_CLOSED after TCPTO (@ref Sn_IR[TIMEOUT] = '1') is occurred.
 */
#define SOCK_SYNSENT                 0x15

/**
 * @brief Connection state
 * @details It indicates Socket n successfully received the connect-request packet (SYN packet) from a peer.\n
 * If socket n sends the response (SYN/ACK  packet) to the peer successfully,  it changes to @ref SOCK_ESTABLISHED. \n
 * If not, it changes to @ref SOCK_CLOSED after timeout (@ref Sn_IR[TIMEOUT] = '1') is occurred.
 */
#define SOCK_SYNRECV                 0x16

/**
 * @brief Success to connect
 * @details This indicates the status of the connection of Socket n.\n
 * It changes to @ref SOCK_ESTABLISHED when the <b>TCP SERVER</b>processed the SYN packet from the <b>TCP CLIENT</b>during @ref SOCK_LISTEN, or
 * when the CONNECT command is successful.\n
 * During @ref SOCK_ESTABLISHED, DATA packet can be transferred using SEND or RECV command.
 */
#define SOCK_ESTABLISHED             0x17

/**
 * @brief Closing state
 * @details These indicate Socket n is closing.\n
 * These are shown in disconnect-process such as active-close and passive-close.\n
 * When Disconnect-process is successfully completed, or when timeout occurs, these change to @ref SOCK_CLOSED.
 */
#define SOCK_FIN_WAIT                0x18

/**
 * @brief Closing state
 * @details These indicate Socket n is closing.\n
 * These are shown in disconnect-process such as active-close and passive-close.\n
 * When Disconnect-process is successfully completed, or when timeout occurs, these change to @ref SOCK_CLOSED.
 */
#define SOCK_CLOSING                 0x1A

/**
 * @brief Closing state
 * @details These indicate Socket n is closing.\n
 * These are shown in disconnect-process such as active-close and passive-close.\n
 * When Disconnect-process is successfully completed, or when timeout occurs, these change to @ref SOCK_CLOSED.
 */
#define SOCK_TIME_WAIT               0x1B

/**
 * @brief Closing state
 * @details This indicates Socket n received the disconnect-request (FIN packet) from the connected peer.\n
 * This is half-closing status, and data can be transferred.\n
 * For full-closing, DISCON command is used. But For just-closing, CLOSE command is used.
 */
#define SOCK_CLOSE_WAIT              0x1C

/**
 * @brief Closing state
 * @details This indicates Socket n is waiting for the response (FIN/ACK packet) to the disconnect-request (FIN packet) by passive-close.\n
 * It changes to @ref SOCK_CLOSED when Socket n received the response successfully, or when timeout(@ref Sn_IR[TIMEOUT] = '1') is occurred.
 */
#define SOCK_LAST_ACK                0x1D
#endif
