/*******************************************************************************
* File Name:   udp_server.h
*
* Description: This file contains declaration of task and functions related to
*              UDP server operation.
*
********************************************************************************
 * (c) 2024-2025, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
*******************************************************************************/
#ifndef UDP_SERVER_H_
#define UDP_SERVER_H_

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#include <task.h>


/*******************************************************************************
* Macros
*******************************************************************************/
/* Wi-Fi Credentials: Modify WIFI_SSID, WIFI_PASSWORD, and WIFI_SECURITY_TYPE
 * to match your Wi-Fi network credentials.
 * Note: Maximum length of the Wi-Fi SSID and password is set to
 * CY_WCM_MAX_SSID_LEN and CY_WCM_MAX_PASSPHRASE_LEN as defined in cy_wcm.h file.
 */

#define WIFI_SSID                                 "WIFI_SSID"
#define WIFI_PASSWORD                             "WIFI_PASSWORD"

/* Security type of the Wi-Fi access point. See 'cy_wcm_security_t' structure
 * in "cy_wcm.h" for more details.
 */
#define WIFI_SECURITY_TYPE                        CY_WCM_SECURITY_WPA2_AES_PSK

/* Maximum number of connection retries to a Wi-Fi network. */
#define MAX_WIFI_CONN_RETRIES                     (10U)

/* Wi-Fi re-connection time interval in milliseconds */
#define WIFI_CONN_RETRY_INTERVAL_MSEC             (1000U)

#define UDP_SERVER_PORT                           (57345U)
#define UDP_SERVER_MAX_PENDING_CONNECTIONS        (3U)
#define UDP_SERVER_RECV_TIMEOUT_MS                (500U)

/*******************************************************************************
* Global Variables
*******************************************************************************/ 
/* UDP Server task handle. */
extern TaskHandle_t server_task_handle;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void udp_server_task(void *arg);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* UDP_SERVER_H_ */

/* [] END OF FILE */
