/*******************************************************************************
* File Name:   udp_server.c
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
/* Header file includes */
#include "cybsp.h"
#include "retarget_io_init.h"
#include <inttypes.h>

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>

/* Cypress secure socket header file */
#include "cy_secure_sockets.h"

/* Wi-Fi connection manager header files */
#include "cy_wcm.h"
#include "cy_wcm_error.h"

/* UDP server task header file. */
#include "udp_server.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* Length of the LED ON/OFF command issued from the UDP server. */
#define UDP_LED_CMD_LEN                           (1U)

/* LED ON and LED OFF commands. */
#define LED_ON_CMD                                ('1')
#define LED_OFF_CMD                               ('0')

#define LED_ON_ACK_MSG                            "LED ON ACK"

/* Initial message sent to UDP Server to confirm client availability. */
#define START_COMM_MSG                            'A'

/* Buffer size to store the incoming messages from server, in bytes. */
#define MAX_UDP_RECV_BUFFER_SIZE                  (20U)

#define DEBOUNCE_DELAY                            (250U)

#define SHIFT_BIT_8                               (8U)
#define SHIFT_BIT_16                              (16U)
#define SHIFT_BIT_24                              (24U)
#define RESET_VALUE                               ('\0')

#define GPIO_INTERRUPT_PRIORITY                   (7U)

#define APP_SDIO_INTERRUPT_PRIORITY               (7U)
#define APP_HOST_WAKE_INTERRUPT_PRIORITY          (2U)
#define APP_SDIO_FREQUENCY_HZ                     (25000000U)
#define SDHC_SDIO_64BYTES_BLOCK                   (64U)

#define SDHC_SDIO_64B_BLOCK                       (64U)
#define RESET_VAL                                 (0U)
#define COMPARE_VAL                               (0U)
#define DEBOUNCE_TIME_MS                          (1U)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
/* Secure socket variables. */
cy_socket_sockaddr_t udp_server_addr, peer_addr;
cy_socket_t server_handle;

/* Flag variable to track client connection status,
 * set to True when START_COMM_MSG is received from client. */
bool client_connected = false;

/* Flags to tack the LED state and command. */
bool led_state = CYBSP_LED_STATE_OFF;

bool button_pressed = false;
static mtb_hal_sdio_t sdio_instance;
static cy_stc_sd_host_context_t sdhc_host_context;
static cy_wcm_config_t wcm_config;
volatile bool button_debouncing = false;
volatile uint32_t button_debounce_timestamp = 0;

/* Interrupt config structure for USER BUTTON*/
cy_stc_sysint_t sysint_cfg =
{
    .intrSrc = CYBSP_USER_BTN_IRQ,
    .intrPriority = GPIO_INTERRUPT_PRIORITY
};

#if (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP)

/* SysPm callback parameter structure for SDHC */
static cy_stc_syspm_callback_params_t sdcardDSParams =
{
    .context   = &sdhc_host_context,
    .base      = CYBSP_WIFI_SDIO_HW
};

/* SysPm callback structure for SDHC*/
static cy_stc_syspm_callback_t sdhcDeepSleepCallbackHandler =
{
    .callback           = Cy_SD_Host_DeepSleepCallback,
    .skipMode           = SYSPM_SKIP_MODE,
    .type               = CY_SYSPM_DEEPSLEEP,
    .callbackParams     = &sdcardDSParams,
    .prevItm            = NULL,
    .nextItm            = NULL,
    .order              = SYSPM_CALLBACK_ORDER
};
#endif

/*******************************************************************************
* Function Name: sdio_interrupt_handler
********************************************************************************
* Summary:
* Interrupt handler function for SDIO instance.
*******************************************************************************/
static void sdio_interrupt_handler(void)
{
    mtb_hal_sdio_process_interrupt(&sdio_instance);
}

/*******************************************************************************
* Function Name: host_wake_interrupt_handler
********************************************************************************
* Summary:
* Interrupt handler function for the host wake up input pin.
*******************************************************************************/
static void host_wake_interrupt_handler(void)
{
    mtb_hal_gpio_process_interrupt(&wcm_config.wifi_host_wake_pin);
}

/*******************************************************************************
* Function Name: app_sdio_init
********************************************************************************
* Summary:
* This function configures and initializes the SDIO instance used in 
* communication between the host MCU and the wireless device.
*******************************************************************************/
static void app_sdio_init(void)
{
    cy_rslt_t result;
    mtb_hal_sdio_cfg_t sdio_hal_cfg;
    
    cy_stc_sysint_t sdio_intr_cfg =
    {
        .intrSrc = CYBSP_WIFI_SDIO_IRQ,
        .intrPriority = APP_SDIO_INTERRUPT_PRIORITY
    };

    cy_stc_sysint_t host_wake_intr_cfg =
    {
            .intrSrc = CYBSP_WIFI_HOST_WAKE_IRQ,
            .intrPriority = APP_HOST_WAKE_INTERRUPT_PRIORITY
    };

    /* Initialize the SDIO interrupt and specify the interrupt handler. */
    cy_en_sysint_status_t interrupt_init_status = Cy_SysInt_Init(&sdio_intr_cfg, sdio_interrupt_handler);

    /* SDIO interrupt initialization failed. Stop program execution. */
    if(CY_SYSINT_SUCCESS != interrupt_init_status)
    {
        handle_app_error();
    }

    /* Enable NVIC interrupt. */
    NVIC_EnableIRQ(CYBSP_WIFI_SDIO_IRQ);

    /* Setup SDIO using the HAL object and desired configuration */
    result = mtb_hal_sdio_setup(&sdio_instance, &CYBSP_WIFI_SDIO_sdio_hal_config, NULL, &sdhc_host_context);

    /* SDIO setup failed. Stop program execution. */
    if(CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Initialize and Enable SD HOST */
    Cy_SD_Host_Enable(CYBSP_WIFI_SDIO_HW);
    Cy_SD_Host_Init(CYBSP_WIFI_SDIO_HW, CYBSP_WIFI_SDIO_sdio_hal_config.host_config, &sdhc_host_context);
    Cy_SD_Host_SetHostBusWidth(CYBSP_WIFI_SDIO_HW, CY_SD_HOST_BUS_WIDTH_4_BIT);

    sdio_hal_cfg.frequencyhal_hz = APP_SDIO_FREQUENCY_HZ;
    sdio_hal_cfg.block_size = SDHC_SDIO_64BYTES_BLOCK;

    /* Configure SDIO */
    mtb_hal_sdio_configure(&sdio_instance, &sdio_hal_cfg);
    
#if (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP)
    /* SDHC SysPm callback registration */
    Cy_SysPm_RegisterCallback(&sdhcDeepSleepCallbackHandler);
#endif /* (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP) */

    /* Setup GPIO using the HAL object for WIFI WL REG ON  */
    mtb_hal_gpio_setup(&wcm_config.wifi_wl_pin, CYBSP_WIFI_WL_REG_ON_PORT_NUM, CYBSP_WIFI_WL_REG_ON_PIN);

    /* Setup GPIO using the HAL object for WIFI HOST WAKE PIN  */
    mtb_hal_gpio_setup(&wcm_config.wifi_host_wake_pin, CYBSP_WIFI_HOST_WAKE_PORT_NUM, CYBSP_WIFI_HOST_WAKE_PIN);

    /* Initialize the Host wakeup interrupt and specify the interrupt handler. */
    cy_en_sysint_status_t interrupt_init_status_host_wake =  Cy_SysInt_Init(&host_wake_intr_cfg, host_wake_interrupt_handler);

    /* Host wake up interrupt initialization failed. Stop program execution. */
    if(CY_SYSINT_SUCCESS != interrupt_init_status_host_wake)
    {
        handle_app_error();
    }

    /* Enable NVIC interrupt. */
    NVIC_EnableIRQ(CYBSP_WIFI_HOST_WAKE_IRQ);
}

/*******************************************************************************
* Function Name: udp_server_recv_handler
*******************************************************************************
* Summary:
*  Callback function to handle incoming  message from UDP client
*
* Parameters:
* cy_socket_t socket_handle: Connection handle for the UDP server socket
* void *args : Task parameter defined during task creation (unused).
*
* Return:
* cy_rslt_t :  Returns CY_RSLT_SUCCESS if the message is successfully 
*  received from the UDP client.
*******************************************************************************/
cy_rslt_t udp_server_recv_handler(cy_socket_t socket_handle, void *arg)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Variable to store the number of bytes received. */
    uint32_t bytes_received = 0;

    /* Buffer to store data received from Client. */
    char message_buffer[MAX_UDP_RECV_BUFFER_SIZE] = {0};

    /* Receive incoming message from UDP server. */
    result = cy_socket_recvfrom(server_handle, message_buffer, MAX_UDP_RECV_BUFFER_SIZE,
                                CY_SOCKET_FLAGS_NONE, &peer_addr, NULL,
                                &bytes_received);

    message_buffer[bytes_received] = RESET_VALUE;

    if(CY_RSLT_SUCCESS == result)
    {
        if(START_COMM_MSG == message_buffer[0])
        {
            client_connected = true;
            printf("UDP Client available on IP Address: %d.%d.%d.%d \n", (uint8)peer_addr.ip_address.ip.v4,
                    (uint8)(peer_addr.ip_address.ip.v4 >> SHIFT_BIT_8), (uint8)(peer_addr.ip_address.ip.v4 >> SHIFT_BIT_16),
                    (uint8)(peer_addr.ip_address.ip.v4 >> SHIFT_BIT_24));
        }
        else
        {
            printf("\nAcknowledgement from UDP Client:\n");

            /* Print the message received from UDP client. */
            printf("%s",message_buffer);

            /* Set the LED state based on the acknowledgment received from the UDP client. */
            if(strcmp(message_buffer, LED_ON_ACK_MSG) == COMPARE_VAL)
            {
                led_state = CYBSP_LED_STATE_ON;
            }
            else
            {
                led_state = CYBSP_LED_STATE_OFF;
            }
            printf("\n");
        }

    }
    else
    {
        printf("Failed to receive message from client. Error: %"PRIu32"\n", result);
        return result;
    }

    printf("===============================================================\n");
    printf("Press the user button to send LED ON/OFF command to the UDP client\n");

    return result;
}

/*******************************************************************************
* Function Name: create_udp_server_socket
*******************************************************************************
* Summary:
*  Function to create a socket and set the socket options
*
* Parameters:
*  void : Task parameter defined during task creation (unused)
*
* Return:
* cy_rslt_t : Returns CY_RSLT_SUCCESS if the UDP server socket is created
* successfully
*
*******************************************************************************/
cy_rslt_t create_udp_server_socket(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Variable used to set socket options. */
    cy_socket_opt_callback_t udp_recv_option = {
            .callback = udp_server_recv_handler,
            .arg = NULL
    };

    /* Create a UDP server socket. */
    result = cy_socket_create(CY_SOCKET_DOMAIN_AF_INET, CY_SOCKET_TYPE_DGRAM, CY_SOCKET_IPPROTO_UDP, &server_handle);
    if (CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Register the callback function to handle messages received from UDP client. */
    result = cy_socket_setsockopt(server_handle, CY_SOCKET_SOL_SOCKET,
            CY_SOCKET_SO_RECEIVE_CALLBACK,
            &udp_recv_option, sizeof(cy_socket_opt_callback_t));
    if (CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Bind the UDP socket created to Server IP address and port. */
    result = cy_socket_bind(server_handle, &udp_server_addr, sizeof(udp_server_addr));
    if (CY_RSLT_SUCCESS == result)
    {
         printf("Socket bound to port: %d\n", udp_server_addr.port);
    }

    return result;
}

/*******************************************************************************
* Function Name: connect_to_wifi_ap()
*******************************************************************************
* Summary:
*  Connects to Wi-Fi AP using the user-configured credentials, retries up to a
*  configured number of times until the connection succeeds.
*
*  Parameters:
*  void : Task parameter defined during task creation (unused)
*
* Return:
* cy_rslt_t : Returns CY_RSLT_SUCCESS if the Wi-Fi AP connection is successful.
*******************************************************************************/
cy_rslt_t connect_to_wifi_ap(void)
{
    cy_rslt_t result;

    /* Variables used by Wi-Fi connection manager. */
    cy_wcm_connect_params_t wifi_conn_param;

    cy_wcm_ip_address_t ip_address;

    /* Variable to track the number of connection retries to the Wi-Fi AP specified
     * by WIFI_SSID macro */
    int conn_retries = 0;

    app_sdio_init();

    wcm_config.interface = CY_WCM_INTERFACE_TYPE_STA;
    wcm_config.wifi_interface_instance = &sdio_instance;

    /* Initialize Wi-Fi connection manager. */
    result = cy_wcm_init(&wcm_config);

    if (CY_RSLT_SUCCESS != result)
    {
        printf("Wi-Fi Connection Manager initialization failed!\n");
        handle_app_error();
    }
    printf("Wi-Fi Connection Manager initialized. \n");

    /* Set the Wi-Fi SSID, password and security type. */
    memset(&wifi_conn_param, RESET_VAL, sizeof(cy_wcm_connect_params_t));
    memcpy(wifi_conn_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
    memcpy(wifi_conn_param.ap_credentials.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
    wifi_conn_param.ap_credentials.security = WIFI_SECURITY_TYPE;

    /* Join the Wi-Fi AP. */
    for(conn_retries = 0; conn_retries < MAX_WIFI_CONN_RETRIES; conn_retries++ )
    {
        result = cy_wcm_connect_ap(&wifi_conn_param, &ip_address);

        if(CY_RSLT_SUCCESS == result)
        {
            printf("Successfully connected to Wi-Fi network '%s'.\n",
                    wifi_conn_param.ap_credentials.SSID);
            printf("IP Address Assigned: %d.%d.%d.%d\n", (uint8)ip_address.ip.v4,
                    (uint8)(ip_address.ip.v4 >> SHIFT_BIT_8), (uint8)(ip_address.ip.v4 >> SHIFT_BIT_16),
                    (uint8)(ip_address.ip.v4 >> SHIFT_BIT_24));

            /* IP address and UDP port number of the UDP server */
            udp_server_addr.ip_address.ip.v4 = ip_address.ip.v4;
            udp_server_addr.ip_address.version = CY_SOCKET_IP_VER_V4;
            udp_server_addr.port = UDP_SERVER_PORT;
            return result;
        }

        printf("Connection to Wi-Fi network failed with error code %d."
                "Retrying in %d ms...\n", (int)result, WIFI_CONN_RETRY_INTERVAL_MSEC);

        vTaskDelay(pdMS_TO_TICKS(WIFI_CONN_RETRY_INTERVAL_MSEC));
    }

    /* Stop retrying after maximum retry attempts. */
    printf("Exceeded maximum Wi-Fi connection attempts\n");

    return result;
}

/*******************************************************************************
* Function Name: user_button_interrupt_handler
********************************************************************************
*
* Summary:
*   This interrupt handler enables or disables GATT notifications upon button
*   press.
*
*******************************************************************************/
void user_button_interrupt_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Variable to hold the LED ON/OFF command to be sent to the UDP client. */
    uint32_t led_state_cmd;

    if (Cy_GPIO_GetInterruptStatus(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN))
    {
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN);
        NVIC_ClearPendingIRQ(CYBSP_USER_BTN1_IRQ);

        if ((!Cy_GPIO_Read(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN)))
        {

            if (!button_debouncing)
            {
                /* Set the debouncing flag */
                button_debouncing = true;

                /* Record the current timestamp */
                button_debounce_timestamp = (uint32_t) (xTaskGetTickCount()
                    * portTICK_PERIOD_MS);
            }

            if (button_debouncing && (((xTaskGetTickCount() *
                    portTICK_PERIOD_MS)) - button_debounce_timestamp <=
                    DEBOUNCE_TIME_MS * portTICK_PERIOD_MS))
            {
                button_debouncing = false;

                /* Set the command to be sent to UDP client. */
                if(led_state == CYBSP_LED_STATE_ON)
                {
                led_state_cmd = LED_OFF_CMD;
                }
                else
                {
                led_state_cmd = LED_ON_CMD;
                }

                /* Update the button pressed flag*/
                button_pressed = true;

                /* Set the flag to send command to UDP client. */
                xTaskNotifyFromISR(server_task_handle, led_state_cmd,
                eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);
           }
        }
    }

    /* CYBSP_USER_BTN1 (SW2) and CYBSP_USER_BTN2 (SW4) share the same port and
    * hence they share the same NVIC IRQ line. Since both the buttons are
    * configured for falling edge interrupt in the BSP, pressing any button
    * will trigger the execution of this ISR. Therefore, we must clear the
    * interrupt flag of the user button (CYBSP_USER_BTN2) to avoid issues in
    * case if user presses BTN2 by mistake.
    */
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_PIN);
    NVIC_ClearPendingIRQ(CYBSP_USER_BTN2_IRQ);

    /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE. */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************
* Function Name: udp_server_task
*******************************************************************************
* Summary:
*  Task used to establish a connection to a remote UDP client.
*
* Parameters:
*  void *args : Task parameter defined during task creation (unused)
*
* Return:
*  void
*
*******************************************************************************/
void udp_server_task(void *arg)
{
    cy_rslt_t result;
    uint8_t tasknotifybits_to_clearonentry = 0;
    uint8_t tasknotifybits_to_clearonexit = 0;

    /* Variable to store number of bytes sent over UDP socket. */
    uint32_t bytes_sent = 0;

    /* Variable to receive LED ON/OFF command from the user button ISR. */
    uint32_t led_state_cmd = LED_OFF_CMD;

    /* CYBSP_USER_BTN1 (SW2) and CYBSP_USER_BTN2 (SW4) share the same port and
    * hence they share the same NVIC IRQ line. Since both are configured in the BSP
    * via the Device Configurator, the interrupt flags for both the buttons are set
    * right after they get initialized through the call to cybsp_init(). The flags
    * must be cleared otherwise the interrupt line will be constantly asserted.
    */
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN);
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_PIN);
    NVIC_ClearPendingIRQ(CYBSP_USER_BTN1_IRQ);
    NVIC_ClearPendingIRQ(CYBSP_USER_BTN2_IRQ);

    /* Initialize the interrupt and register interrupt callback */
    cy_en_sysint_status_t btn_interrupt_init_status = Cy_SysInt_Init(&sysint_cfg,
         &user_button_interrupt_handler);

    if(CY_SYSINT_SUCCESS != btn_interrupt_init_status)
    {
        handle_app_error();
    }

    /* Enable the interrupt in the NVIC */
    NVIC_EnableIRQ(sysint_cfg.intrSrc);

    /* Connect to Wi-Fi AP */
    if(CY_RSLT_SUCCESS != connect_to_wifi_ap())
    {
        printf("\n Failed to connect to Wi-Fi AP.\n");
        handle_app_error();
    }

    /* Secure Sockets initialization */
    result = cy_socket_init();
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Secure Sockets initialization failed!\n");
        handle_app_error();
    }
    printf("Secure Sockets initialized\n");

    /* Create UDP Server*/
    result = create_udp_server_socket();
    if (CY_RSLT_SUCCESS != result)
    {
        printf("UDP Server Socket creation failed. Error: %"PRIu32"\n", result);
        handle_app_error();
    }

    /* Toggle the LED on/off command sent to client on every button press */
    while(true)
    {
        /* Wait until a notification is received from the user button ISR. */
        xTaskNotifyWait(tasknotifybits_to_clearonentry, tasknotifybits_to_clearonexit, &led_state_cmd, portMAX_DELAY);

        if(true == button_pressed)
        {
            
            button_pressed = false;

            /* Send LED ON/OFF command to UDP client. */
            if(client_connected)
            {
                result = cy_socket_sendto(server_handle, &led_state_cmd, UDP_LED_CMD_LEN, CY_SOCKET_FLAGS_NONE,
                                          &peer_addr, sizeof(cy_socket_sockaddr_t), &bytes_sent);
                if(CY_RSLT_SUCCESS == result)
                {
                    if(led_state_cmd == LED_ON_CMD)
                    {
                        printf("LED ON command sent to UDP client\n");
                    }
                    else
                    {
                        printf("LED OFF command sent to UDP client\n");
                    }
                }
                else
                {
                    printf("Failed to send command to client. Error: %"PRIu32"\n", result);
                }
            }
        }
    }
 }

/* [] END OF FILE */
