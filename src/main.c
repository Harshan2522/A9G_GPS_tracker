#include <string.h>
#include <stdio.h>
// #include "math.h"
#include "time.h"
#include <ctype.h>

#include "api_hal_gpio.h"
#include <api_os.h>
#include <api_event.h>
#include <api_debug.h>
#include "api_hal_pm.h"
#include "api_info.h"
#include "api_sms.h"

#include "api_network.h"
#include "api_socket.h"

#include <api_gps.h>
#include <api_hal_uart.h>

#include "gps_parse.h"
#include "gps.h"
#include "buffer.h"
#include "assert.h"

// #include "sms_lib.c"
// #include "str_utils.c"

/**
 * gps tracker, use an open source tracker server traccar:https://www.traccar.org/
 * the server in the code(`#define SERVER_IP   "ss.neucrack.com"`) may invalid someday, you can download the server and deploy youself
 * How to use:
 *          compile and download to A9G dev board, open browser, access http://ss.neucrack.com:8082 ,
 *          then register and login, add devices and the number is IMEI e.g. `867959033006999`, finally the position of your device will be show in the map
 *
 * @attention The code below just a demo, please read and check the code carefully before copy to your project directly(DO NOT copy directly)!!!!!!
 *
 *
 */
#define MAIN_TASK_STACK_SIZE (2048 * 2)
#define MAIN_TASK_PRIORITY 0
#define MAIN_TASK_NAME "device_main_task"

#define SECOND_TASK_STACK_SIZE (2048 * 2)
#define SECOND_TASK_PRIORITY 1
#define SECOND_TASK_NAME "device__task"

#define REPORT_INTERVAL 2000

// const uint8_t nmea[]="$GNGGA,000021.263,2228.7216,N,11345.5625,E,0,0,,153.3,M,-3.3,M,,*4E\r\n$GPGSA,A,1,,,,,,,,,,,,,,,*1E\r\n$BDGSA,A,1,,,,,,,,,,,,,,,*0F\r\n$GPGSV,1,1,00*79\r\n$BDGSV,1,1,00*68\r\n$GNRMC,000021.263,V,2228.7216,N,11345.5625,E,0.000,0.00,060180,,,N*5D\r\n$GNVTG,0.00,T,,M,0.000,N,0.000,K,N*2C\r\n";

#define MASTER_NUMBER_COUNT 2
#define LED_STATE_ON GPIO_LEVEL_LOW

char *MASTER_NUMBERS[MASTER_NUMBER_COUNT] = {"+917904817218", "+919486644311"};

const GPIO_PIN SOS_button = GPIO_PIN1;
const GPIO_PIN GPIO_GPS = GPIO_PIN27;
const GPIO_PIN GPIO_NETWORK = GPIO_PIN28;

static HANDLE eventTaskHandle = NULL;
// static HANDLE smsGpioTaskHandle = NULL;

double latitude;
double longitude;

bool networkFlag = false;
bool isGprsReady = false;
bool SOS_enabled = false;
bool isGpsReady = false;

void lowerStr(char *str)
{
    for (int i = 0; str[i]; i++)
    {
        if (str[i] >= 'A' && str[i] <= 'Z')
        {
            str[i] -= 'A' - 'a';
        }
    }
}

void clearStr(char *str)
{
    for (int i = 0; str[i]; i++)
    {
        if (str[i] == '\r' || str[i] == '\n' || (str[i] >= ' ' && str[i] <= '~'))
        {
            continue;
        }

        str[i] = '?';
    }
}

void removeChar(char *s, int c)
{
    // in:abcd123def,'d'
    // out:abc123ef
    int j, n = strlen(s);
    for (int i = j = 0; i < n; i++)
        if (s[i] != c)
            s[j++] = s[i];

    s[j] = '\0';
}

void InitSMS()
{
    Trace(1, "InitSMS Start");

    if (!SMS_SetFormat(SMS_FORMAT_TEXT, SIM0))
    {
        Trace(1, "Setting format failed");
        return;
    }
    SMS_Parameter_t smsParam = {
        .fo = 17,
        .vp = 167,
        .pid = 0,
        .dcs = 8, // 0:English 7bit, 4:English 8 bit, 8:Unicode 2 Bytes
    };
    if (!SMS_SetParameter(&smsParam, SIM0))
    {
        Trace(1, "Setting parameter failed");
        return;
    }
    if (!SMS_SetNewMessageStorage(SMS_STORAGE_SIM_CARD))
    {
        Trace(1, "Setting message storage failed");
        return;
    }
    Trace(1, "InitSMS End");
}

void SendUtf8Sms(char *phoneNumber, char *msg)
{
    uint8_t *unicode = NULL;
    uint32_t unicodeLen;
    clearStr(msg);
    Trace(1, "SMS to send:length=%d, msg:%s", strlen(msg), msg);

    if (!SMS_LocalLanguage2Unicode(msg, strlen(msg), CHARSET_UTF_8, &unicode, &unicodeLen))
    {
        Trace(1, "Converting local to unicode failed");
        return;
    }
    if (!SMS_SendMessage(phoneNumber, unicode, unicodeLen, SIM0))
    {
        Trace(1, "Sending SMS failed");
    }
    OS_Free(unicode);
}

void ClearSmsStorage()
{
    SMS_Storage_Info_t storageInfo;

    SMS_GetStorageInfo(&storageInfo, SMS_STORAGE_SIM_CARD);
    Trace(1, "SMS storage of sim card: used=%d, total=%d", storageInfo.used, storageInfo.total);

    if (storageInfo.used > 10)
    {
        for (int i = 1; i <= storageInfo.total; i++)
        {
            if (!SMS_DeleteMessage(i, SMS_STATUS_ALL, SMS_STORAGE_SIM_CARD))
                Trace(1, "Deleting SIM sms fail at index %d", i);
            // else
            // Trace(1, "delete SIM sms success");
        }
        Trace(1, "Cleaning all SIM sms done");
    }
}

void Get_PhoneNumer(char *msgHeader, char *phoneNumber)
{
    // Trace(1, "msgHeader:%s", msgHeader);
    //"+91_sender_phone_number",fdsfsd,fdsfdsf,"+91_sms_center"
    int len = strlen(msgHeader);
    for (int i = 1; i < len; i++)
    {
        if (msgHeader[i] == '\"')
        {
            phoneNumber[i - 1] = 0;
            break;
        }
        phoneNumber[i - 1] = msgHeader[i];
    }
    // Trace(1, "phoneNumber:%s", phoneNumber);
}

bool Is_Master_Phone_Number(char *number)
{
    for (int i = 0; i < MASTER_NUMBER_COUNT; i++)
    {
        if (!strcmp(MASTER_NUMBERS[i], number))
            return true;
    }

    return false;
}

void Init_Pins()
{
    // wait for System to be ready
    GPIO_config_t gpioConfig = {
        .mode = GPIO_MODE_OUTPUT,
        .defaultLevel = GPIO_LEVEL_LOW};

    GPIO_config_t gpioConfigbutton = {
        .mode = GPIO_MODE_INPUT,
        .defaultLevel = GPIO_LEVEL_HIGH};

    PM_PowerEnable(GPIO_GPS, true);
    PM_PowerEnable(GPIO_NETWORK, true);
    PM_PowerEnable(SOS_button, true);

    gpioConfig.pin = GPIO_GPS;
    GPIO_Init(gpioConfig);

    gpioConfig.pin = GPIO_NETWORK;
    GPIO_Init(gpioConfig);

    gpioConfigbutton.pin = SOS_button;
    GPIO_Init(gpioConfigbutton);
}

// send the system report
void Send_SMS_Report(char *phoneNumber)
{
    GPIO_LEVEL level;
    char *status1, *status2, msg[300];

    // Get the status of LED 1
    GPIO_Get(GPIO_GPS, &level);
    status1 = level == LED_STATE_ON ? "on" : "off";

    // Get the status of LED 2
    GPIO_Get(GPIO_NETWORK, &level);
    status2 = level == LED_STATE_ON ? "on" : "off";

    uint8_t percent;
    uint16_t v = PM_Voltage(&percent);

    // build into message content
    snprintf(msg, sizeof(msg), "Status:\nGPS is %s\nNETWORK is %s \nBattery Power \nVoltage = %d volts \nPercentage = %d %% ", status1, status2, v, percent);
    /*
        status
        example:-
        GPS on
        NTEWORK off
        battery Volt & % power
    */

    // Send a message
    SendUtf8Sms(phoneNumber, msg);
    Trace(1, "Report SMS: %s", msg);
}

// processing an SMS message
bool Send_SMS_GPS(char *phoneNumber)
{
    char msg[100];
    snprintf(msg, sizeof(msg), "http://maps.google.com/maps?q=%lf+%lf", latitude, longitude);
    SendUtf8Sms(phoneNumber, msg);
    Trace(1, "Report SMS: %s", msg);
    return true;
}
void Handle_SMS(char *phoneNumber, char *smsContent)
{
    Trace(1, "Command: %s", smsContent);

    // Convert messages to lowercase
    lowerStr(smsContent);
    bool shouldReport = false;
    char *line;

    // Cut each line of the message, each line is a command
    // Example of a message:
    /*
        GPS led1
        NETWORK led2
        battery power
        report
    */
    while ((line = strsep(&smsContent, "\n")))
    {
        // If there is a "report" command, after processing all commands, you will have to send a report
        if (!strcmp(line, "report"))
        {
            shouldReport = true;
        }
        else if (!strcmp(line, "location"))
        {
            if (!Send_SMS_GPS(phoneNumber))
            {
                Trace(1, "failed to send gps location");
            }
        }
        else
        {
            // Process on/off commands
            Trace(1, "the cmt is : %s", line);
            // Handle_Command(line);
        }
    }
    char *phone = phoneNumber;
    if (shouldReport && Is_Master_Phone_Number(phone))
    {
        // Submit Report

        Send_SMS_Report(phoneNumber);
    }

    Trace(1, "Command handled");
}

void Event_Dispatch(API_Event_t *pEvent)
{
    switch (pEvent->id)
    {
    case API_EVENT_ID_NO_SIMCARD:
        Trace(10, "!!NO SIM CARD %d!!!!", pEvent->param1);
        networkFlag = false;
        break;
    case API_EVENT_ID_NETWORK_REGISTER_SEARCHING:
        Trace(2, "network register searching");
        networkFlag = false;
        break;
    case API_EVENT_ID_NETWORK_REGISTER_DENIED:
        Trace(2, "network register denied");
    case API_EVENT_ID_NETWORK_REGISTER_NO:
        Trace(2, "network register no");
        break;
    case API_EVENT_ID_SYSTEM_READY:
        networkFlag = true;
        // Initialize GPIO
        Init_Pins();

        // Clear the message saver
        ClearSmsStorage();

        Trace(1, "System initialized");
        break;
    case API_EVENT_ID_SMS_SENT:
        // Log successful delivery
        Trace(1, "SMS sent");
        break;
    case API_EVENT_ID_NETWORK_REGISTERED_HOME:
    case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
    {
        uint8_t status;
        Trace(2, "network register success");
        bool ret = Network_GetAttachStatus(&status);
        if (!ret)
            Trace(1, "get attach staus fail");
        Trace(1, "attach status:%d", status);
        if (status == 0)
        {
            ret = Network_StartAttach();
            if (!ret)
            {
                Trace(1, "network attach fail");
                isGprsReady = false;
            }
        }
        else
        {
            Network_PDP_Context_t context = {
                .apn = "www",
                .userName = "",
                .userPasswd = ""};
            Network_StartActive(context);
        }
        break;
    }
    case API_EVENT_ID_NETWORK_ATTACHED:
        Trace(2, "network attach success");
        Network_PDP_Context_t context = {
            .apn = "www",
            .userName = "",
            .userPasswd = ""};
        Network_StartActive(context);
        isGprsReady = true;
        break;

    case API_EVENT_ID_SMS_RECEIVED: // modification need to be done
        Trace(1, "message recived");
        char *header = pEvent->pParam1;
        char *content = pEvent->pParam2;

        Trace(1, "Received msg length:%d, content:%s", strlen(content), content);
        char phoneNumber[20];

        // extract the phonenumber
        Get_PhoneNumer(header, phoneNumber);

        // check if the phonenumber is the master's phonenumber
        if (Is_Master_Phone_Number(phoneNumber))
        {
            Trace(1, "SMS from the master");

            // handle the commentform the master
            Handle_SMS(phoneNumber, content);
        }
        else
        {
            Trace(1, "Message form the stranger or [QC],header:%s", header);
        }
        OS_Free(phoneNumber);
        break;
    case API_EVENT_ID_GPS_UART_RECEIVED:
        GPS_Update(pEvent->pParam1, pEvent->param1);
        break;
    default:
        break;
    }
}

// start of the GPS
bool Start_GPS(GPS_Info_t *gpsInfo)
{
    uint8_t buffer[300];
    bool result = false;

    // open GPS hardware(UART2 open either)
    GPS_Init();
    GPS_Open(NULL);

    // wait for gps start up, or gps will not response command
    while (gpsInfo->rmc.latitude.value == 0)
    {
        Trace(1, "Starting GPS");
        OS_Sleep(1000);
    }
    // init GPS: try 5 times setting gps nmea output interval to 10s, break at first success
    for (uint8_t i = 0; i < 5; ++i)
    {
        if (GPS_SetOutputInterval(10000))
        {
            result = true;
            break;
        }
        else
        {
            Trace(1, "Setting nmea output interval failed");
            result = false;
        }
        OS_Sleep(1000);
    }
    if (!result)
        return result;

    if (!GPS_ClearInfoInFlash())
    {
        Trace(1, "erase gps fail");
        return false;
    }

    if (!GPS_SetQzssOutput(false))
    {
        Trace(1, "enable qzss nmea output fail");
        return false;
    }

    if (!GPS_SetSearchMode(true, false, true, false))
    {
        Trace(1, "set search mode fail");
        return false;
    }

    if (!GPS_SetSBASEnable(true))
    {
        Trace(1, "enable sbas fail");
        return false;
    }

    if (!GPS_SetFixMode(GPS_FIX_MODE_LOW_SPEED))
    {
        Trace(1, "set fix mode fail");
        return false;
    }

    if (!GPS_GetVersion(buffer, 150))
    {
        Trace(1, "Getting GPS firmware version failed");
        return false;
    }
    else
        Trace(1, "GPS firmware version:%s", buffer);

    // set gps nmea output interval to 1s
    if (!GPS_SetOutputInterval(1000))
    {
        Trace(1, "Setting nmea output interval failed");
        return false;
    }

    return true;
}

void gpsTaskHandle(void *pData)
{
    GPS_Info_t *gpsInfo = Gps_GetInfo();
    static int count = 0;
    char buffer[100];
    if (count > 60)
    {
        count = 0;
        if (isGpsReady)
        {
            uint8_t isFixed = gpsInfo->gsa[0].fix_type > gpsInfo->gsa[1].fix_type ? gpsInfo->gsa[0].fix_type : gpsInfo->gsa[1].fix_type;
            char *isFixedStr = NULL;
            if (isFixed == 2)
                isFixedStr = "2D fix";
            else if (isFixed == 3)
            {
                if (gpsInfo->gga.fix_quality == 1)
                    isFixedStr = "3D fix";
                else if (gpsInfo->gga.fix_quality == 2)
                    isFixedStr = "3D/DGPS fix";
            }
            else
                isFixedStr = "no fix";

            // convert unit ddmm.mmmm to degree(°)
            int temp = (int)(gpsInfo->rmc.latitude.value / gpsInfo->rmc.latitude.scale / 100);
            latitude = temp + (double)(gpsInfo->rmc.latitude.value - temp * gpsInfo->rmc.latitude.scale * 100) / gpsInfo->rmc.latitude.scale / 60.0;
            temp = (int)(gpsInfo->rmc.longitude.value / gpsInfo->rmc.longitude.scale / 100);
            longitude = temp + (double)(gpsInfo->rmc.longitude.value - temp * gpsInfo->rmc.longitude.scale * 100) / gpsInfo->rmc.longitude.scale / 60.0;

            // you can copy ` latitude,longitude ` to http://www.gpsspg.com/maps.htm check location on map

            snprintf(buffer, sizeof(buffer), "GPS fix mode:%d, BDS fix mode:%d, fix quality:%d, satellites tracked:%d, gps sates total:%d, is fixed:%s, coordinate:WGS84, Latitude:%f, Longitude:%f, unit:degree,altitude:%f", gpsInfo->gsa[0].fix_type, gpsInfo->gsa[1].fix_type,
                     gpsInfo->gga.fix_quality, gpsInfo->gga.satellites_tracked, gpsInfo->gsv[0].total_sats, isFixedStr, latitude, longitude, gpsInfo->gga.altitude);
            // show in tracer
            Trace(2, buffer);
        }
    }
    else if (count <= 60)
    {
        ++count;
    }

    OS_StartCallbackTimer(eventTaskHandle, 1000, gpsTaskHandle, NULL);
}

void Button_task(void *pData)
{
    static int cnt = 0;
    GPIO_LEVEL state;
    GPIO_LEVEL stateNET, stateGPS;
    // wait till the NETWORK to get attached with the nearest service provider
    while (!networkFlag)
        OS_Sleep(1000);
    GPS_Info_t *gpsInfo = Gps_GetInfo();

    // wait for gprs register complete
    // The process of GPRS registration network may cause the power supply voltage of GPS to drop,
    // which resulting in GPS restart.
    while (!isGprsReady)
    {
        Trace(1, "Registering GPRS");
        OS_Sleep(2000);
    }
    isGpsReady = Start_GPS(gpsInfo);

    stateGPS = (isGpsReady) ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW;
    GPIO_Set(GPIO_GPS, stateGPS);

    Trace(1, "Start of GPS is %s", isGpsReady ? "OK" : "FAILED");

    while (GPIO_Get(SOS_button, &state))
    {
        stateNET = (networkFlag) ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW;
        GPIO_Set(GPIO_NETWORK, stateNET);
        if (state)
        {
            SOS_enabled = true;
            for (int i = 0; i < 5; i++)
            {
                OS_Sleep(1000);
                Trace(1, "SOS is pressed: %d sec", i);
            }
            GPIO_Get(SOS_button, &state);
            if (state)
            {
                while (cnt < 6)
                {

                    Send_SMS_Report(MASTER_NUMBERS[0]);
                    //  Send_SMS_Report(MASTER_NUMBERS[1]); // sending data to dharaneesh
                    if (!Send_SMS_GPS(MASTER_NUMBERS[0]))
                    {
                        Trace(1, "failed to send gps location");
                    }
                    else
                    {
                        Trace(1, "Sent the gps location");
                    }
                    // if (!Send_SMS_GPS(MASTER_NUMBERS[1]))
                    // {
                    //     Trace(1, "failed to send gps location");
                    // }
                    ++cnt;
                    OS_Sleep(5000);
                }
                cnt = 0;
            }
        }
    }
    Trace(1, "out of the button task , restart the device");
    OS_Sleep(5000);
}

void Event_Task(void *pData)
{
    API_Event_t *event = NULL;

    // Launch a new stream named Blink_Task to flash LED indicating active module status
    OS_CreateTask(Button_task, NULL, NULL, SECOND_TASK_STACK_SIZE, SECOND_TASK_PRIORITY, 0, 0, SECOND_TASK_NAME);

    OS_StartCallbackTimer(eventTaskHandle, 1000, gpsTaskHandle, NULL);
    // endless loops, listening and processing events.
    while (1)
    {
        // continuosly listenting to the occuring event
        if (OS_WaitEvent(eventTaskHandle, (void **)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            Event_Dispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}

void tracker_Main()
{
    eventTaskHandle = OS_CreateTask(Event_Task,
                                    NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
    OS_SetUserMainHandle(&eventTaskHandle);
}