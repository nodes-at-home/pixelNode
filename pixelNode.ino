// junand 07.01.2017

// ------------------------------------------------------------------------------------------------------------------------------------------

#define HOME_WLAN

// ------------------------------------------------------------------------------------------------------------------------------------------

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ESP8266HTTPClient.h>

#include <Streaming.h>
#include <Ticker.h>
#include <Time.h>
#include <TimeLib.h>

#include <ArduinoJson.h>

// #define MQTT_MAX_PACKET_SIZE 256
#include <PubSubClient.h>

#include "MaxMatrix.h"
#include "Sprites.h"

#include "WLAN.h"
//const char* ssid     = "your_ssid";
//const char* password = "your_password";

// ------------------------------------------------------------------------------------------------------------------------------------------

#define VERSION "V0.10 (pixelNode)"

// ------------------------------------------------------------------------------------------------------------------------------------------

#define MY_SERIAL Serial

// ------------------------------------------------------------------------------------------------------------------------------------------

// const int MATRIX_DATA_PIN = 14;      // grün DIN pin of MAX7219 module     -> D5
// const int MATRIX_LOAD_PIN = 12;      // gelb CS pin of MAX7219 module      -> D6
// const int MATRIX_CLOCK_PIN = 13;     // schwarz CLK pin of MAX7219 module  -> D7

const int MATRIX_DATA_PIN = 13;      // grün DIN pin of MAX7219 module     -> D7
const int MATRIX_LOAD_PIN = 15;      // gelb CS pin of MAX7219 module      -> D8
const int MATRIX_CLOCK_PIN = 14;     // schwarz CLK pin of MAX7219 module  -> D5

const int MATRIX_MAX_IN_USE = 8;    //change this variable to set how many MAX7219's you'll use

MaxMatrix matrix ( MATRIX_DATA_PIN, MATRIX_LOAD_PIN, MATRIX_CLOCK_PIN, MATRIX_MAX_IN_USE ); // define module

// erste Spalte zum Anzeigen, die ersten 64 Pixelspalten werden leer gelassen
int matrixInsertColumn = 0;

int alertDisplayDuration = -1; // in sec, -1 no effect, 0 is forever  !!!like an alert!!!
long alertDisplayBegin;

volatile int matrixDisplayColumn = 0;
volatile int matrixDisplayRow = 0;

// ISR, wird alle 100 ms gerufen
Ticker ticker;
#define DEFAULT_TICKER_DELAY 10
volatile int tickerShiftDelay = DEFAULT_TICKER_DELAY;
const int TICKER_RESOLUTION = 10; // one tick every 10 ms
int tickerCounter = 0;

#define NO_DIRECTION 0
#define HORIZONTAL_DIRECTION 1
#define VERTICAL_DIRECTION 2

struct Command {

    int id;
    int direction;
    int from;
    int to;
    int delay;
    int nextCommandId;
    Command *nextCommand;

    int d;
    boolean running;

};

const int COMMANDS_SIZE = 20;
volatile Command commands [COMMANDS_SIZE];
volatile Command *commandInExecution;
int commandInExecutionIndex = 0;

// ------------------------------------------------------------------------------------------------------------------------------------------

WiFiClient wifiClient;

// ------------------------------------------------------------------------------------------------------------------------------------------

#if defined HOME_WLAN or defined DEV_WLAN
static byte SNTP_SERVER_IP[]    = { 192, 168, 2, 1 }; // ntpd@mauzi
//static byte SNTP_SERVER_IP[]    = { 192, 168, 2, 8 }; // ntpd@glutexo
#else
//uint8_t SNTP_SERVER_IP[]    = { 192, 43, 244, 18};   // time.nist.gov
byte SNTP_SERVER_IP[]    = { 130,149,17,21};      // ntps1-0.cs.tu-berlin.de xxx
//byte SNTP_SERVER_IP[]    = { 192,53,103,108};     // ptbtime1.ptb.de
//byte SNTP_SERVER_IP[]    = { 64, 90, 182, 55 };   // nist1-ny.ustiming.org
//byte SNTP_SERVER_IP[]    = { 66, 27, 60, 10 };    // ntp2d.mcc.ac.uk
//byte SNTP_SERVER_IP[]    = { 130, 88, 200, 4 };   // ntp2c.mcc.ac.uk
//byte SNTP_SERVER_IP[]    = { 31, 193, 9, 10 };    // clock02.mnuk01.burstnet.eu
//byte SNTP_SERVER_IP[]    = { 82, 68, 133, 225 };  // ntp0.borg-collective.org.uk
#endif

const unsigned long SEVENTY_YEARS = 2208988800UL; // offset between ntp and unix time
const unsigned long OFFSET_CEST = 7200L;          // offset in sec for MESZ / CEST
const unsigned long OFFSET_CET = 3600L;           // offset in sec for MEZ  / CET

const unsigned long SYNC_INTERVAL = SECS_PER_HOUR;

WiFiUDP udp;

time_t currentTime, lastTime;

// ------------------------------------------------------------------------------------------------------------------------------------------

const int DEFAULT_DISPLAY_CATEGORY_PERIOD = 5; // sec
int displayCategoryPeriod = DEFAULT_DISPLAY_CATEGORY_PERIOD;
int displayCategoryPeriodCounter = 100;
int displayCategory = -1;
const int MAX_DISPLAY_CATEGORY = 12;

boolean isDisplayCategoryTime = false;

// ------------------------------------------------------------------------------------------------------------------------------------------

const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "ESP8266_pixel_node";

#if defined HOME_WLAN
const IPAddress MQTT_SERVER ( 192, 168, 2, 117 );
#endif

PubSubClient mqttClient ( wifiClient );
long lastMqttReconnect;
const long MQTT_RECONNECT_PERIOD = 5000L;

const char* MQTT_TOPIC_BASE =    "nodes@home/display/pixel/kitchen";
const char* MQTT_TOPIC_COMMAND = "nodes@home/display/pixel/kitchen/command";

const int DEFAULT_ALERT_DISPLAY_DURATION = 15;

// ------------------------------------------------------------------------------------------------------------------------------------------

const int NUM_MESSAGES = 10;
char* msgText [NUM_MESSAGES] [200];
boolean isMsgText [NUM_MESSAGES];
boolean isMsgEnabled [NUM_MESSAGES];

boolean isDisplayTime = true;
boolean isDisplayDate = true;
boolean isDisplayWeekday = false;
int displayBrightness = 0;

// ------------------------------------------------------------------------------------------------------------------------------------------

// const int SETUP_DELAY = 10000;
const int SETUP_DELAY = 1000;

void setup () {

    delay ( 1000 );

    // --- init led matrix ----------------------------------------------------
    matrix.init ( 0x00 ); // module initialize, dot matix intensity 0-15
    clearMatrix ();

    // --- init MY_SERIAL --------------------------------------------------------
    Serial.begin ( 9600 );
    delay ( 2 );
    MY_SERIAL.print ( "\n[SETUP] start version=" ); // first MY_SERIAL print!
    MY_SERIAL.println ( VERSION );

    // --- init interrupt routine ---------------------------------------------
    // Timer1.initialize ( 50 ); // 50 µs, prescale is set by lib
    // Timer1.attachInterrupt ( doInterrupt );
    ticker.attach_ms ( TICKER_RESOLUTION, doInterrupt );

    commandInExecution = NULL;

    // --- version text -------------------------------------------------------
    clearMatrix ();
    matrix.init ();
    matrixAppendText ( "version " );
    matrixAppendText ( VERSION );
    delay ( SETUP_DELAY );

    // --- init wifi ----------------------------------------------------------
    MY_SERIAL.print ( "[SETUP] connecting " );
    MY_SERIAL.print ( ssid );

    clearMatrix ();
    matrix.init ();

    WiFi.mode ( WIFI_STA );
    WiFi.begin ( ssid, password );
    while ( WiFi.status () != WL_CONNECTED ) {
        delay ( 500 );
        matrixAppendText ( "." );
        MY_SERIAL.print ( '.' );
    }

    MY_SERIAL << endl << "[SETUP] connected with ip ";
    MY_SERIAL.println ( WiFi.localIP () );

    clearMatrix ();
    matrix.init ();
    matrixAppendText ( "connected with ip " );
    matrixAppendText ( WiFi.localIP ().toString ().c_str () );

    // --- init ntp -----------------------------------------------------------
    MY_SERIAL.print ( "[SETUP] ntp server=" );
    for ( int i = 0; i < 4; i++ ) {
        MY_SERIAL.print ( SNTP_SERVER_IP [i] );
        if ( i < 3 ) MY_SERIAL.print ( "." );
        else MY_SERIAL.println ();
    }
    udp.begin ( 8888 ); // for connect to time server
    setSyncInterval ( SYNC_INTERVAL );
    setSyncProvider ( getNtpTime );
    MY_SERIAL.print ( "[SETUP] now: " );
    digitalClockDisplay ( now () );

	// --- msgText -------------------------------------------------------------
	
	for ( int i = 0; i < NUM_MESSAGES; i++ ) {
		isMsgText [i] = false;
		isMsgEnabled [i] = true;
	}

    // --- init mqtt -----------------------------------------------------------
    MY_SERIAL << "[SETUP] mqtt: server=" << MQTT_SERVER << " port=" << MQTT_PORT << " version=" << MQTT_VERSION << endl;
    mqttClient.setServer ( MQTT_SERVER, MQTT_PORT );
    mqttClient.setCallback ( mqttCallback );
    // mqttClient.setClient ( wifiClient );

    // --- display message -----------------------------------------------------

    delay ( SETUP_DELAY );
    clearMatrix ();

}

// ------------------------------------------------------------------------------------------------------------------------------------------

/*
// all top level attributes are optional
{
    "display" : {
        "duration" : nn,
		"brightness" : nn,
        "time" : true,
        "date" : true,
        "weekday" : true,
		"enabled" : [ true, ..., false ]     // flags for 10 messages
    },
    "messages" : [ // no is 0 .. 9
        { "line" : n, "text" : "string", "clear" : true },      // clear "overwrites" text, clear = false has no effect
    ],
    "alert" : {
        "duration" : nn,
        "text" : "string"
    }
}

*/

void mqttCallback ( char* topic, byte* payload, unsigned int payloadLength ) {

    MY_SERIAL.print ( "[MQTT] payloadLength=" );
    MY_SERIAL.println ( payloadLength );

    MY_SERIAL.print ( "]" );
    for ( int i = 0; i < payloadLength; i++ ) MY_SERIAL.print ( (char) payload [i] );
    MY_SERIAL.println ( "[" );

    char buf [512] = { 0 };
    int len = payloadLength < sizeof ( buf ) ? payloadLength : sizeof ( buf ) - 1;
    strncpy ( buf, (const char*) payload, len );
    buf [len] = '\0';

    StaticJsonBuffer<512> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject ( buf );

    if ( strcmp ( topic, MQTT_TOPIC_COMMAND ) == 0 ) {

        MY_SERIAL << "[MQTT] command topic" << endl;

        if  ( root.success () ) {
            // MY_SERIAL << "[MQTT] success" << endl;
            if ( root.containsKey ( "display" ) ) {
                // MY_SERIAL << "[MQTT] display" << endl;
                JsonObject& display = root ["display"];
                if ( display.containsKey ( "duration" ) ) {
                    int duration = display ["duration"];
                    if ( duration > 0 && duration < 20 ) {
                        displayCategoryPeriod = duration;
                    }
                }
                if ( display.containsKey ( "brightness" ) ) {
                    int brightness = display ["brightness"];
                    if ( brightness >= 0 && brightness < 16 ) {
                        displayBrightness = brightness;
                    }
                }
                if ( display.containsKey ( "time" ) ) {
                    String state = display ["time"];
                    isDisplayTime = state == "on" ? true : false;
                }
                if ( display.containsKey ( "date" ) ) {
                    String state = display ["date"];
                    isDisplayDate = state == "on" ? true : false;
                }
                if ( display.containsKey ( "weekday" ) ) {
                    String state = display ["weekday"];
                    isDisplayWeekday = state == "on" ? true : false;
                }
				if ( display.containsKey ( "enabled" ) && display ["enabled"].is<JsonArray&> () ) {
					JsonArray& enabled = display ["enabled"];
					for ( int i = 0; i < NUM_MESSAGES; i++ ) {
						String state = enabled.get<String> ( i );
						// MY_SERIAL << "i=" << i << " state=" << state << endl;
						isMsgEnabled [i] = state == "off" ? false : true;
					}
				}
            }
            if ( root.containsKey ( "messages" ) ) {
                // MY_SERIAL << "[MQTT] messages" << endl;
                const JsonArray& jsonArray = root ["messages"];
                const int size = jsonArray.size ();
                MY_SERIAL.printf ( "messages [] size=%d\n", size );
                for ( int i = 0; i < size; i++ ) {
                    const int line = jsonArray [i] ["line"];
                    const String text = jsonArray [i] ["text"];
                    const boolean clear = jsonArray [i] ["clear"];
                    // MY_SERIAL << "i=" << i << " line=" << line << " text=" << text << " clear=" << clear << endl;
                    if ( line >= 0 && line < 10 ) {
                        if ( clear ) {
                            isMsgText [line] = false;
                        }
                        else {
                            isMsgText [line] = true;
                            snprintf ( (char*) (msgText + line), 200, "%s", text.c_str () );
                        }
                    }
                }
            }
            if ( root.containsKey ( "alert" ) ) {
                MY_SERIAL << "[MQTT] alert" << endl;
                JsonObject& alert = root ["alert"];
                if ( alert.containsKey ( "text" ) ) {
                    const int duration = alert ["duration"];
                    alertDisplayDuration = duration > 0 ? duration : DEFAULT_ALERT_DISPLAY_DURATION;
                    const char* text = alert ["text"];
                    clearMatrix ();
                    matrix.init ();
                    matrixAppendText ( text );
                }
            }
        }
        else {
            MY_SERIAL.println ( "parsing failed" );
        }

    }

}

void mqttReconnect () {

    // Loop until we're reconnected
    if ( !mqttClient.connected () && (lastMqttReconnect + MQTT_RECONNECT_PERIOD) < millis () ) {

        MY_SERIAL.print ( "[MQTT] Attempting MQTT connection ... " );

        lastMqttReconnect = millis ();

        // Attempt to connect
        if ( mqttClient.connect ( MQTT_CLIENT_ID ) ) {

            MY_SERIAL.println ( "connected" );
            // Once connected, publish an announcement...
            mqttClient.publish ( MQTT_TOPIC_BASE, VERSION, true ); // retained = true	
            // ... and resubscribe
            boolean rc = mqttClient.subscribe ( MQTT_TOPIC_COMMAND );
            MY_SERIAL << "[MQTT] mqttClient.subscribe rc=" << rc << endl;

        }
        else {

            MY_SERIAL.print ( "failed, rc=" );
            MY_SERIAL.print ( mqttClient.state () );
            MY_SERIAL.println ( " try again in 5 seconds" );

        }
    }
}

// ------------------------------------------------------------------------------------------------------------------------------------------

void clearMatrix () {

    matrix.clear ();
    // matrixInsertColumn = NUM_DISPLAY_COLS;
    matrixInsertColumn = 0;
    matrixDisplayColumn = 0;
    matrixDisplayRow = 0;

}

// ------------------------------------------------------------------------------------------------------------------------------------------

void clearCommand ( int i ) {

    commands [i].id = -1;
    commands [i].direction = NO_DIRECTION;
    commands [i].from = -1;
    commands [i].to = -1;
    commands [i].delay = -1;
    commands [i].nextCommandId = -1;
    commands [i].nextCommand = NULL;
    commands [i].d = 0;
    commands [i].running = false;

}

void clearAllCommands () {

    commandInExecutionIndex = 0;

    for ( int i = 0; i < COMMANDS_SIZE; i++ ) {
        clearCommand ( i );
    }

}

struct Command* findCommand ( int id ) {

    Command *result = NULL;

    for ( int i = 0; i < COMMANDS_SIZE; i++ ) {
        if ( commands [i].id == id ) {
            result = (Command*) &commands [i];
        }
    }

    return result;

}

// ------------------------------------------------------------------------------------------------------------------------------------------

void doInterrupt () {

    volatile int* val;

    tickerCounter++;
    if ( tickerCounter > tickerShiftDelay ) {

        tickerCounter = 0;

        matrix.shiftOutMatrix ( matrixDisplayColumn, matrixDisplayRow );

        volatile Command* cmd = commandInExecution;

        if ( cmd ) {

            if ( cmd->direction == HORIZONTAL_DIRECTION ) val = &matrixDisplayColumn;
            else if ( cmd->direction == VERTICAL_DIRECTION ) val = &matrixDisplayRow;

            if ( cmd->running ) {
                if ( *val == cmd->to ) {
                    setNextCommand ();
                }
                else {
                    *val += cmd->d;
                }
            }
            else { // !running
                cmd->running = true;
                *val = cmd->from;
            }

        }

    }

}

void initCommand ( int id, int direction, int from, int to, int next_id ) {

        commands [commandInExecutionIndex].id = id;
        commands [commandInExecutionIndex].direction = direction;
        commands [commandInExecutionIndex].from = from;
        commands [commandInExecutionIndex].to = to;
        commands [commandInExecutionIndex].delay = DEFAULT_TICKER_DELAY;
        if ( commands [commandInExecutionIndex].from < commands [commandInExecutionIndex].to ) {
            commands [commandInExecutionIndex].d = 1;
        }
        else {
            commands [commandInExecutionIndex].d = -1;
        }
        commands [commandInExecutionIndex].nextCommandId = next_id;

        commandInExecutionIndex++;

}

void startCommand ( int id ) {

    Command *command = findCommand ( id );
    command->running = false;
    commandInExecution = command;

}

void setNextCommand () {

    commandInExecution->running = false;

    if ( commandInExecution->nextCommandId > -1 ) {
        if ( commandInExecution->nextCommand == NULL ) {
            commandInExecution->nextCommand = findCommand ( commandInExecution->nextCommandId );
        }
        commandInExecution = commandInExecution->nextCommand;
        if ( commandInExecution ) commandInExecution->running = false; // damit sich from initialisiert
    }

}

// ------------------------------------------------------------------------------------------------------------------------------------------

void matrixAppendText ( const String str ) {

    matrixAppendText ( str.c_str (), str.length () );

}

void matrixAppendText ( const char* text ) {

    matrixAppendText ( text, strlen ( text ) );

}

/*

    UTF-8 Umlaute

    Ä       C384
    Ö       C396
    Ü       C39C
    ä       C3A4
    ö       C3B6
    ü       C3BC
    ß       C39F

*/

void matrixAppendText ( const char* text, const int length ) {

    commandInExecution = NULL;

    for ( int i = 0; i < length; i++ ) {

        if ( text [i] == 0xC3 ) { // special handling for german umlaute

            switch ( text [++i] ) {
                case 0x84: // Ä     sprite index 95
                    matrixInsertColumn += matrix.printChar ( 32 + 95, matrixInsertColumn );
                    break;
                case 0x96: // Ö     sprite index 96
                    matrixInsertColumn += matrix.printChar ( 32 + 96, matrixInsertColumn );
                    break;
                case 0x9C: // Ü     sprite index 97
                    matrixInsertColumn += matrix.printChar ( 32 + 97, matrixInsertColumn );
                    break;
                case 0xA4: // ä     sprite index 98
                    matrixInsertColumn += matrix.printChar ( 32 + 98, matrixInsertColumn );
                    break;
                case 0xB6: // ö     sprite index 99
                    matrixInsertColumn += matrix.printChar ( 32 + 99, matrixInsertColumn );
                    break;
                case 0xBC: // ü     sprite index 100
                    matrixInsertColumn += matrix.printChar ( 32 + 100, matrixInsertColumn );
                    break;
                case 0x9F: // ü     sprite index 101
                    matrixInsertColumn += matrix.printChar ( 32 + 101, matrixInsertColumn );
                    break;
                default:
                    break;
            }

        }
        else if ( text [i] == 0xC2 ) { // special handling for german umlaute

            switch ( text [++i] ) {
                case 0xB0: // °     sprite index 102
                    matrixInsertColumn += matrix.printChar ( 32 + 102, matrixInsertColumn );
                    break;
                default:
                    break;
            }

        }
        else {

            matrixInsertColumn += matrix.printChar ( text [i], matrixInsertColumn );

        }

    }

    if ( matrixInsertColumn > NUM_DISPLAY_COLS ) {

        clearAllCommands ();

        int col1 = 0;
        int col2 = matrixInsertColumn - NUM_DISPLAY_COLS;
        initCommand ( 1, HORIZONTAL_DIRECTION, col1, col2, 2 );
        initCommand ( 2, HORIZONTAL_DIRECTION, col2, col2, 3 );
        initCommand ( 3, HORIZONTAL_DIRECTION, col2, col1, 4 );
        initCommand ( 4, HORIZONTAL_DIRECTION, col1, col1, 1 );

        startCommand ( 4 );

    }

    alertDisplayBegin = millis ();

}

void matrixDisplayTimeDateChar ( char c ) {

    if ( c == '1' ) matrixInsertColumn += matrix.printEmptyCol ( matrixInsertColumn );
    if ( c == ' ' ) {
        matrixInsertColumn += matrix.printEmptyCol ( matrixInsertColumn );
        matrixInsertColumn += matrix.printEmptyCol ( matrixInsertColumn );
        matrixInsertColumn += matrix.printEmptyCol ( matrixInsertColumn );
    }
    else {
        matrixInsertColumn += matrix.printChar ( c, matrixInsertColumn );
    }

}

void matrixDisplayTime ( time_t time ) {

    commandInExecution = NULL;

    int len = 30;
    char buf [len];

    sprintf ( buf, "%02d%c%02d", hour ( time ), second ( time ) % 2 == 0 ? ':' : ' ', minute ( time ) );
    // MY_SERIAL.println ( buf );

    matrixInsertColumn = 21;
    matrixDisplayColumn = 0;

    for ( int i = 0; i < len; i++ ) {
        if ( buf [i] == 0x00 ) break;
        matrixDisplayTimeDateChar ( buf [i] );
    }

}

void matrixDisplayDate ( time_t time ) {

    commandInExecution = NULL;

    int len = 30;
    char buf [len];

    sprintf ( buf, "%02d.%02d.%04d", day ( time ), month ( time ), year ( time ) );
    // MY_SERIAL.println ( buf );

    matrixInsertColumn = 9;

    for ( int i = 0; i < len; i++ ) {
        if ( buf [i] == 0x00 ) break;
        matrixDisplayTimeDateChar ( buf [i] );
    }

}

// ------------------------------------------------------------------------------------------------------------------------------------------
// time

boolean isCEST ( unsigned long t ) {

  boolean isCest = false;

  int m = month ( t );
  switch ( m ) {

    // CET
    case 1:
    case 2:
    case 11:
    case 12:
      isCest = false;
      break;

    // CEST
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
      isCest = true;
      break;

    // CEST -> CES / CES -> CEST
    case 10:
    case 3:
      isCest = (m==10); // Anfang Oktober ist Sommerzeit / Anfang MÃ¤rz ist Winterzeit
      if ( day ( t ) > 24 ) { // wir sind in der letzten Woche des Monats inkl. letztem Sonntag
        if ( weekday ( t ) + 31 - day ( t ) < 8 ) { // wir sind am So oder danach; 8 ist der kleinste Wert
          isCest = !isCest; // fast schon Winterzeit / Sommerzeit
          if ( weekday ( t ) == 1 && hour ( t ) < 1 ) { // es ist Sonntag und noch nicht 2 Uhr Ortszeit!
              isCest = !isCest; // aber doch noch ein bischen Sommerzeit / Winterzeit
          }
        }
      }
      break;

    default:
      break;

  }

  return isCest;

}

time_t getNtpTime () {

  sendNTPpacket ( SNTP_SERVER_IP );

  delay ( 1000 );

  unsigned long time = recvNtpTime ();
  if ( time != 0L ) {
    time -= SEVENTY_YEARS;
//    time += (isCEST ( time ) ? OFFSET_CEST : OFFSET_CET); // erst hier ist time initial gesetzt
    if ( isCEST ( time ) ) {
      time += OFFSET_CEST;
    }
    else {
      time += OFFSET_CET;
    }
  }

  return time;

}

const int NTP_PACKET_SIZE = 48;                   // NTP time stamp is in first 48 bytes of message
byte packetBuffer [NTP_PACKET_SIZE];              // buffer to hold incoming(outgoing packets

void sendNTPpacket ( byte *address ) {

  memset ( packetBuffer, 0, NTP_PACKET_SIZE );

  // Init for NTP Request
  packetBuffer [0] = B11100011; // LI, Version, Mode 0xE3
  packetBuffer [1] = 0; // Stratum
  packetBuffer [2] = 6; // max intervall between messages in sec
  packetBuffer [3] = 0xEC; // clock precision
  // bytes 4 - 11 are for root delay ad dispersion and were set to 0 by memset
  packetBuffer [12] = 49; // four byte reference id
  packetBuffer [13] = 0x4E;
  packetBuffer [14] = 49;
  packetBuffer [15] = 52;

  // send the packet requesting a timestamp
  // port 123
  udp.beginPacket ( address, 123 );
  udp.write ( packetBuffer, NTP_PACKET_SIZE );
  udp.endPacket ();

}

unsigned long recvNtpTime () {

  if ( udp.parsePacket () ) {

    udp.read ( packetBuffer, NTP_PACKET_SIZE );

    // the time starts at byte 40, convert four bytes into long
    unsigned long hi = word ( packetBuffer [40], packetBuffer [41] );
    unsigned long lo = word ( packetBuffer [42], packetBuffer [43] );

    // this is NTP time (seconds since Jan 1 1900
    unsigned long secsSince1900 = hi << 16 | lo;

    return secsSince1900;

  }

  return 0L; // return 0 if unable to get the time

}

void digitalClockDisplay ( time_t time ){

  // digital clock display of the time
  MY_SERIAL.print ( hour ( time ) );
  printDigits ( minute ( time ) );
  printDigits ( second ( time ) );
  MY_SERIAL.print ( " " );
  MY_SERIAL.print ( day ( time ));
  MY_SERIAL.print ( "." );
  MY_SERIAL.print ( month ( time ) );
  MY_SERIAL.print ( "." );
  MY_SERIAL.print ( year ( time ) );
  MY_SERIAL.println ();

}



void printDigits ( int digits ) {

  // utility for digital clock display: prints preceding colon and leading 0
  MY_SERIAL.print ( ":" );
  if ( digits < 10 ) MY_SERIAL.print ( '0' );
  MY_SERIAL.print ( digits );

}

// ------------------------------------------------------------------------------------------------------------------------------------------

char* getMsgText ( int msg ) {

    char* result = (char*) "no msg text";

    if ( msg >= 0 && msg < NUM_MESSAGES ) {
        if ( isMsgText [msg] ) result = (char*) msgText [msg];
    }

    // MY_SERIAL.printf ( "getMsgText: msg=%d result=%s\n", msg, result );

    return result;

}

// ------------------------------------------------------------------------------------------------------------------------------------------

boolean dontWait = false;

void loop() {

    mqttReconnect ();
    mqttClient.loop ();

    currentTime = now ();
    // tick every second
    if ( dontWait || second ( currentTime ) != second ( lastTime ) ) {

        dontWait = false;

        displayCategoryPeriodCounter++;

        if ( alertDisplayDuration < 0 || alertDisplayDuration > 0 && alertDisplayBegin + alertDisplayDuration * 1000L < millis () ) {

            if ( alertDisplayDuration > 0 ) {
                alertDisplayDuration = -1;
                clearMatrix ();
            }

            if ( isDisplayCategoryTime ) matrixDisplayTime ( currentTime ); // toggle the colon

            if ( displayCategoryPeriodCounter >= displayCategoryPeriod ) {

                displayCategoryPeriodCounter = 0;
                displayCategory++;
                if ( displayCategory > MAX_DISPLAY_CATEGORY ) displayCategory = 0;
                clearMatrix ();
                matrix.init ( displayBrightness );
                isDisplayCategoryTime = false;
                int msg = displayCategory - 3;

                switch ( displayCategory ) {
                    case 0: // time
                        if ( isDisplayTime ) {
                            matrixDisplayTime ( currentTime );
                            isDisplayCategoryTime = true;
                        }
                        else {
                            displayCategoryPeriodCounter = displayCategoryPeriod;
                            dontWait = true;
                        }
                        break;
                    case 1: // date
                        if ( isDisplayDate ) {
                            matrixDisplayDate ( currentTime );
                        }
                        else {
                            displayCategoryPeriodCounter = displayCategoryPeriod;
                            dontWait = true;
                        }
                        break;
                    case 2: // day of week
                        if ( isDisplayWeekday ) {
                            matrixInsertColumn = 13;
                            matrixAppendText ( dayStr ( weekday ( currentTime ) ) );
                        }
                        else {
                            displayCategoryPeriodCounter = displayCategoryPeriod;
                            dontWait = true;
                        }
                        break;
                    case 3: // msg [0]
                    case 4: // msg [1]
                    case 5: // msg [2]
                    case 6: // msg [3]
                    case 7: // msg [4]
                    case 8: // msg [5]
                    case 9: // msg [6]
                    case 10: // msg [7]
                    case 11: // msg [8]
                    case 12: // msg [9]
                        matrixInsertColumn = 0;
                        alertDisplayDuration = -1;
                        if ( isMsgText [msg] && isMsgEnabled [msg] ) {
                            matrixAppendText ( getMsgText ( msg ) );
                        }
                        else {
                            // displayCategory++;
                            displayCategoryPeriodCounter = displayCategoryPeriod;
                            dontWait = true;
                        }
                        break;
                    default:
                        displayCategoryPeriodCounter = 0;
                        break;
                }

            }

        }

        lastTime = currentTime;

    }

}

// ------------------------------------------------------------------------------------------------------------------------------------------