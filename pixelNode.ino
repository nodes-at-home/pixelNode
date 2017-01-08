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

#include <JsonStreamingParser.h>
#include <JsonListener.h>
#include <ArduinoJson.h>

#include <PubSubClient.h>

#include "MaxMatrix.h"
#include "Sprites.h"

#include "WLAN.h"
//const char* ssid     = "your_ssid";
//const char* password = "your_password";

// ------------------------------------------------------------------------------------------------------------------------------------------

// #define VERSION "V0.10 (" __DATE__ ", " __TIME__ ")"
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

int displayTextDuration = -1; // in sec, -1 no effect, 0 is forever
long textDisplayBegin;

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

const int DISPLAY_CATEGORY_PERIOD = 10;
int displayCategoryPeriodCounter = 100;
int displayCategory = -1;
const int MAX_DISPLAY_CATEGORY = 4 + 9;

boolean isDisplayCategoryTime = false;

// ------------------------------------------------------------------------------------------------------------------------------------------

const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "ESP8266_pixel_node";

#if defined HOME_WLAN
const IPAddress MQTT_SERVER ( 192, 168, 2, 117 );
#elif defined DEV_WLAN
const IPAddress MQTT_SERVER ( 192, 168, 2, 101 );
#elif defined XPERIA_WLAN
const IPAddress MQTT_SERVER ( 192, 168, 43, 78 );
#elif defined IPHONE_WLAN
const IPAddress MQTT_SERVER ( 172, 20, 10, 2 );
#endif

PubSubClient mqttClient ( wifiClient );
long lastMqttReconnect;
const long MQTT_RECONNECT_PERIOD = 5000L;

const char* MQTT_TOPIC_BASE = "nodes@home/display/pixel/kitchen";
const char* MQTT_TOPIC_TEXT = "nodes@home/display/pixel/kitchen/text";
const char* MQTT_TOPIC_TEMPERATURE = "nodes@home/sensor/+/+/value/temperature";
const char* MQTT_TOPIC_HUMIDITY = "nodes@home/sensor/+/+/value/humidity";
const char* MQTT_TOPIC_VOLTAGE = "nodes@home/sensor/+/+/value/voltage";

const int DEFAULT_DISPLAY_TEXT_DURATION = 15;

// ------------------------------------------------------------------------------------------------------------------------------------------

const int NUM_SENSORS = 3 * 3;
char* sensorText [NUM_SENSORS] [200];
bool isSensorText [NUM_SENSORS];

// ------------------------------------------------------------------------------------------------------------------------------------------

void setup () {
    
    delay ( 1000 );

    // --- init led matrix ----------------------------------------------------
    matrix.init (); // module initialize
    matrix.setIntensity ( 0x0 ); // dot matix intensity 0-15
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
    delay ( 10000 );

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
    
    // --- init mqtt -----------------------------------------------------------
    MY_SERIAL << "[SETUP] mqtt: server=" << MQTT_SERVER << " port=" << MQTT_PORT << " version=" << MQTT_VERSION << endl;
    mqttClient.setServer ( MQTT_SERVER, MQTT_PORT );
    mqttClient.setCallback ( mqttCallback );
    // mqttClient.setClient ( wifiClient );

    // --- display message -----------------------------------------------------
    
    delay ( 10000 );
    clearMatrix ();
  
}

// ------------------------------------------------------------------------------------------------------------------------------------------

// const char* http_request ( const char* url ) {
    
    // const char* result = NULL;
    
    // MY_SERIAL << "[HTTP] Start Http Client url=" << url << endl;
    
    // http.begin ( url );
    // int http_code = http.GET (); // send a GET request
    // MY_SERIAL << "[HTTP] http_code=" << http_code << endl;
    
    // if ( http_code == HTTP_CODE_OK ) {
        
        // int len = http.getSize ();
        // MY_SERIAL << "payload size=" << len << endl;
        
        // String payload = http.getString ();
        // MY_SERIAL.println ( payload );
        // result = payload.c_str ();
                
        // MY_SERIAL.println ( "[HTTP] connection closed or file end." );
        
    // }
    // else {
        // MY_SERIAL << "[HTTP] hhtp request with http_code=" << http_code << " (" << http.errorToString ( http_code ) << ")" << endl;
    // }
    
    // http.end ();
    
    // return result;
    
// }

// ------------------------------------------------------------------------------------------------------------------------------------------

void mqttCallback ( char* topic, byte* payload, unsigned int payloadLength ) {
    
    MY_SERIAL.print ( "[MQTT] payloadLength=" );
    MY_SERIAL.println ( payloadLength );
    
    MY_SERIAL.print ( "[MQTT] Message arrived [" );
    MY_SERIAL.print ( topic );
    MY_SERIAL.print ( "] " );
    for ( int i = 0; i < payloadLength; i++ ) MY_SERIAL.print ( (char) payload [i] );
    MY_SERIAL.println();

    char buf [200] = { 0 };
    int len = payloadLength < sizeof ( buf ) ? payloadLength : sizeof ( buf ) - 1;
    strncpy ( buf, (const char*) payload, len );
    buf [len] = '\0';
        
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject ( buf );

    if ( strcmp ( topic, MQTT_TOPIC_TEXT ) == 0 ) {

        MY_SERIAL << "[MQTT] display text topic" << endl;
        
        if  ( root.success () ) {
            clearMatrix ();
            matrix.init ();
            const int duration = root ["duration"];
            const char* text = root ["text"];
            displayTextDuration = duration > 0 ? duration : DEFAULT_DISPLAY_TEXT_DURATION;
            matrixAppendText ( text );
        }
        else {
            MY_SERIAL.printf ( "parsing failed payload=%s\n", payload );
        }
        
    }
    else {
        char* token = strtok ( topic, "/" );
        int i = 0;
        char topicBase [20];
        char sensorMark [20];
        char sensorName [20];
        char sensorLocation [20];
        char sensorType [20];
        while ( token ) {
            // MY_SERIAL.printf ( "[MQTT] token=%s\n", token );
            if ( i == 0 ) {
                strncpy ( topicBase, token, 20 );
            }
            else if ( i == 1 ) {
                strncpy ( sensorMark, token, 20 );
            }
            else if ( i == 2 ) {
                strncpy ( sensorName, token, 20 );
            }
            else if ( i == 3 ) {
                strncpy ( sensorLocation, token, 20 );
            }

            else if ( i == 5 ) {
                strncpy ( sensorType, token, 20 );
            }
            token = strtok ( NULL, "/" );
            i++;
        }
        
        if ( strcmp ( topicBase, "nodes@home" ) == 0 && strcmp ( sensorMark, "sensor" ) == 0 && strcmp ( sensorName, "DHT11" ) == 0 ) {

            int index = 0;
            
            if ( strcmp ( sensorLocation, "lounge" ) == 0 ) {
                index += 0 * 3;
            }
            else if ( strcmp ( sensorLocation, "roof" ) == 0 ) {
                index += 1 * 3;
            }
            else if ( strcmp ( sensorLocation, "terrace" ) == 0 ) {
                index += 2 * 3;
            }
            else {
                index = -1;
            }
            
            char unit [20];
            if ( index > -1 ) {
                if ( strcmp ( sensorType, "temperature" ) == 0 ) {
                    strcpy ( unit, "°C" );
                    index += 0;
                }
                else if ( strcmp ( sensorType, "humidity" ) == 0 ) {
                    strcpy ( unit, "%" );
                    index += 1;
                }
                else if ( strcmp ( sensorType, "voltage" ) == 0 ) {
                    strcpy ( unit, "mV" );
                    index += 2;
                }
                else {
                    index = -1;
                }
            }
            
            MY_SERIAL.printf ( "sensor data: i=%d loc=%s type=%s\n", index, sensorLocation, sensorType );

            if ( index > -1 ) {
                isSensorText [index] = true;
                int value = root ["value"];
                snprintf ( (char*) (sensorText + index), 200, "%s@%s: %d%s", sensorName, sensorLocation, value, unit );
            }
            
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
            mqttClient.publish ( MQTT_TOPIC_BASE, VERSION );
            // ... and resubscribe
            mqttClient.subscribe ( MQTT_TOPIC_TEXT );
            mqttClient.subscribe ( MQTT_TOPIC_TEMPERATURE );
            mqttClient.subscribe ( MQTT_TOPIC_HUMIDITY );
            mqttClient.subscribe ( MQTT_TOPIC_VOLTAGE );
      
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

    textDisplayBegin = millis ();

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

const long WEATHER_REQUEST_PERIOD = 1L * 60L * 60L * 1000L; // every hour
#define JSON_PARSER_DEBUG false

void requestJsonDataFromUrl ( const char* url, JsonListener* jsonListener ) {
    
    MY_SERIAL << "[HTTP] Start Http Client url=" << url << endl;
    
    JsonStreamingParser jsonParser;
    jsonParser.setListener ( jsonListener );
    
    HTTPClient http;

    http.begin ( url );
    int http_code = http.GET (); // send a GET request
    MY_SERIAL << "[HTTP] http_code=" << http_code << endl;
    
    if ( http_code == HTTP_CODE_OK ) {
        
        int payloadSize = http.getSize ();
        MY_SERIAL << "[HTTP] payload size=" << payloadSize << endl;
        
        // create buffer for read
        uint8_t buf [128] = { 0 };

        // get tcp stream
        WiFiClient* stream = http.getStreamPtr ();

        // read all data from server
        while ( http.connected () && ( payloadSize > 0 || payloadSize == -1 ) ) {
            
            // get available data size
            size_t size = stream->available ();
            // MY_SERIAL.printf ( "size=%d\n", size );            

            if ( size ) {
                
                // read up to 128 byte
                int readBytesLen = stream->readBytes ( buf, ( ( size > sizeof ( buf ) ) ? sizeof ( buf ) : size ) );

                // write it to MY_SERIAL
                // MY_SERIAL.write ( buf, readBytesLen );
                for ( int i = 0; i < readBytesLen; i++ ) {
                    jsonParser.parse ( buf [i] );
                }

                if ( payloadSize > 0 ) {
                    payloadSize -= readBytesLen;
                }
                
            }
            delay ( 1 );
        }

        MY_SERIAL.println ( "[HTTP] connection closed or file end." );
        
    }
    else {
        MY_SERIAL << "[HTTP] hhtp request with http_code=" << http_code << " (" << http.errorToString ( http_code ) << ")" << endl;
    }
    
    http.end ();
    
}

char temperature [10];
char pressure [10];
char humidity [10];

char lastKey [20];

bool inMain;
bool isTempValue;
bool isPressureValue;
bool isHumidityValue;

class CurrentWeatherJsonListener: public JsonListener {
    
    public:
    
        void whitespace ( char c ) {
            // noop
        }
      
        void startDocument () {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.println ( "[CurrentWeatherJsonListener] startDocument ()" );
            
            strcpy ( temperature, "" );
            strcpy ( pressure, "" );
            strcpy ( humidity, "" );
            
            inMain = false;
            
            strcpy ( lastKey, "" );
            
        }

        void endDocument () {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.println ( "[CurrentWeatherJsonListener] endDocument ()" );

            // this is never called, why ever
            // snprintf ( weatherTextCache, sizeof ( weatherTextCache ), "min %f C max %f C", minTemp, maxTemp );
            
        }

        void startArray () {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.println ( "[CurrentWeatherJsonListener] startArray ()" );

        }

        void endArray () {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.println ( "[CurrentWeatherJsonListener] endArray ()" );

        }

        void startObject () {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.printf ( "\n[CurrentWeatherJsonListener] startObject () lastKey=%s\n", lastKey );

            inMain = false;
            if ( strcmp ( lastKey, "main" ) == 0 ) inMain = true;

        }
        
        void endObject () {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.println ( "[CurrentWeatherJsonListener] endObject ()" );

        }

        void key ( String key ) {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.print ( "[CurrentWeatherJsonListener] key () key=" );
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.print ( key );
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.print ( ": " );
            
            strcpy ( lastKey, key.c_str () );
            
            if ( inMain ) {
                isTempValue = false;
                isHumidityValue = false;
                isPressureValue = false;
                if ( key == "temp" ) isTempValue = true;
                if ( key == "humidity" ) isHumidityValue = true;
                if ( key == "pressure" ) isPressureValue = true;
            }
                        
        }

        void value ( String value ) {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.print ( "[CurrentWeatherJsonListener] value () value=" );
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.println ( value );
            
            if ( isTempValue ) strcpy ( temperature, value.c_str () );
            if ( isHumidityValue ) strcpy ( humidity, value.c_str () );
            if ( isPressureValue ) strcpy ( pressure, value.c_str () );

        }

};

CurrentWeatherJsonListener currentWeatherJsonListener;

static char currentWeatherTextCache [200] = { 0 };
long lastCurrentWeatherHttpRequest = 0L;

char* getCurrentWeatherText () {
    

    if ( strlen ( currentWeatherTextCache ) == 0 || lastCurrentWeatherHttpRequest + WEATHER_REQUEST_PERIOD < millis () ) {
        
        MY_SERIAL.println ( "[WEATHER] request new current weather report" );
        
        // const char* url = "http://192.168.2.8/website/weather_test/current_weather_test.html";
        const char* url = "http://api.openweathermap.org/data/2.5/weather?id=2804759&appid=29ab3e72b07b4dd4c3aca6db6ff3290d&lang=en&units=metric";
        
        // snprintf ( currentWeatherTextCache, sizeof ( currentWeatherTextCache ), "there is no weather today" );
        
        requestJsonDataFromUrl ( url, &currentWeatherJsonListener );
        
        snprintf ( currentWeatherTextCache, sizeof ( currentWeatherTextCache ), "%s°C %s%% %shPa", temperature, humidity, pressure );
        MY_SERIAL.printf ( "[WEATHER] temperature=%s humidity=%s pressure=%s\n", temperature, humidity, pressure );

        lastCurrentWeatherHttpRequest = millis ();
        
    }
    
    return currentWeatherTextCache;

}


float minTemp;
float maxTemp; 
bool inList;
int listIndex;
float temp;

class ForecastWeatherJsonListener: public JsonListener {
    
    public:
    
        void whitespace ( char c ) {
            // noop
        }
      
        void startDocument () {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.println ( "[ForecastWeatherJsonListener] startDocument ()" );
            
            minTemp = 999.9;
            maxTemp = -999.9; 
            inList = false;
            inMain = false; // ???
            
        }

        void endDocument () {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.println ( "[ForecastWeatherJsonListener] endDocument ()" );

            // this is never called, why ever
            // snprintf ( weatherTextCache, sizeof ( weatherTextCache ), "min %f C max %f C", minTemp, maxTemp );
            
        }

        void startArray () {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.println ( "[ForecastWeatherJsonListener] startArray ()" );

        }

        void endArray () {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.println ( "[ForecastWeatherJsonListener] endArray ()" );
            if ( inList ) inList = false;

        }

        void startObject () {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.println ( "[ForecastWeatherJsonListener] startObject ()" );
            if ( inList ) temp = 0.0;

        }
        
        void endObject () {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.println ( "[ForecastWeatherJsonListener] endObject ()" );

        }

        void key ( String key ) {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.print ( "[ForecastWeatherJsonListener] key () key=" );
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.print ( key );
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.print ( ": " );
            
            if ( key == "list" ) {
                inList = true;
                listIndex = 0;
            }

            if ( inList && key == "main" ) inMain = true;
            
            isTempValue = false;
            if ( inMain && listIndex < 8 && key == "temp" ) isTempValue = true;
            
        }

        void value ( String value ) {
            
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.print ( "[ForecastWeatherJsonListener] value () value=" );
            if ( JSON_PARSER_DEBUG ) MY_SERIAL.println ( value );
            
            if ( isTempValue ) {
                
                temp = atof ( value.c_str () );
                
                listIndex++;
                if ( temp < minTemp ) minTemp = temp;
                if ( temp > maxTemp ) maxTemp = temp;
                
            }

        }

};

ForecastWeatherJsonListener forecastWeatherJsonListener;

static char forecastWeatherTextCache [200] = { 0 };
long lastForecastWeatherHttpRequest = 0L;

char* getForecastWeatherText () {
    
    if ( strlen ( forecastWeatherTextCache ) == 0 || lastForecastWeatherHttpRequest + WEATHER_REQUEST_PERIOD < millis () ) {
        
        MY_SERIAL.println ( "[WEATHER] request new forecast weather report" );
        
        // const char* url = "http://192.168.2.8/website/weather_test/forecast_weather_test.html";
        const char* url = "http://api.openweathermap.org/data/2.5/forecast?id=2804759&appid=29ab3e72b07b4dd4c3aca6db6ff3290d&lang=en&units=metric";
        
        // snprintf ( forecastWeatherTextCache, sizeof ( forecastWeatherTextCache ), "there is no forecast today" );
        
        requestJsonDataFromUrl ( url, &forecastWeatherJsonListener );
        
        char minTempStr [10];
        dtostrf ( minTemp, 4, 1, minTempStr );

        char maxTempStr [10];
        dtostrf ( maxTemp, 4, 1, maxTempStr );

        MY_SERIAL.printf ( "[WEATHER] min=%sC max=%sC\n", minTempStr, maxTempStr );
        sprintf ( forecastWeatherTextCache, "min %s°C max %s°C", minTempStr, maxTempStr );
            

        lastForecastWeatherHttpRequest = millis ();
        
    }
    
    return forecastWeatherTextCache;

}

// ------------------------------------------------------------------------------------------------------------------------------------------

char* getSensorText ( int sensor ) {
    
    char* result = (char*) "no sensor text";
    
    if ( sensor >= 0 && sensor < NUM_SENSORS ) {
        if ( isSensorText [sensor] ) result = (char*) sensorText [sensor];
    }
    
    MY_SERIAL.printf ( "getSensorText: sensor=%d result=%s\n", sensor, result );
    
    return result;
    
}

// ------------------------------------------------------------------------------------------------------------------------------------------

bool dontWait = false;

void loop() {
    
    mqttReconnect ();
    mqttClient.loop ();

    currentTime = now ();
    // tick every second
    if ( dontWait || second ( currentTime ) != second ( lastTime ) ) {
        
        dontWait = false;
        
        displayCategoryPeriodCounter++;
        
        if ( displayTextDuration < 0 || displayTextDuration > 0 && textDisplayBegin + displayTextDuration * 1000L < millis () ) {
            
            if ( displayTextDuration > 0 ) {
                displayTextDuration = -1;
                clearMatrix ();
            }
            
            if ( isDisplayCategoryTime ) matrixDisplayTime ( currentTime ); // toggle the colon
        
            if ( displayCategoryPeriodCounter >= DISPLAY_CATEGORY_PERIOD ) {
                
                displayCategoryPeriodCounter = 0;
                displayCategory++;
                if ( displayCategory > MAX_DISPLAY_CATEGORY ) displayCategory = 0;
                clearMatrix ();
                matrix.init ();
                isDisplayCategoryTime = false;
                int sensor = displayCategory - 5;

                switch ( displayCategory ) {
                    case 0: // time
                        matrixDisplayTime ( currentTime );
                        isDisplayCategoryTime = true;
                        break;
                    case 1: // date
                        matrixDisplayDate ( currentTime );
                        break;
                    case 2: // day of week
                        // matrix_show_day_of_week ( currentTime );
                        matrixInsertColumn = 13;
                        matrixAppendText ( dayStr ( weekday ( currentTime ) ) );
                        break;
                    case 3: // weather
                        matrixInsertColumn = 0;
                        matrixAppendText ( getForecastWeatherText () );
                        displayTextDuration = -1;
                        break;
                    case 4: // forecast
                        matrixInsertColumn = 0;
                        matrixAppendText ( getCurrentWeatherText () );
                        displayTextDuration = -1;
                        break;
                    case 5: // sensor lounge temerature
                    case 6: // sensor lounge humidity
                    case 7: // sensor lounge voltage
                    case 8: // sensor roof temerature
                    case 9: // sensor roof humidity
                    case 10: // sensor roof voltage
                    case 11: // sensor terrace temerature
                    case 12: // sensor terrace humidity
                    case 13: // sensor terrace voltage
                        matrixInsertColumn = 0;
                        displayTextDuration = -1;
                        if ( isSensorText [sensor] ) {
                            matrixAppendText ( getSensorText ( sensor ) );
                        }
                        else {
                            // displayCategory++;
                            displayCategoryPeriodCounter = DISPLAY_CATEGORY_PERIOD;
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

