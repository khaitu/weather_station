#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_MPL115A2.h>
#include <Adafruit_SI1145.h>
#include <HTU21D.h>

#define kLoopInterval 1
#define kBarDisplayFrequency 6
#define kMatrixRotation 3

// SENSORS
Adafruit_BicolorMatrix matrix1 = Adafruit_BicolorMatrix();
Adafruit_BicolorMatrix matrix2 = Adafruit_BicolorMatrix();
Adafruit_MPL115A2 barometerSensor;
Adafruit_SI1145 uvSensor = Adafruit_SI1145();
HTU21D humiditySensor;

// VARIABLES
float   pressure                 = 0,
        temperature              = 0,
        humidity                 = 0,
        light                    = 0,
        uv                       = 0,
        ir                       = 0;
uint8_t periodCounter            = 0;
int8_t  moleculePositions[2][2]  = { { 2, 2 }, { 6, 6 } },
        moleculeDirections[2][2] = { { 1, 1 }, { -1, -1 } };


void numberBarDisplay( Adafruit_BicolorMatrix &matrix, int numeral, int16_t offset, uint16_t colour )
{
	int16_t i = 0;

	for ( i = 0; i < 8 && i < numeral; i++ ) matrix.drawPixel( i, 0 + ( 2 * offset ), colour );

	if ( numeral > 7 ) for ( i = 7; i < numeral; i++ ) matrix.drawPixel( ( i - 8 ), 1 + ( 2 * offset ), colour );
}


void moleculeMotionDisplay( Adafruit_BicolorMatrix &matrix, int8_t *position, int8_t *direction, uint16_t colour )
{
	long randoms[2] = { random( 0, 100 ), random( 0, 100 ) };

	if ( randoms[0] > 87 && direction[0] != 0 ) direction[0] = 0;
	else if ( randoms[0] > 75 ) direction[0] = -direction[0];
	else if ( randoms[0] < 25 && direction[0] == 0 ) direction[0] = randoms[0] % 2 == 0 ? 1 : -1;
	if ( randoms[1] > 87 && direction[1] != 0 ) direction[1] = 0;
	else if ( randoms[1] > 75 ) direction[1] = -direction[1];
	else if ( randoms[1] < 25 && direction[1] == 0 ) direction[1] = randoms[1] % 2 == 0 ? 1 : -1;

	if ( position[0] >= 7 && direction[0] == 1 ) direction[0] = -1;
	else if ( position[0] <= 0 && direction[0] == -1 ) direction[0] = 1;
	if ( position[1] >= 7 && direction[1] == 1 ) direction[1] = -1;
	else if ( position[1] <= 0 && direction[1] == -1 ) direction[1] = 1;

	position[0] += direction[0];
	position[1] += direction[1];

	matrix.drawPixel( position[0], position[1], colour );
}


void prependStr( char* s, const char* t )
{
	size_t len = strlen(t);
	size_t i;

	memmove(s + len, s, strlen(s) + 1);

	for (i = 0; i < len; ++i) s[i] = t[i];
}



void ftoa( char *str, float f, int precision )
{
	int numeral = (int)f;
	int decimal = (int)( ( f - numeral ) * pow( 10, precision ) );
	char *decimalStr;

	decimalStr = (char*)malloc( precision );

	// sort out decimals
	sprintf( decimalStr, "%i", decimal );
	while ( strlen( decimalStr ) < precision ) prependStr( decimalStr, "0" );

	sprintf( str, "%i.%s", numeral, decimalStr );

	free( decimalStr );
}


void setup()
{
	Serial.begin( 9600 );

	matrix1.begin( 0x70 );
	matrix2.begin( 0x71 );

	uvSensor.begin();

	delay( 250 );

	barometerSensor.begin();
	humiditySensor.begin();

//	TWBR = 12;

	matrix1.setRotation( kMatrixRotation );
	matrix2.setRotation( kMatrixRotation );
}


void loop()
{
	uint8_t  period      = 0;
	uint16_t colour      = LED_GREEN;
	char     output[2][256],
	         tempStr[10],
	         pressStr[10],
	         humidityStr[10],
	         altTempStr[10],
	         lightStr[10],
	         uvStr[10],
	         irStr[10],
	         serialStr[245];

	pressure    = periodCounter % kBarDisplayFrequency == 0 ? barometerSensor.getPressure()    : pressure,
	temperature = periodCounter % kBarDisplayFrequency == 1 ? humiditySensor.readTemperature() : temperature,
	humidity    = periodCounter % kBarDisplayFrequency == 2 ? humiditySensor.readHumidity()    : humidity;
	light       = periodCounter % kBarDisplayFrequency == 3 ? uvSensor.readVisible()           : light;
	uv          = periodCounter % kBarDisplayFrequency == 4 ? (float)uvSensor.readUV() / 100   : uv;
	ir          = periodCounter % kBarDisplayFrequency == 5 ? uvSensor.readIR()                : ir;
	//altTemp     = barometerSensor.getTemperature();

	if ( periodCounter % kBarDisplayFrequency == 0 )
	{
		ftoa( tempStr, temperature, 4 );
		ftoa( pressStr, pressure, 4 );
		ftoa( humidityStr, humidity, 4 );
		ftoa( lightStr, light, 4 );
		ftoa( uvStr, uv, 4 );
		ftoa( irStr, ir, 4 );
		//ftoa( altTempStr, altTemp, 4 );

		// output sensor data to tty
		sprintf( serialStr, "%s,%s,%s,%s,%s,%s", tempStr, pressStr, humidityStr, lightStr, uvStr, irStr /*, altTempStr */ );
		Serial.println( serialStr );
	}

	// select colour for bars
	if ( ( temperature < 22.0f && temperature >= 20.0f ) || ( temperature >= 25.0f && temperature < 27.0f ) ) colour = LED_YELLOW;
	else if ( temperature >= 22.0f && temperature < 25.0f ) colour = LED_GREEN;
	else colour = LED_RED;

	// clear displays
	matrix1.clear();
	matrix2.clear();

	// display temperature as bar visualisation on the first matrix
	if ( periodCounter % 2 == 0 )
	{
		numberBarDisplay( matrix1, (int)( temperature / 10 ), 0, colour );
		numberBarDisplay( matrix1, (int)( (int)temperature % 10 ), 1, colour );
		numberBarDisplay( matrix1, (int)( ( temperature - (int)temperature ) * 10 ), 2, colour );
		numberBarDisplay( matrix1, (int)( (int)( ( temperature - (int)temperature ) * 100 ) % 10 ), 3, colour );
		matrix1.writeDisplay();
	}

	// display temperature molecule on the second matrix
	moleculeMotionDisplay( matrix2, moleculePositions[0], moleculeDirections[0], LED_RED );
	moleculeMotionDisplay( matrix2, moleculePositions[1], moleculeDirections[1], LED_YELLOW );
	matrix2.writeDisplay();

	// increment period counter
	if ( ++periodCounter > kBarDisplayFrequency ) periodCounter = 0;

	delay( kLoopInterval );
}
