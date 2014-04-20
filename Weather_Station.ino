#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_MPL115A2.h>
#include <Adafruit_SI1145.h>
#include <HTU21D.h>

#define kLoopInterval             1
#define kBarDisplayFrequency      6
#define kMatrixRotation           3
#define kTemperatureLowerLimit    20.0f
#define kTemperatureUpperLimit    26.0f
#define kTemperatureRange         ( kTemperatureUpperLimit - kTemperatureLowerLimit )
#define kMoleculeMultiplier       100

#define COUNT(b) (int)(sizeof(b) / sizeof(b[0]))


// SENSORS
Adafruit_BicolorMatrix matrix1 = Adafruit_BicolorMatrix();
Adafruit_BicolorMatrix matrix2 = Adafruit_BicolorMatrix();
Adafruit_SI1145 uvSensor       = Adafruit_SI1145();
Adafruit_MPL115A2 barometerSensor;
HTU21D humiditySensor;

// VARIABLES
float    _pressure                 = 0,
         _temperature              = 0,
         _humidity                 = 0,
         _light                    = 0,
         _uv                       = 0,
         _ir                       = 0,
         _temperatureBuffer[100];
uint8_t  _periodCounter            = 0;
uint16_t _sensorCounter            = 0,
         _sensorMaxCounter;
int8_t   _moleculePositions[3][2]  = { { 2, 2 }, { 6, 6 }, { 4, 4 } },
         _moleculeDirections[3][2] = { { 1, 1 }, { -1, -1 }, { 1, -1 } },
         _moleculeModuli[3]        = { kMoleculeMultiplier, kMoleculeMultiplier, kMoleculeMultiplier };


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
	else if ( randoms[0] < 15 && direction[0] == 0 ) direction[0] = randoms[0] % 2 == 0 ? 1 : -1;
	if ( randoms[1] > 87 && direction[1] != 0 ) direction[1] = 0;
	else if ( randoms[1] > 75 ) direction[1] = -direction[1];
	else if ( randoms[1] < 15 && direction[1] == 0 ) direction[1] = randoms[1] % 2 == 0 ? 1 : -1;

	// don't let molecules be still
	if ( direction[0] == 0 && direction[1] == 0 ) direction[ randoms[0] % 2 ] = randoms[1] % 2 == 0 ? 1 : -1;

	// check molecule isn't going off the edge
	if ( position[0] >= 7 && direction[0] == 1 ) direction[0] = -1;
	else if ( position[0] <= 0 && direction[0] == -1 ) direction[0] = 1;
	if ( position[1] >= 7 && direction[1] == 1 ) direction[1] = -1;
	else if ( position[1] <= 0 && direction[1] == -1 ) direction[1] = 1;

	position[0] += direction[0];
	position[1] += direction[1];

	matrix.drawPixel( position[0], position[1], colour );
}


void moleculeStaticDisplay( Adafruit_BicolorMatrix &matrix, int8_t *position, uint16_t colour )
{
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

void pushValueToBuffer( float* buffer, float value, int count )
{
	int i;

	// shift values
	for ( i = 0; i < count; i++ ) buffer[ i ] = buffer[ i + 1 ];

	buffer[ count - 1 ] = value;
}

float bufferAverage( float* buffer, int count )
{
	float sum = 0;
	int i;

	for ( i = 0; i < count; i++ ) sum += buffer[i];

	return sum / (float)count;
}



void setup()
{
	int i;

	Serial.begin( 9600 );

	matrix1.begin( 0x70 );
	matrix2.begin( 0x71 );

	uvSensor.begin( 0x61 );

	delay( 250 );

	barometerSensor.begin(); // 0x60
	humiditySensor.begin();  // 0x40

//	TWBR = 12;

	matrix1.setRotation( kMatrixRotation );
	matrix2.setRotation( kMatrixRotation );

	// initialise buffers
	for( i = 0; i < COUNT( _temperatureBuffer ); i++ ) _temperatureBuffer[i] = 0;

	// set up maximum sensor counter value
	_sensorMaxCounter = pow( 2, 8 * sizeof( _sensorCounter ) );
}


void loop()
{
	uint8_t  period = 0;
	uint16_t colour = LED_GREEN;
	float    averageTemperature,
	         deltaValue;
	char     output[2][256],
	         tempStr[10],
	         pressStr[10],
	         humidityStr[10],
	         altTempStr[10],
	         lightStr[10],
	         uvStr[10],
	         irStr[10],
	         serialStr[245];

	_pressure    = _periodCounter % kBarDisplayFrequency == 0 ? barometerSensor.getPressure()    : _pressure,
	_temperature = _periodCounter % kBarDisplayFrequency == 1 ? humiditySensor.readTemperature() : _temperature,
	_humidity    = _periodCounter % kBarDisplayFrequency == 2 ? humiditySensor.readHumidity()    : _humidity;
	_light       = _periodCounter % kBarDisplayFrequency == 3 ? uvSensor.readVisible()           : _light;
	_uv          = _periodCounter % kBarDisplayFrequency == 4 ? (float)uvSensor.readUV() / 100   : _uv;
	_ir          = _periodCounter % kBarDisplayFrequency == 5 ? uvSensor.readIR()                : _ir;
	//altTemp     = barometerSensor.getTemperature();

	// output sensor data to tty
	if ( _periodCounter % kBarDisplayFrequency == 0 )
	{
		ftoa( tempStr, _temperature, 4 );
		ftoa( pressStr, _pressure, 4 );
		ftoa( humidityStr, _humidity, 4 );
		ftoa( lightStr, _light, 4 );
		ftoa( uvStr, _uv, 4 );
		ftoa( irStr, _ir, 4 );
		//ftoa( altTempStr, altTemp, 4 );

		// output to tty
		sprintf( serialStr, "%s,%s,%s,%s,%s,%s", tempStr, pressStr, humidityStr, lightStr, uvStr, irStr /*, altTempStr */ );
//		Serial.println( serialStr );
	}

	// clear displays
	matrix1.clear();
	matrix2.clear();

	// display temperature as bar visualisation on the first matrix
//	if ( _periodCounter % 2 == 0 )
	{
		// push temperature value to buffer
		pushValueToBuffer( _temperatureBuffer, _temperature, COUNT( _temperatureBuffer ) );

		// get average temperature
		averageTemperature = bufferAverage( _temperatureBuffer, COUNT( _temperatureBuffer ) );

		// select colour for bars
		if ( ( averageTemperature < 22.0f && averageTemperature >= 20.0f ) || ( averageTemperature >= 24.0f && averageTemperature < 26.0f ) ) colour = LED_YELLOW;
		else if ( averageTemperature >= 22.0f && averageTemperature < 24.0f ) colour = LED_GREEN;
		else colour = LED_RED;

		// output temperature bars to display
		numberBarDisplay( matrix1, (int)( averageTemperature / 10 ), 0, colour );
		numberBarDisplay( matrix1, (int)( (int)averageTemperature % 10 ), 1, colour );
		numberBarDisplay( matrix1, (int)( ( averageTemperature - (int)averageTemperature ) * 10 ), 2, colour );
		numberBarDisplay( matrix1, (int)( (int)( ( averageTemperature - (int)averageTemperature ) * 100 ) % 10 ), 3, colour );
		matrix1.writeDisplay();
	}

	// calculate moduli
	deltaValue = averageTemperature - kTemperatureLowerLimit;
	if ( deltaValue < 0 ) deltaValue = 0;
	else if ( deltaValue > kTemperatureRange ) deltaValue = kTemperatureRange;
	_moleculeModuli[0] = (uint8_t)( 1 + kMoleculeMultiplier - kMoleculeMultiplier * log10( 1 + 9 * ( deltaValue / kTemperatureRange ) ) );

	// display temperature molecule on the second matrix
	if ( _sensorCounter % _moleculeModuli[0] == 0 ) moleculeMotionDisplay( matrix2, _moleculePositions[0], _moleculeDirections[0], LED_RED );
	else moleculeStaticDisplay( matrix2, _moleculePositions[0], LED_RED );
	if ( _sensorCounter % _moleculeModuli[1] == 0 ) moleculeMotionDisplay( matrix2, _moleculePositions[1], _moleculeDirections[1], LED_YELLOW );
	else moleculeStaticDisplay( matrix2, _moleculePositions[1], LED_YELLOW );
	// if ( _sensorCounter % _moleculeModuli[2] == 0 ) moleculeMotionDisplay( matrix2, _moleculePositions[2], _moleculeDirections[2], LED_GREEN );
	// else moleculeStaticDisplay( matrix2, _moleculePositions[2], LED_GREEN );
	matrix2.writeDisplay();

	// increment period counter
	if ( ++_periodCounter > kBarDisplayFrequency ) _periodCounter = 0;

	// reset sensor counter if it reaches maximum value
	if ( ++_sensorCounter >= _sensorMaxCounter ) _sensorCounter = 0;
}
