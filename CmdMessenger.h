/*
  CmdMessenger - library that provides command based messaging

  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  */

#ifndef CmdMessenger_h
#define CmdMessenger_h

#include <inttypes.h>
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

//#include "Stream.h"

extern "C"
{
	// callback functions always follow the signature: void cmd(void);
	typedef void(*messengerCallbackFunction) (void);
}

#define MESSENGERBUFFERSIZE 192   // The length of the commandbuffer  (default: 64)
#define MAXSTREAMBUFFERSIZE 512  // The length of the streambuffer   (default: 64)
#define DEFAULT_TIMEOUT     5000 // Time out on unanswered messages. (default: 5s)

// Message States
enum
{
	kProccesingMessage,            // Message is being received, not reached command separator
	kEndOfMessage,				 // Message is fully received, reached command separator
	kProcessingArguments,			 // Message is received, arguments are being read parsed
};

#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')

class CmdMessenger
{
private:
	// **** Private variables ***

	uint8_t startCommand:1;						// Indicates if sending of a command is underway
	uint8_t pauseProcessing:1;					// pauses processing of new commands, during sending
	uint8_t print_newlines:1;					// Indicates if \r\n should be added after send command
	uint8_t dumped:1;							// Indicates if last argument has been externally read
	uint8_t ArgOk:1;							// Indicated if last fetched argument could be read
	uint8_t lastCommandId;						// ID of last received command
	uint8_t bufferIndex;						// Index where to write data in buffer
	uint8_t bufferLength;						// Is set to MESSENGERBUFFERSIZE
	uint8_t bufferLastIndex;					// The last index of the buffer
	char ArglastChar;							// Bookkeeping of argument escape char
	char CmdlastChar;							// Bookkeeping of command escape char
	char commandBuffer[MESSENGERBUFFERSIZE];	// Buffer that holds the data
	char streamBuffer[MAXSTREAMBUFFERSIZE];		// Buffer that holds the data
	uint8_t messageState;						// Current state of message processing
	char *current;								// Pointer to current buffer position
	char *last;									// Pointer to previous buffer position
	char prevChar;								// Previous char (needed for unescaping)
	Stream *comms;								// Serial data stream

	char command_separator;	// Character indicating end of command (default: ';')
	char field_separator;	// Character indicating end of argument (default: ',')
	char escape_character;	// Character indicating escaping of special chars

	messengerCallbackFunction default_callback;	// default callback function


	// **** Initialize ****

	/**
	 * Enables printing newline after a sent command
	 */
	void init(Stream & ccomms, const char fld_separator, const char cmd_separator, const char esc_character) {
		default_callback = NULL;
		comms = &ccomms;
		print_newlines = false;
		field_separator = fld_separator;
		command_separator = cmd_separator;
		escape_character = esc_character;
		bufferLength = MESSENGERBUFFERSIZE;
		bufferLastIndex = MESSENGERBUFFERSIZE - 1;
		reset();

		default_callback = NULL;

		pauseProcessing = false;
	}

	/**
	 * Resets the command buffer and message state
	 */
	void reset() {
		bufferIndex = 0;
		current = NULL;
		last = NULL;
		dumped = true;
	}

	// **** Command processing ****

	/**
	 * Processes bytes and determines message state
	 */
	__attribute__((always_inline)) inline
	uint8_t processLine(char serialChar) {
		messageState = kProccesingMessage;
		//char serialChar = (char)serialByte;
		bool escaped = isEscaped(&serialChar, escape_character, &CmdlastChar);
		if ((serialChar == command_separator) && !escaped) {
			commandBuffer[bufferIndex] = 0;
			if (bufferIndex > 0) {
				messageState = kEndOfMessage;
				current = commandBuffer;
				CmdlastChar = '\0';
			}
			reset();
		}
		else {
			commandBuffer[bufferIndex] = serialChar;
			bufferIndex++;
			if (bufferIndex >= bufferLastIndex) reset();
		}
		return messageState;
	}

	/**
	 * Dispatches attached callbacks based on command
	 */
	__attribute__((always_inline)) inline
	void handleMessage() {
		lastCommandId = readInt16Arg();
		if (default_callback != NULL)
			(*default_callback)();
	}

	/**
	 * Waits for reply from sender or timeout before continuing
	 */
	__attribute__((always_inline)) inline
	bool blockedTillReply(unsigned int timeout = DEFAULT_TIMEOUT, byte ackCmdId = 1) {
		unsigned long time = millis();
		unsigned long start = time;
		bool receivedAck = false;
		while ((time - start) < timeout && !receivedAck) {
			time = millis();
			receivedAck = checkForAck(ackCmdId);
		}
		return receivedAck;
	}

	/**
	 *   Loops as long data is available to determine if acknowledge has come in
	 */
	__attribute__((always_inline)) inline
	bool checkForAck(byte ackCommand) {
		while (comms->available()) {
			//Processes a byte and determines if an acknowlegde has come in
			int messageState = processLine(comms->read());
			if (messageState == kEndOfMessage) {
				int id = readInt16Arg();
				if (ackCommand == id && ArgOk) {
					return true;
				}
				else {
					return false;
				}
			}
			return false;
		}
		return false;
	}

	// **** Command sending ****

	/**
	 * Print variable of type T binary in binary format
	 */
	template < class T >
	void writeBin(const T & value)
	{
		const byte *bytePointer = (const byte *)(const void *)&value;
		for (unsigned int i = 0; i < sizeof(value); i++)
		{
			printEsc(*bytePointer);
			bytePointer++;
		}
	}

	// **** Command receiving ****

	/**
	 * Find next argument in command
	 */
	int findNext(char *str, char delim) {
		int pos = 0;
		bool escaped = false;
		bool EOL = false;
		ArglastChar = '\0';
		while (true) {
			escaped = isEscaped(str, escape_character, &ArglastChar);
			EOL = (*str == '\0' && !escaped);
			if (EOL) {
				return pos;
			}
			if (*str == field_separator && !escaped) {
				return pos;
			}
			else {
				str++;
				pos++;
			}
		}
		return pos;
	}

	/**
	 * Read a variable of any type in binary format
	 */
	template < class T >
	T readBin(char *str)
	{
		T value;
		unescape(str);
		byte *bytePointer = (byte *)(const void *)&value;
		for (unsigned int i = 0; i < sizeof(value); i++)
		{
			*bytePointer = str[i];
			bytePointer++;
		}
		return value;
	}

	template < class T >
	T empty()
	{
		T value;
		byte *bytePointer = (byte *)(const void *)&value;
		for (unsigned int i = 0; i < sizeof(value); i++)
		{
			*bytePointer = '\0';
			bytePointer++;
		}
		return value;
	}

	// **** Escaping tools ****

	/**
	 * Split string in different tokens, based on delimiter
	 * Note that this is basically strtok_r, but with support for an escape character
	 */
	char *split_r(char *str, const char delim, char **nextp) {
		char *ret;
		// if input null, this is not the first call, use the nextp pointer instead
		if (str == NULL) {
			str = *nextp;
		}
		// Strip leading delimiters
		while (findNext(str, delim) == 0 && *str) {
			str++;
		}
		// If this is a \0 char, return null
		if (*str == '\0') {
			return NULL;
		}
		// Set start of return pointer to this position
		ret = str;
		// Find next delimiter
		str += findNext(str, delim);
		// and exchange this for a a \0 char. This will terminate the char
		if (*str) {
			*str++ = '\0';
		}
		// Set the next pointer to this char
		*nextp = str;
		// return current pointer
		return ret;
	}

	/**
	 * Indicates if the current character is escaped
	 */
	bool isEscaped(char *currChar, const char escapeChar, char *lastChar) {
		bool escaped;
		escaped = (*lastChar == escapeChar);
		*lastChar = *currChar;

		// special case: the escape char has been escaped:
		if (*lastChar == escape_character && escaped) {
			*lastChar = '\0';
		}
		return escaped;
	}

	/**
	 * Escape and print a string
	 */
	void printEsc(char *str) {
		while (*str != '\0') {
			printEsc(*str++);
		}
	}

	/**
	 * Escape and print a character
	 */
	void printEsc(char str) {
		if (str == field_separator || str == command_separator || str == escape_character || str == '\0') {
			comms->print(escape_character);
		}
		comms->print(str);
	}

public:

	// ****** Public functions ******

	// **** Initialization ****

	/**
	 * CmdMessenger constructor
	 */
	CmdMessenger(
		Stream & ccomms,
		const char fld_separator = ',',
		const char cmd_separator = ';',
		const char esc_character = '/'
	) {
		init(ccomms, fld_separator, cmd_separator, esc_character);
	}

	/**
	 * Enables printing newline after a sent command
	 */
	void printLfCr(bool addNewLine = true) {
		print_newlines = addNewLine;
	}

	/**
	 * Attaches an default function for commands that are not explicitly attached
	 */
	void attach(messengerCallbackFunction newFunction) {
		default_callback = newFunction;
	}

	// **** Command processing ****

	/**
	 * Feeds serial data in CmdMessenger
	 */
	void feedinSerialData() {
		while (!pauseProcessing && comms->available())
		{
			// The Stream class has a readBytes() function that reads many bytes at once. On Teensy 2.0 and 3.0, readBytes() is optimized.
			// Benchmarks about the incredible difference it makes: http://www.pjrc.com/teensy/benchmark_usb_serial_receive.html

			size_t bytesAvailable = min(comms->available(), MAXSTREAMBUFFERSIZE);
			comms->readBytes(streamBuffer, bytesAvailable);

			// Process the bytes in the stream buffer, and handles dispatches callbacks, if commands are received
			for (size_t byteNo = 0; byteNo < bytesAvailable; byteNo++)
			{
				int messageState = processLine(streamBuffer[byteNo]);

				// If waiting for acknowledge command
				if (messageState == kEndOfMessage)
				{
					handleMessage();
				}
			}
		}
	}

	/**
	 * Gets next argument. Returns true if an argument is available
	 */
	bool next() {
		char * temppointer = NULL;
		// Currently, cmd messenger only supports 1 char for the field seperator
		switch (messageState) {
		case kProccesingMessage:
			return false;
		case kEndOfMessage:
			temppointer = commandBuffer;
			messageState = kProcessingArguments;
		default:
			if (dumped)
				current = split_r(temppointer, field_separator, &last);
			if (current != NULL) {
				dumped = true;
				return true;
			}
		}
		return false;
	}

	/**
	 * Returns if an argument is available. Alias for next()
	 */
	__attribute__((always_inline)) inline
	bool available() { return next(); }

	/**
	 * Returns if the latest argument is well formed.
	 */
	__attribute__((always_inline)) inline
	bool isArgOk() { return ArgOk; }

	/**
	 * Returns the commandID of the current command
	 */
	__attribute__((always_inline)) inline
	uint8_t commandID() { return lastCommandId; }

	// ****  Command sending ****

	/**
	 * Send a command with a single argument of any type
	 * Note that the argument is sent as string
	 */
	template < class T >
	bool sendCmd(byte cmdId, T arg, bool reqAc = false, byte ackCmdId = 1,
		unsigned int timeout = DEFAULT_TIMEOUT)
	{
		if (!startCommand) {
			sendCmdStart(cmdId);
			sendCmdArg(arg);
			return sendCmdEnd(reqAc, ackCmdId, timeout);
		}
		return false;
	}

	/**
	 * Send a command with a single argument of any type
	 * Note that the argument is sent in binary format
	 */
	template < class T >
	bool sendBinCmd(byte cmdId, T arg, bool reqAc = false, byte ackCmdId = 1,
		unsigned int timeout = DEFAULT_TIMEOUT)
	{
		if (!startCommand) {
			sendCmdStart(cmdId);
			sendCmdBinArg(arg);
			return sendCmdEnd(reqAc, ackCmdId, timeout);
		}
		return false;
	}

	/**
	 * Send a command without arguments, without acknowledge
	 */
	bool sendCmd(byte cmdId) {
		if (!startCommand) {
			sendCmdStart(cmdId);
			return sendCmdEnd(false, 1, DEFAULT_TIMEOUT);
		}
		return false;
	}

	/**
	 * Send a command without arguments, with acknowledge
	 */
	bool sendCmd(byte cmdId, bool reqAc, byte ackCmdId) {
		if (!startCommand) {
			sendCmdStart(cmdId);
			return sendCmdEnd(reqAc, ackCmdId, DEFAULT_TIMEOUT);
		}
		return false;
	}
	// **** Command sending with multiple arguments ****

	/**
	 * Send start of command. This makes it easy to send multiple arguments per command
	 */
	void sendCmdStart(byte cmdId) {
		if (!startCommand) {
			startCommand = true;
			pauseProcessing = true;
			comms->print(cmdId);
		}
	}

	/**
	 * Send an escaped command argument
	 */
	void sendCmdEscArg(char *arg) {
		if (startCommand) {
			comms->print(field_separator);
			printEsc(arg);
		}
	}

	/**
	 * Send formatted argument.
	 *  Note that floating points are not supported and resulting string is limited to 128 chars
	 */
	void sendCmdfArg(char *fmt, ...) {
		const int maxMessageSize = 128;
		if (startCommand) {
			char msg[maxMessageSize];
			va_list args;
			va_start(args, fmt);
			vsnprintf(msg, maxMessageSize, fmt, args);
			va_end(args);

			comms->print(field_separator);
			comms->print(msg);
		}
	}

	/**
	 * Send end of command
	 */
	bool sendCmdEnd(bool reqAc = false, byte ackCmdId = 1, unsigned int timeout = DEFAULT_TIMEOUT) {
		bool ackReply = false;
		if (startCommand) {
			comms->print(command_separator);
			if (print_newlines)
				comms->println(); // should append BOTH \r\n
			if (reqAc) {
				ackReply = blockedTillReply(timeout, ackCmdId);
			}
		}
		pauseProcessing = false;
		startCommand = false;
		return ackReply;
	}

	/**
	 * Send a single argument as string
	 *  Note that this will only succeed if a sendCmdStart has been issued first
	 */
	template < class T > void sendCmdArg(T arg)
	{
		if (startCommand) {
			comms->print(field_separator);
			comms->print(arg);
		}
	}

	/**
	 * Send a single argument as string with custom accuracy
	 *  Note that this will only succeed if a sendCmdStart has been issued first
	 */
	template < class T > void sendCmdArg(T arg, unsigned int n)
	{
		if (startCommand) {
			comms->print(field_separator);
			comms->print(arg, n);
		}
	}

	/**
	 * Send double argument in scientific format.
	 *  This will overcome the boundary of normal d sending which is limited to abs(f) <= MAXLONG
	 */
	void sendCmdSciArg(double arg, unsigned int n = 6) {
		if (startCommand)
		{
			comms->print(field_separator);
			printSci(arg, n);
		}
	}


	/**
	 * Send a single argument in binary format
	 *  Note that this will only succeed if a sendCmdStart has been issued first
	 */
	template < class T > void sendCmdBinArg(T arg)
	{
		if (startCommand) {
			comms->print(field_separator);
			writeBin(arg);
		}
	}

	// **** Command receiving ****

	/**
	 * Read the next argument as bool
	 */
	bool readBoolArg() {
		return (readInt16Arg() != 0) ? true : false;
	}

	/**
	 * Read the next argument as int
	 */
	int16_t readInt16Arg() {
		if (next()) {
			dumped = true;
			ArgOk = true;
			return atoi(current);
		}
		ArgOk = false;
		return 0;
	}

	/**
	 * Read the next argument as int
	 */
	int32_t readInt32Arg() {
		if (next()) {
			dumped = true;
			ArgOk = true;
			return atol(current);
		}
		ArgOk = false;
		return 0L;
	}

	/**
	 * Read the next argument as char
	 */
	char readCharArg() {
		if (next()) {
			dumped = true;
			ArgOk = true;
			return current[0];
		}
		ArgOk = false;
		return 0;
	}

	/**
	 * Read the next argument as float
	 */
	float readFloatArg() {
		if (next()) {
			dumped = true;
			ArgOk = true;
			//return atof(current);
			return strtod(current, NULL);
		}
		ArgOk = false;
		return 0;
	}

	/**
	 * Read the next argument as double
	 */
	double readDoubleArg() {
		if (next()) {
			dumped = true;
			ArgOk = true;
			return strtod(current, NULL);
		}
		ArgOk = false;
		return 0;
	}

	/**
	 * Read next argument as string.
	 * Note that the String is valid until the current command is replaced
	 */
	char *readStringArg() {
		if (next()) {
			dumped = true;
			ArgOk = true;
			return current;
		}
		ArgOk = false;
		return '\0';
	}

	/**
	 * Return next argument as a new string
	 * Note that this is useful if the string needs to be persisted
	 */
	void copyStringArg(char *string, uint8_t size) {
		if (next()) {
			dumped = true;
			ArgOk = true;
			strlcpy(string, current, size);
		}
		else {
			ArgOk = false;
			if (size) string[0] = '\0';
		}
	}

	/**
	 * Compare the next argument with a string
	 */
	uint8_t compareStringArg(char *string) {
		if (next()) {
			if (strcmp(string, current) == 0) {
				dumped = true;
				ArgOk = true;
				return 1;
			}
			else {
				ArgOk = false;
				return 0;
			}
		}
		return 0;
	}

	/**
	 * Read an argument of any type in binary format
	 */
	template < class T > T readBinArg()
	{
		if (next()) {
			dumped = true;
			return readBin < T >(current);
		}
		else {
			return empty < T >();
		}
	}

	// **** Escaping tools ****

	/**
	 * Unescapes a string
	 * Note that this is done inline
	 */
	void unescape(char *fromChar) {
		// Move unescaped characters right
		char *toChar = fromChar;
		while (*fromChar != '\0') {
			if (*fromChar == escape_character) {
				fromChar++;
			}
			*toChar++ = *fromChar++;
		}
		// Pad string with \0 if string was shortened
		for (; toChar < fromChar; toChar++) {
			*toChar = '\0';
		}
	}

	/**
	 * Print float and double in scientific format
	 */
	void printSci(double f, unsigned int digits) {
		// handle sign
		if (f < 0.0)
		{
			comms->print('-');
			f = -f;
		}

		// handle infinite values
		if (isinf(f))
		{
			comms->print("INF");
			return;
		}
		// handle Not a Number
		if (isnan(f))
		{
			comms->print("NaN");
			return;
		}

		// max digits
		if (digits > 6) digits = 6;
		long multiplier = pow(10, digits);     // fix int => long

		int exponent;
		if (abs(f) < 10.0) {
			exponent = 0;
		}
		else {
			exponent = int(log10(f));
		}
		float g = f / pow(10, exponent);
		if ((g < 1.0) && (g != 0.0))
		{
			g *= 10;
			exponent--;
		}

		long whole = long(g);                     // single digit
		long part = long((g - whole)*multiplier + 0.5);  // # digits
		// Check for rounding above .99:
		if (part == 100) {
			whole++;
			part = 0;
		}
		char format[16];
		sprintf(format, "%%ld.%%0%dldE%%+d", digits);
		char output[16];
		sprintf(output, format, whole, part, exponent);
		comms->print(output);
	}
};
#endif
