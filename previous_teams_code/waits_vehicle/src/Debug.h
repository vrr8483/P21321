#define DEBUG_ENABLED (1)
#define BAUD_RATE (38400)

void initSerial();
void Serial_Printf(const char * str, ...);
uint8_t characterAvailable();
char getChar();
void printPrompt();
void putChar(char c);
void printUsage();
void handleCommand(char * str);
void testPlots();