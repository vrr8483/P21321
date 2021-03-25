#include "Arduino.h"
#include "Encoder.h"
#include "MainLoop.h"

volatile uint32_t encoderTicks[4] = {0,0,0,0};

/**
 * Generalized ISR that can be used for all encoder
 * @param encoderID - ID for the encoder
 */
void encoderISR(uint8_t encoderID)
{
    encoderTicks[encoderID]++;
}

void encoder0ISR()
{
    encoderISR(0);
}

void encoder1ISR()
{
    encoderISR(1);
}

void encoder2ISR()
{
    encoderISR(2);
}

void encoder3ISR()
{
    encoderISR(3);
}

//It compiles, no matter what VSCode says
void initEncoders()
{
    attachInterrupt(digitalPinToInterrupt(LEFT_TOP_ENC_PIN), encoder0ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(LEFT_BOT_ENC_PIN), encoder1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_TOP_ENC_PIN), encoder2ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_BOT_ENC_PIN), encoder3ISR, RISING);

}