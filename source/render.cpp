/*
 * assignment2_drums
 * ECS7012 Music and Audio Programming
 *
 * Second assignment, to create a sequencer-based
 * drum machine which plays sampled drum sounds in loops.
 *
 * This code runs on the Bela embedded audio platform (bela.io).
 *
 * Andrew McPherson, Becky Stewart and Victor Zappi
 * 2015-2020
 */

#include <Bela.h>
#include <cmath>
#include "drums.h"
#include "filter.h"


/* Variables which are given to you: */

// Initialise arrays with external linkage (of size NUMBER_OF_DRUMS) to contain drum sample buffers, lengths of those buffers, beat patterns and
// the lengths of those patterns
extern float *gDrumSampleBuffers[NUMBER_OF_DRUMS];
extern int gDrumSampleBufferLengths[NUMBER_OF_DRUMS];
extern int *gPatterns[NUMBER_OF_PATTERNS];
extern int gPatternLengths[NUMBER_OF_PATTERNS];

int gIsPlaying = 0;	 // Initialise start/stop variable

int gReadPointers[16];	// Initialise array of read pointers
int gDrumBufferForReadPointer[16];	// Initialise array of values designating buffers that are assigned to read pointers

// Initialiase variables to store drum pattern information, used in startNextEvent
int gCurrentPattern = 0;
int gCurrentIndexInPattern = 0;	

extern int gIsPlaying;

/* This indicates whether we should play the samples backwards.
 */
int gPlaysBackwards = 0;

/* For bonus step only: these variables help implement a fill
 * (temporary pattern) which is triggered by tapping the board.
 */
int gShouldPlayFill = 0;
int gPreviousPattern = 0;

/* TODO: Declare any further global variables you need here */

// Counters used in audio variable initialisations and reassignments inside render()
int gMetronomeCounter = 0;
int gEventIntervalMilliseconds = 500;
int printCounter = 0;
int initialiseCounter = 0;
int movingAverageCounter = 0;
int tapCounter = 0;
int orientationCounter = 0;
int tapInitialiseCounter = 0;

float *audioOutputBuffer;  // Pointer to audio buffer allocated with dynamic memory

// Arrays to contain X, Y and Z accelerometer data
float accelerometerPinReference[3];
float accelerometerPinMapped[3];
float movingAverageResult[3];

const int movingAverageOrder = 11;	// Moving average filter order initialisation

// Two dimensional array to contain accelerometer previous states (filter order x accelerometer dimensions)
float movingAverageArray[movingAverageOrder][3];

// y[n] for filters 1 and 2, combined in series to create a biquad filter
float F1ZPinHighPass = 0;
float F2ZPinHighPass = 0;

const int LEDPin = 0;	// LED digital pin value
const int buttonPin = 1;	// Button digital pin value
const int metronomeTogglePin = 0;	// Metronome toggle analog pin value
const int tempoTogglePin = 1;	// Tempo adjust analog pin value
const int accelerometerXPin = 2;	// Accelerometer X value analog input 
const int accelerometerYPin = 3;	// Accelerometer Y value analog input
const int accelerometerZPin = 4;	// Accelerometer Z value analog input

int previousLEDState = 0;	// Keep track of the previous LED state in render()
int previousButtonState = 0;	// Keep track of the previous button state in render()

// Button debouncing state machine states
enum {
	stateOff = 0,
	stateOn
};

int debounceState = stateOff;
int debounceCounter = 0;
int	debounceInterval = 882;

// Orientation pattern selection state machine states
enum orientationState {
	flat = 0,
	left,
	right,
	back,
	front,
	upsideDown
};

// Orientation state and threshold initialisation
int orientationState = flat;
float orientationStateMachineThreshold = 0.5;

// Declaring objects 'Filter1' and 'Filter2' of class type Filter
Filter Filter1, Filter2;

// setup() is called once before the audio rendering starts.
// Use it to perform any initialisation and allocation which is dependent
// on the period size or sample rate.
//
// userData holds an opaque pointer to a data structure that was passed
// in from the call to initAudio().
//
// Return true on success; returning false halts the program.

bool setup(BelaContext *context, void *userData)
{
	// Set LED to an output
	pinMode(context, 0, LEDPin, OUTPUT);
	// Set button to an input
	pinMode(context, 0, buttonPin, INPUT);
	
	// Declare audioOutputBuffer to be the same length as the Bela Context's buffer, as inside render
	// buffer information will be copied into it
	audioOutputBuffer = new float[context->audioFrames];
	audioOutputBuffer = 0;	// initialise this buffer to 0;
	
	// These 3 for loops set every index in their respective arrays to 0
	for (int i = 0; i < 16; i++) {
		gReadPointers[i] = 0;
		gDrumBufferForReadPointer[i] = -10;  // A value of -10 designates the index is inactive
	}
	
	for (int i = 0; i < movingAverageOrder; i++) {
		for (int j = 0; j < 3; j++) {
			movingAverageArray[i][j] = 0;
		}
	}
	
	for (int i = 0; i < 3; i++) {
		accelerometerPinReference[i] = 0;
		accelerometerPinMapped[i] = 0;
		movingAverageResult[i] = 0;
	}
	
	// The private variable sampleRate_ is set as the Bela Context's sample rate through the public method setSampleRate of
	// Filter1 and Filter2 of class type Filter
	Filter1.setSampleRate(context->audioSampleRate);
	Filter2.setSampleRate(context->audioSampleRate);
	
	rt_printf("\n");
	rt_printf("Place device flat on table initially!\n");
	rt_printf("\n");
	
	return true;
}

// render() is called regularly at the highest priority by the audio engine.
// Input and output are given from the audio hardware and the other
// ADCs and DACs (if available). If only audio is available, numMatrixFrames
// will be 0.

void render(BelaContext *context, void *userData)
{
	// This initial for loop iterates through the number of audio frames in the Bela buffer
	for (unsigned int n = 0; n < context->audioFrames; n++) {
		
	// Read analog pins 2, 3 and 4, and save their values at half the sample rate
	float accelerometerXPinVoltage = analogRead(context,n/2,accelerometerXPin);
	float accelerometerYPinVoltage = analogRead(context,n/2,accelerometerYPin);
	float accelerometerZPinVoltage = analogRead(context,n/2,accelerometerZPin);
	
	// A potentiometer (due to lack of 2nd button) is used to toggle the gIsPlaying variable
	float metronomeRead = analogRead(context,n/2,metronomeTogglePin);
		if (metronomeRead > 0.5) {
			gIsPlaying = 1;
		} else if (metronomeRead < 0.5) {
			gIsPlaying = 0;
		}
	
	// Values read from a pontentiometer at half the sample rate are used to control tempo
	float tempoRead = analogRead(context,n/2,tempoTogglePin);
	gEventIntervalMilliseconds = map(tempoRead,0,1,50,1000);	// Map potentiometer values to be between 50 and 1000
	gMetronomeCounter++;
		if (gMetronomeCounter <= 2500 && gIsPlaying == 1) {
			// Turn on LED for first 2500 samples of beat
			digitalWrite(context, n, LEDPin, 1);
		} else if (gMetronomeCounter >= 2500 && gIsPlaying == 1) {
			// Turn off LED after 2500 samples
			digitalWrite(context, n, LEDPin, 0);
		}
		// Reset metronome counter after user defined number of samples (gEventIntervalMilliseconds value converted to samples by
		// multiplying it by Fs/1000)
		if (gMetronomeCounter >= (gEventIntervalMilliseconds*context->audioSampleRate)/1000) {
			startNextEvent(gIsPlaying);
			gMetronomeCounter = 0;
		}
	
	// Take 1 reference reading of accelerometer pin voltage values at sample 10, to be used as a normalising value for
	// mapping accelerometer voltage to acceleration
	if (initialiseCounter++ == 10) {
		accelerometerPinReference[0] = accelerometerXPinVoltage;
		accelerometerPinReference[1] = accelerometerYPinVoltage;
		accelerometerPinReference[2] = accelerometerZPinVoltage;
		rt_printf("XPinReference: %f\n",accelerometerPinReference[0]);
		rt_printf("YPinReference: %f\n",accelerometerPinReference[1]);
		rt_printf("ZPinReference: %f\n",accelerometerPinReference[2]);
		rt_printf("\n");
	}
	
	// Map accelerometer voltage to acceleration. All 90° shifts in axes correspond to a ~0.25 change in analog input voltage.
	// For X and Y readings: in_min = 0.5 * reference, in_max = 1.5 * reference reading (when the board is flat); these values are then
	// mapped between -1 and 1. The Z value's reference value is +1g acceleration due to gravity, so a 180° rotation to -1g is
	// (in theory) reference - reference (experimentally I found this to be reference*0.27)
	if (movingAverageCounter++ >= context->audioSampleRate*0.1) {
		accelerometerPinMapped[0] = map(accelerometerXPinVoltage,accelerometerPinReference[0]*0.5,accelerometerPinReference[0]*1.5,-1,1);
		accelerometerPinMapped[1] = map(accelerometerYPinVoltage,accelerometerPinReference[1]*0.5,accelerometerPinReference[1]*1.45,-1,1);
		accelerometerPinMapped[2] = map(accelerometerZPinVoltage,accelerometerPinReference[2],accelerometerPinReference[2]*0.27,1,-1);
		movingAverageResult[0] = movingAverage(accelerometerPinMapped[0],0);
		movingAverageResult[1] = movingAverage(accelerometerPinMapped[1],1);
		movingAverageResult[2] = movingAverage(accelerometerPinMapped[2],2);
		movingAverageCounter = 0;
	}
	
	// Print to console every 441 samples
	if (printCounter++ == context->audioSampleRate*0.1) {
		rt_printf("Orientation State: %i\n",orientationState);
		rt_printf("\n");
		printCounter = 0;
	}
	
	// Because of the initial spike in the accelerometer Z value reading due to gravity, taps (to enable a temporary drum fill) are only
	// read after 1 second
	
	if (tapCounter++ == context->audioSampleRate*0.1) {
		// Using 2 2nd order high pass filters in series creates a 4th order biquad high pass filter
		F1ZPinHighPass = Filter1.highPass(accelerometerPinMapped[2]);  // Filter 1 high passing non-averaged accelerometer Z pin data
		F2ZPinHighPass = Filter2.highPass(F1ZPinHighPass);	// Filter 2 high passing Filter 1's data
	tapCounter = 0;
	}
	if (tapInitialiseCounter++ > context->audioSampleRate) {
		if (F2ZPinHighPass >= 0.8) {
			gShouldPlayFill = 1;  // If Filter 2's output spikes above the thresold of 1.0, enable global variable gShouldPlayFill
		}
	}

	// Button state machine
	int value = digitalRead(context, n, buttonPin);
   	if(debounceState == stateOff) {
   		// Button is not being interacted with
   		}
   		if (value == 0) {
   			debounceState = stateOn;
   			debounceCounter = 0;
   	}
   	else if(debounceState == stateOn) {
   		// Button was just pressed, wait for debounce
   		// Input: run counter, wait for timeout
   		if (debounceCounter++ >= debounceInterval) {
     		startPlayingDrum(2);
	   		debounceState = stateOff;
	   	}
	}
	
  	
	// Orientation pattern selection state machine. The board has 6 possible states: flat, left, right, back, front, upside down
	if (orientationCounter++ == context->audioSampleRate*0.1) {
		// Do not bypass orientation pattern selection state machine
		if (gShouldPlayFill == 0) {
			if (orientationState == flat) {
				gPlaysBackwards = 0;	// Play forwards
				gCurrentPattern = 0;
				if (movingAverageResult[0] <= -orientationStateMachineThreshold) {
					orientationState = left;	// If X <= -0.5, orientation state goes to left
				} else if (movingAverageResult[0] >= orientationStateMachineThreshold) {
					orientationState = right;    // If X >= 0.5, orientation state goes to right
				} else if (movingAverageResult[1] <= -orientationStateMachineThreshold) {
					orientationState = back;	// If Y <= -0.5, orientation state goes to back
				} else if (movingAverageResult[1] >= orientationStateMachineThreshold) {
					orientationState = front;    // If Y >= 0.5, orientation state goes to front
				}
			} else if (orientationState == left) {
				gPlaysBackwards = 0;	// Play forwards
				gCurrentPattern = 1;
				if (movingAverageResult[0] >= -orientationStateMachineThreshold && movingAverageResult[2] >= orientationStateMachineThreshold*1.5) {
					orientationState = flat;	// If X >= -0.5 & Z >= 0.75, orientation state goes to flat
				} else if (movingAverageResult[1] <= -orientationStateMachineThreshold) {
					orientationState = back;	// If Y <= -0.5, orientation state goes to back
				} else if (movingAverageResult[1] >= orientationStateMachineThreshold) {
					orientationState = front;	// If Y >= 0.5, orientation state goes to front
				} else if (movingAverageResult[0] >= -orientationStateMachineThreshold && movingAverageResult[2] <= -orientationStateMachineThreshold*1.5) {
					orientationState = upsideDown;	// If X >= -0.5 & Z <= -0.75, orientation state goes to upside down
				}
			} else if (orientationState == right) {
				gPlaysBackwards = 0;	// Play forwards
				gCurrentPattern = 2;
				if (movingAverageResult[0] <= orientationStateMachineThreshold && movingAverageResult[2] >= orientationStateMachineThreshold*1.5) {
					orientationState = flat;	// If X <= 0.5 & Z >= 0.75, orientation state goes to flat
				} else if (movingAverageResult[1] <= -orientationStateMachineThreshold) {
					orientationState = back;	// If Y <= -0.5, orientation state goes to back
				} else if (movingAverageResult[1] >= orientationStateMachineThreshold) {
					orientationState = front;	// If Y >= 0.5, orientation state goes to front
				} else if (movingAverageResult[0] >= -orientationStateMachineThreshold && movingAverageResult[2] <= -orientationStateMachineThreshold*1.5) {
					orientationState = upsideDown;	// If X >= -0.5 & Z <= -0.75, orientation state goes to upside down
				}
			} else if (orientationState == back) {
				gPlaysBackwards = 0;	// Play forwards
				gCurrentPattern = 3;
				if (movingAverageResult[0] <= -orientationStateMachineThreshold) {
					orientationState = left;	// If X <= -0.5, orientation state goes to left
				} else if (movingAverageResult[0] >= orientationStateMachineThreshold) {
					orientationState = right;	// If X >= 0.5, orientation state goes to right
				} else if (movingAverageResult[1] >= -orientationStateMachineThreshold && movingAverageResult[2] >= orientationStateMachineThreshold*1.5) {
					orientationState = flat;	// If Y >= -0.5 & Z >= 0.75, orientation state goes to flat
				} else if (movingAverageResult[1] >= -orientationStateMachineThreshold && movingAverageResult[2] <= -orientationStateMachineThreshold*1.5) {
					orientationState = upsideDown;    // If Y >= -0.5 & Z <= -0.75, orientation state goes to flat
				}		
			} else if (orientationState == front) {
				gPlaysBackwards = 0;	// Play forwards
				gCurrentPattern = 4;
				if (movingAverageResult[0] <= -orientationStateMachineThreshold) {
					orientationState = left;	// If X <= -0.5, orientation state goes to left
				} else if (movingAverageResult[0] >= orientationStateMachineThreshold) {
					orientationState = right;	// If X >= 0.5, orientation state goes to right
				} else if (movingAverageResult[1] <= orientationStateMachineThreshold && movingAverageResult[2] >= orientationStateMachineThreshold*1.5) {
					orientationState = flat;	// If Y <= 0.5 & Z >= 0.75, orientation state goes to flat
				} else if (movingAverageResult[1] >= -orientationStateMachineThreshold && movingAverageResult[2] <= -orientationStateMachineThreshold*1.5) {
					orientationState = upsideDown;    // If Y >= -0.5 & Z <= -0.75, orientation state goes to flat
				}	
			} else if (orientationState == upsideDown) {
				gPlaysBackwards = 1;	// Play backwards
				if (movingAverageResult[0] <= -orientationStateMachineThreshold) {
					orientationState = left;	// If X <= -0.5, orientation state goes to left
				} else if (movingAverageResult[0] >= orientationStateMachineThreshold) {
					orientationState = right;	// If X >= 0.5, orientation state goes to right
				} else if (movingAverageResult[1] <= -orientationStateMachineThreshold) {
					orientationState = back;	// If Y <= -0.5, orientation state goes to back
				} else if (movingAverageResult[1] >= orientationStateMachineThreshold) {
					orientationState = front;	// If Y >= 0.5, orientation state goes to front
				}
			}
		// Bypass orientation pattern selection state machine
		} else if (gShouldPlayFill == 1) {
			// If this if statement's conditions are met (through tapping the board), the current drum pattern is saved and a special value FILL_PATTERN is
			// set as the current pattern
			gPreviousPattern = gCurrentPattern;
			gCurrentPattern = FILL_PATTERN;
		}
	orientationCounter = 0;
	}

	float out = 0;    // Reset primary audio buffer out to 0
   	// Read through all 16 indexes in the gReadPointers and gDrumBufferForReadPointer arrays
   	for (int i = 0; i < 16; i++) {
   		// Check if the drum buffer at index i is inactive
   		if (gDrumBufferForReadPointer[i] != -10) {
   			// If it is, copy the contents of the drum buffer at index gDrumBufferForReadPointer[i] to the secondary audio buffer audioOutputBuffer
   			audioOutputBuffer = gDrumSampleBuffers[gDrumBufferForReadPointer[i]];
   			if (gPlaysBackwards == 0) {
   				// If we are not playing the sample backwards (i.e. if the board is being held in any orientation that is not upside down), read
   				// through the samples in audioOutputBuffer with the read pointer at index i, and add them to out through compound addition
	  			out += audioOutputBuffer[gReadPointers[i]++];
	  			// If the read pointer reaches the end of the buffer, indicate gDrumBufferForReadPointer[i] is no longer active
	  			if (gReadPointers[i] >= gDrumSampleBufferLengths[gDrumBufferForReadPointer[i]]) {
	  				gDrumBufferForReadPointer[i] = -10;
	   			}
   			} else if (gPlaysBackwards == 1) {
   				// If we are playing the sample backwards (i.e. the board is being help upside down), count through the samples in the buffer
   				// negatively
   				out += audioOutputBuffer[gReadPointers[i]--];
   				// If the read pointer reaches 0 or goes beyond it, indicate gDrumBufferForReadPointer[i] is no longer active
   				if (gReadPointers[i] <= 0) {
	   				gDrumBufferForReadPointer[i] = -10;
	   			}
   			}	
   		}
   	}
   	
   	// Iterate through all channels specified in the Bela Context, and add the audio data from variable out to all
   	for(unsigned int channel = 0; channel < context->audioOutChannels; channel++) {
		// Write the sample to every audio output channel
    	audioWrite(context, n, channel, out);
  		}
	}
}


/* Start playing a particular drum sound given by drumIndex.
 */
void startPlayingDrum(int drumIndex) {
	// Read through all 16 indexes in the gReadPointers and gDrumBufferForReadPointer arrays
	for (int i = 0; i < 16; i++) {
		if (gPlaysBackwards == 0) {
			// If it is not indicated that samples are playing backwards, check that the gDrumBufferForReadPointer at index i is inactive,
			// and if it is, assign the drum index passed as an argument into the startPlayingDrum() function to it and reset gReadPointers[i]
			if (gDrumBufferForReadPointer[i] == -10) {
				gDrumBufferForReadPointer[i] = drumIndex;
				gReadPointers[i] = 0;
				break;
			}
		} else if (gPlaysBackwards == 1) {
			// If it is indicated that samples are playing backwards, check that the gDrumBufferForReadPointer at index i is inactive,
			// and if it is, assign the drum index passed as an argument into the startPlayingDrum() function to it and set gReadPointers[i]
			// to length of the drum buffer (so it can start playing backwards)
			if (gDrumBufferForReadPointer[i] == -10) {
				gDrumBufferForReadPointer[i] = drumIndex;
				gReadPointers[i] = gDrumSampleBufferLengths[gDrumBufferForReadPointer[i]];
				break;
			}
		}	
	}
}

/* Start playing the next event in the pattern */
void startNextEvent(int toggle) {
	if (toggle == 1) {
		// If gIsPlaying is 1 and passed as an argument into startNextEvent, iterate through number of drum buffers
		for (int i = 0; i < NUMBER_OF_DRUMS-1; i++) {
			// If the eventContainsDrum() function (at specific index of specific drum pattern, in relation to drum at index i) returns
			// a 1, call the startPlayingDrum() function at that index i
			if (eventContainsDrum(gPatterns[gCurrentPattern][gCurrentIndexInPattern], i) == 1) {
				startPlayingDrum(i);
			}
		}
		
		// Modulo arithmetic ensures the pattern index wraps around the length of the pattern, hence staying within it
		gCurrentIndexInPattern = (gCurrentIndexInPattern + 1 + gPatternLengths[gCurrentPattern]) % gPatternLengths[gCurrentPattern];
		
		// If modulo is 0 (i.e. the end of pattern has been reached) set the current index in pattern to 0, and if that pattern was a
		// pattern (initiated by tapping the board), also set gShouldPlayFill to 0 and reinstate the pattern that was playing before the fill
		if (gCurrentIndexInPattern == 0) {
			if (gCurrentPattern == FILL_PATTERN) {
				gShouldPlayFill = 0;
				gCurrentPattern = gPreviousPattern;
			}
			gCurrentIndexInPattern = 0;
		}
	}	
}

/* Returns whether the given event contains the given drum sound */
int eventContainsDrum(int event, int drum) {
	if(event & (1 << drum))
		return 1;
	return 0;
}

/* Moving average filter */
float movingAverage(float Xn, int arrayIndex) {
	// Reset private sum variable to 0 whenever function is called
	float averagedSum = 0;

	// Sum every element in movingAverageArray array
	for (int i = 0; i < movingAverageOrder; i++) {
		averagedSum += movingAverageArray[i][arrayIndex];
	}
	// Divide this sum by the order of the moving average (the size of the array)
	float averagedResult = averagedSum/movingAverageOrder;
	
	// Input the input sample Xn into index 0, and move every element in the array forward 1 index (back 1 time step)
	for (int i = movingAverageOrder; i > 0; i--) {
		movingAverageArray[i][arrayIndex] = movingAverageArray[i-1][arrayIndex];
	}
	movingAverageArray[0][arrayIndex] = Xn;
	 	
	return averagedResult;
}

// cleanup_render() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in initialise_render().

void cleanup(BelaContext *context, void *userData)
{
	delete[] audioOutputBuffer;
}
