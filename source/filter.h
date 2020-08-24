/*
 * assignment2_drums
 * ECS7012 Music and Audio Programming
 *
 * Second assignment, to create a sequencer-based
 * drum machine which plays sampled drum sounds in loops.
 *
 * This code runs on the Bela embedded audio platform (bela.io).
 * Code partially inspired by Andrew McPherson
 *
 * Jack Walters
 */
 
#include <cmath>


class Filter {

public:
	// Constructor
	Filter();
	
	// Constructor with argument, calls for coefficients to be recalculated
	Filter(float sampleRate);
	
	// Set the sample rate, calls for coefficients to be recalculated
	void setSampleRate(float rate);
	
	// Calculate the next sample of output
	float highPass(float input); 
	
	// Destructor
	~Filter();

private:
	// Calculate coefficients
	void calculateCoefficients(float frequency, float q);

	// State variables
	float sampleRate_;
	float frequency_;
	float q_;

	float b0_;
	float b1_;
	float b2_;
	
	float a0_;
	float a1_;
	float a2_;

	float previousX_[2];
	float previousY_[2];
};
