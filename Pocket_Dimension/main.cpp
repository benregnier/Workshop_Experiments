// TODO: new card using Computer Card
// Description: cv / midi looper with decay
// two channels of pitch/ envelope / trigger
// looping with variable length and start
// decay feature where triggers and envelopes slowly reduxe each loop
// loop is event based - a midi or trigger in will sample
// loop keeps note and velocity info
// new event overwrites existing 
// determine plausible max buffer with samples at 100ms
// decay keeps note but reduces output trigger and envelope voltage / midi velocity by a percentage
// audio ins/ outs are velocity in and envelope out
// cv ins/ outs are pitch
// triggers are triggers
// midi and cv share common loops
// works on midi channel 1 and 2 as device
// main knob is loop length
// x knob is loop position (scrub)
// y is decay amount