// set the MUX logic pins - reading channels 0,1,2,3 = 00 01 10 11
void muxUpdate(int num)
{
  digitalWrite(MUX_LOGIC_A, ((num >> 0)) & 1);
  digitalWrite(MUX_LOGIC_B, ((num >> 1)) & 1);
  // NB this seems to need 1us delay for pins to 'settle' before reading.
}


// Read the Mux IO pins into the value store
void muxRead(int num)
{
  switch (num)
  {

    case 0:
      // read Main Knob
      analog[4]->update(analogRead(A2));
      // Read CV 1
      analog[2]->update(maxRange-analogRead(A3)); // inverted 
      break;

    case 1:
      // read Knob X
      analog[5]->update(analogRead(A2));
      // Read CV 2
      analog[3]->update(maxRange-analogRead(A3)); // inverted 
      break;

    case 2:
      // read Knob Y
      analog[6]->update(analogRead(A2));
      // Read CV 1
      analog[2]->update(maxRange-analogRead(A3)); // inverted 

      break;

    case 3:
      // read Switch
      analog[7]->update(analogRead(A2));
      // Read CV 2
      analog[3]->update(maxRange-analogRead(A3)); // inverted 

      break;
  }
}
