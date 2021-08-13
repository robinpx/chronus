void toPython() {
  if (currentMillis - prevSendMillis >= sendInterval) {
    String buf;
    buf += String(myX, 1);
    buf += F(";");
    buf += String(myY, 1);

    // Send buffer of Roomba's coordinates to Python
    Serial3.println(buf);
    Serial3.flush();
          
    // Update send timer
    prevSendMillis += sendInterval;
  }
}

void getClearestSNR(int &satNum, int &satSNR) { // pass addresses
  for (int i=0; i<4; ++i) {
    int no = atoi(satNumber[i].value());
    if (no >= 1 && no <= MAX_SATELLITES) {
          sats[no-1].snr = atoi(snr[i].value());
          sats[no-1].active = true;
    }
  }
      
  int totalMessages = atoi(totalGPGSVMessages.value());
  int currentMessage = atoi(messageNumber.value());
  //int inView = atoi(satsInView.value());

  // initalize as empty
  satNum=0;
  satSNR=0;
  int maximum = 0;
  
  if (totalMessages == currentMessage) {
    //Serial.print(F("Sats in View=")); Serial.println(atoi(satsInView.value())); // for debugging
    for (int i=0; i<MAX_SATELLITES; ++i) {
        if (sats[i].active) {
           //Serial.print(F("Nums="));
           //Serial.print(i+1);
           //Serial.print(F(" SNR = "));
           //Serial.print(sats[i].snr);
           //Serial.println();
           // if (sats[i].snr >= 30 && maximum < sats[i].snr)
           if (sats[i].snr >= 30 && maximum < sats[i].snr) { // get clearest snr
             satNum=i+1;
             satSNR=sats[i].snr;
             maximum=satSNR;
           }
        }
    }

    for (int i=0; i<MAX_SATELLITES; ++i) {
       sats[i].active = false;
    }
  }
}
