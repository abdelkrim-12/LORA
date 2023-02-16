        currentMillis4 = millis();
        if ((check4 == 0) && (previousMillis4 != 0) && ((currentMillis4 - previousMillis4) >= interval))
        {
          sendDataToGW(totalMilliLitres4, total_duration4);
          previousMillis4 = 0;
          totalMilliLitres4 = 0;
          total_duration4 = 0;
          esp_deep_sleep_start();
        }


  
  /*//char str[10];
  std::stringstream sstream1;
  std::stringstream sstream2;
  std::stringstream sstream3;

  sstream1 << val1;
  sstream2 << val2;
  sstream3 << id;
  string str_val1 = sstream1.str();
  string str_val2 = sstream2.str();
  string str_val3 = sstream3.str();

  int size = str_val1.length() + str_val2.length() + str_val3.length();
  string str = str_val1 + '/' + str_val2 + '/' + str_val3;

  char str2[size+1];

  strcpy(str2, str.c_str());

  Serial.print("str2= ");
  Serial.print(str2);*/        