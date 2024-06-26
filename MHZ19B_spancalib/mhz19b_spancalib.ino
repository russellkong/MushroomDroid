/*----------------------------------------------------------
    MH-Z19 CO2 sensor  Span calibration SAMPLE
  ----------------------------------------------------------*/

#include <MHZ19B_uart.h>

const int rx_pin = 2;  //Serial rx pin no
const int tx_pin = 3;  //Serial tx pin no

const int waitingMinutes = 1;  //waiting 30 minutes

MHZ19B_uart mhz19;

/*----------------------------------------------------------
    MH-Z19 CO2 sensor  setup
  ----------------------------------------------------------*/
void setup() {
  Serial.begin(9600);
  mhz19.begin(rx_pin, tx_pin);

  mhz19.setAutoCalibration(false);
  Serial.println();
}

/*----------------------------------------------------------
    MH-Z19 CO2 sensor  loop
  ----------------------------------------------------------*/
long cnt = 0;
long avg_sum = 0L;
const long waitingSeconds = waitingMinutes * 60L;
void loop() {
  if ( ++cnt % 60 == 0) {
    int ppm = mhz19.getPPM();
    avg_sum += (long)ppm;
    Serial.print(cnt / 60); Serial.println("min.");
    Serial.print("co2: "); Serial.print(ppm); Serial.println("ppm now.");
  } else {
    Serial.print(".");
  }
  delay(1000);

  if (cnt > waitingSeconds) {
    long avg = avg_sum / waitingMinutes;
    if (avg < 1000) {
      Serial.print("CO2 Avg. :");Serial.println(avg);
      Serial.println("Can't Span point calibration because CO2 average is lower than 1000ppm");
      cnt = 0;
      avg_sum = 0L;
      return;
    }

    Serial.println("");
    mhz19.calibrateSpan(avg);
    Serial.println("span calibration now .");

    for ( int i = 0; i < 10; i++) {
      Serial.print("co2: "); Serial.print(mhz19.getPPM()); Serial.println("ppm now.");
      delay(10000);
    }
    cnt = 0;
    avg_sum = 0L;
  }

}
