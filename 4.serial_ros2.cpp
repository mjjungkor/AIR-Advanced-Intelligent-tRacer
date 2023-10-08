const int SW=13;
const int LED=12;
int nlastSwitchState=0;

void setup()
{
    Serial.begin(115200);
    pinMode(SW,INPUT);//INPUT_PULLUP
    pinMode(LED,OUTPUT);
    //Serial.print("Initialized\n");
}

void loop()
{
    int nReadSwitchSignal=digitalRead(SW);

    if (nlastSwitchState != nReadSwitchSignal) // 스위치 상태 변화 감지(Toggle)
    {
        nlastSwitchState=nReadSwitchSignal;
        if (nReadSwitchSignal == 1) // 스위치가 'On'인 경우
            Serial.println("ON");
    }
}

void serialEvent(void)
{
    /*char data;
    data=Serial.read();
    if (data == 'ON') {
        bFlag=true;
    }*/
    String receivedMessage = Serial.readStringUntil('\n');
    if (receivedMessage == 'OK') {
        for(int i=0;i<3; i++)
        {
            digitalWrite(LED, HIGH);
            delay(300);//300ms
            digitalWrite(LED, LOW);
            delay(300);//300ms
        }
    }
}