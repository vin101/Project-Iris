#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <NewPing.h>
#include <Servo.h>

#define TRIGGER_PIN  12    //digital
#define ECHO_PIN     11    //~pwm
#define MAX_DISTANCE 400

#define SERVO_PIN    10    //~pwm

Servo servo0;
int thresh=20;

int override=0;
int headingerror=5;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int pos0; 
int st_time=0;

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

float declinationAngle = 0.025;
sensors_event_t event;

int lfip=9,rfip=6,lbip=5,rbip=3;


void setup(void)
{
    Serial.begin(9600);

    servo0.detach();
    servo0.attach(SERVO_PIN);
    Serial.begin(9600);

    /* Initialise the sensor */
    if(!mag.begin())
    {
        /* There was a problem detecting the HMC5883 ... check your connections */
        Serial.println("HMC5883 not detected");
        while(1);
    }


    servo0.write(90);
    delay(1000);
}

int getObDist(int pos0)
{
    servo0.write(pos0);
    delay(50);
    int ObDist = sonar.ping_cm();

    return ObDist;
}

void ArSweep (int &maxD, int &posA)
{
    maxD=0; //near range
    int ObDist;
    servo0.write(0);
    delay(500);
    for(pos0 = 0; pos0 <= 180; pos0+=10)
    {
        ObDist = getObDist(pos0);
        if( maxD < ObDist )
        {
            maxD = ObDist;
            posA = pos0;
        }
        delay(1);
    }  
}

int getHeading()
{
    mag.getEvent(&event);
    float heading = atan2(event.magnetic.y, event.magnetic.x);
    heading += declinationAngle;
    if(heading < 0)
        heading += 2*PI;    
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
        heading -= 2*PI;   
    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/M_PI;
    return headingDegrees;
}

void halt()
{
    digitalWrite(lfip,LOW);
    digitalWrite(rfip,LOW);
    digitalWrite(lbip,LOW);
    digitalWrite(rbip,LOW);
}

void forward(int dist=0)
{
    digitalWrite(lfip,LOW);
    analogWrite(lbip,120);
    digitalWrite(rfip,LOW);
    analogWrite(rbip,130);
}

void backward(int dist=0)
{
    analogWrite(lfip,130);
    digitalWrite(lbip,LOW);
    analogWrite(rfip,130);
    digitalWrite(rbip,LOW);
}

void left_inf()
{
    analogWrite(lfip,90);
    digitalWrite(lbip,LOW);
    digitalWrite(rfip,LOW);
    analogWrite(rbip,90);
}

void right_inf()
{
    digitalWrite(lfip,LOW);
    analogWrite(lbip,90);
    analogWrite(rfip,90);
    digitalWrite(rbip,LOW);
}

int trueheading(int heading)
{
    if(heading>360)
        heading=heading-360;
    else if(heading<0)
        heading=heading+360;

    return heading;
}

int whereis(int curheading, int finalheading)
{
    int point180=trueheading(finalheading+180);
    int dir=0;

    if(finalheading>0 && finalheading<=180)
    {
        if(curheading>finalheading && curheading<point180)
            dir=1;
        else if((curheading>0 && curheading<finalheading) || (curheading>point180 && curheading<=360))
            dir=-1;
    }
    else if(finalheading>180 && finalheading<=360)
    {
        if((curheading>0 && curheading<point180) || (curheading>finalheading && curheading<=360))
            dir=1;
        else if(curheading>point180 && curheading<finalheading)
            dir=-1;
    }

    return dir;
}

int isheading(int curheading ,int finalheading ,int error)
{
    int ulim=trueheading(finalheading+error);
    int llim=trueheading(finalheading-error);

    if(whereis(curheading,ulim)==-1 && whereis(curheading,llim)==1)
        return 1;
    else
        return 0;
}

void left(int deg)
{
    int finalHeading=getHeading()-deg;  
    finalHeading= trueheading(finalHeading);
    analogWrite(lfip,90);
    digitalWrite(lbip,LOW);
    digitalWrite(rfip,LOW);
    analogWrite(rbip,90);  
    while(!isheading(getHeading(),finalHeading,headingerror));
    halt();

}

void right(int deg)
{
    int finalHeading=getHeading()+deg;
    finalHeading= trueheading(finalHeading);
    digitalWrite(lfip,LOW);
    analogWrite(lbip,90);
    analogWrite(rfip,90);
    digitalWrite(rbip,LOW);
    while(!isheading(getHeading(),finalHeading,headingerror));
    halt();
}

void loop()
{
    right(90);
    delay(500);
    /*
    servo0.write(90);
    int fdist=sonar.ping_cm();

    if(fdist>=thresh)
        forward();

    while(sonar.ping_cm()>thresh)
    {
        if(Serial.available())
        {
            char cmd=Serial.read();
            if(cmd=='t')
                manualoverride();
        }
    } 
    halt();
    int maxd=0;
    int posA;
    ArSweep(maxd, posA);
    delay(500);
    servo0.write(90);

    if(posA>90)
        left(posA-90);
    else if(posA<90)
        right(90-posA);
    /*
    
    /*Serial.println(getObDist(90));
    delay(500);*/
}

void manualoverride()
{
    char cmd;
    halt();

    do
    {
        cmd='-';
        
        if(Serial.available())
        {
            cmd=Serial.read();

            switch(cmd)
            {
            case 'f': 
                forward(); 
                break;
            case 'b': 
                backward(); 
                break;
            case 'l': 
                left_inf(); 
                break;
            case 'r': 
                right_inf(); 
                break;
            case 'i': 
                halt();
                break;
            case 't':
                return;
            }
        }
    }
    while(cmd!='t');
}

