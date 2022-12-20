#define ITERATIONS 500

class IkSensorDist
{
  float last_depend = 0.0;
  
public:
  IkSensorDist()
  {
    pinMode(A0, INPUT);
  }

  float get_distance(float K = 12.298)
  {
    float d_cm = 0.0;

    int16_t analog_value  = analogRead(A0);
    last_depend = analog_value * 0.0048828125;
    last_depend = pow(last_depend, -1.10);
    d_cm = K * last_depend;

    return d_cm;
 }

 float calibrate_K(const float ideal_distance)
 {
    float k = ideal_distance / this->last_depend;
    return k;
 }
};

class UltraSensorDist
{
  uint8_t trigPin;
  uint8_t echoPin;
  
public:
  UltraSensorDist(uint8_t _trigPin = 12, uint8_t _echoPin = 11)
  {
    this->trigPin = _trigPin;
    this->echoPin = _echoPin;
    
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
  }

  float get_distance()
  {
    long duration;
    float d_cm;

    digitalWrite(this->trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(this->trigPin, HIGH);

    delayMicroseconds(10);
    digitalWrite(this->trigPin, LOW);

    duration = pulseIn(this->echoPin, HIGH);

    d_cm = (duration / 2) / 29.1;
    
    return d_cm;
  }
};

UltraSensorDist u_sensor;
IkSensorDist ik_sensor;

float K = 1.0;
  
void setup()
{
  Serial.begin (9600);  

  Serial.println("Calibrate start!");

  float dist_u = u_sensor.get_distance();
  float dist_ik = ik_sensor.get_distance(K);
  int i = 0;
  bool is_first_it = true;

  while (i < ITERATIONS)
  {
    do
    {
      dist_u = u_sensor.get_distance();
      dist_ik = ik_sensor.get_distance(K);
    } while (dist_u > 80);
    
    Serial.print("dist_u = ");
    Serial.println(dist_u);
    Serial.print("dist_ik = ");
    Serial.println(dist_ik);
    Serial.print("K = ");
    
    if (is_first_it)
    {
      K = ik_sensor.calibrate_K(dist_u);
      is_first_it = false;
    }
    else
    {
      K = (K + ik_sensor.calibrate_K(dist_u)) / 2.0;
    }
    
    Serial.println(K);
  
    delay(50);

    ++i;
  }

  Serial.println("RangeCalibration Done!");

  delay(500);
}

void loop()
{
  float cm_1 = u_sensor.get_distance();
  Serial.print("Ultra Sensor Readings: ");
  Serial.print(cm_1);
  Serial.println(" см.");
  
  float cm_2 = ik_sensor.get_distance();
  Serial.print("IK Sensor Readings:  ");
  Serial.print(cm_2);
  Serial.println(" cm.");
  
  delay(750);
}
