boolean check_of_MCU6050_GY87_PRINTLN = false;
#ifdef MCU6050_GY87_PRINTLN
check_of_MCU6050_GY87_PRINTLN = true;
#endif

#ifdef NEW_MCU6050_GET
#ifdef TRIAXIALSENSORTEST_INO
int step_count = 0;
#define accurate 2
#define diff 30
#define dly 1000
#define print_raw_data 1
#define InitialCalibration 1

MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;
double theta_x, theta_y, theta_z;
float Acc_angle_error_x, Acc_angle_error_y, Acc_angle_error_z;
float Acc_rawX, Acc_rawY, Acc_rawZ;          //Here we store the raw data read
float rad_to_deg = 180.0 / PI;               //This value is for pasing from radians to degrees values
int acc_error = 0;                           //We use this variable to only calculate once the Acc data error
float Acc_angle_x, Acc_angle_y, Acc_angle_z; //Here we store the angle value obtained with Acc data
int noDIV4096 = 0;
#endif
#ifdef GY_87_V8
QMC5883L compass;
MPU6050 accelgyro;
//The BMP180 Digital Barometer
SFE_BMP180 pressure;
#define ALTITUDE 1655.0

//all area
//QMC5883L
int16_t meg_x, meg_y, meg_z;
int16_t meg_no_value_tmp;
int meg_h;
//MPU6050
int16_t pose_ax, pose_ay, pose_az;
int16_t pose_gx, pose_gy, pose_gz;
//bmp180
char pres_status;
double pres_T, pres_P, pres_p0, pres_a;
unsigned long pres_last_time;
uint8_t pres_step = 0;
#endif
#endif
void imu_update()
{
#ifdef NEW_MCU6050_GET
#ifdef TRIAXIALSENSORTEST_INO
    tr_ino_loop();
    angleX = Acc_angle_x;
    angleY = Acc_angle_y;
    angleZ = Acc_angle_z;
#endif
#ifdef GY_87_V8
    V8_loop();
    angleX = pose_ax; //...(?)v
    angleY = pose_ay; //...(?)
    angleZ = pose_az; //...(?)
#endif
#else
    Wire.requestFrom(ADDR_SLAVE_I2C, PACKET_SIZE);
    byte data[PACKET_SIZE];
    int i = 0;
    while (Wire.available())
    {
        data[i] = Wire.read();
        i++;
    }

    // we use a c union to convert between byte[] and float
    union ROL_tag
    {
        byte b[4];
        float fval;
    } ROL_Union;
    union PIT_tag
    {
        byte b[4];
        float fval;
    } PIT_Union;
    union YAW_tag
    {
        byte b[4];
        float fval;
    } YAW_Union;

    ROL_Union.b[0] = data[0];
    ROL_Union.b[1] = data[1];
    ROL_Union.b[2] = data[2];
    ROL_Union.b[3] = data[3];
    if (isnan(ROL_Union.fval) != 1)
    {
        angleX = ROL_Union.fval;
    }

    PIT_Union.b[0] = data[4];
    PIT_Union.b[1] = data[5];
    PIT_Union.b[2] = data[6];
    PIT_Union.b[3] = data[7];
    if (isnan(PIT_Union.fval) != 1)
    {
        angleY = PIT_Union.fval;
    }

    YAW_Union.b[0] = data[8];
    YAW_Union.b[1] = data[9];
    YAW_Union.b[2] = data[10];
    YAW_Union.b[3] = data[11];
    if (isnan(YAW_Union.fval) != 1)
    {
        angleZ = YAW_Union.fval;
    }
#endif
}

void V8_init()
{
#ifdef NEW_MCU6050_GET

#ifdef GY_87_V8
    Wire.begin();
    accelgyro.setI2CMasterModeEnabled(false); //need 、 must set by pass or HMC5883L won't move
    accelgyro.setI2CBypassEnabled(true);
    accelgyro.setSleepEnabled(false);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("\n===============================\nGY87 checker!\n");

    // initialize device
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("Testing device connections...");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println(accelgyro.getI2CBypassEnabled() ? "MPU6050 I2C Bypass Enabled" : "ERROR!!! MPU6050 I2C Bypass DISabled !!!");

    delay(100); // Wait 5 seconds for next scan
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("*****i2c_scanner******");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("Scanning...");

    int nDevices = 0;
    for (byte address = 1; address < 127; ++address)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0)
        {
            if (check_of_MCU6050_GY87_PRINTLN)
                Serial.print("I2C device found at address 0x");
            if (address < 16)
            {
                if (check_of_MCU6050_GY87_PRINTLN)
                    Serial.print("0");
            }
            if (check_of_MCU6050_GY87_PRINTLN)
                Serial.print(address, HEX);
            if (check_of_MCU6050_GY87_PRINTLN)
                Serial.println("  !");

            ++nDevices;
        }
        else if (error == 4)
        {
            if (check_of_MCU6050_GY87_PRINTLN)
                Serial.print("Unknown error at address 0x");
            if (address < 16)
            {
                if (check_of_MCU6050_GY87_PRINTLN)
                    Serial.print("0");
            }
            if (check_of_MCU6050_GY87_PRINTLN)
                Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
    {
        if (check_of_MCU6050_GY87_PRINTLN)
            Serial.println("No I2C devices found\n");
    }
    else
    {
        if (check_of_MCU6050_GY87_PRINTLN)
            Serial.println("done\n");
    }

    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("total GOT");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(nDevices);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("Devices ; request 3");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print((nDevices == 3) ? "gy-87 powered up success" : "gy-87 powered up FAILED !!!");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println();
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("=========================================");
    delay(100); // Wait 5 seconds for next scan

    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("*****QMC5883L tset ********");
    // initialize device
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("Initializing I2C devices...");
    /*mag.initialize();
   if(check_of_MCU6050_GY87_PRINTLN)Serial.print("HMC5883L 's current address is : 0x");
   if(check_of_MCU6050_GY87_PRINTLN)Serial.print((char)((mag.get_address()/16<10)?mag.get_address()/16+(int)'0':mag.get_address()/16-10+(int)'A'));
   if(check_of_MCU6050_GY87_PRINTLN)Serial.println((char)((mag.get_address()%16<10)?mag.get_address()%16+(int)'0':mag.get_address()%16-10+(int)'A'));
   if(check_of_MCU6050_GY87_PRINTLN)Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");*/
    compass.init();
    int setup_temp_r;
    int16_t setup_temp_x, setup_temp_y, setup_temp_z, setup_temp_t;
    setup_temp_r = compass.readRaw(&setup_temp_x, &setup_temp_y, &setup_temp_z, &setup_temp_t);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("compass.x=");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println(setup_temp_x);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("compass.y=");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println(setup_temp_y);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("compass.z=");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println(setup_temp_z);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("compass.temp=");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println(setup_temp_t);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("compass.readRaw=");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println(setup_temp_r);
    if (compass.ready())
    {
        int heading = compass.readHeading();
        if (check_of_MCU6050_GY87_PRINTLN)
            Serial.println("compass.ready()");
        if (check_of_MCU6050_GY87_PRINTLN)
            Serial.print("heading=");
        if (check_of_MCU6050_GY87_PRINTLN)
            Serial.println(heading);
    }
    else
    {
        if (check_of_MCU6050_GY87_PRINTLN)
            Serial.println("compass IS NOT ready !!!");
    }
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("*****************************");

    delay(100);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("------------------------\nMPU6050 data get try ->");
    //MPU6050 data get try ->
    int16_t setup_temp_ax, setup_temp_ay, setup_temp_az;
    int16_t setup_temp_gx, setup_temp_gy, setup_temp_gz;
    accelgyro.getMotion6(&setup_temp_ax, &setup_temp_ay, &setup_temp_az, &setup_temp_gx, &setup_temp_gy, &setup_temp_gz);

    // display tab-separated accel/gyro x/y/z values
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("a/g:\n");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("ax:");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(setup_temp_ax);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("\n");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("ay:");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(setup_temp_ay);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("\n");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("az:");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(setup_temp_az);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("\n");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("gx:");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(setup_temp_gx);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("\n");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("gy:");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(setup_temp_gy);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("\n");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("gz:");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(setup_temp_gz);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("\n");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("------------------------");

    delay(100);
    //Initialise the BMP180 Barometer (and Temperature Sensor)
    // Initialize the sensor (it is important to get calibration values stored on the device).
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("BMP180 init and deta get...");
    if (pressure.begin())
        if (check_of_MCU6050_GY87_PRINTLN)
            Serial.println("BMP180 init success");
        else
        {
            // Oops, something went wrong, this is usually a connection problem,
            // see the comments at the top of this sketch for the proper connections.

            if (check_of_MCU6050_GY87_PRINTLN)
                Serial.println("BMP180 init fail\n\n");
            // Pause forever.
        }

    //get data...
    char setup_temp_status;
    double setup_temp_T, setup_temp_P, setup_temp_p0, setup_temp_a;

    // Loop here getting pressure readings every 10 seconds.

    // If you want sea-level-compensated pressure, as used in weather reports,
    // you will need to know the altitude at which your measurements are taken.
    // We're using a constant called ALTITUDE in this sketch:

    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println();
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("provided altitude: ");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(ALTITUDE, 0);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(" meters, ");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(ALTITUDE * 3.28084, 0);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println(" feet");

    // If you want to measure altitude, and not pressure, you will instead need
    // to provide a known baseline pressure. This is shown at the end of the sketch.

    // You must first get a temperature measurement to perform a pressure reading.

    // Start a temperature measurement:
    // If request is successful, the number of ms to wait is returned.
    // If request is unsuccessful, 0 is returned.

    setup_temp_status = pressure.startTemperature();
    if (setup_temp_status != 0)
    {
        // Wait for the measurement to complete:
        delay(setup_temp_status);

        // Retrieve the completed temperature measurement:
        // Note that the measurement is stored in the variable T.
        // Function returns 1 if successful, 0 if failure.

        setup_temp_status = pressure.getTemperature(setup_temp_T);
        if (setup_temp_status != 0)
        {
            // Print out the measurement:
            if (check_of_MCU6050_GY87_PRINTLN)
                Serial.print("temperature: ");
            if (check_of_MCU6050_GY87_PRINTLN)
                Serial.print(setup_temp_T, 2);
            if (check_of_MCU6050_GY87_PRINTLN)
                Serial.print(" deg C, ");
            if (check_of_MCU6050_GY87_PRINTLN)
                Serial.print((9.0 / 5.0) * setup_temp_T + 32.0, 2);
            if (check_of_MCU6050_GY87_PRINTLN)
                Serial.println(" deg F");

            // Start a pressure measurement:
            // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
            // If request is successful, the number of ms to wait is returned.
            // If request is unsuccessful, 0 is returned.

            setup_temp_status = pressure.startPressure(3);
            if (setup_temp_status != 0)
            {
                // Wait for the measurement to complete:
                delay(setup_temp_status);

                // Retrieve the completed pressure measurement:
                // Note that the measurement is stored in the variable P.
                // Note also that the function requires the previous temperature measurement (T).
                // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
                // Function returns 1 if successful, 0 if failure.

                setup_temp_status = pressure.getPressure(setup_temp_P, setup_temp_T);
                if (setup_temp_status != 0)
                {
                    // Print out the measurement:
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.print("absolute pressure: ");
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.print(setup_temp_P, 2);
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.print(" mb, ");
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.print(setup_temp_P * 0.0295333727, 2);
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.println(" inHg");

                    // The pressure sensor returns abolute pressure, which varies with altitude.
                    // To remove the effects of altitude, use the sealevel function and your current altitude.
                    // This number is commonly used in weather reports.
                    // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
                    // Result: p0 = sea-level compensated pressure in mb

                    setup_temp_p0 = pressure.sealevel(setup_temp_P, ALTITUDE); // we're at 1655 meters (Boulder, CO)
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.print("relative (sea-level) pressure: ");
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.print(setup_temp_p0, 2);
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.print(" mb, ");
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.print(setup_temp_p0 * 0.0295333727, 2);
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.println(" inHg");

                    // On the other hand, if you want to determine your altitude from the pressure reading,
                    // use the altitude function along with a baseline pressure (sea-level or other).
                    // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
                    // Result: a = altitude in m.

                    setup_temp_a = pressure.altitude(setup_temp_P, setup_temp_p0);
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.print("computed altitude: ");
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.print(setup_temp_a, 0);
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.print(" meters, ");
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.print(setup_temp_a * 3.28084, 0);
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.println(" feet");
                }
                else if (check_of_MCU6050_GY87_PRINTLN)
                    Serial.println("error retrieving pressure measurement\n");
            }
            else if (check_of_MCU6050_GY87_PRINTLN)
                Serial.println("error starting pressure measurement\n");
        }
        else if (check_of_MCU6050_GY87_PRINTLN)
            Serial.println("error retrieving temperature measurement\n");
    }
    else if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("error starting temperature measurement\n");

    pres_status = setup_temp_status;
    pres_T = setup_temp_T;
    pres_P = setup_temp_P;
    pres_p0 = setup_temp_p0;
    pres_a = setup_temp_a;
    pres_last_time = millis();

    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("=========================================\n\n\n");

#endif
#endif
}

void tr_ino_init()
{
#ifdef NEW_MCU6050_GET
#ifdef TRIAXIALSENSORTEST_INO
    Wire.begin();
    accelgyro.initialize();
    if (InitialCalibration == 1)
    {
        for (int a = 0; a < 200; a++)
        {

            Acc_rawX = ax_() / 4096.0;
            Acc_rawY = ay_() / 4096.0;
            Acc_rawZ = az_() / 4096.0;

            /*---X---*/
            Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2))) * rad_to_deg));
            /*---Y---*/
            Acc_angle_error_y = Acc_angle_error_y + ((atan(-1 * (Acc_rawX) / sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2))) * rad_to_deg));
            /*---Y---*/
            Acc_angle_error_z = Acc_angle_error_z + ((sqrt(pow((Acc_rawY), 2) + pow((Acc_rawX), 2)) / atan((Acc_rawZ))) * rad_to_deg);

            if (a == 199)
            {
                Acc_angle_error_x = Acc_angle_error_x / 200;
                Acc_angle_error_y = Acc_angle_error_y / 200;
                Acc_angle_error_z = Acc_angle_error_z / 200;
                acc_error = 1;
            }
        }
    }
#endif
#endif
}

void tr_ino_loop()
{
#ifdef NEW_MCU6050_GET
#ifdef TRIAXIALSENSORTEST_INO

    update_Acc();
#endif
#endif
}

void V8_loop()
{
#ifdef GY_87_V8
#ifdef TRIAXIALSENSORTEST_INO
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("^^^^^^");
    //QMC5883L
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println((compass.readRaw(&meg_x, &meg_y, &meg_z, &meg_no_value_tmp) == 1) ? "QMC5883L:" : "QMC5883L...ERROR!");
    if (compass.ready())
    {
        meg_h = compass.readHeading();
    }
    else
    {
        if (check_of_MCU6050_GY87_PRINTLN)
            Serial.println("compass ERROR !!!");
    }
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("compass.x = ");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(meg_x);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(" ; compass.y = ");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(meg_y);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(" ; compass.z = ");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(meg_z);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(" ; compass.heading = ");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println(meg_h);
    //MPU6050
    accelgyro.getMotion6(&pose_ax, &pose_ay, &pose_az, &pose_gx, &pose_gy, &pose_gz);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("MPU6050:\n");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("ax : ");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(pose_ax);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(" ; ay : ");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(pose_ay);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(" ; az : ");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(pose_az);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("\n");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("gx : ");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(pose_gx);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(" ; gy : ");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(pose_gy);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(" ; gz : ");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(pose_gz);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("\n");
    //bmp180
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println("bmp180:");
    switch (pres_step)
    {
    case 0:
        pres_status = pressure.startTemperature();
        if (pres_status != 0)
        {
            pres_step++;
            pres_last_time = millis();
            break;
        }
        else
        {
            if (check_of_MCU6050_GY87_PRINTLN)
                Serial.println("bmp180 : error starting temperature measurement");
            break;
        }
        delay(pres_status);
    case 1:
        if (abs(pres_last_time - millis()) > pres_status)
        {
            pres_status = pressure.getTemperature(pres_T);
            if (pres_status != 0)
            {

                pres_status = pressure.startPressure(3);
                if (pres_status != 0)
                {
                    pres_step++;
                    pres_last_time = millis();
                    break;
                }
                else
                {
                    if (check_of_MCU6050_GY87_PRINTLN)
                        Serial.println("bmp180 : error starting pressure measurement");
                    pres_step = 0;
                    break;
                }
            }
            else
            {
                if (check_of_MCU6050_GY87_PRINTLN)
                    Serial.println("bmp180 : error retrieving temperature measurement");
                pres_step = 0;
                break;
            }
        }
        else
        {
            //skip
            break;
        }
    case 2:
        if (abs(pres_last_time - millis()) > pres_status)
        {
            pres_status = pressure.getPressure(pres_P, pres_T);
            if (pres_status != 0)
            {

                if (check_of_MCU6050_GY87_PRINTLN)
                    Serial.print(" > ");
                pres_step = 0;
                break;
            }
            else
            {
                if (check_of_MCU6050_GY87_PRINTLN)
                    Serial.println("bmp180 : retrieving pressure measurement");
                pres_step = 0;
                break;
            }
        }
        else
        {
            //skip
            break;
        }

    default:
        break;
    }
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print("temperature : ");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(pres_T, 2);
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.print(" ; absolute pressure : ");
    if (check_of_MCU6050_GY87_PRINTLN)
        Serial.println(pres_P, 2);
#endif
#endif
}

int16_t ax_()
{
#ifdef NEW_MCU6050_GET
#ifdef TRIAXIALSENSORTEST_INO
    /** @brief 回傳x軸加速度

  * 需求:
  * 全域變數accelgyro必須存在
  * int16_t 指標 ax, ay, az, gx, gy, gz 皆須存在
  * 若 noDIV4096 == 1 傾印未除4096前的初始數據

  * @return 回傳x軸加速度(int16_t) */
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (noDIV4096 == 1)
    {
        if (check_of_MCU6050_GY87_PRINTLN)
            Serial.println("ax without any calculation: " + String(ax));
    }
    return ax;
#endif
#endif
}

int16_t ay_()
{
#ifdef NEW_MCU6050_GET
#ifdef TRIAXIALSENSORTEST_INO
    /** @brief 回傳y軸加速度

  * 需求:
  * 全域變數accelgyro必須存在
  * int16_t 指標 ax, ay, az, gx, gy, gz 皆須存在
  * 若 noDIV4096 == 1 傾印未除4096前的初始數據

  * @return 回傳y軸加速度(int16_t) */
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (noDIV4096 == 1)
    {
        if (check_of_MCU6050_GY87_PRINTLN)
            Serial.println("ay without any calculation: " + String(ay));
    }
    return ay;
#endif
#endif
}

int16_t az_()
{
#ifdef NEW_MCU6050_GET
#ifdef TRIAXIALSENSORTEST_INO
    /** @brief 回傳z軸加速度

  * 需求:
  * 全域變數accelgyro必須存在
  * int16_t 指標 ax, ay, az, gx, gy, gz 皆須存在
  * 若 noDIV4096 == 1 傾印未除4096前的初始數據

  * @return 回傳z軸加速度(int16_t) */
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (noDIV4096 == 1)
    {
        if (check_of_MCU6050_GY87_PRINTLN)
            Serial.println("az without any calculation: " + String(az));
    }
    return az;
#endif
#endif
}

void update_Acc()
{
#ifdef NEW_MCU6050_GET
#ifdef TRIAXIALSENSORTEST_INO
    Acc_rawX = ax_() / 4096.0;
    Acc_rawY = ay_() / 4096.0;
    Acc_rawZ = az_() / 4096.0;

    //---X---
    Acc_angle_x = (atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2))) * rad_to_deg) - Acc_angle_error_x;
    //---Y---
    Acc_angle_y = (atan(-1 * (Acc_rawX) / sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2))) * rad_to_deg) - Acc_angle_error_y;
    //---Z---
    Acc_angle_z = ((sqrt(pow((Acc_rawY), 2) + pow((Acc_rawX), 2)) / atan((Acc_rawZ))) * rad_to_deg) - Acc_angle_error_z;

    if (print_raw_data)
    {
        if (check_of_MCU6050_GY87_PRINTLN)
            Serial.println("rawX:" + String(Acc_rawX) + "  rawY:" + String(Acc_rawY) + "  rawZ:" + String(Acc_rawZ) + "  angX:" + String(Acc_angle_x) + "  angY:" + String(Acc_angle_y) + "  angZ:" + String(Acc_angle_z));
    }
#endif
#endif
}
