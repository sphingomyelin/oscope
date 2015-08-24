
String COM_PORT = "/dev/ttyUSB0";

/*
 * Oscilloscope
 * Gives a visual rendering of analog pin in realtime.
 *
 * ---------------- IMPROVEMENTS ------------------
 * Updates by Jan Hermann (jan.hermann.91@gmail.com), 8/3/2015
 * Added functionality for higher sampling frequency
 * Changed usability to something closer to a real oscilloscope
 * Added basic measurements like min, max and RMS value
 * Added a help screen
 * Some improvements in speed (circular buffer)
 *
 * ---------------- IMPROVEMENTS ------------------
 * Updates by John Porter, 2/7/2014
 * Added ability to move waveform left or right.
 * Added gridlines (bounds and minor).
 * Added ability to pause/resume.
 * Added ability to measure time.
 * General usability improvements.
 *
 * --------------- ORIGINAL PROJECT ---------------
 * This project is part of Accrochages
 * See http://accrochages.drone.ws
 * (c) 2008 Sofian Audry (info@sofianaudry.com)
 * ------------------------------------------------
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

// * ------------------ HOT KEYS ------------------
final char T_UP       = 'w'; // Translate waveform up
final char T_DOWN     = 's'; //                    down
final char T_LEFT     = 'a'; //                    left
final char T_RIGHT    = 'd'; //                    right
final char Z_IN       = 'c'; // Horizontal zoom in
final char Z_OUT      = 'z'; //                 out
final char S_IN       = 'e'; // Vertical scale in
final char S_OUT      = 'q'; //                out
final char MGL_UP     = 'r'; // Minor voltage grid lines increase
final char MGL_DOWN   = 'f'; //                          decrease
final char MGL_T_UP   = 't'; // Minor time grid lines increase
final char MGL_T_DOWN = 'g'; //                       decrease
final char TOG_PAUSE  = 'p'; // Toggle pause (unpause resets waveform)
final char TOG_HELP   = 'h'; // Toggle help screen
final char RESET_AXIS = ' '; // Reset axis settings
final char MEAS_TIME  = 'x'; // Adds and/or highlights vertical bars (time measurement)
final char BAR_LEFT   = ','; // Move highlighted vertical bar left (can also mouse click)
final char BAR_RIGHT  = '.'; //                               right
// * ----------------------------------------------

// * --------------- STARTING STATE ---------------
float volt_div_ar[]  = {1.0, 0.5, 0.2, 0.1, 0.05, 0.02, 0.01};
int volt_div_idx     = 0;
float time_div_ar[] = {2.0, 1.0, 0.5, 0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001};
int time_div_idx    = 1;
float centerV      = 0;
float centerH      = 0;
int gridLines    = 5;
int gridLinesX   = 9;
int com_port     = 1;   // Index number in Serial.list
// * ----------------------------------------------

// Small data structure
class DataPair {
  int value;
  int utime;
  DataPair() {
    this.value=0;
    this.utime=0;
  }
  DataPair(int value, int utime) {
    this.value=value;
    this.utime=utime;
  }
}

// Global vars
import processing.serial.*;
Serial port;                    // Create object from Serial class
DataPair val;                   // Time data was received
DataPair[] data;
int data_index = 0;
float voltage;
float measTime = 0;
int   timeMode = 0;
int[] timeBars = {0, 0};
PFont f;
boolean pause;
boolean help_screen;
int nr_data = 250000;
float margin_percent = 0.1;

// Setup
void setup() {
  // Print out com ports
  println("Modify com. port according to your needs at the top of the file");
  println("Possible com. ports are:");
  for(int i=0; i < Serial.list().length; i++) {
    println(Serial.list()[i]);
  }
  
  // Window
  size(1280, 480);
  frameRate(30);
  smooth();
  f = createFont("Monospaced.plain", 16, true);
  
  // Serial communication
  port = new Serial(this, COM_PORT,  2000000);    // Com port specified here
  
  // Array for data
  data = new DataPair[nr_data];
  for (int i=0; i<nr_data; i++){
    data[i] = new DataPair();
  }
  
  // Initialization
  timeBars[0] = width/3;
  timeBars[1] = 2*width/3;
  pause = false;
}

int mod(int a, int b) {
  return (a % b + b) % b;
}

String pad(String string, int length) {
  for (int i=0; i<(length-string.length()); i++) {
    string = " " + string;
  }
  return string;
}

// Read value from serial stream
DataPair getValue() {
  int value = -1;
  int utime = -1;
  boolean found_value = false;
  while (port.available () >= 8 && found_value == false) {
    if (port.read() == 0xff) {
      value = (port.read() << 8) | (port.read());
      utime = (port.read() << 24) | (port.read() << 16) | (port.read() << 8) | (port.read());
//      println("Voltage: " + value * 5.0/1023.0 + "V, Time: " + utime + "us, data_index: " + data_index);
     //if (port.read() == 0x00) {
       found_value = true;
     //}
     //else
       //println("Error: Transmission error");
    }
  }
  DataPair pair = new DataPair(value, utime);
  return pair;
}

float getSampFreq() {
  if (data[mod(data_index-5, nr_data)].utime > 0 && data[mod(data_index, nr_data)].utime > 0) {
    return 5000000.0/(data[mod(data_index, nr_data)].utime - data[mod(data_index-5, nr_data)].utime);
  } else {
    return -1.0;
  }
}

float getVoltFromInt(int val) {
  return val * 5.0f/1023.0f;
}

// Get a x-value from the time and time div settings
int getX(int utime) {
  return (int)(width - width * ((data[data_index].utime - utime)/1000000.0 - centerH) / ((gridLinesX+1) * time_div_ar[time_div_idx]));
}

float getMicroTimeFromX(int pixel) {
  return (-(width-pixel) / (float)(width/(gridLinesX+1)) * time_div_ar[time_div_idx]*1000000.0f);
}

// Get a y-value for the datapoint, varies based on axis settings
int getY(int val) {
  return (int)(height*(1-margin_percent) - (getVoltFromInt(val)+centerV) / volt_div_ar[volt_div_idx] * height*(1-2*margin_percent)/(gridLines+1));
}

float getVoltsFromY(int pixel) {
  return (-(height*(1-margin_percent)-pixel) / (height/(gridLines+1)) * volt_div_ar[volt_div_idx]);
}

// Push the values in the data array
void pushDataPair(DataPair pair) {
  data_index = mod(data_index+1, nr_data);
  data[mod(data_index, nr_data)] = pair;
}

// Draw waveform
void drawLines() {
  int x0 = getX(data[data_index].utime), x1 = x0;
  int y0 = getY(data[data_index].value), y1 = y0;
  stroke(255,255,0);
  for (int i=1; i<nr_data; i++) {
    x1 = getX(data[mod(-i+data_index, nr_data)].utime);
    y1 = getY(data[mod(-i+data_index, nr_data)].value);
    if (x0 < 0) break;
    if (x1 > width) continue;
    if (!(x1 < x0 || (x1 == x0 && y1 != y0))) continue; //(x1 == x0) continue; // (x1 < x0 || (x1 == x0 && y1 != y0))
    if (data[mod(-i+data_index, nr_data)].utime > 0)
      line(x0, y0, x1, y1);
    x0 = x1;
    y0 = y1;
  }
}

// Draw gridlines (bounds, minor)
void drawGrid() {
  // Draw window bounds
  stroke(255, 0, 0);
  line(0, height*margin_percent, width, height*margin_percent);
  line(0, height*(1-margin_percent), width, height*(1-margin_percent));
  
  // Get scaled values for bounds
  //int pFive = getY(1023);
  int zero  = getY(0);

  // Draw voltage bounds
  stroke(255, 0, 0);
//  line(0, pFive-1, width, pFive-1);
  line(0, zero+1, width, zero+1);

  // Add voltage bound text
  textFont(f, 10);
  fill(255, 0, 0);
//  text("+5V", 5, pFive+12);
  text(" 0V", 5, zero-4);

  // Draw minor grid lines - horizontal
  int gridVal = 0;
  stroke(75, 75, 75);
  for (int i = 0; i < gridLines; i++) {
    if(i == (gridLines+1)/2-1 && ((gridLines+1) % 2) == 0) stroke(150, 75, 75);
    else stroke(75, 75, 75);
    gridVal = round((i+1.0)*(height*(1-2*margin_percent) / (gridLines+1.0)) + height*margin_percent);
    line(0, gridVal, width, gridVal);
//    text("" + truncate(i * volt_div_ar[volt_div_idx], 2), 5, gridVal-4);
  }

  // Add minor grid line text
  if (gridLines > 0) {
    textFont(f, 16);
    fill(204, 102, 0);
    text("Div Voltage: " + truncate(volt_div_ar[volt_div_idx], 2) + "V", 1080, height-12);
  }
  
  // Draw minor grid lines - vertical
  gridVal = 0;
  stroke(75, 75, 75);
  for (int i = 0; i < gridLinesX; i++) {
    if(i == (gridLinesX+1)/2-1 && ((gridLinesX+1) % 2) == 0) stroke(150, 75, 75);
    else stroke(75, 75, 75);
    gridVal = round((i+1.0)*(width / (gridLinesX+1.0)));
    line(gridVal, height*margin_percent, gridVal, height*(1-margin_percent));
  }

  // Add minor grid line text
  if (gridLinesX > 0) {
    textFont(f, 16);
    fill(204, 102, 0);
    
    text("Div Time:    " + truncate(time_div_ar[time_div_idx]*1000.0, 0) + "ms", 535, height-12);
  }
  
  // Print difference between vertical 'time' bars
  if (timeMode > 0) {
    textFont(f, 16);
    fill(204, 102, 0);
    
    float timeDiff = truncate((getMicroTimeFromX(timeBars[1]) - getMicroTimeFromX(timeBars[0]))/1000.0, 2);
    text("Time: " + timeDiff + "ms", 30, height-12);
  }
}

// Draw vertical 'time bars' (seperate from above for better layering)
void drawVertLines() {
  stroke(200, 200, 200);
  if (timeMode == 1) {
    line(timeBars[1], height*margin_percent, timeBars[1], height*(1-margin_percent));
    stroke(50, 50, 255);
    line(timeBars[0], height*margin_percent, timeBars[0], height*(1-margin_percent));
  }
  else if (timeMode == 2) {
    line(timeBars[0], height*margin_percent, timeBars[0], height*(1-margin_percent));
    stroke(50, 255, 50);
    line(timeBars[1], height*margin_percent, timeBars[1], height*(1-margin_percent));
  }
}

void drawCurrent() {
  if (!pause) {
    // Print current voltage reading
    textFont(f, 16);
    fill(204, 102, 0);
    voltage = truncate(getVoltFromInt(data[data_index].value), 2);
    text("Voltage:   " + pad(nf(voltage, 0, 2) + "V", 5), 1080, 20);
    
    // Print current sampling frequency
    float fs = getSampFreq();
    if (fs > 0)
      text("Samp Freq: " + pad((int)getSampFreq() + "Hz", 6), 1080, 40);
    else
      text("Samp Freq: " + pad("---Hz", 6), 1080, 40);
  } else {
    textFont(f, 16);
    fill(204, 102, 0);
    text("--- PAUSED ---", 1080, 20);
  }
}

void drawStats() {
  // Print RMS voltage of whole window
  float rms_voltage = 0.0f;
  float mean_voltage = 0.0f;
  float max_voltage = getVoltFromInt(0);
  float min_voltage = getVoltFromInt(1024);
  DataPair current;
  float current_volt = 0.0f;
  int n = 0;
  for (int i=0; i<nr_data; i++) {
    current = data[mod(-i+data_index, nr_data)];
    current_volt = getVoltFromInt(current.value);
    if (getX(current.utime) < width) {
      if (getX(current.utime) < 0 || current.utime <= 0) {
        break;
      } else {
        rms_voltage += current_volt*current_volt;
        mean_voltage += current_volt;
        max_voltage = max(max_voltage, current_volt);
        min_voltage = min(min_voltage, current_volt);
        n++;
      }
    } else {
      continue;
    }
  }
  
  rms_voltage /= n;
  rms_voltage = sqrt(rms_voltage);
  mean_voltage /= n;
  
  text("RMS voltage:  " + pad(nfs(truncate(rms_voltage, 2), 0, 2) + "V", 6), 800, 20);
  text("mean voltage: " + pad(nfs(truncate(rms_voltage, 2), 0, 2) + "V", 6), 800, 40);
  text("max. voltage: " + pad(nfs(truncate(max_voltage, 2), 0, 2) + "V", 6), 520, 20);
  text("min. voltage: " + pad(nfs(truncate(min_voltage, 2), 0, 2) + "V", 6), 520, 40);
  
  text("GND shifted by:  " + nf(truncate(centerV, 2), 0, 2) + "V", 200, 20);
  text("Time shifted by: " + nf(truncate(centerH*1000, 2), 0, 2) + "ms", 200, 40);
  
  text("Toggle HELP", 10, 20);
  text("   with <h>", 10, 40);
}

void drawHelp() {
  fill(204, 102, 0);
  rectMode(RADIUS);
  rect(width/2, height/2, 320, 155, 7);
  
  textFont(f, 16);
  fill(0, 0, 0);
  int xpos = 330, ypos = 95;
  int spacing = 20;
  
  text("Help Screen",                                                      xpos, ypos + 1 * spacing);
  text("----------------------------------------------------------------", xpos, ypos + 2 * spacing);
  text("Waveform translation:              < w : UP    -----  DOWN : s >", xpos, ypos + 3 * spacing);
  text("Waveform translation:              < a : LEFT  ----- RIGHT : d >", xpos, ypos + 4 * spacing);
  text("Zoom horizontal (voltage):         < c : OUT   -----    IN : z >", xpos, ypos + 5 * spacing);
  text("Zoom vertical   (time):            < e : OUT   -----    IN : q >", xpos, ypos + 6 * spacing);
  text("Reset Display:                                 <   > (Space)    ", xpos, ypos + 7 * spacing);
  
  text("Toggle PAUSE:                                  < p >            ", xpos, ypos + 8 * spacing);
  text("Toggle HELP:                                   < h >            ", xpos, ypos + 9 * spacing);
  
  text("Toggle TIME MEAS. / Bar Selection:             < x >            ", xpos, ypos + 10 * spacing);
  text("Move time bars:                    < , : LEFT  ----- RIGHT : . >", xpos, ypos + 11 * spacing);
  text(" OR Move time bars with the mouse cursor"                        , xpos, ypos + 12 * spacing);
  
  text("In-/decrease Grid Lines (voltage): < r : UP    -----  DOWN : f >", xpos, ypos + 13 * spacing);
  text("In-/decrease Grid Lines (time):    < t : UP    -----  DOWN : g >", xpos, ypos + 14 * spacing);
}

// Truncate a floating point number
float truncate(float x, int digits) {
  float temp = pow(10.0, digits);
  return round( x * temp ) / temp;
}

// When a key is pressed down or held...
void keyPressed() {
  switch (key) {
  case T_UP: centerV += 0.05*volt_div_ar[volt_div_idx]; break;   // Move waveform up
  case T_DOWN: centerV -= 0.05*volt_div_ar[volt_div_idx]; break; // Move waveform down
  case T_RIGHT: if (pause) centerH += 0.05*time_div_ar[time_div_idx]; break;// Move waveform right
  case T_LEFT: if (pause) centerH -= 0.05*time_div_ar[time_div_idx]; break; // Move waveform left
  case MGL_UP:                                               // Increase minor grid lines
    if (gridLines < 49)
      gridLines += 2;
    break;
  case MGL_DOWN:                                             // Decrease minor grid lines
    if (gridLines > 1)
      gridLines -= 2;
    break;
  case MGL_T_UP:                                             // Increase minor grid lines
    if (gridLinesX < 49)
      gridLinesX += 2;
    break;
  case MGL_T_DOWN:                                           // Decrease minor grid lines
    if (gridLinesX > 1)
      gridLinesX -= 2;
    break;
  case BAR_LEFT:                                             // Move the time bar left (also mouse click)
    if (timeMode == 1 && timeBars[0] > 0)
      timeBars[0] -= 1;
    else if (timeMode == 2 && timeBars[1] > 0)
      timeBars[1] -= 1; 
    break;
  case BAR_RIGHT:                                            // Move the time bar right (also mouse click)
    if (timeMode == 1 && timeBars[0] < width-1)
      timeBars[0] += 1;
    else if (timeMode == 2 && timeBars[1] < width-1)
      timeBars[1] += 1; 
    break;
  }
}

// When a key is released...
void keyReleased() {
  //println(key+": "+(int)key);
  switch (key) {
  case Z_IN:                                                 // Zoom in: Volts
    if (volt_div_idx < volt_div_ar.length-1) {
      volt_div_idx++;
    }
    break;
  case Z_OUT:                                                // Zoom out: Volts
    if (volt_div_idx >= 1) {
      volt_div_idx--;
    }
    break;
  case S_IN:                                                 // Zoom in: Time
    if (time_div_idx < time_div_ar.length-1) {
      time_div_idx++;
      if (pause) centerH += (time_div_ar[time_div_idx-1] - time_div_ar[time_div_idx]) * (gridLinesX+1)/2;
      else centerH = 0;
    }
    break;
  case S_OUT:                                                // Zoom out: Time
    if (time_div_idx >= 1) {
      if (pause) centerH -= (time_div_ar[time_div_idx-1] - time_div_ar[time_div_idx]) * (gridLinesX+1)/2;
      else centerH = 0;
      time_div_idx--;
    }
    break;
  case RESET_AXIS:                                           // Reset all scaling
    centerV = 0; centerH = 0;
    volt_div_idx = 0; time_div_idx = 1;
    gridLines = 5; gridLinesX = 9;
    break;
  case MEAS_TIME: timeMode = (timeMode + 1) % 3; break;      // Change the vertical bars (off, left bar, right bar)
  case TOG_PAUSE:                                            // Toggle waveform pausing
    if (pause) {
      centerH = 0;
      for (int i=0; i<nr_data; i++){
        data[i].value = 0;                                   // Clear data on resume
        data[i].utime = 0;
      }
    }
    pause = !pause;
    break;
  case TOG_HELP:
    help_screen = !help_screen;
  }
}

// Use mouse clicks to quickly move vertical bars (if highlighted)
void mousePressed() {
  if(timeMode == 1)
    timeBars[0] = mouseX;
  else if(timeMode == 2)
    timeBars[1] = mouseX;
}

void mouseDragged() {
  if(timeMode == 1)
    timeBars[0] = mouseX;
  else if(timeMode == 2)
    timeBars[1] = mouseX;
}

// Primary drawing function
void draw() {
  background(0);
  drawGrid();
  // Get voltages, times of reading
  boolean last_value = false;
  while (!pause && !last_value) {
    val = getValue();
    // Push value/time onto array
    if (val.value != -1 && val.utime != -1)
      pushDataPair(val);
    else 
      last_value = true;
  }
  drawCurrent();
  drawStats();
  drawLines();
  drawVertLines();
  if (help_screen) {
    drawHelp();
  }
}